// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// TODO:
//  1. Support sloped scooping.

#define _USE_MATH_DEFINES
#include <cmath>
#include <sstream>
#include <string_view>
#include <exception>

#include "ow_materials/material_utils.h"

#include "RegolithPlugin.h"

using namespace ow_regolith;
using namespace ow_dynamic_terrain;

using namespace std::literals;

using std::string, std::string_view, std::endl, std::runtime_error, std::sqrt,
      std::abs;

using ignition::math::Vector3d, ignition::math::Quaterniond;

// service paths served by this class
const string SRV_SPAWN_REGOLITH      = "/ow_regolith/spawn_regolith";
const string SRV_REMOVE_ALL_REGOLITH = "/ow_regolith/remove_regolith";

// topic paths used in class
const string TOPIC_TERRAIN_CONTACT   = "/ow_regolith/contacts/terrain";
const string TOPIC_BULK_EXCAVATION   = "/ow_materials/bulk_excavation/visual";
const string TOPIC_DIG_PHASE         = "/ow_dynamic_terrain/scoop_dig_phase";
const string TOPIC_MATERIAL_INGESTED = "/ground_truth/material_ingested";

const string NAMESPACE_MATERIALS = "/ow_materials/materials";

// constants specific to the scoop end-effector
const Vector3d SCOOP_SPAWN_OFFSET(0.0, 0.0, -0.05);

constexpr string_view PLUGIN_NAME{"RegolithPlugin"};
constexpr string_view SCOOP_LINK_NAME{"lander::l_scoop_tip"};

// Time before a BulkExcavation is sent on /ow_materials/material_ingested
// following receipt of a removal request where ingested=True
// NOTE: This interval intentionally matches the interval in
//  DockIngestSampleServer in ow_lander/src/ow_lander/actions.py such that
//  there will only be one BulkExcavation message on this topic per call to
//  DockIngestSample.
const ros::Duration CONSOLIDATE_INGESTED_TIMEOUT = ros::Duration(3.0);

#define GZERR(msg) gzerr << PLUGIN_NAME << ": " << msg
#define GZWARN(msg) gzwarn << PLUGIN_NAME << ": " << msg
#define GZMSG(msg) gzmsg << PLUGIN_NAME << ": " << msg

RegolithPlugin::RegolithPlugin() : m_spawn_offsets(1, SCOOP_SPAWN_OFFSET),
    m_task_queue(std::bind(&RegolithPlugin::processBulkExcavation, this,
                      std::placeholders::_1))
{
  // do nothing
}

template<typename T>
static T get_required_parameter(sdf::ElementPtr sdf, string_view name)
{
  if (!sdf->HasElement(name.data())) {
    std::stringstream ss;
    ss << "Required SDF parameter " << name
       << " missing. Fix the SDF implementation of this plugin.";
    throw runtime_error(ss.str());
  }
  return sdf->Get<T>(name.data());
}

void RegolithPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  constexpr string_view LANDER_MODEL_NAME{"lander"};
  constexpr double SCOOP_WIDTH{0.08}; // meters

  m_node_handle = std::make_unique<ros::NodeHandle>(PLUGIN_NAME.data());

  m_scoop_link = model->GetLink(SCOOP_LINK_NAME.data());
  if (m_scoop_link == nullptr) {
    GZERR("Link by the name " << SCOOP_LINK_NAME << " is not present in "
          << "model " << LANDER_MODEL_NAME << endl);
    return;
  }

  // get plugin parameters from SDF
  string model_uri;
  try {
    model_uri = get_required_parameter<string>(sdf, "regolith_model_uri"sv);
    m_spawn_threshold = get_required_parameter<double>(
      sdf, "spawn_volume_threshold"sv);
  } catch (runtime_error e) {
    GZERR("Failed to build material database: " << e.what() << endl);
    return;
  }

  // if the optional spawn_spacing parameter is present, generate spawn points
  double spawn_spacing;
  if (sdf->HasElement("spawn_spacing")) {
    spawn_spacing = sdf->Get<double>("spawn_spacing");
    if (spawn_spacing < 0.0) {
      GZERR("Spawn spacing must be positive." << endl);
      return;
    }
    const auto additional_spawns = static_cast<int>(
      (SCOOP_WIDTH - spawn_spacing) / (2 * spawn_spacing)
    );
    for (int i = 0; i < additional_spawns; ++i) {
      // populate both sides of the central spawn point with additional points
      Vector3d dy(0.0f, (i + 1) * static_cast<float>(spawn_spacing), 0.0f);
      m_spawn_offsets.push_back(SCOOP_SPAWN_OFFSET + dy);
      m_spawn_offsets.push_back(SCOOP_SPAWN_OFFSET - dy);
    }
  }
  // initialize offset selector
  m_spawn_offset_selector = m_spawn_offsets.begin();

  if (!m_pool.initialize("regolith", model->GetWorld(), model_uri)) {
    GZERR("ParticlePool failed to connect to gazebo_ros services" << endl);
    return;
  }

  // populate materials database
  try {
    m_material_db.populate_from_rosparams(NAMESPACE_MATERIALS);
  } catch (const ow_materials::MaterialConfigError &e) {
    GZERR("Failed initialize material database: " << e.what() << endl);
    return;
  }
  
  // Compute the initial velocity a regolith particle will need to reach the
  // rear of the scoop around the same time it falls to the floor of the scoop.
  const double LENGTH_OF_SCOOP_FLOOR = 0.116; // meters
  const double SPAWN_HEIGHT_ABOVE_SCOOP_FLOOR = 0.05;
  m_regolith_spawn_velocity = LENGTH_OF_SCOOP_FLOOR * sqrt(
    abs(model->GetWorld()->Gravity().Length())
    / (2 * SPAWN_HEIGHT_ABOVE_SCOOP_FLOOR)
  );

  // Compute the time it takes for a particle moving under the above velocity
  // to exit the spawn volume. This tells us how frequently we can spawn a
  // particle at a given spawn location without causing overlap.
  m_spawn_overlap_interval = m_regolith_spawn_velocity > 0.0 ?
    ros::Duration(
      spawn_spacing / m_regolith_spawn_velocity
    )
    : ros::Duration(0.0);

  m_time_of_last_central_spawn = ros::Time::now();

  //// setup ROS facilities
  // advertise services served by this class
  m_srv_spawn_regolith = m_node_handle->advertiseService(
    SRV_SPAWN_REGOLITH, &RegolithPlugin::spawnRegolithSrv, this);
  m_srv_remove_all_regolith = m_node_handle->advertiseService(
    SRV_REMOVE_ALL_REGOLITH, &RegolithPlugin::removeRegolithSrv, this);
  // advertise published topics
  m_pub_material_ingested = m_node_handle
    ->advertise<ow_materials::BulkExcavation>(TOPIC_MATERIAL_INGESTED, 1, true);
  // create timers
  m_consolidate_ingested_timeout = m_node_handle->createTimer(
    CONSOLIDATE_INGESTED_TIMEOUT, &RegolithPlugin::onConsolidateIngestedTimeout,
    this, true, false
  );
  // subscribe to all ROS topics
  m_sub_terrain_contact = m_node_handle->subscribe(TOPIC_TERRAIN_CONTACT,
    1, &RegolithPlugin::onTerrainContact, this);
  m_sub_bulk_excavation = m_node_handle->subscribe(TOPIC_BULK_EXCAVATION,
    10, &RegolithPlugin::onBulkExcavationVisualMsg, this);
  m_sub_dig_phase = m_node_handle->subscribe(TOPIC_DIG_PHASE,
    1, &RegolithPlugin::onDigPhaseMsg, this);

  GZMSG("Successfully loaded!" << endl);
}

void RegolithPlugin::resetDisplacedBulk()
{
  m_bulk_displaced.clear();
}

bool RegolithPlugin::spawnRegolithSrv(SpawnRegolithRequest &request,
                                       SpawnRegolithResponse &response)
{
  Vector3d position(request.position.x, request.position.y, request.position.z);
  auto ret = m_pool.spawn(position, request.reference_frame,
                          ow_materials::Bulk());
  response.success = !ret.empty();
  return true;
}

bool RegolithPlugin::removeRegolithSrv(RemoveRegolithRequest &request,
                                       RemoveRegolithResponse &)
{
  if (request.link_names.empty()) {
    // remove all regolith models
    m_pool.clear();
  } else {
    // remove specific regolith models
    if (request.ingested) {
      m_bulk_ingested.mix(m_pool.removeAndConsolidate(request.link_names));
      resetConsolidatedIngestedTimeout();
    } else {
      m_pool.remove(request.link_names);
    }
  }
  return true;
}

void RegolithPlugin::onTerrainContact(const Contacts::ConstPtr &msg)
{
  if (msg->model_names.empty()) {
    return;
  }
  m_pool.remove(msg->model_names);
}

void RegolithPlugin::onBulkExcavationVisualMsg(
  const ow_materials::BulkExcavation::ConstPtr& msg)
{
  if (msg->header.seq != m_next_expected_seq) {
    GZWARN("Modification message on topic " << m_sub_bulk_excavation.getTopic()
      << " was dropped! At least " << (msg->header.seq - m_next_expected_seq)
      << " message(s) may have been missed." << endl);
  }
  m_next_expected_seq = msg->header.seq + 1;
  if (!m_scoop_is_digging) {
    // do not spawn regolith when the motion does not look like a dig
    return;
  }
  m_task_queue.addTask(*msg);
}

void RegolithPlugin::processBulkExcavation(ow_materials::BulkExcavation *bulk)
{
  const Vector3d SCOOP_FORWARD(1.0, 0.0, 0.0);

  m_bulk_displaced.mix(ow_materials::Bulk(*bulk));
  // Use a for-loop so a maximum number of iterations can be enforced. The
  // maximum iteration is the number of unique spawns, so that more than one
  // particle will never spawn at the same location in the same frame.
  for (uint i = 0u; i != m_spawn_offsets.size(); ++i) {
    // Once volume is less than threshold, this function has done its job,
    if (m_bulk_displaced.getVolume() < m_spawn_threshold) {
      return;
    }
    // reduce tracked bulk by the threshold
    double particle_volume = m_bulk_displaced.reduce(m_spawn_threshold);
    // The returned value is the actual mass take from the bulk, so that is the
    // particle's mass, which also has an identical blend to the reduced bulk.
    ow_materials::Bulk particle_bulk(m_bulk_displaced.getBlend(),
                                     particle_volume);
    // compute particle mass, which requires database lookups
    double mass;
    try {
      mass = ow_materials::computeBulkMass(particle_bulk, m_material_db);
    } catch (ow_materials::MaterialRangeError const &e) {
      GZERR("Material in displaced bulk is somehow not present in the database."
            " MaterialRangeError: " << e.what() << endl);
      return;
    }

    if (ros::Time::now() - m_time_of_last_central_spawn
          <= m_spawn_overlap_interval) {
      // Regolith is being spawned too rapidly and spawning more now may cause a
      // a particle to spawn inside of another particle. This check serves the
      // additional benefit of throttling API calls, which may cause a Gazebo
      // crash if called too rapidly.
      GZWARN(
        "Regolith particle spawn skipped to avoid particle overlap." << endl);
      return;
    }
    // select spawn offset and wrap from end to beginning
    auto offset = *(m_spawn_offset_selector++);
    if (m_spawn_offset_selector ==  m_spawn_offsets.end()) {
      m_spawn_offset_selector = m_spawn_offsets.begin();
      // reset time of central spawn to track how fast regolith spawns are
      // using up space in the spawning region
      m_time_of_last_central_spawn = ros::Time::now();
    }
    // spawn in particle pool
    auto model_name = m_pool.spawn(offset, SCOOP_LINK_NAME.data(),
                                   particle_bulk, mass);
    if (model_name.empty()) {
      GZERR("Failed to spawn regolith particle!" << endl);
      continue;
    }

    const Quaterniond scoop_orientation = m_scoop_link->WorldPose().Rot();
    Vector3d scooping_direction = scoop_orientation * SCOOP_FORWARD;
    scooping_direction.Z() = 0.0; // flatten against X-Y plane
    const Vector3d velocity = -m_regolith_spawn_velocity
                               * scooping_direction.Normalized();

    if (!m_pool.setParticleVelocity(model_name, velocity)) {
      GZERR("Failed to set velocity for regolith " << model_name << endl);
    }
  }
  // The nominal behavior of the previous loop is to return from inside.
  // If execution arrives here and there remains displaced volume above the
  // threshold. This means the volume displaced per frame is larger than the
  // particle spawning system can handle. A warning should be issued.
  if (m_bulk_displaced.getVolume() >= m_spawn_threshold) {
    GZWARN("More volume is being displaced per frame than can be spawned as "
           "regolith. Consider increasing the value of spawn_volume_threshold "
           "in common.launch to avoid this warning." << endl);
  }
}

void RegolithPlugin::onDigPhaseMsg(
  const ow_dynamic_terrain::scoop_dig_phase::ConstPtr &msg)
{
  using ow_dynamic_terrain::scoop_dig_phase;
  m_scoop_is_digging = msg->phase != scoop_dig_phase::NOT_DIGGING;
  switch (msg->phase) {
    // do nothing phases
    case scoop_dig_phase::SINKING:
    case scoop_dig_phase::PLOWING:
    case scoop_dig_phase::RETRACTING:
      break;
    case scoop_dig_phase::NOT_DIGGING:
      resetDisplacedBulk();
      break;
    default:
      GZERR("Unhandled dig state received. This should never happen!" << endl);
  }
}

void RegolithPlugin::onConsolidateIngestedTimeout(const ros::TimerEvent&)
{
  ow_materials::BulkExcavation msg;
  try {
    msg = ow_materials::bulkToBulkExcavationMsg(m_bulk_ingested, m_material_db);
  } catch (ow_materials::MaterialRangeError const &e) {
    gzerr << e.what() << endl;
    return;
  }
  msg.header.stamp = ros::Time::now();
  m_pub_material_ingested.publish(msg);
  m_bulk_ingested.clear();
}

void RegolithPlugin::resetConsolidatedIngestedTimeout()
{
  m_consolidate_ingested_timeout.stop();
  m_consolidate_ingested_timeout.setPeriod(CONSOLIDATE_INGESTED_TIMEOUT);
  m_consolidate_ingested_timeout.start();
}

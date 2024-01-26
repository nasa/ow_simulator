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

#include "RegolithPlugin.h"
#include "ServiceClientFacade.h"

using namespace ow_regolith;
using namespace ow_dynamic_terrain;

using namespace std::literals;
using namespace std::literals::chrono_literals;

using std::string, std::string_view, std::endl, std::runtime_error;

using ignition::math::Vector3d, ignition::math::Quaterniond;

// service paths served by this class
const string SRV_SPAWN_REGOLITH      = "/ow_regolith/spawn_regolith";
const string SRV_REMOVE_ALL_REGOLITH = "/ow_regolith/remove_regolith";

// topic paths used in class
const string TOPIC_TERRAIN_CONTACT = "/ow_regolith/contacts/terrain";
const string TOPIC_BULK_EXCAVATION = "/ow_materials/bulk_excavation/visual";
const string TOPIC_DIG_PHASE       = "/ow_dynamic_terrain/scoop_dig_phase";

// constants specific to the scoop end-effector
const Vector3d SCOOP_SPAWN_OFFSET(0.0, 0.0, -0.05);

constexpr string_view PLUGIN_NAME{"RegolithPlugin"};
constexpr string_view SCOOP_LINK_NAME{"lander::l_scoop_tip"};

#define GZERR(msg) gzerr << PLUGIN_NAME << ": " << msg
#define GZWARN(msg) gzwarn << PLUGIN_NAME << ": " << msg
#define GZMSG(msg) gzmsg << PLUGIN_NAME << ": " << msg

RegolithPlugin::RegolithPlugin() : m_spawn_offsets(1, SCOOP_SPAWN_OFFSET),
    m_queue(std::bind(&RegolithPlugin::processBulkExcavation, this,
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
  constexpr string_view REGOLITH_TAG{"regolith"};
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
  if (sdf->HasElement("spawn_spacing")) {
    double spacing = sdf->Get<double>("spawn_spacing");
    const auto additional_spawns = static_cast<int>((SCOOP_WIDTH - spacing)
                                                    / (2 * spacing));
    for (int i = 0; i < additional_spawns; ++i) {
      // populate both sides of the central spawn point with additional points
      Vector3d dy(0.0f, (i + 1) * static_cast<float>(spacing), 0.0f);
      m_spawn_offsets.push_back(SCOOP_SPAWN_OFFSET + dy);
      m_spawn_offsets.push_back(SCOOP_SPAWN_OFFSET - dy);
    }
  }
  // initialize offset selector
  m_spawn_offset_selector = m_spawn_offsets.begin();

  if (!m_model_pool.initialize("regolith", model->GetWorld(),
                               model_uri, m_node_handle.get())) {
    GZERR("ParticlePool failed to connect to gazebo_ros services" << endl);
    return;
  }

  // create a material database from ROS parameters
  try {
    m_material_db.populate_from_rosparams("/ow_materials");
  } catch (const ow_materials::MaterialConfigError &e) {
    GZERR(e.what() << endl);
    return;
  }

  // advertise services served by this class
  m_srv_spawn_regolith = m_node_handle->advertiseService(
    SRV_SPAWN_REGOLITH, &RegolithPlugin::spawnRegolithSrv, this);
  m_srv_remove_all_regolith = m_node_handle->advertiseService(
    SRV_REMOVE_ALL_REGOLITH, &RegolithPlugin::removeRegolithSrv, this);

  // subscribe to all ROS topics
  m_sub_terrain_contact = m_node_handle->subscribe(TOPIC_TERRAIN_CONTACT,
    1, &RegolithPlugin::onTerrainContact, this);
  m_sub_bulk_excavation = m_node_handle->subscribe(TOPIC_BULK_EXCAVATION,
    10, &RegolithPlugin::onBulkExcavationVisualMsg, this);
  m_sub_dig_phase = m_node_handle->subscribe(TOPIC_DIG_PHASE,
    1, &RegolithPlugin::onDigPhaseMsg, this);

  // set the maximum scoop inclination that the psuedo force can counteract
  // NOTE: do not use a value that makes cosine zero!
  constexpr auto MAX_SCOOP_INCLINATION_DEG = 80.0; // degrees
  constexpr auto MAX_SCOOP_INCLINATION_RAD = MAX_SCOOP_INCLINATION_DEG * M_PI
                                             / 180.0; // radians
  constexpr auto PSUEDO_FORCE_WEIGHT_FACTOR = sin(MAX_SCOOP_INCLINATION_RAD);
  // psuedo force magnitude = model's weight X weight factor
  m_psuedo_force_mag = m_model_pool.getModelMass()
                       * model->GetWorld()->Gravity().Length()
                       * PSUEDO_FORCE_WEIGHT_FACTOR;

  const ros::Duration TASK_QUEUE_PERIOD(0.005);
  m_queue_manager_timer = m_node_handle->createTimer(TASK_QUEUE_PERIOD,
    &RegolithPlugin::manageQueue, this, false, true);

  GZMSG("Successfully loaded!" << endl);
}

void RegolithPlugin::resetDisplacedBulk()
{
  m_bulk_displaced.clear();
}

void RegolithPlugin::clearAllPsuedoForces()
{
  m_model_pool.clearAllForces();
}

bool RegolithPlugin::spawnRegolithSrv(SpawnRegolithRequest &request,
                                       SpawnRegolithResponse &response)
{
  Vector3d position(request.position.x, request.position.y, request.position.z);
  auto ret = m_model_pool.spawn(position, request.reference_frame,
                                 ow_materials::Bulk());
  response.success = !ret.empty();
  return true;
}

bool RegolithPlugin::removeRegolithSrv(RemoveRegolithRequest &request,
                                        RemoveRegolithResponse &response)
{
  if (request.link_names.empty()) {
    // remove all regolith models
    m_model_pool.clear();
  } else {
    // remove specific regolith models
    m_model_pool.remove(request.link_names, request.ingested);
  }
  response.success = response.not_removed.empty();
  return true;
}

void RegolithPlugin::onTerrainContact(const Contacts::ConstPtr &msg)
{
  m_model_pool.remove(msg->link_names, false);
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
  m_queue.addTask(*msg);
}

void RegolithPlugin::manageQueue(const ros::TimerEvent&)
{
  m_queue.manage();
}

void RegolithPlugin::processBulkExcavation(
  const ow_materials::BulkExcavation bulk)
{
  const Vector3d SCOOP_FORWARD(1.0, 0.0, 0.0);

  // DEBUG
  auto start_time = ros::Time::now();

  m_bulk_displaced.mix(ow_materials::Bulk(bulk));

  if (m_bulk_displaced.getVolume() < m_spawn_threshold) {
    // nothing more to do until the threshold is reached
    return;
  }

  // DEBUG LOGGING
  // std::stringstream ss;
  // ss << "{\n";
  // for (auto const &b : m_bulk_displaced.getComposition()) {
  //   ss << "\t" << static_cast<int>(b.first) << ":" << b.second << "\n";
  // }
  // ss << "}";
  // ROS_INFO(ss.str().c_str());

  // Use a for-loop so a maximum number of iterations can be enforced. The
  // maximum iteration is the number of unique spawns, so that more than one
  // particle will never spawn at the same location in the same frame.
  for (uint i = 0u; i != m_spawn_offsets.size(); ++i) {
    // Once volume is less than threshold, this function has done its job,
    if (m_bulk_displaced.getVolume() < m_spawn_threshold) {
      // DEBUG
      ROS_INFO_STREAM("processBulkExcavation call took "
        << (ros::Time::now() - start_time).toSec() * 1000 << " ms");
      return;
    }
    // select spawn offset and wrap from end to beginning
    auto offset = *(m_spawn_offset_selector++);
    if (m_spawn_offset_selector ==  m_spawn_offsets.end()) {
      m_spawn_offset_selector = m_spawn_offsets.begin();
    }
    // reduce tracked bulk by the threshold
    double particle_volume = m_bulk_displaced.reduce(m_spawn_threshold);
    // spawn a regolith model
    ow_materials::Bulk particle_bulk(m_bulk_displaced.getBlend(),
                                     particle_volume);
    auto link_name = m_model_pool.spawn(offset, SCOOP_LINK_NAME.data(),
                                         particle_bulk);
    if (link_name.empty()) {
      GZERR("Failed to spawn regolith particle!" << endl);
      continue;
    }
    // if scoop is retracting, psuedo forces are not need
    if (m_psuedo_force_required) {
      // compute psuedo force direction from scoop orientation
      const Quaterniond scoop_orientation = m_scoop_link->WorldPose().Rot();
      Vector3d scooping_direction = scoop_orientation * SCOOP_FORWARD;
      scooping_direction.Z() = 0.0; // flatten against X-Y plane
      Vector3d pushback = -m_psuedo_force_mag * scooping_direction.Normalized();
      if (!m_model_pool.applyMaintainedForce(link_name, pushback)) {
        GZERR("Failed to apply force to regolith " << link_name << endl);
      }
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
  m_psuedo_force_required = msg->phase == scoop_dig_phase::SINKING ||
                            msg->phase == scoop_dig_phase::PLOWING;
  switch (msg->phase) {
    // do nothing phases
    case scoop_dig_phase::SINKING:
    case scoop_dig_phase::PLOWING:
      break;
    case scoop_dig_phase::RETRACTING:
      clearAllPsuedoForces();
      break;
    case scoop_dig_phase::NOT_DIGGING:
      resetDisplacedBulk();
      clearAllPsuedoForces();
      break;
    default:
      ROS_ERROR("Unhandled dig state received. This should never happen!");
  }
}

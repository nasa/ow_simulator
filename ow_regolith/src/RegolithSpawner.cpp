// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// TODO:
//  1. Support sloped scooping.

#define _USE_MATH_DEFINES
#include <cmath>
#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <gazebo_msgs/GetPhysicsProperties.h>

#include <RegolithSpawner.h>
#include <ServiceClientFacade.h>

using namespace ow_regolith;
using namespace ow_dynamic_terrain;

using namespace sensor_msgs;
using namespace cv_bridge;

using namespace std::literals::chrono_literals;

using std::string, std::begin, std::end, std::distance, std::find,
      std::stringstream, std::make_unique, std::make_shared;

using std::chrono::duration_cast, std::chrono::seconds;

using tf::Vector3, tf::Point, tf::vector3MsgToTF, tf::pointMsgToTF,
      tf::quatRotate, tf::quaternionMsgToTF, tf::tfDot;

using gazebo_msgs::LinkStates, gazebo_msgs::GetPhysicsProperties;

// service paths served by this class
const string SRV_SPAWN_REGOLITH      = "/ow_regolith/spawn_regolith";
const string SRV_REMOVE_ALL_REGOLITH = "/ow_regolith/remove_regolith";
const string SRV_GET_PHYS_PROPS      = "/gazebo/get_physics_properties";

// topic paths used in class
const string TOPIC_LINK_STATES           = "/gazebo/link_states";
const string TOPIC_TERRAIN_CONTACT       = "/ow_regolith/contacts/terrain";
const string TOPIC_MODIFY_TERRAIN_VISUAL = "/ow_dynamic_terrain/modification_differential/visual";
const string TOPIC_DIG_PHASE             = "/ow_dynamic_terrain/scoop_dig_phase";

const string REGOLITH_TAG  = "regolith";

// constants specific to the scoop end-effector
const string SCOOP_LINK_NAME       = "lander::l_scoop_tip";
const Vector3 SCOOP_FORWARD        = Vector3(1.0, 0.0, 0.0);
const Vector3 SCOOP_SPAWN_OFFSET   = Vector3(0.0, 0.0, -0.05);
const double SCOOP_WIDTH           = 0.08; // meters

const ros::Duration SERVICE_CONNECT_TIMEOUT = ros::Duration(5.0);

RegolithSpawner::RegolithSpawner(const string &node_name)
  : m_node_handle(make_shared<ros::NodeHandle>(node_name)),
    m_volume_displaced(0.0),
    m_spawn_offsets(1, SCOOP_SPAWN_OFFSET)
{
  // do nothing
}

bool RegolithSpawner::initialize()
{
  // get node parameters
  if (!m_node_handle->getParam("spawn_volume_threshold", m_spawn_threshold)) {
    ROS_ERROR("Regolith node requires the spawn_volume_threshold parameter.");
    return false;
  }
  string model_uri;
  if (!m_node_handle->getParam("regolith_model_uri", model_uri)) {
    ROS_ERROR("Regolith node requires the regolith_model_uri parameter.");
    return false;
  }
  // if the optional spawn_spacing parameter is present, generate spawn points
  double spawn_spacing;
  if (m_node_handle->getParam("spawn_spacing", spawn_spacing)) {
    const auto additional_spawns = static_cast<int>(
      (SCOOP_WIDTH - spawn_spacing) / (2 * spawn_spacing)
    );
    for (int i = 0; i < additional_spawns; ++i) {
      // populate both sides of the central spawn point with additional points
      Vector3 dy{0.0f, (i + 1) * static_cast<float>(spawn_spacing), 0.0f};
      m_spawn_offsets.push_back(SCOOP_SPAWN_OFFSET + dy);
      m_spawn_offsets.push_back(SCOOP_SPAWN_OFFSET - dy);
    }
  }
  // initialize offset selector
  m_spawn_offset_selector = m_spawn_offsets.begin();

  m_model_pool = make_unique<ModelPool>(m_node_handle);
  if (!m_model_pool->connectServices()) {
    ROS_ERROR("ModelPool failed to connect to gazebo_ros services.");
    return false;
  }

  if (!m_model_pool->setModel(model_uri, REGOLITH_TAG)) {
    ROS_ERROR("Failed to set model.");
    return false;
  }

  // advertise services served by this class
  m_srv_spawn_regolith = m_node_handle->advertiseService(
    SRV_SPAWN_REGOLITH, &RegolithSpawner::spawnRegolithSrv, this);
  m_srv_remove_all_regolith = m_node_handle->advertiseService(
    SRV_REMOVE_ALL_REGOLITH, &RegolithSpawner::removeRegolithSrv, this);

  // subscribe to all ROS topics
  m_sub_link_states = m_node_handle->subscribe(TOPIC_LINK_STATES,
    1, &RegolithSpawner::onLinkStatesMsg, this);
  m_sub_terrain_contact = m_node_handle->subscribe(TOPIC_TERRAIN_CONTACT,
    1, &RegolithSpawner::onTerrainContact, this);
  m_sub_mod_diff_visual = m_node_handle->subscribe(TOPIC_MODIFY_TERRAIN_VISUAL,
    10, &RegolithSpawner::onModDiffVisualMsg, this);
  m_sub_dig_phase = m_node_handle->subscribe(TOPIC_DIG_PHASE,
    1, &RegolithSpawner::onDigPhaseMsg, this);


  // set the maximum scoop inclination that the psuedo force can counteract
  // NOTE: do not use a value that makes cosine zero!
  constexpr auto MAX_SCOOP_INCLINATION_DEG = 80.0f; // degrees
  constexpr auto MAX_SCOOP_INCLINATION_RAD = MAX_SCOOP_INCLINATION_DEG * M_PI
                                             / 180.0f; // radians
  // FIXME: this is not the right formula and results in a force much larger
  //  than the particle weight
  constexpr auto PSUEDO_FORCE_WEIGHT_FACTOR = sin(MAX_SCOOP_INCLINATION_RAD);
  // query gazebo for the gravity vector
  ServiceClientFacade gz_get_phys_props;
  GetPhysicsProperties phys_prop_msg;
  if (!gz_get_phys_props.connect<GetPhysicsProperties>(
        m_node_handle, SRV_GET_PHYS_PROPS, SERVICE_CONNECT_TIMEOUT, false) ||
      !gz_get_phys_props.call(phys_prop_msg)) {
    ROS_ERROR("Failed to connect Gazebo for gravity vector");
    return false;
  }
  Vector3 gravity;
  vector3MsgToTF(phys_prop_msg.response.gravity, gravity);
  // psuedo force magnitude = model's weight X weight factor
  m_psuedo_force_mag = m_model_pool->getModelMass()
                       * gravity.length()
                       * PSUEDO_FORCE_WEIGHT_FACTOR;

  return true;
}

void RegolithSpawner::resetTrackedVolume()
{
  m_volume_displaced = 0.0;
}

void RegolithSpawner::clearAllPsuedoForces()
{
  if (!m_model_pool->clearAllForces()) {
    ROS_WARN("Failed to clear force on one or more regolith particles");
  }
}

bool RegolithSpawner::spawnRegolithSrv(SpawnRegolithRequest &request,
                                       SpawnRegolithResponse &response)
{
  Point position;
  pointMsgToTF(request.position, position);
  auto ret = m_model_pool->spawn(position, request.reference_frame);
  response.success = !ret.empty();
  return true;
}

bool RegolithSpawner::removeRegolithSrv(RemoveRegolithRequest &request,
                                        RemoveRegolithResponse &response)
{
  if (request.link_names.empty()) {
    // remove all regolith models
    response.not_removed = m_model_pool->clear();
  } else {
    // remove specific regolith models
    response.not_removed = m_model_pool->remove(request.link_names);
  }
  response.success = response.not_removed.empty();
  return true;
}

void RegolithSpawner::onLinkStatesMsg(const LinkStates::ConstPtr &msg) 
{
  auto name_it = find(begin(msg->name), end(msg->name), SCOOP_LINK_NAME);
  if (name_it == end(msg->name)) {
    ROS_WARN("Failed to find %s in link states", SCOOP_LINK_NAME.c_str());
    return;
  }
  auto pose_it = begin(msg->pose) + distance(begin(msg->name), name_it);
  quaternionMsgToTF(pose_it->orientation, m_scoop_orientation);
}

void RegolithSpawner::onTerrainContact(const Contacts::ConstPtr &msg)
{
  m_model_pool->remove(msg->link_names);
}

void RegolithSpawner::onModDiffVisualMsg(
  const modified_terrain_diff::ConstPtr& msg)
{
  if (msg->header.seq != m_next_expected_seq) {
    ROS_WARN_STREAM(
      "Modification message on topic " << m_sub_mod_diff_visual.getTopic()
      << " was dropped! At least " << (msg->header.seq - m_next_expected_seq)
      << " message(s) may have been missed."
    );
  }
  m_next_expected_seq = msg->header.seq + 1;

  // if not digging, ignore this modification as it could be caused by a grind
  if (!m_digging) {
    return;
  }

  // import image to so we can traverse it
  auto image_handle = CvImageConstPtr();
  try {
    image_handle = toCvShare(msg->diff, msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  auto rows = image_handle->image.rows;
  auto cols = image_handle->image.cols;
  if (rows <= 0 || cols <= 0) {
    ROS_WARN("Differential image dimensions are zero or negative");
    return;
  }

  const auto pixel_height = msg->height / rows;
  const auto pixel_width = msg->width / cols;
  const auto pixel_area = pixel_height * pixel_height;

  // estimate the total volume displaced using a Riemann sum over the image
  for (auto y = 0; y < rows; ++y) {
    for (auto x = 0; x < cols; ++x) {
      const auto volume = -image_handle->image.at<float>(y, x) * pixel_area;
      m_volume_displaced += volume;
    }
  }

  if (m_volume_displaced >= m_spawn_threshold) {
    // select spawn offset
    auto offset = *(m_spawn_offset_selector++);
    // wrap selector
    if (m_spawn_offset_selector ==  m_spawn_offsets.end())
      m_spawn_offset_selector = m_spawn_offsets.begin();
    // spawn a regolith model
    auto link_name = m_model_pool->spawn(offset, SCOOP_LINK_NAME);
    if (link_name.empty()) {
      ROS_ERROR("Failed to spawn regolith particle");
      return;
    }
    // deduct threshold from tracked volume
    m_volume_displaced -= m_spawn_threshold;
    // if scoop is exiting terrain, adding psuedo forces is unnecesssary
    if (m_retracting) {
      return;
    }
    // compute psuedo force direction from scoop orientation
    Vector3 scooping_vec(quatRotate(m_scoop_orientation, SCOOP_FORWARD));
    scooping_vec.setZ(0.0); // flatten against X-Y plane
    Vector3 pushback_force = -m_psuedo_force_mag * scooping_vec.normalize();
    // choose a long ros duration (1 yr) to delay the disabling of wrench force
    constexpr auto one_year_in_seconds = duration_cast<seconds>(1h*24*365).count();
    if (!m_model_pool->applyForce(link_name, pushback_force,
                                  ros::Duration(one_year_in_seconds))) {
      ROS_ERROR("Failed to apply force to regolith %s", link_name.c_str());
      return;
    }
  }
}

void RegolithSpawner::onDigPhaseMsg(
  const ow_dynamic_terrain::scoop_dig_phase::ConstPtr &msg)
{
  m_digging = msg->digging;
  m_retracting = msg->phase == msg->RETRACTING;
  using ow_dynamic_terrain::scoop_dig_phase;
  switch (msg->phase) {
    // do nothing phases
    case scoop_dig_phase::SINKING:
    case scoop_dig_phase::PLOWING:
      break;
    case scoop_dig_phase::NOT_DIGGING:
      resetTrackedVolume();
      clearAllPsuedoForces();
      break;
    case scoop_dig_phase::RETRACTING:
      clearAllPsuedoForces();
      break;
    default:
      ROS_ERROR("Unhandled dig state received. This should never happen!");
  }
}

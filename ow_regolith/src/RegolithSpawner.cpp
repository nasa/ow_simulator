// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// TODO:
//  1. Support sloped scooping.
//  2. Work around reliance on action callbacks from ow_lander.

#define _USE_MATH_DEFINES
#include <cmath>
#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <gazebo_msgs/GetPhysicsProperties.h>

#include <RegolithSpawner.h>

using namespace ow_regolith;
using namespace ow_dynamic_terrain;
using namespace ow_lander;

using namespace sensor_msgs;
using namespace cv_bridge;

using namespace std::literals::chrono_literals;

using std::string, std::begin, std::end, std::distance, std::find,
      std::stringstream, std::make_unique;

using std::chrono::duration_cast, std::chrono::seconds;

using tf::Vector3, tf::Point, tf::vector3MsgToTF, tf::pointMsgToTF,
      tf::quatRotate, tf::quaternionMsgToTF, tf::tfDot;

using gazebo_msgs::LinkStates, gazebo_msgs::GetPhysicsProperties;

// service paths served by this class
const static string SRV_SPAWN_REGOLITH      = "/ow_regolith/spawn_regolith";
const static string SRV_REMOVE_ALL_REGOLITH = "/ow_regolith/remove_regolith";
const static string SRV_GET_PHYS_PROPS      = "/gazebo/get_physics_properties";

// topic paths used in class
const static string TOPIC_LINK_STATES           = "/gazebo/link_states";
const static string TOPIC_TERRAIN_CONTACT       = "/ow_regolith/contacts/terrain";

const static string TOPIC_MODIFY_TERRAIN_VISUAL = "/ow_dynamic_terrain/modification_differential/visual";
const static string TOPIC_DIG_LINEAR_RESULT     = "/DigLinear/result";
const static string TOPIC_DIG_CIRCULAR_RESULT   = "/DigCircular/result";

const static string REGOLITH_TAG = "regolith";

// constants specific to the scoop end-effector
const static string SCOOP_LINK_NAME       = "lander::l_scoop_tip";
const static Vector3 SCOOP_FORWARD        = Vector3(1.0, 0.0, 0.0);
const static Vector3 SCOOP_DOWNWARD       = Vector3(0.0, 0.0, 1.0);

const static Vector3 WORLD_DOWNWARD = Vector3(0.0, 0.0, -1.0);

const static Vector3 AUTO_SPAWN_OFFSET = Point(0.0, 0.0, 0.03);

const static ros::Duration SERVICE_CONNECT_TIMEOUT = ros::Duration(5.0);

RegolithSpawner::RegolithSpawner(const string &node_name)
  : m_node_handle(new ros::NodeHandle(node_name)),
    m_volume_displaced(0.0), m_volume_center(0.0, 0.0, 0.0)
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
  string model_uri = "";
  if (!m_node_handle->getParam("regolith_model_uri", model_uri)) {
    ROS_ERROR("Regolith node requires the regolith_model_uri parameter.");
    return false;
  }

  m_pool = make_unique<ModelPool>(m_node_handle);
  if (!m_pool->connectServices()) {
    ROS_ERROR("ModelPool failed to connect to gazebo_ros services.");
    return false;
  }

  if (!m_pool->setModel(model_uri, REGOLITH_TAG)) {
    ROS_ERROR("Failed to set model.");
    return false;
  }

  // advertise services served by this class
  m_srv_spawn_regolith = m_node_handle->advertiseService(
    SRV_SPAWN_REGOLITH, &RegolithSpawner::spawnRegolithSrv, this);
  m_srv_remove_all_regolith = m_node_handle->advertiseService(
    SRV_REMOVE_ALL_REGOLITH, &RegolithSpawner::removeRegolithSrv, this);

  // subscribe to all ROS topics
  m_sub_link_states = m_node_handle->subscribe(
    TOPIC_LINK_STATES, 1, &RegolithSpawner::onLinkStatesMsg, this);
  m_sub_terrain_contact = m_node_handle->subscribe(
    TOPIC_TERRAIN_CONTACT, 1, &RegolithSpawner::onTerrainContact, this);
  m_sub_mod_diff_visual = m_node_handle->subscribe(
    TOPIC_MODIFY_TERRAIN_VISUAL, 1, &RegolithSpawner::onModDiffVisualMsg, this);
  m_sub_dig_linear_result = m_node_handle->subscribe(
    TOPIC_DIG_LINEAR_RESULT, 1, &RegolithSpawner::onDigLinearResultMsg, this);
  m_sub_dig_circular_result = m_node_handle->subscribe(
    TOPIC_DIG_CIRCULAR_RESULT, 1, &RegolithSpawner::onDigCircularResultMsg, this);

  // set the maximum scoop inclination that the psuedo force can counteract
  // NOTE: do not use a value that makes cosine zero!
  constexpr auto MAX_SCOOP_INCLINATION_DEG = 80.0f; // degrees
  constexpr auto MAX_SCOOP_INCLINATION_RAD = MAX_SCOOP_INCLINATION_DEG * M_PI / 180.0f; // radians
  constexpr auto PSUEDO_FORCE_WEIGHT_FACTOR = 1.0f / cos(MAX_SCOOP_INCLINATION_RAD);
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
  m_psuedo_force_mag
    = m_pool->getModelMass() * gravity.length() * PSUEDO_FORCE_WEIGHT_FACTOR;

  return true;
}

bool RegolithSpawner::spawnRegolithSrv(SpawnRegolithRequest &request,
                                       SpawnRegolithResponse &response)
{
  Point position;
  pointMsgToTF(request.position, position);
  auto ret = m_pool->spawn(position, request.reference_frame);
  response.success = !ret.empty();
  return true;
}

bool RegolithSpawner::removeRegolithSrv(RemoveRegolithRequest &request,
                                        RemoveRegolithResponse &response)
{
  response.not_removed = m_pool->remove(request.link_names);
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
  m_pool->remove(msg->link_names);
}

void RegolithSpawner::onModDiffVisualMsg(const modified_terrain_diff::ConstPtr& msg)
{
  // check if visual terrain modification was caused by the scoop
  Vector3 scoop_bottom(quatRotate(m_scoop_orientation, SCOOP_DOWNWARD));
  if (tfDot(scoop_bottom, WORLD_DOWNWARD) < 0.0)
    // Scoop bottom is pointing up, which is not a proper scooping orientation.
    // This can be the result of a deep grind, and should not result in 
    // particles being spawned.
    return;

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
    ROS_DEBUG("Differential image dimensions are zero or negative");
    return;
  }

  const auto pixel_height = msg->height / rows;
  const auto pixel_width = msg->width / cols;
  const auto pixel_area = pixel_height * pixel_height;

  const Point image_corner_to_midpoint(msg->height / 2, msg->width / 2, 0);

  // estimate the total volume displaced using a Riemann sum over the image
  for (auto y = 0; y < rows; ++y) {
    for (auto x = 0; x < cols; ++x) {
      const auto volume = -image_handle->image.at<float>(y, x) * pixel_area;
      const auto image_position = Point(pixel_width * x, pixel_height * y, 0) - image_corner_to_midpoint;
      m_volume_displaced += volume;
      // position of the changed volume relative modification center
      m_volume_center += volume * image_position;
    }
  }

  Point world_position;
  pointMsgToTF(msg->position, world_position);

  if (m_volume_displaced >= m_spawn_threshold) {
    // calculate weighted center of volume displacement in world coordinates
    m_volume_center = m_volume_center / m_volume_displaced + world_position;
    // spawn a regolith model
    auto link_name = m_pool->spawn(m_volume_center + AUTO_SPAWN_OFFSET, "world");
    if (link_name.empty()) {
      ROS_ERROR("Failed to spawn regolith particle");
      return;
    }
    // deduct threshold from tracked volume and recent volume center to zero
    m_volume_displaced -= m_spawn_threshold;
    m_volume_center = Point(0, 0, 0);
    // compute psuedo force direction from scoop orientation
    Vector3 scooping_vec(quatRotate(m_scoop_orientation, SCOOP_FORWARD));
    scooping_vec.setZ(0.0); // flatten against X-Y plane
    Vector3 pushback_force = -m_psuedo_force_mag * scooping_vec.normalize();
    // Choose a long ros duration (1 year), to delay the disabling of wrench force
    constexpr auto one_year_in_seconds = duration_cast<seconds>(1h*24*365).count();
    if (!m_pool->applyForce(link_name, pushback_force,
                            ros::Duration(one_year_in_seconds))) {
      ROS_ERROR("Failed to apply force to regolith %s", link_name.c_str());
      return;
    }
  }
}

void RegolithSpawner::reset()
{
  if (!m_pool->clearAllForces())
    ROS_WARN("Failed to clear force on one or more regolith particles");
  // reset to avoid carrying over volume from one digging event to the next
  m_volume_displaced = 0.0;
  m_volume_center = Vector3(0.0, 0.0, 0.0);
}

void RegolithSpawner::onDigLinearResultMsg(const DigLinearActionResult::ConstPtr &msg)
{
  reset();
}

void RegolithSpawner::onDigCircularResultMsg(const DigCircularActionResult::ConstPtr &msg)
{
  reset();
}

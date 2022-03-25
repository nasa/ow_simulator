// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// TODO:
//  1. Support sloped scooping.
//  2. Work around reliance on action callbacks from ow_lander.

#define _USE_MATH_DEFINES
#include <cmath>

#include <cv_bridge/cv_bridge.h>

#include <RegolithSpawner.h>

using namespace ow_regolith;
using namespace ow_dynamic_terrain;
using namespace ow_lander;

using namespace sensor_msgs;
using namespace cv_bridge;
using namespace std;

using namespace tf;

using gazebo_msgs::LinkStates;

// service paths served by this class
const static string SRV_SPAWN_REGOLITH      = "/ow_regolith/spawn_regolith";
const static string SRV_REMOVE_ALL_REGOLITH = "/ow_regolith/remove_regolith";

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

  if (!m_pool->setModel(model_uri, REGOLITH_TAG)) {
    ROS_ERROR("Failed to create model pool.");
    return false;
  }

  // advertise services served by this class
  m_srv_spawn_regolith = m_node_handle->advertiseService(
    SRV_SPAWN_REGOLITH, &RegolithSpawner::spawnRegolithSrv, this);
  m_srv_remove_all_regolith     = m_node_handle->advertiseService(
    SRV_REMOVE_ALL_REGOLITH, &RegolithSpawner::removeRegolithSrv, this);

  // subscribe to all ROS topics
  m_sub_link_states     = m_node_handle->subscribe(
    TOPIC_LINK_STATES, 1, &RegolithSpawner::onLinkStatesMsg, this);
  m_sub_terrain_contact = m_node_handle->subscribe(
    TOPIC_TERRAIN_CONTACT, 1, &RegolithSpawner::onTerrainContact, this);
  m_mod_diff_visual     = m_node_handle->subscribe(
    TOPIC_MODIFY_TERRAIN_VISUAL, 1, &RegolithSpawner::onModDiffVisualMsg, this);
  m_dig_linear_result   = m_node_handle->subscribe(
    TOPIC_DIG_LINEAR_RESULT, 1, &RegolithSpawner::onDigLinearResultMsg, this);
  m_dig_circular_result = m_node_handle->subscribe(
    TOPIC_DIG_CIRCULAR_RESULT, 1, &RegolithSpawner::onDigCircularResultMsg, this);

  return true;
}

// bool RegolithSpawner::applyScoopPushback(string body_name) {
//   // compute scooping direction from scoop orientation
//   Vector3 scooping_vec(quatRotate(m_scoop_orientation, SCOOP_FORWARD));
//   // flatten scooping_vec against the X-Y plane
//   scooping_vec.setZ(0.0);
//   // define pushback direction as opposite to the scooping direction
//   Vector3 pushback_vec(-scooping_vec.normalize());

//   // apply psuedo force to keep model in the scoop
//   ApplyBodyWrench wrench_msg;
//   vector3TFToMsg(m_psuedo_force_mag * pushback_vec,
//                  wrench_msg.request.wrench.force);

//   wrench_msg.request.body_name         = body_name;

//   // Choose a long ros duration (1 year), to delay the disabling of wrench force
//   auto one_year_in_seconds = duration_cast<seconds>(1h*24*365).count();
//   wrench_msg.request.duration          = ros::Duration(one_year_in_seconds);

//   wrench_msg.request.reference_point.x = 0.0;
//   wrench_msg.request.reference_point.y = 0.0;
//   wrench_msg.request.reference_point.z = 0.0;

//   wrench_msg.request.wrench.torque.x   = 0.0;
//   wrench_msg.request.wrench.torque.y   = 0.0;
//   wrench_msg.request.wrench.torque.z   = 0.0;

//   return m_gz_apply_wrench.call(wrench_msg);
// }

// bool RegolithSpawner::clearAllPsuedoForces()
// {
//   bool service_call_error_occurred = false;
//   BodyRequest msg;
//   for (auto &regolith : m_active_models) {
//     msg.request.body_name = regolith.body_name;
//     if (!m_gz_clear_wrench.call(msg)) {
//       ROS_WARN("Failed to clear force on %s", regolith.body_name.c_str());
//       service_call_error_occurred = true;
//     }
//   }
//   return !service_call_error_occurred;
// }

bool RegolithSpawner::spawnRegolithSrv(SpawnRegolithRequest &request,
                                       SpawnRegolithResponse &response)
{
  Point position;
  pointMsgToTF(request.position, position);
  response.success = m_pool->spawn(position, request.reference_frame);
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
    if (!m_pool->spawn(m_volume_center + AUTO_SPAWN_OFFSET, "world")) {
      ROS_ERROR("Failed to spawn regolith in scoop");
      return;
    }
    // deduct threshold from tracked volume and recent volume center to zero
    m_volume_displaced -= m_spawn_threshold;
    m_volume_center = Point(0, 0, 0);
    // pushback most recently spawned model
    // if (!applyScoopPushback(m_active_models.back().body_name)) {
    //   ROS_ERROR("Failed apply pushback force to regolith model");
    //   return;
    // }
  }
}

void RegolithSpawner::resetTrackedVolume()
{
  // reset to avoid carrying over volume from one digging event to the next
  m_volume_displaced = 0.0;
  m_volume_center = Vector3(0.0, 0.0, 0.0);
}

void RegolithSpawner::onDigLinearResultMsg(const DigLinearActionResult::ConstPtr &msg)
{
  // clearAllPsuedoForces();
  resetTrackedVolume();
}

void RegolithSpawner::onDigCircularResultMsg(const DigCircularActionResult::ConstPtr &msg)
{
  // clearAllPsuedoForces();
  resetTrackedVolume();
}

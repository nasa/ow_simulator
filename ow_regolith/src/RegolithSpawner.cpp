// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "RegolithSpawner.h"

#include "sdf_utility.h"

#include <cmath>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/BodyRequest.h>

using namespace ow_dynamic_terrain;
using namespace ow_lander;
using namespace gazebo_msgs;
using namespace sensor_msgs;
using namespace cv_bridge;
using namespace sdf_utility;

using tf::Vector3;

using std::string;
using std::stringstream;
using std::acos;
using std::cos;
using std::unique_ptr;

// service paths used in class
const static string SRV_GET_PHYS_PROPS = "/gazebo/get_physics_properties";
const static string SRV_SPAWN_MODEL    = "/gazebo/spawn_sdf_model";
const static string SRV_DELETE_MODEL   = "/gazebo/delete_model";
const static string SRV_APPLY_WRENCH   = "/gazebo/apply_body_wrench";
const static string SRV_CLEAR_WRENCH   = "/gazebo/clear_body_wrenches";

// topic paths used in class
const static string TOPIC_MODIFY_TERRAIN_VISUAL = "/ow_dynamic_terrain/modification_differential/visual";
const static string TOPIC_DIG_LINEAR_RESULT     = "/DigLinear/result";
const static string TOPIC_DIG_CIRCULAR_RESULT   = "/DigCircular/result";
const static string TOPIC_DELIVER_RESULT        = "/Deliver/result";

template <class T>
static bool callRosService(ros::ServiceClient &srv, T &msg) 
{
  if (!srv.isValid()) {
    ROS_ERROR("Connection to service %s has been lost", 
      srv.getService().c_str()
    );
    return false;
  }
  if (!srv.call(msg)) {
    ROS_ERROR("Failed to call service %s", srv.getService().c_str());
    return false;
  } 
  return true;
}

template <class T>
static bool createRosServiceClient(unique_ptr<ros::NodeHandle> &nh, 
  string service_path, ros::ServiceClient &out_srv, bool persistent = false)
{
  // maximum time we will wait for the service to exist
  constexpr auto SERVICE_TIMEOUT = 5.0f; // seconds
  out_srv = nh->serviceClient<T>(service_path, persistent);
  if (!out_srv.waitForExistence(ros::Duration(SERVICE_TIMEOUT))) {
    ROS_ERROR("Timed out waiting for service %s to advertise", 
      service_path.c_str()
    );
    return false;
  } 
  // DEBUG CODE
  // if (!out_srv.exists()) {
  //   ROS_DEBUG("Service does not exist");
  //   return false;
  // }
  return true;
}

RegolithSpawner::RegolithSpawner(ros::NodeHandle* nh)
  : m_volume_displaced(0.0), m_node_handle(nh) 
{
  // get node parameters, which mirror class data member
  if (!m_node_handle->getParam("spawn_volume_threshold", m_spawn_threshold))
    ROS_ERROR("Regolith node requires the spawn_volume_threshold parameter.");
  if (!m_node_handle->getParam("regolith_model_uri", m_model_uri))
    ROS_ERROR("Regolith node requires the regolith_model_uri paramter.");
}

bool RegolithSpawner::initialize(void) 
{
  // set the maximum scoop inclination that the psuedo force can counteract
  // to be 45 degrees (excluding friction)
  constexpr auto MAX_SCOOP_INCLINATION_DEG = 45.0f; // degrees
  constexpr auto MAX_SCOOP_INCLINATION_RAD 
    = MAX_SCOOP_INCLINATION_DEG * acos(-1) / 180; // radians
  constexpr auto PSUEDO_FORCE_WEIGHT_FACTOR 
    = 1.0f / cos(MAX_SCOOP_INCLINATION_RAD);

  // load SDF model
  if (!getSdfFromUri(m_model_uri, m_model_sdf)) {
    ROS_ERROR("Failed to load SDF for regolith model");
    return false;
  }
  // parse as SDF object to query properties about the model
  auto sdf = parseSdf(m_model_sdf);
  float model_mass;
  if (!getModelLinkMass(sdf, model_mass) ||
      !getModelLinkName(sdf, m_model_linkname)) {
    ROS_ERROR("Failed to acquire regolith model SDF parameters");
    return false;
  }

  // connect to all ROS services
  ros::ServiceClient gz_get_phys_prop;
  if (!createRosServiceClient<GetPhysicsProperties>(m_node_handle, 
        SRV_GET_PHYS_PROPS, gz_get_phys_prop, false) ||
      !createRosServiceClient<SpawnModel>(m_node_handle, 
        SRV_SPAWN_MODEL, m_gz_spawn_model, true) ||
      !createRosServiceClient<DeleteModel>(m_node_handle, 
        SRV_DELETE_MODEL, m_gz_delete_model, true) ||
      !createRosServiceClient<ApplyBodyWrench>(m_node_handle, 
        SRV_APPLY_WRENCH, m_gz_apply_wrench, true) ||
      !createRosServiceClient<BodyRequest>(m_node_handle, 
        SRV_CLEAR_WRENCH, m_gz_clear_wrench, true)) {
    ROS_ERROR("Failed to connect to all required ROS services");
    return false;
  }

  // query gazebo for the gravity vector
  GetPhysicsProperties msg;
  if (!callRosService(gz_get_phys_prop, msg)) {
    ROS_ERROR("Failed to query Gazebo for gravity vector");
    return false;
  }
  Vector3 gravity(
    msg.response.gravity.x, msg.response.gravity.y, msg.response.gravity.z
  );
  // set psuedo force to the model's weight multiplied by the weight factor
  m_psuedo_force_mag 
    = model_mass * gravity.length() * PSUEDO_FORCE_WEIGHT_FACTOR;

  // DEBUG CODE
  // ROS_DEBUG("model mass       = %2f", model_mass);
  // ROS_DEBUG("psuedo force mag = %2f", m_psuedo_force_mag);

  // subscribe callbacks to ROS topics
  m_modify_terrain_visual = m_node_handle->subscribe(
    TOPIC_MODIFY_TERRAIN_VISUAL, 1, &RegolithSpawner::terrainVisualModCb, this);
  m_dig_linear_result = m_node_handle->subscribe(
    TOPIC_DIG_LINEAR_RESULT, 1, &RegolithSpawner::armDigLinearResultCb, this);
  m_dig_circular_result = m_node_handle->subscribe(
    TOPIC_DIG_CIRCULAR_RESULT, 1, &RegolithSpawner::armDigCircularResultCb, this);
  m_deliver_result = m_node_handle->subscribe(
    TOPIC_DELIVER_RESULT, 1, &RegolithSpawner::armDeliverResultCb, this);

  return true;
}

bool RegolithSpawner::spawnRegolithInScoop(Vector3 pushback_direction)
{
  ROS_INFO("Spawning regolith");

  // spawn model
  static auto spawn_count = 0;
  stringstream model_name;
  model_name << "regolith_" << spawn_count++;

  SpawnModel spawn_msg;
  
  // DEBUG CODE
  // ROS_DEBUG("SDF = %s", m_model_sdf.c_str());
  // ROS_DEBUG("model_name = %s", model_name.str().c_str());

  spawn_msg.request.model_name                  = model_name.str();
  spawn_msg.request.model_xml                   = m_model_sdf;
  spawn_msg.request.robot_namespace             = "/regolith";
  spawn_msg.request.reference_frame             = "lander::l_scoop_tip";

  spawn_msg.request.initial_pose.position.x     = 0.0;
  spawn_msg.request.initial_pose.position.y     = 0.0;
  spawn_msg.request.initial_pose.position.z     = -0.05;
  
  spawn_msg.request.initial_pose.orientation.x  = 0.0;
  spawn_msg.request.initial_pose.orientation.y  = 0.0;
  spawn_msg.request.initial_pose.orientation.z  = 0.0;
  spawn_msg.request.initial_pose.orientation.w  = 0.0;
  
  if (!callRosService(m_gz_spawn_model, spawn_msg))
    return false;

  // apply a force to keep model in the scoop
  stringstream body_name;
  body_name << model_name.str() << "::" << m_model_linkname;

  ApplyBodyWrench wrench_msg;
  
  wrench_msg.request.body_name         = body_name.str();
  wrench_msg.request.reference_frame   = "world";
  
  wrench_msg.request.reference_point.x = 0.0;
  wrench_msg.request.reference_point.y = 0.0;
  wrench_msg.request.reference_point.z = 0.0;
  
  tf::vector3TFToMsg(
    m_psuedo_force_mag * pushback_direction, 
    wrench_msg.request.wrench.force
  );
  
  wrench_msg.request.wrench.torque.x   = 0.0;
  wrench_msg.request.wrench.torque.y   = 0.0;
  wrench_msg.request.wrench.torque.z   = 0.0;
  
  // negative duration means the force is applied until we clear it
  wrench_msg.request.duration          = ros::Duration(-1);

  // DEBUG CODE
  ROS_DEBUG("Psuedo force = (%2f, %2f, %2f)", 
    wrench_msg.request.wrench.force.x,
    wrench_msg.request.wrench.force.y,
    wrench_msg.request.wrench.force.z
  );
  // ROS_DEBUG("Duration = %4f seconds", wrench_msg.request.duration.toSec());

  m_active_models.push_back({model_name.str(), body_name.str()});

  return callRosService(m_gz_apply_wrench, wrench_msg);
}


void RegolithSpawner::clearAllPsuedoForces(void)
{
  for (auto &regolith : m_active_models) {
    BodyRequest msg;
    msg.request.body_name = regolith.body_name;
    if (!callRosService(m_gz_clear_wrench, msg))
      ROS_WARN("Failed to clear force on %s", regolith.body_name.c_str());
  }
}

void RegolithSpawner::removeAllRegolithModels(void)
{
  auto it = m_active_models.begin();
  while (it != m_active_models.end()) {
    DeleteModel msg;
    msg.request.model_name = it->model_name;
    if (callRosService(m_gz_delete_model, msg)) {
      it = m_active_models.erase(it);
    } else {
      ROS_WARN("Failed to delete model %s", it->model_name.c_str());
      ++it;
    }
  }
}

void RegolithSpawner::terrainVisualModCb(const modified_terrain_diff::ConstPtr& msg)
{
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

  auto pixel_area = (msg->height / rows) * (msg->width / cols);

  Vector3 center(msg->position.x, msg->position.y, msg->position.z);

  for (auto y = 0; y < rows; ++y)
    for (auto x = 0; x < cols; ++x)
      m_volume_displaced += -image_handle->image.at<float>(y, x) * pixel_area;

  // DEBUG OUTPUT
  // ROS_DEBUG("Volume displaced: %f", m_volume_displaced);

  if (m_volume_displaced >= m_spawn_threshold) {
    // deduct threshold from tracked volume
    m_volume_displaced -= m_spawn_threshold;
    // compute scooping direction
    auto scooping_dir = center - m_previous_center;
    scooping_dir.setZ(0.0); // flatten against X-Y plane (cheap projection)

    if (!spawnRegolithInScoop(-scooping_dir.normalize()))
      ROS_ERROR("Failed to spawn regolith in scoop");
  }

  // cache center so on next call we know in what direction the scoop has moved
  m_previous_center = center;
}

void RegolithSpawner::armDigLinearResultCb(const DigLinearActionResult::ConstPtr &msg)
{
  clearAllPsuedoForces();
}

void RegolithSpawner::armDigCircularResultCb(const DigCircularActionResult::ConstPtr &msg)
{
  clearAllPsuedoForces();
}

void RegolithSpawner::armDeliverResultCb(const DeliverActionResult::ConstPtr &msg)
{
  removeAllRegolithModels();
}
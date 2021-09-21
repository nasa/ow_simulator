// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "RegolithSpawner.h"

#include <fstream>
#include <cmath>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/GetPhysicsProperties.h>

using namespace ow_dynamic_terrain;
using namespace sensor_msgs;
using namespace gazebo_msgs;
using namespace cv_bridge;

using tf::Vector3;

using std::string;
using std::ifstream;
using std::stringstream;
using std::acos;
using std::cos;

static string getSdfFromUri(const string &model_uri) {
  auto model_path = gazebo::common::ModelDatabase::Instance()
    ->GetModelFile(model_uri);
  ifstream file(model_path);
  if (file) {
    stringstream ss;
    ss << file.rdbuf();
    return ss.str();
  } else {
    ROS_ERROR("SDF file %s could not be opened!", model_uri.c_str());
    ROS_ERROR("Error code: %s", strerror(errno));
    return "";
  }
}

static float getModelMass(const string &model_sdf) {
  // parse SDF string for all links
  sdf::SDFPtr sdf_data(new sdf::SDF());
  sdf_data->SetFromString(model_sdf);
  auto root = sdf_data->Root();
  auto model = root->GetElement("model");
  auto link = model->GetElement("link");

  // sum the mass of all links
  auto model_mass = 0.0f;
  while (link) {
    // DEBUG CODE
    // ROS_DEBUG("Link %s", link->GetAttribute("name")->GetAsString());
    float link_mass;
    link->GetElement("inertial")->GetElement("mass")->GetValue()
      ->Get(link_mass);
    model_mass += link_mass;
    // go to next link element
    link = link->GetNextElement("link");
  }

  return model_mass;
}

template <class T>
static bool callRosService(ros::ServiceClient &srv, T &msg) {
  if (!srv.call(msg)) {
    ROS_ERROR("Service %s failed: %s", 
      srv.getService().c_str(), msg.response.status_message.c_str()
    );
    return false;
  } else {
    return true;
  }
};

RegolithSpawner::RegolithSpawner(ros::NodeHandle* nh)
  : m_volume_displaced(0.0), m_node_handle(nh) {

  // get node parameters, which mirror class data member
  if (!m_node_handle->getParam("spawn_volume_threshold", m_spawn_threshold))
    ROS_ERROR("Regolith node requires the spawn_volume_threshold parameter.");
  if (!m_node_handle->getParam("regolith_model_uri", m_model_uri))
    ROS_ERROR("Regolith node requires the regolith_model_uri paramter.");

  // connect to services for spawning models and applying forces to models
  // NOTE: Using persistence produces an error each time the service is called
  // TODO: Keep experimenting with persistence, should provide speed boost if it works
  m_gz_spawn = m_node_handle->serviceClient<SpawnModel>(
    "/gazebo/spawn_model"
  );
  m_gz_wrench = m_node_handle->serviceClient<ApplyBodyWrench>(
    "/gazebo/apply_body_wrench"
  );
}

bool RegolithSpawner::initializeRegolith(void) {
  // set the maximum scoop inclination that the psuedo force can counteract
  // to be 45 degrees
  constexpr float MAX_SCOOP_INCLINATION_DEG = 45; // degrees
  constexpr float MAX_SCOOP_INCLINATION_RAD 
    = MAX_SCOOP_INCLINATION_DEG * acos(-1) / 180; // radians
  constexpr float PSUEDO_FORCE_WEIGHT_FACTOR 
    = 1 / cos(MAX_SCOOP_INCLINATION_RAD);

  // load SDF model
  m_model_sdf = getSdfFromUri(m_model_uri);
  if (m_model_sdf.empty()) {
    ROS_ERROR("Failed to acquire regolith model SDF");
    return false;
  }

  // set psuedo force to the model's weight
  auto model_mass = getModelMass(m_model_sdf);
  auto gz_get_phys_prop = m_node_handle->serviceClient<GetPhysicsProperties>(
    "/gazebo/get_physics_properties"
  );
  GetPhysicsProperties msg;
  // FIXME: blocks execution, is there a better way?
  gz_get_phys_prop.waitForExistence();
  if (!callRosService(gz_get_phys_prop, msg)) {
    ROS_ERROR("Failed to query Gazebo for gravity vector");
    return false;
  }
  Vector3 gravity(
    msg.response.gravity.x, msg.response.gravity.y, msg.response.gravity.z
  );
  m_psuedo_force_mag 
    = model_mass * gravity.length() * PSUEDO_FORCE_WEIGHT_FACTOR;

  // DEBUG CODE
  ROS_DEBUG("model mass       = %2f", model_mass);
  ROS_DEBUG("psuedo force mag = %2f", m_psuedo_force_mag);

  return true;
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

bool RegolithSpawner::spawnRegolithInScoop(Vector3 pushback_direction)
{
  ROS_INFO("Spawning regolith");

  // spawn model
  static auto spawn_count = 0;
  stringstream model_name;
  model_name << "regolith_" << spawn_count++;

  SpawnModel spawn_msg;
  
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
  
  if (!callRosService(m_gz_spawn, spawn_msg))
    return false;

  // apply a force to keep model in the scoop
  stringstream body_name;
  body_name << model_name.str() << "::link";

  ApplyBodyWrench wrench_msg;
  
  wrench_msg.request.body_name                  = body_name.str();
  wrench_msg.request.reference_frame            = "world";
  
  wrench_msg.request.reference_point.x          = 0.0;
  wrench_msg.request.reference_point.y          = 0.0;
  wrench_msg.request.reference_point.z          = 0.0;
  
  tf::vector3TFToMsg(
    m_psuedo_force_mag * pushback_direction, 
    wrench_msg.request.wrench.force
  );
  
  wrench_msg.request.wrench.torque.x            = 0.0;
  wrench_msg.request.wrench.torque.y            = 0.0;
  wrench_msg.request.wrench.torque.z            = 0.0;
  
  // TODO: Compute appropriate duration based on dig_linear arguments
  //       Forces should stop about when dig_linear stops
  //       Or when scoop is upright and out of the ground
  wrench_msg.request.duration                   = ros::Duration(10.0);

  // DEBUG CODE
  ROS_DEBUG("Psuedo force = (%2f, %2f, %2f)", 
    wrench_msg.request.wrench.force.x,
    wrench_msg.request.wrench.force.y,
    wrench_msg.request.wrench.force.z
  );
  ROS_DEBUG("Duration = %4f seconds", wrench_msg.request.duration.toSec());

  return callRosService(m_gz_wrench, wrench_msg);
}
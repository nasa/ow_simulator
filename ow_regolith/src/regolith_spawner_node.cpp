// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ApplyBodyWrench.h>

#include <gazebo/common/common.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

#include "ow_dynamic_terrain/modified_terrain_diff.h"

using namespace sensor_msgs;
using namespace gazebo_msgs;
using namespace cv_bridge;
using namespace ow_dynamic_terrain;

static std::string getGazeboModelSdf(std::string const &model_uri) {
  auto model_path = gazebo::common::ModelDatabase::Instance()->GetModelFile(model_uri);
  std::ifstream file(model_path);
  if (file) {
    std::stringstream ss;
    ss << file.rdbuf();
    return ss.str();
  } else {
    ROS_ERROR("SDF file %s could not be opened!", model_uri.c_str());
    ROS_ERROR("Error code: %s", strerror(errno));
    return "";
  }
}

// TODO: How to make this function gauruntee msg won't be modified?
//       Making the msg reference constant causes a compile error
template <class T>
static bool call_ros_service(ros::ServiceClient &srv, T &msg) {
  if (!srv.call(msg)) {
    ROS_ERROR("Failed to call service %s!", srv.getService().c_str());
    ROS_ERROR("Service failed with status: %s", msg.response.status_message.c_str());
    return false;
  } else {
    return true;
  }
};

class RegolithSpawner
{
public:
  RegolithSpawner(ros::NodeHandle* nh)
    : m_volume_displaced(0), m_spawn_threshold(0.0005), m_node_handle(nh)
  {
    m_gazebo_spawn = m_node_handle->serviceClient<SpawnModel>("/gazebo/spawn_sdf_model");//, true);
    m_gazebo_wrench = m_node_handle->serviceClient<ApplyBodyWrench>("/gazebo/apply_body_wrench");//, true);
  }

  void terrainVisualModCb(const modified_terrain_diff::ConstPtr& msg)
  {
    // import image to so we can traverse it
    auto image_handle = CvImageConstPtr();
    try
    {
      image_handle = toCvShare(msg->diff, msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    auto rows = image_handle->image.rows;
    auto cols = image_handle->image.cols;
    if (rows <= 0 || cols <= 0) {
      ROS_ERROR("Differential image dimensions are zero or negative");
      return;
    }

    auto pixel_area = (msg->height / rows) * (msg->width / cols);
    auto center = msg->position;

    for (auto y = 0; y < rows; ++y)
      for (auto x = 0; x < cols; ++x)
        m_volume_displaced += -image_handle->image.at<float>(y, x) * pixel_area;

    // DEBUG
    // ROS_INFO("Volume displaced: %f", m_volume_displaced);

    if (m_volume_displaced >= m_spawn_threshold)
    {
      m_volume_displaced -= m_spawn_threshold;
      if (!spawnRegolithInScoop())
        ROS_ERROR("Failed to spawn regolith in scoop");
    }
  }

  bool spawnRegolithInScoop(void)
  {
    ROS_INFO("Spawning regolith");

    // load SDF model
    // TODO: load this once and save as data member
    auto sdf = getGazeboModelSdf("model://ball_icefrag_2cm");
    if (sdf.empty()) {
      ROS_ERROR("Failed to acquire regolith SDF");
      return false;
    }

    // spawn model
    static auto spawn_count = 0;
    std::stringstream model_name;
    model_name << "regolith_" << spawn_count++;

    SpawnModel spawn_msg;
    
    spawn_msg.request.model_name                  = model_name.str();
    spawn_msg.request.model_xml                   = sdf;
    spawn_msg.request.robot_namespace             = "/regolith";
    spawn_msg.request.reference_frame             = "lander::l_scoop_tip";

    spawn_msg.request.initial_pose.position.x     = 0.0;
    spawn_msg.request.initial_pose.position.y     = 0.0;
    spawn_msg.request.initial_pose.position.z     = -0.05;
    
    spawn_msg.request.initial_pose.orientation.x  = 0.0;
    spawn_msg.request.initial_pose.orientation.y  = 0.0;
    spawn_msg.request.initial_pose.orientation.z  = 0.0;
    spawn_msg.request.initial_pose.orientation.w  = 0.0;
    
    if (!call_ros_service(m_gazebo_spawn, spawn_msg))
      return false;

    // apply a force to keep model in the scoop
    std::stringstream body_name;
    body_name << model_name.str() << "::link";

    ApplyBodyWrench wrench_msg;
    
    wrench_msg.request.body_name                  = body_name.str();
    wrench_msg.request.reference_frame            = "world";
    
    wrench_msg.request.reference_point.x          = 0.0;
    wrench_msg.request.reference_point.y          = 0.0;
    wrench_msg.request.reference_point.z          = 0.0;
    
    // TOOD: Compute the appropriate force direction based on the direction of 
    //       the dig.
    //       Could simply use the direction of the scoop relative to the x-y 
    //       plane.
    wrench_msg.request.wrench.force.x             = -1.0;
    wrench_msg.request.wrench.force.y             = 0.0;
    wrench_msg.request.wrench.force.z             = 0.0;
    
    wrench_msg.request.wrench.torque.x            = 0.0;
    wrench_msg.request.wrench.torque.y            = 0.0;
    wrench_msg.request.wrench.torque.z            = 0.0;
    
    // TODO: Compute appropriate duration based on dig_linear arguments
    //       Forces should stop about when dig_linear stops
    //       Or when scoop is upright and out of the ground
    wrench_msg.request.duration                   = ros::Duration(10.0);

    if (!call_ros_service(m_gazebo_wrench, wrench_msg))
      return false;

    return true;
  }

private:
  float m_volume_displaced;
  
  // TODO: make into topic value that can be adjusted during runtime
  float m_spawn_threshold; 

  std::unique_ptr<ros::NodeHandle> m_node_handle;
  ros::ServiceClient m_gazebo_spawn;
  ros::ServiceClient m_gazebo_wrench;
};

int main(int argc, char* argv[]) 
{
  // initialize ROS
  ros::init(argc, argv, "regolith_spawner_node");
  ros::NodeHandle nh("regolith_spawner_node");

  auto rs = RegolithSpawner(&nh);

  auto sub = nh.subscribe("/ow_dynamic_terrain/modification_differential/visual", 
    1, &RegolithSpawner::terrainVisualModCb, &rs);

  // DEBUG
  // sleep(10.0);
  // rs.spawnRegolithInScoop();

  ros::spin();
}
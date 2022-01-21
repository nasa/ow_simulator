// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <string>

#include <ros/ros.h>

#include <RegolithSpawner.h>

using namespace ow_regolith;

const static std::string NODE_NAME = "regolith_node";

int main(int argc, char* argv[]) 
{
  // initialize ROS
  ros::init(argc, argv, NODE_NAME);

  RegolithSpawner rs(NODE_NAME);
  
  if (!rs.initialize()) {
    ROS_ERROR("Failed to initialize regolith node");
    return 1;
  }

  ros::spin();

  return 0;
}

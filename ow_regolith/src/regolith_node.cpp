// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// TODO:
//   1. Suppress or prevent "ODE Message 3: LCP internal error, s <= 0 (s=0.0000e+00)"
//   2. Have a service for regolith_node that allows spawning of regolith

#include <ros/ros.h> 

#include "RegolithSpawner.h"

int main(int argc, char* argv[]) 
{
  // initialize ROS
  ros::init(argc, argv, "regolith_node");
  ros::NodeHandle nh("regolith_node");

  RegolithSpawner rs(&nh);
  
  if (!rs.initialize()) {
    ROS_ERROR("Failed to initialize regolith node");
    return 1;
  }

  ros::spin();

  return 0;
}

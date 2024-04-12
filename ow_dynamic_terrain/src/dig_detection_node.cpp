// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ros/ros.h"

#include "DigDetector.h"

using namespace ow_dynamic_terrain;

const std::string NODE_NAME = "dig_detection_node";

int main(int argc, char* argv[])
{
  // initialize ROS
  ros::init(argc, argv, NODE_NAME);

  DigDetector dd(NODE_NAME);

  ros::spin();

  return 0;
}

// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// ROS includes
#include <ros/ros.h>

// rosbag includes
#include <rosbag/recorder.h>

#include <string>

#include "config.h"

int main(int argc, char* argv[]) 
{
  ros::init(argc, argv, "bag_recorder_node");
  ros::NodeHandle nh("bag_recorder_node");

  rosbag::RecorderOptions recorder_options;
  if (!get_recorder_options(recorder_options))
  {
    ROS_ERROR("Recorder options are invalid. Check configuration file.");
    return 1;
  }

  // both record_all and node options will override topics.yaml
  if (!recorder_options.record_all && recorder_options.node == std::string(""))
    get_recorder_topics(recorder_options.topics);

  rosbag::Recorder recorder(recorder_options);
 
  ROS_INFO("Bag recorder node is running");

  return recorder.run();
}
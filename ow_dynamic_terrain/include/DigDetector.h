// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef DIG_DETECTOR_H
#define DIG_DETECTOR_H

#include "ros/ros.h"

#include "gazebo_msgs/LinkStates.h"

#include "DigStateMachine.h"

namespace ow_dynamic_terrain {

class DigDetector
{
public:
  DigDetector(const std::string &node_name);
  ~DigDetector() = default;

  DigDetector() = delete;
  DigDetector(const DigDetector&) = delete;
  DigDetector& operator=(const DigDetector&) = delete;

  void onLinkStatesMessage(const gazebo_msgs::LinkStates::ConstPtr &msg);

  void onDigStateTransition(DigStateId new_state);

private:
  std::unique_ptr<ros::NodeHandle> m_node_handle;

  std::unique_ptr<DigStateMachine> m_machine;

  ros::Subscriber m_sub_link_states;
  ros::Publisher m_pub_dig_phase;

};

}

#endif // DIG_DETECTOR_H

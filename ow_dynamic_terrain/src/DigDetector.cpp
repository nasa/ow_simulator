// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <functional>

#include <tf/tf.h>

#include "ow_dynamic_terrain/scoop_dig_phase.h"

#include "DigDetector.h"

using namespace ow_dynamic_terrain;

DigDetector::DigDetector(const std::string &node_name)
  : m_node_handle(std::make_unique<ros::NodeHandle>(node_name))
{
  m_sub_link_states = m_node_handle->subscribe(
    "/gazebo/link_states", 1, &DigDetector::onLinkStatesMessage, this);
  // NOTE: this topic must be advertised before the DigStateMachine is created
  //  so the value of scoop_dig_phase is properly latched at NOT_DIGGING
  m_pub_dig_phase = m_node_handle->advertise<scoop_dig_phase>(
    "/ow_dynamic_terrain/scoop_dig_phase", 1, true);
  m_machine = std::make_unique<DigStateMachine>(m_node_handle.get(),
    std::bind(&DigDetector::onDigStateTransition, this, std::placeholders::_1));
}

void DigDetector::onLinkStatesMessage(
  const gazebo_msgs::LinkStates::ConstPtr &msg)
{
  const std::string SCOOP_LINK_NAME = "lander::l_scoop_tip";
  auto name_it = find(begin(msg->name), end(msg->name), SCOOP_LINK_NAME);
  if (name_it == end(msg->name)) {
    ROS_WARN("Failed to find %s in link states", SCOOP_LINK_NAME.c_str());
    return;
  }
  tf::Pose scoop_pose;
  auto pose_it = begin(msg->pose) + distance(begin(msg->name), name_it);
  tf::poseMsgToTF(*pose_it, scoop_pose);

  m_machine->handleScoopPoseUpdate(scoop_pose);
}

void DigDetector::onDigStateTransition(DigStateId new_state)
{
  scoop_dig_phase msg;
  switch(new_state) {
    case DigStateId::NOT_DIGGING:
      msg.digging = false;
      msg.phase = scoop_dig_phase::NOT_DIGGING;
      break;
    case DigStateId::SINKING:
      msg.digging = true;
      msg.phase = scoop_dig_phase::SINKING;
      break;
    case DigStateId::PLOWING:
      msg.digging = true;
      msg.phase = scoop_dig_phase::PLOWING;
      break;
    case DigStateId::RETRACTING:
      msg.digging = true;
      msg.phase = scoop_dig_phase::RETRACTING;
      break;
    default:
      ROS_ERROR("Unknown dig state transitioned into."
                "This should never happen!");
  }
  m_pub_dig_phase.publish(msg);
}

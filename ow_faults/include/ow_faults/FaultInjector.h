/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 ******************************************************************************/
#ifndef FaultInjector_h
#define FaultInjector_h


#include <ros/ros.h>
#include <ow_faults/FaultsConfig.h>
#include <sensor_msgs/JointState.h>
#include <unordered_map>


// This class injects simple message faults that don't need to be simulated
// at their source. Modified topics are prefixed with "/faults". This could be
// accomplished on the original topics, but, for example, in the case of
// /joint_states, this would require forking and modifying joint_state_publisher.
// Creating a modified version of the original message is simpler, and it
// leaves the original message intact in case we want to use it as part of a
// placeholder fault estimator.
class FaultInjector
{
public:
  FaultInjector(ros::NodeHandle node_handle);
  ~FaultInjector(){}

  void faultsConfigCb(ow_faults::FaultsConfig& faults, uint32_t level);

private:
  // Output /faults/joint_states, a modified version of /joint_states, injecting
  // simple message faults that don't need to be simulated at their source.
  void jointStateCb(const sensor_msgs::JointStateConstPtr& msg);

  // Find an item in an std::vector or other find-able data structure, and
  // return its index. Return -1 if not found.
  template<typename group_t, typename item_t>
  int findPositionInGroup(const group_t& group, const item_t& item);

  // Get index from m_joint_index_map. If found, modify out_index and return
  // true. Otherwise, return false.
  bool findJointIndex(const std::string& name, int& out_index);

  ow_faults::FaultsConfig m_faults;

  ros::Subscriber m_joint_state_sub;
  ros::Publisher m_joint_state_pub;

  std::unordered_map<std::string, int> m_joint_index_map;
};


#endif

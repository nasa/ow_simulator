/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 ******************************************************************************/
#include "ow_faults/FaultInjector.h"
#include <algorithm>


using namespace std;


FaultInjector::FaultInjector(ros::NodeHandle node_handle)
{
  m_joint_state_sub = node_handle.subscribe("/joint_states", 10, &FaultInjector::jointStateCb, this);
  m_joint_state_pub = node_handle.advertise<sensor_msgs::JointState>("/faults/joint_states", 10);
}

void FaultInjector::faultsConfigCb(ow_faults::FaultsConfig& faults, uint32_t level)
{
  // This is where we would check to see if faults is different from m_faults
  // if we wanted to change some state based on that.

  // Store current set of faults for later use
  m_faults = faults;
}

void FaultInjector::jointStateCb(const sensor_msgs::JointStateConstPtr& msg)
{
  // Populate the map once here.
  // This assumes the collection of joints will never change.
  if (m_joint_index_map.empty()) {
    string names[] = {"j_ant_pan", "j_ant_tilt", "j_dist_pitch", "j_hand_yaw",
                      "j_prox_pitch", "j_scoop_yaw", "j_shou_pitch", "j_shou_yaw"};
    for (auto& n : names) {
      int index = findPositionInGroup(msg->name, n);
      if (index >= 0)
        m_joint_index_map[n] = index;
    }
  }

  sensor_msgs::JointState output(*msg);

  // Set failed sensor values to 0
  int index;
  if (m_faults.ant_pan_encoder_failure && findJointIndex("j_ant_pan", index)) {
    output.position[index] = 0.0;
  }
  if (m_faults.ant_pan_torque_sensor_failure && findJointIndex("j_ant_pan", index)) {
    output.effort[index] = 0.0;
  }

  if (m_faults.ant_tilt_encoder_failure && findJointIndex("j_ant_tilt", index)) {
    output.position[index] = 0.0;
  }
  if (m_faults.ant_tilt_torque_sensor_failure && findJointIndex("j_ant_tilt", index)) {
    output.effort[index] = 0.0;
  }

  if (m_faults.shou_yaw_encoder_failure && findJointIndex("j_shou_yaw", index)) {
    output.position[index] = 0.0;
  }
  if (m_faults.shou_yaw_torque_sensor_failure && findJointIndex("j_shou_yaw", index)) {
    output.effort[index] = 0.0;
  }

  if (m_faults.shou_pitch_encoder_failure && findJointIndex("j_shou_pitch", index)) {
    output.position[index] = 0.0;
  }
  if (m_faults.shou_pitch_torque_sensor_failure && findJointIndex("j_shou_pitch", index)) {
    output.effort[index] = 0.0;
  }

  if (m_faults.prox_pitch_encoder_failure && findJointIndex("j_prox_pitch", index)) {
    output.position[index] = 0.0;
  }
  if (m_faults.prox_pitch_torque_sensor_failure && findJointIndex("j_prox_pitch", index)) {
    output.effort[index] = 0.0;
  }

  if (m_faults.dist_pitch_encoder_failure && findJointIndex("j_dist_pitch", index)) {
    output.position[index] = 0.0;
  }
  if (m_faults.dist_pitch_torque_sensor_failure && findJointIndex("j_dist_pitch", index)) {
    output.effort[index] = 0.0;
  }

  if (m_faults.hand_yaw_encoder_failure && findJointIndex("j_hand_yaw", index)) {
    output.position[index] = 0.0;
  }
  if (m_faults.hand_yaw_torque_sensor_failure && findJointIndex("j_hand_yaw", index)) {
    output.effort[index] = 0.0;
  }

  if (m_faults.scoop_yaw_encoder_failure && findJointIndex("j_scoop_yaw", index)) {
    output.position[index] = 0.0;
  }
  if (m_faults.scoop_yaw_torque_sensor_failure && findJointIndex("j_scoop_yaw", index)) {
    output.effort[index] = 0.0;
  }

  m_joint_state_pub.publish(output);
}

template<typename group_t, typename item_t>
int FaultInjector::findPositionInGroup(const group_t& group, const item_t& item)
{
  auto iter = std::find(group.begin(), group.end(), item);
  if (iter == group.end())
    return -1;
  return iter - group.begin();
}

bool FaultInjector::findJointIndex(const string& name, int& out_index)
{
  auto iter = m_joint_index_map.find(name);

  if (iter == m_joint_index_map.end())
    return false;

  out_index = iter->second;
  return true;
}

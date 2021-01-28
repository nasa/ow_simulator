// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ow_faults/FaultInjector.h"
#include <algorithm>

using namespace std;
using namespace ow_lander;

FaultInjector::FaultInjector(ros::NodeHandle node_handle)
{
  m_joint_state_sub = node_handle.subscribe("/_original/joint_states", 10, &FaultInjector::jointStateCb, this);
  // 'new' joint states topic where fault data is now publishes on
  m_joint_state_pub = node_handle.advertise<sensor_msgs::JointState>("/joint_states", 10); 

  //power fault publishers and subs
  m_fault_power_state_of_charge_pub = node_handle.advertise<std_msgs::Float64>("temporary/power_fault/state_of_charge", 10);
  m_fault_power_temp_pub = node_handle.advertise<std_msgs::Float64>("temporary/power_fault/temp_increase", 10);

  // topic for system fault messages, see Faults.msg
  m_fault_status_pub = node_handle.advertise<ow_faults::SystemFaults>("/system_faults_status", 10); 
  // topic for arm fault status, see ArmFaults.msg
  m_arm_fault_status_pub = node_handle.advertise<ow_faults::ArmFaults>("/arm_faults_status", 10); 
  // topic for power fault status, see PowerFaults.msg
  m_power_fault_status_pub = node_handle.advertise<ow_faults::PowerFaults>("/power_faults_status", 10); 
  
  srand (static_cast <unsigned> (time(0)));

  m_arm_state_sub = node_handle.subscribe("/system_faults_status", 10, &FaultInjector::checkSystemFaults, this);
  // m_fault_arm_plan_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/fake_arm_plan", 10); 
  m_fault_arm_plan_pub = node_handle.advertise<ow_faults::SystemFaults>("/fake_arm_plan", 10); 

}

void FaultInjector::faultsConfigCb(ow_faults::FaultsConfig& faults, uint32_t level)
{
  // This is where we would check to see if faults is different from m_faults
  // if we wanted to change some state based on that.

  // Store current set of faults for later use
  m_faults = faults;
}

void FaultInjector::setSytemFaultsMessage(ow_faults::SystemFaults& msg, int value) {
  // for now only arm execution errors
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/world";
  msg.value = value; //should be ARM EXECUTION ERROR for now
}

void FaultInjector::setArmFaultsMessage(ow_faults::ArmFaults& msg, int value) {
  // for now only arm execution errors
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/world";
  msg.value = value; //should be HARDWARE for now
}

void FaultInjector::setPowerFaultsMessage(ow_faults::PowerFaults& msg, int value) {
  // for now only arm execution errors
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/world";
  msg.value = value; //should be HARDWARE for now
}

void FaultInjector::setPowerTemperatureFaultValue(bool getTempBool){
  float thermal_val;
  if (isnan(powerTemperatureOverloadValue)) {
    thermal_val =  50.0 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(90.0-50.0)));
    powerTemperatureOverload = thermal_val;
  } else if (!getTempBool) {
    powerTemperatureOverload = NAN;
  }
}

void FaultInjector::checkSystemFaults(const ow_faults::SystemFaults& msg)
{
  ow_faults::SystemFaults system_faults_msg;
  SystemFaults sf = ArmExecutionError;
  if (msg.value == sf) {
    system_faults_msg.value = 100;
  }
  m_fault_arm_plan_pub.publish(system_faults_msg);

}

void FaultInjector::jointStateCb(const sensor_msgs::JointStateConstPtr& msg)
{
  // Populate the map once here.
  // This assumes the collection of joints will never change.
  if (m_joint_state_indices.empty()) {
    for (int j = 0; j < NUM_JOINTS; j ++) {
      int index = findPositionInGroup(msg->name, joint_names[j]);
      if (index >= 0)
        m_joint_state_indices.push_back(index);
    }
  }

  sensor_msgs::JointState output(*msg);
  ow_faults::SystemFaults system_faults_msg;
  ow_faults::ArmFaults arm_faults_msg;
  ow_faults::PowerFaults power_faults_msg;
  SystemFaults sf = ArmExecutionError;
  ComponentFaults hardwareFault = Hardware;

  //arm faults
  // Set failed sensor values to 0
  unsigned int index;
  if (m_faults.ant_pan_encoder_failure && findJointIndex(J_ANT_PAN, index)) {
    output.position[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
  }
  if (m_faults.ant_pan_torque_sensor_failure && findJointIndex(J_ANT_PAN, index)) {
    output.effort[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
  }

  if (m_faults.ant_tilt_encoder_failure && findJointIndex(J_ANT_TILT, index)) {
    output.position[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
  }
  if (m_faults.ant_tilt_torque_sensor_failure && findJointIndex(J_ANT_TILT, index)) {
    output.effort[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
  }

  if (m_faults.shou_yaw_encoder_failure && findJointIndex(J_SHOU_YAW, index)) {
    output.position[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
    setArmFaultsMessage(arm_faults_msg,hardwareFault);
  }
  if (m_faults.shou_yaw_torque_sensor_failure && findJointIndex(J_SHOU_YAW, index)) {
    output.effort[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
    setArmFaultsMessage(arm_faults_msg,hardwareFault);
  }

  if (m_faults.shou_pitch_encoder_failure && findJointIndex(J_SHOU_PITCH, index)) {
    output.position[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
    setArmFaultsMessage(arm_faults_msg,hardwareFault);
  }
  if (m_faults.shou_pitch_torque_sensor_failure && findJointIndex(J_SHOU_PITCH, index)) {
    output.effort[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
    setArmFaultsMessage(arm_faults_msg,hardwareFault);
  }

  if (m_faults.prox_pitch_encoder_failure && findJointIndex(J_PROX_PITCH, index)) {
    output.position[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
    setArmFaultsMessage(arm_faults_msg,hardwareFault);
  }
  if (m_faults.prox_pitch_torque_sensor_failure && findJointIndex(J_PROX_PITCH, index)) {
    output.effort[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
    setArmFaultsMessage(arm_faults_msg,hardwareFault);
  }

  if (m_faults.dist_pitch_encoder_failure && findJointIndex(J_DIST_PITCH, index)) {
    output.position[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
    setArmFaultsMessage(arm_faults_msg,hardwareFault);
  }
  if (m_faults.dist_pitch_torque_sensor_failure && findJointIndex(J_DIST_PITCH, index)) {
    output.effort[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
    setArmFaultsMessage(arm_faults_msg,hardwareFault);
  }

  if (m_faults.hand_yaw_encoder_failure && findJointIndex(J_HAND_YAW, index)) {
    output.position[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
    setArmFaultsMessage(arm_faults_msg,hardwareFault);
  }
  if (m_faults.hand_yaw_torque_sensor_failure && findJointIndex(J_HAND_YAW, index)) {
    output.effort[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
    setArmFaultsMessage(arm_faults_msg,hardwareFault);
  }

  if (m_faults.scoop_yaw_encoder_failure && findJointIndex(J_SCOOP_YAW, index)) {
    output.position[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
    setArmFaultsMessage(arm_faults_msg,hardwareFault);
  }
  if (m_faults.scoop_yaw_torque_sensor_failure && findJointIndex(J_SCOOP_YAW, index)) {
    output.effort[index] = 0.0;
    setSytemFaultsMessage(system_faults_msg, sf);
    setArmFaultsMessage(arm_faults_msg,hardwareFault);
  }

  std_msgs::Float64 soc_msg;

  // power faults
  if(m_faults.low_state_of_charge_power_failure) {
    // Fault is a range ( anything < 10%)
    soc_msg.data = 2.2;
    m_fault_power_state_of_charge_pub.publish(soc_msg);
    setPowerFaultsMessage(power_faults_msg, hardwareFault);
  }
  if(m_faults.instantaneous_capacity_loss_power_failure) {
    // (most recent and current). If the % difference is > 5% and no other tasks in progress, then fault. 
    soc_msg.data = 98.5; //random now but should be >5% more than the previous value
    m_fault_power_state_of_charge_pub.publish(soc_msg);
    setPowerFaultsMessage(power_faults_msg, hardwareFault);
  }
  if(m_faults.thermal_power_failure){
    // if > 50 degrees C, then consider fault. 
    std_msgs::Float64 thermal_msg;
    setPowerTemperatureFaultValue(true);
    thermal_msg.data = powerTemperatureOverloadValue;
    m_fault_power_temp_pub.publish(thermal_msg);
    setPowerFaultsMessage(power_faults_msg, hardwareFault);
  } else {
    setPowerTemperatureFaultValue(false);
  }

  m_joint_state_pub.publish(output);
  m_fault_status_pub.publish(system_faults_msg);
  m_arm_fault_status_pub.publish(arm_faults_msg);
  m_power_fault_status_pub.publish(power_faults_msg);
}

template<typename group_t, typename item_t>
int FaultInjector::findPositionInGroup(const group_t& group, const item_t& item)
{
  auto iter = std::find(group.begin(), group.end(), item);
  if (iter == group.end())
    return -1;
  return iter - group.begin();
}

bool FaultInjector::findJointIndex(const unsigned int joint, unsigned int& out_index)
{
  if(joint >= NUM_JOINTS)
    return false;

  out_index = m_joint_state_indices[joint];
  return true;
}

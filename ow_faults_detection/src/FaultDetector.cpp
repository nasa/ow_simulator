// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ow_faults_detection/FaultDetector.h"
#include <algorithm>
#include <iomanip>
#include <iostream> 

using namespace ow_lander;

using std::bitset;
using std::string;

constexpr bitset<10> FaultDetector::isCamExecutionError;
constexpr bitset<10> FaultDetector::isPanTiltExecutionError;
constexpr bitset<10> FaultDetector::isArmExecutionError;
constexpr bitset<10> FaultDetector::isPowerSystemFault;

constexpr bitset<3> FaultDetector::islowVoltageError;
constexpr bitset<3> FaultDetector::isCapLossError;
constexpr bitset<3> FaultDetector::isThermalError;

FaultDetector::FaultDetector(ros::NodeHandle& nh)
{
  srand (static_cast <unsigned> (time(0)));
  // arm and antenna
  m_joint_states_sub = nh.subscribe( "/flags/joint_states",
                                      10,
                                      &FaultDetector::jointStatesFlagCb,
                                      this);
  // camera
  const char* image_str = "/StereoCamera/left/image_";
  m_camera_original_trigger_sub = nh.subscribe( image_str + string("trigger"),
                                                10, 
                                                &FaultDetector::camerTriggerCb, 
                                                this);
  m_camera_raw_sub = nh.subscribe( image_str + string("raw"),
                                   10, 
                                   &FaultDetector::cameraRawCb, 
                                   this);
  
  m_camera_trigger_timer = nh.createTimer(ros::Duration(0.1), &FaultDetector::cameraTriggerPublishCb, this);

  //  power fault publishers and subs
  m_power_soc_sub = nh.subscribe( "/power_system_node/state_of_charge",
                                  10,
                                  &FaultDetector::powerSOCListener,
                                  this);
  m_power_temperature_sub = nh.subscribe( "/power_system_node/battery_temperature",
                                          10,
                                          &FaultDetector::powerTemperatureListener,
                                          this);

  // topics for JPL msgs: system fault messages, see Faults.msg, Arm.msg, Power.msg, PTFaults.msg
  m_arm_fault_msg_pub = nh.advertise<ow_faults_detection::ArmFaults>("/faults/arm_faults_status", 10);
  m_antenna_fault_msg_pub = nh.advertise<ow_faults_detection::PTFaults>("/faults/pt_faults_status", 10);
  m_camera_fault_msg_pub = nh.advertise<ow_faults_detection::CamFaults>("/faults/cam_faults_status", 10);
  m_power_fault_msg_pub = nh.advertise<ow_faults_detection::PowerFaults>("/faults/power_faults_status", 10);
  m_system_fault_msg_pub = nh.advertise<ow_faults_detection::SystemFaults>("/faults/system_faults_status", 10);

}

// Creating Fault Messages
template<typename fault_msg>
void FaultDetector::setFaultsMessageHeader(fault_msg& msg)
{
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
}

template<typename bitsetFaultsMsg, typename bitmask>
void FaultDetector::setBitsetFaultsMessage(bitsetFaultsMsg& msg, bitmask bm) 
{
  setFaultsMessageHeader(msg);
  msg.value = bm.to_ullong();
}

template<typename fault_msg>
void FaultDetector::setComponentFaultsMessage(fault_msg& msg, ComponentFaults value) 
{
  setFaultsMessageHeader(msg);
  msg.value = static_cast<uint>(value);
}

// publish system messages
void FaultDetector::publishSystemFaultsMessage()
{
  ow_faults_detection::SystemFaults system_faults_msg;
  setBitsetFaultsMessage(system_faults_msg, m_system_faults_bitset);
  m_system_fault_msg_pub.publish(system_faults_msg);
}

//// Publish Camera Messages
void FaultDetector::cameraTriggerPublishCb(const ros::TimerEvent& t)
{
  ow_faults_detection::CamFaults camera_faults_msg;
  auto diff = m_cam_raw_time - m_cam_trigger_time;
  if (m_cam_trigger_time <= m_cam_raw_time &&  
    m_cam_raw_time <= m_cam_trigger_time + ros::Duration(2) || 
    diff < ros::Duration(0) && ros::Duration(-1) < diff) {
    m_system_faults_bitset &= ~isCamExecutionError;
  } else {
    m_system_faults_bitset |= isCamExecutionError;
    setComponentFaultsMessage(camera_faults_msg, ComponentFaults::Hardware);
  }

  publishSystemFaultsMessage();
  m_camera_fault_msg_pub.publish(camera_faults_msg);
}

//// Publish Power Faults Messages
void FaultDetector::publishPowerSystemFault()
{
  ow_faults_detection::PowerFaults power_faults_msg;
  //update if fault
  if (m_temperature_fault || m_soc_fault) {
    //system
    m_system_faults_bitset |= isPowerSystemFault;
    //power
    setComponentFaultsMessage(power_faults_msg, ComponentFaults::Hardware);
  } else {
    m_system_faults_bitset &= ~isPowerSystemFault;
  }
  publishSystemFaultsMessage();
  m_power_fault_msg_pub.publish(power_faults_msg);
}

// Listeners
// Arm listeners
bool FaultDetector::isFlagSet(uint joint, const std::vector<uint8_t>& flags) 
{
  unsigned int index;
  auto found = findJointIndex(joint, index);

  if (!found){
    return false;
  }
  if (flags[index] == true){
    return true;
  }
  return false;
}

void FaultDetector::jointStatesFlagCb(const ow_faults_detection::JointStatesFlagConstPtr& msg)
{
  unsigned int index;
  
  // Populate the map once here.
  // This assumes the collection of joints will never change.
  if (m_joint_state_indices.empty()) {
    for (int j = 0; j < NUM_JOINTS; j ++) {
      int index = findPositionInGroup(msg->name, joint_names[j]);
      if (index >= 0)
        m_joint_state_indices.push_back(index);
    }
  }

  bool armFault = false;
  auto armList = {J_SHOU_YAW, J_SHOU_PITCH, J_PROX_PITCH, 
                  J_DIST_PITCH, J_HAND_YAW, J_SCOOP_YAW};
  //ant faults
  m_pan_fault = isFlagSet( J_ANT_PAN, msg->flags);
  m_tilt_fault = isFlagSet( J_ANT_TILT, msg->flags);
  antPublishFaultMessages();

  //arm faults
  for (auto& name : armList) {
    armFault = armFault || isFlagSet( name, msg->flags);
  }

  ow_faults_detection::ArmFaults arm_faults_msg;
  if (armFault){
    m_system_faults_bitset |= isArmExecutionError;
    setComponentFaultsMessage(arm_faults_msg,  ComponentFaults::Hardware);
  } else {
    m_system_faults_bitset &= ~isArmExecutionError;
  }

  m_arm_fault_msg_pub.publish(arm_faults_msg);
  publishSystemFaultsMessage();
}



template<typename group_t, typename item_t>
int FaultDetector::findPositionInGroup(const group_t& group, const item_t& item)
{
  auto iter = std::find(group.begin(), group.end(), item);
  if (iter == group.end())
    return -1;
  return iter - group.begin();
}

bool FaultDetector::findJointIndex(const unsigned int joint, unsigned int& out_index)
{
  if(joint >= m_joint_state_indices.size())
    return false;

  out_index = m_joint_state_indices[joint];
  return true;
}

//// Antenna Listeners
void FaultDetector::antPublishFaultMessages()
{
  ow_faults_detection::PTFaults ant_fault_msg;
  if (m_pan_fault || m_tilt_fault) {
    setComponentFaultsMessage(ant_fault_msg, ComponentFaults::Hardware);
    m_system_faults_bitset |= isPanTiltExecutionError;
  }else {
    m_system_faults_bitset &= ~isPanTiltExecutionError;
  }
  publishSystemFaultsMessage();
  m_antenna_fault_msg_pub.publish(ant_fault_msg);
}

//// Camera listeners
void FaultDetector::camerTriggerCb(const std_msgs::Empty& msg)
{
  m_cam_trigger_time = ros::Time::now();
}

void FaultDetector::cameraRawCb(const sensor_msgs::Image& msg)
{
  m_cam_raw_time = ros::Time::now();
}

//// Power Topic Listeners
void FaultDetector::powerTemperatureListener(const std_msgs::Float64& msg)
{
  m_temperature_fault = msg.data > THERMAL_MAX;
  publishPowerSystemFault();
}
 
void FaultDetector::powerSOCListener(const std_msgs::Float64& msg)
{
  float newSOC = msg.data;
  if (isnan(m_last_SOC)){
    m_last_SOC = newSOC;
  }
  m_soc_fault = ((newSOC <= SOC_MIN)  ||
                (!isnan(m_last_SOC) &&
                ((abs(m_last_SOC - newSOC) / m_last_SOC) >= SOC_MAX_DIFF )));
  publishPowerSystemFault();
  m_last_SOC = newSOC;
}

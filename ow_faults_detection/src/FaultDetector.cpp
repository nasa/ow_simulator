// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ow_faults_detection/FaultDetector.h"
#include <algorithm>
#include <iomanip>
#include <iostream>

using namespace ow_lander;
using namespace owl_msgs;

using std::bitset;
using std::string;

constexpr bitset<3> FaultDetector::islowVoltageError;
constexpr bitset<3> FaultDetector::isCapLossError;
constexpr bitset<3> FaultDetector::isThermalError;

constexpr bitset<2> FaultDetector::isPanLockedError;
constexpr bitset<2> FaultDetector::isTiltLockedError;

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

  //  power fault publishers and subs
  m_power_soc_sub = nh.subscribe( "/power_system_node/state_of_charge",
                                  10,
                                  &FaultDetector::powerSOCListener,
                                  this);
  m_power_temperature_sub = nh.subscribe( "/power_system_node/battery_temperature",
                                          10,
                                          &FaultDetector::powerTemperatureListener,
                                          this);

  // topics for OWLAT/JPL msgs: system fault messages, see owl_msgs/msg
  m_arm_fault_msg_pub = nh.advertise<ow_faults_detection::ArmFaults>("/faults/arm_faults_status", 10);
  m_antenna_fault_msg_pub = nh.advertise<ow_faults_detection::PanTiltFaultsStatus>("/faults/pan_tilt_faults_status", 10);
  m_camera_fault_msg_pub = nh.advertise<ow_faults_detection::CamFaults>("/faults/cam_faults_status", 10);
  m_power_fault_msg_pub = nh.advertise<ow_faults_detection::PowerFaults>("/faults/power_faults_status", 10);
  m_system_faults_msg_pub = nh.advertise<owl_msgs::SystemFaultsStatus>("/system_faults_status", 10);

}

// Creating Fault Messages
template<typename fault_msg>
void FaultDetector::setFaultsMessageHeader(fault_msg& msg)
{
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
}

// publish system messages
void FaultDetector::publishSystemFaultsMessage()
{
  owl_msgs::SystemFaultsStatus system_faults_msg;
  setFaultsMessageHeader(system_faults_msg);
  system_faults_msg.value = m_system_faults_flags;
  m_system_faults_msg_pub.publish(system_faults_msg);
}

//// Publish Camera Messages
void FaultDetector::cameraPublishFaultMessages(bool is_fault)
{
  owl_msgs::CameraFaultsStatus camera_faults_msg;
  if (is_fault) {
    m_system_faults_flags |= SystemFaultsStatus::CAMERA_EXECUTION_ERROR;
  } else {
    m_system_faults_flags &= ~SystemFaultsStatus::CAMERA_EXECUTION_ERROR;
  }
  publishSystemFaultsMessage();
  m_camera_faults_msg_pub.publish(camera_faults_msg);
}

//// Publish Power Faults Messages
void FaultDetector::publishPowerSystemFault()
{
  ow_faults_detection::PowerFaults power_faults_msg;
  //update if fault
  if (m_temperature_fault || m_soc_fault) {
    //system
    m_system_faults_flags |= SystemFaultsStatus::POWER_EXECUTION_ERROR;
    //power
  } else {
    m_system_faults_flags &= ~SystemFaultsStatus::POWER_EXECUTION_ERROR;
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

  owl_msgs::ArmFaultsStatus arm_faults_msg;
  if (armFault){
    m_system_faults_flags |= SystemFaultsStatus::ARM_EXECUTION_ERROR;
    arm_faults_msg.value |= ArmFaultsStatus::HARDWARE;
  } else {
    m_system_faults_flags &= ~SystemFaultsStatus::ARM_EXECUTION_ERROR;
    arm_faults_msg.value &= ~ArmFaultsStatus::HARDWARE;
  }

  setFaultsMessageHeader(arm_faults_msg);
  m_arm_faults_msg_pub.publish(arm_faults_msg);
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
  owl_msgs::PanTiltFaultsStatus ant_faults_msg;
  if (m_pan_fault || m_tilt_fault) {
    // assign system fault and determine/assign individual pan_tilt faults
    m_system_faults_flags |= SystemFaultsStatus::PAN_TILT_EXECUTION_ERROR;
    if (m_pan_fault) {
      ant_faults_msg.value |= PanTiltFaultsStatus::PAN_JOINT_LOCKED;
    } else {
      ant_faults_msg.value &= ~PanTiltFaultsStatus::PAN_JOINT_LOCKED;
    }
    if (m_tilt_fault) {
      ant_faults_msg.value |= PanTiltFaultsStatus::TILT_JOINT_LOCKED;
    } else {
      ant_faults_msg.value &= ~PanTiltFaultsStatus::TILT_JOINT_LOCKED;
    }
  }else {
    // exonerate system and pan_tilt faults
    m_system_faults_flags &= ~SystemFaultsStatus::PAN_TILT_EXECUTION_ERROR;
    ant_faults_msg.value = PanTiltFaultsStatus::NONE;
  }
  publishSystemFaultsMessage();
  setFaultsMessageHeader(ant_faults_msg);
  m_antenna_faults_msg_pub.publish(ant_faults_msg);
}

//// Camera listeners
void FaultDetector::cameraTriggerCb(const std_msgs::Empty& msg)
{
  // fault if camera data is still pending when trigger is received
  if (m_camera_data_pending) {
    cameraPublishFaultMessages(true);
  }
  m_camera_data_pending = true;
}

void FaultDetector::cameraRawCb(const sensor_msgs::Image& msg)
{
  // exonerate fault when camera_raw data is received
  cameraPublishFaultMessages(false);
  m_camera_data_pending = false;
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

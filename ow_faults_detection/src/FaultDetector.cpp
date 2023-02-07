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
                                                &FaultDetector::cameraTriggerCb, 
                                                this);
  m_camera_raw_sub = nh.subscribe( image_str + string("raw"),
                                   10, 
                                   &FaultDetector::cameraRawCb, 
                                   this);
  
  //  power fault publishers and subs
  m_power_soc_sub = nh.subscribe( "/battery_state_of_charge",
                                  10,
                                  &FaultDetector::powerSOCListener,
                                  this);
  m_power_temperature_sub = nh.subscribe( "/battery_temperature",
                                          10,
                                          &FaultDetector::powerTemperatureListener,
                                          this);

  // topics for OWLAT/JPL msgs: system fault messages, see owl_msgs/msg
  m_arm_faults_msg_pub = nh.advertise<ArmFaultsStatus>("/arm_faults_status", 10);
  m_antenna_faults_msg_pub = nh.advertise<PanTiltFaultsStatus>("/pan_tilt_faults_status", 10);
  m_camera_faults_msg_pub = nh.advertise<CameraFaultsStatus>("/camera_faults_status", 10);
  m_power_faults_msg_pub = nh.advertise<PowerFaultsStatus>("/power_faults_status", 10);
  m_system_faults_msg_pub = nh.advertise<SystemFaultsStatus>("/system_faults_status", 10);

}

// Creating Fault Messages
template<typename fault_msg>
void FaultDetector::setFaultsMessageHeader(fault_msg& msg)
{
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
}

template<typename pub_t, typename msg_t, typename flags_t>
void FaultDetector::publishFaultsMessage(pub_t& fault_pub, msg_t fault_msg, flags_t fault_flags)
{
  setFaultsMessageHeader(fault_msg);
  if (fault_flags) {
    fault_msg.value |= fault_flags;
  } else {
    fault_msg.value &= ~fault_flags;
  }
  fault_pub.publish(fault_msg);
}

// Listeners
// Arm and Antenna listeners
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

  // check for arm faults
  bool isFault = false;
  auto armList = {J_SHOU_YAW, J_SHOU_PITCH, J_PROX_PITCH, 
                  J_DIST_PITCH, J_HAND_YAW, J_SCOOP_YAW};
  for (auto& name : armList) {
    isFault = isFault || isFlagSet( name, msg->flags);
  }
  if (isFault) {
    m_system_faults_flags |= SystemFaultsStatus::ARM_EXECUTION_ERROR;
    m_arm_faults_flags |= ArmFaultsStatus::HARDWARE;
  } else {
    m_system_faults_flags &= ~SystemFaultsStatus::ARM_EXECUTION_ERROR;
    m_arm_faults_flags &= ~ArmFaultsStatus::HARDWARE;
  }
  
  // check for antenna faults
  isFault = isFlagSet( J_ANT_PAN, msg->flags);
  if (isFault) {
    m_antenna_faults_flags |= PanTiltFaultsStatus::PAN_JOINT_LOCKED;
  } else {
    m_antenna_faults_flags &= ~PanTiltFaultsStatus::PAN_JOINT_LOCKED;
  }

  isFault = isFlagSet( J_ANT_TILT, msg->flags);
  if (isFault) {
    m_antenna_faults_flags |= PanTiltFaultsStatus::TILT_JOINT_LOCKED;
  } else {
    m_antenna_faults_flags &= ~PanTiltFaultsStatus::TILT_JOINT_LOCKED;
  }

  // update system faults
  if (ArmFaultsStatus::NONE == m_arm_faults_flags) {
    m_system_faults_flags &= ~SystemFaultsStatus::ARM_EXECUTION_ERROR;
  } else {
    m_system_faults_flags |= SystemFaultsStatus::ARM_EXECUTION_ERROR;
  }
  if (PanTiltFaultsStatus::NONE == m_antenna_faults_flags) {
    m_system_faults_flags &= ~SystemFaultsStatus::PAN_TILT_EXECUTION_ERROR;
  } else {
    m_system_faults_flags |= SystemFaultsStatus::PAN_TILT_EXECUTION_ERROR;
  }

  // publish updated faults messages
  publishFaultsMessage(m_arm_faults_msg_pub, ArmFaultsStatus(), m_arm_faults_flags);
  publishFaultsMessage(m_antenna_faults_msg_pub, PanTiltFaultsStatus(), m_antenna_faults_flags);
  publishFaultsMessage(m_system_faults_msg_pub, SystemFaultsStatus(), m_system_faults_flags);
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

//// Camera listeners
void FaultDetector::cameraTriggerCb(const std_msgs::Empty& msg)
{
  // fault if camera data is still pending when trigger is received
  if (m_camera_data_pending) {
    m_camera_faults_flags |= CameraFaultsStatus::NO_IMAGE;
    m_system_faults_flags |= SystemFaultsStatus::CAMERA_EXECUTION_ERROR;

    // publish updated faults messages (exonerated faults published in cameraRawCb)
    publishFaultsMessage(m_camera_faults_msg_pub, CameraFaultsStatus(), m_camera_faults_flags);
    publishFaultsMessage(m_system_faults_msg_pub, SystemFaultsStatus(), m_system_faults_flags);
  }
  m_camera_data_pending = true;
}

void FaultDetector::cameraRawCb(const sensor_msgs::Image& msg)
{
  // exonerate fault when camera_raw data is received
  m_camera_faults_flags &= ~CameraFaultsStatus::NO_IMAGE;
  m_system_faults_flags &= ~SystemFaultsStatus::CAMERA_EXECUTION_ERROR;
  m_camera_data_pending = false;

  // publish updated faults messages
  publishFaultsMessage(m_camera_faults_msg_pub, CameraFaultsStatus(), m_camera_faults_flags);
  publishFaultsMessage(m_system_faults_msg_pub, SystemFaultsStatus(), m_system_faults_flags);
}

//// Power Topic Listeners
void FaultDetector::powerTemperatureListener(const BatteryTemperature& msg)
{
  // check for excessive battery temperature
  if (msg.value > POWER_THERMAL_MAX) {
    m_power_faults_flags |= PowerFaultsStatus::THERMAL_FAULT;
  } else {
    m_power_faults_flags &= ~PowerFaultsStatus::THERMAL_FAULT;
  }

  // update system faults
  if (PowerFaultsStatus::NONE == m_power_faults_flags) {
    m_system_faults_flags &= ~SystemFaultsStatus::POWER_EXECUTION_ERROR;
  } else {
    m_system_faults_flags |= SystemFaultsStatus::POWER_EXECUTION_ERROR;
  }

  // publish updated faults messages
  publishFaultsMessage(m_power_faults_msg_pub, PowerFaultsStatus(), m_power_faults_flags);
  publishFaultsMessage(m_system_faults_msg_pub, SystemFaultsStatus(), m_system_faults_flags);
}
 
void FaultDetector::powerSOCListener(const BatteryStateOfCharge& msg)
{
  // set initial state of charge
  float current_soc = msg.value;
  if (isnan(m_last_soc)){
    m_last_soc = current_soc;
  }

  // check for low state of charge
  if (current_soc <= POWER_SOC_MIN) {
    m_power_faults_flags |= PowerFaultsStatus::LOW_STATE_OF_CHARGE;
  } else {
    m_power_faults_flags &= ~PowerFaultsStatus::LOW_STATE_OF_CHARGE;
  }

  // check for excessive instant capacity loss
  if ((abs(m_last_soc - current_soc) / m_last_soc) >= POWER_SOC_MAX_DIFF) {
    m_power_faults_flags |= PowerFaultsStatus::INSTANTANEOUS_CAPACITY_LOSS;
  } else {
    m_power_faults_flags &= ~PowerFaultsStatus::INSTANTANEOUS_CAPACITY_LOSS;
  }
  m_last_soc = current_soc;

  // update system faults
  if (PowerFaultsStatus::NONE == m_power_faults_flags) {
    m_system_faults_flags &= ~SystemFaultsStatus::POWER_EXECUTION_ERROR;
  } else {
    m_system_faults_flags |= SystemFaultsStatus::POWER_EXECUTION_ERROR;
  }

  // publish updated faults messages
  publishFaultsMessage(m_power_faults_msg_pub, PowerFaultsStatus(), m_power_faults_flags);
  publishFaultsMessage(m_system_faults_msg_pub, SystemFaultsStatus(), m_system_faults_flags);
}




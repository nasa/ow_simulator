// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ow_faults_detection/FaultDetector.h"
#include <algorithm>

using namespace std;
using namespace ow_lander;

constexpr std::bitset<10> FaultDetector::isCamExecutionError;
constexpr std::bitset<10> FaultDetector::isPanTiltExecutionError;
constexpr std::bitset<10> FaultDetector::isArmExecutionError;
constexpr std::bitset<10> FaultDetector::isPowerSystemFault;

constexpr std::bitset<3> FaultDetector::islowVoltageError;
constexpr std::bitset<3> FaultDetector::isCapLossError;
constexpr std::bitset<3> FaultDetector::isThermalError;

FaultDetector::FaultDetector(ros::NodeHandle& node_handle)
{
  srand (static_cast <unsigned> (time(0)));
  // antenna
  auto original_str = "/_original";
  auto ant_pan_str = "/ant_pan_position_controller";
  auto ant_tilt_str = "/ant_tilt_position_controller";
  m_ant_pan_command_sub = node_handle.subscribe( string("/_original") + ant_pan_str + string("/command"),
                                          10,
                                          &FaultDetector::antennaPanCommandCb,
                                          this);
  m_ant_pan_state_sub = node_handle.subscribe(ant_pan_str + string("/state"),
                                          10,
                                          &FaultDetector::antennaPanStateCb,
                                          this);
  m_ant_tilt_command_sub = node_handle.subscribe( string("/_original") + ant_tilt_str + string("/command"),
                                          10,
                                          &FaultDetector::antennaTiltCommandCb,
                                          this);
  m_ant_tilt_state_sub = node_handle.subscribe(ant_tilt_str + string("/state"),
                                          10,
                                          &FaultDetector::antennaTiltStateCb,
                                          this);
  // camera
  auto image_trigger_str = "/StereoCamera/left/image_trigger";
  m_camera_original_trigger_sub = node_handle.subscribe(string("/_original") + image_trigger_str,
    10, &FaultDetector::cameraTriggerOriginalCb, this);
  m_camera_trigger_sub = node_handle.subscribe(image_trigger_str,
    10, &FaultDetector::cameraTriggerCb, this);
  m_camera_trigger_timer = node_handle.createTimer(ros::Duration(0.1), &FaultDetector::cameraTriggerPublishCb, this);

  //  power fault publishers and subs
  m_power_soc_sub = node_handle.subscribe("/power_system_node/state_of_charge",
                                          10,
                                          &FaultDetector::powerSOCListener,
                                          this);
  m_power_temperature_sub = node_handle.subscribe("/power_system_node/battery_temperature",
                                                  10,
                                                  &FaultDetector::powerTemperatureListener,
                                                  this);

  // topics for JPL msgs: system fault messages, see Faults.msg, Arm.msg, Power.msg, PTFaults.msg
  m_antenna_fault_msg_pub = node_handle.advertise<ow_faults::PTFaults>("/faults/pt_faults_status", 10);
  m_camera_fault_msg_pub = node_handle.advertise<ow_faults::CamFaults>("/faults/cam_faults_status", 10);
  m_power_fault_msg_pub = node_handle.advertise<ow_faults::PowerFaults>("/faults/power_faults_status", 10);
  m_system_fault_msg_pub = node_handle.advertise<ow_faults::SystemFaults>("/faults/system_faults_status", 10);

}

// Creating Fault Messages
template<typename fault_msg>
void FaultDetector::setFaultsMessageHeader(fault_msg& msg){
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
}

template<typename bitsetFaultsMsg, typename bitmask>
void FaultDetector::setBitsetFaultsMessage(bitsetFaultsMsg& msg, bitmask bm) {
  setFaultsMessageHeader(msg);
  msg.value = bm.to_ullong();
}

template<typename fault_msg>
void FaultDetector::setComponentFaultsMessage(fault_msg& msg, ComponentFaults value) {
  setFaultsMessageHeader(msg);
  msg.value = static_cast<uint>(value);
}

// publish system messages
void FaultDetector::publishSystemFaultsMessage(){
  ow_faults::SystemFaults system_faults_msg;
  setBitsetFaultsMessage(system_faults_msg, m_system_faults_bitset);
  m_system_fault_msg_pub.publish(system_faults_msg);
}

//// Publish Camera Messages
void FaultDetector::cameraTriggerPublishCb(const ros::TimerEvent& t){
  ow_faults::CamFaults camera_faults_msg;

  if (m_cam_og_trigger_time != m_cam_trigger_time) {
    m_system_faults_bitset |= isCamExecutionError;
    setComponentFaultsMessage(camera_faults_msg, ComponentFaults::Hardware);
  } else {
    m_system_faults_bitset &= ~isCamExecutionError;
  }

  publishSystemFaultsMessage();
  m_camera_fault_msg_pub.publish(camera_faults_msg);
}

//// Publish Power Faults Messages
void FaultDetector::publishPowerSystemFault(){
  ow_faults::PowerFaults power_faults_msg;
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
//// Antenna Listeners
void FaultDetector::antennaPanCommandCb(const std_msgs::Float64& msg){
  antPublishFaultMessages( msg.data, m_ant_pan_set_point);
}

void FaultDetector::antennaTiltCommandCb(const std_msgs::Float64& msg){
  antPublishFaultMessages(msg.data, m_ant_tilt_set_point);
}

void FaultDetector::antPublishFaultMessages(float command, float m_set_point ){
  ow_faults::PTFaults ant_pan_fault_msg;

  if (command != m_set_point) {
    setComponentFaultsMessage(ant_pan_fault_msg, ComponentFaults::Hardware);
    m_system_faults_bitset |= isPanTiltExecutionError;
  }else {
    m_system_faults_bitset &= ~isPanTiltExecutionError;
  }
  publishSystemFaultsMessage();
  m_antenna_fault_msg_pub.publish(ant_pan_fault_msg);
}

void FaultDetector::antennaPanStateCb(const control_msgs::JointControllerState& msg){
  m_ant_pan_set_point = msg.set_point;
}

void FaultDetector::antennaTiltStateCb(const control_msgs::JointControllerState& msg){
  m_ant_tilt_set_point = msg.set_point;
}

//// Camera listeners
void FaultDetector::cameraTriggerOriginalCb(const std_msgs::Empty& msg){
  m_cam_og_trigger_time = ros::Time::now();
}

void FaultDetector::cameraTriggerCb(const std_msgs::Empty& msg){
  m_cam_trigger_time = ros::Time::now();
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

// helper functions
float FaultDetector::getRandomFloatFromRange( float min_val, float max_val){
  return min_val + (max_val - min_val) * (rand() / static_cast<float>(RAND_MAX));
}

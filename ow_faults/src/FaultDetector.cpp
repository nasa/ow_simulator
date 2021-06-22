// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ow_faults/FaultDetector.h"
#include <algorithm>

using namespace std;
// using namespace ow_lander;

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

 auto image_trigger_str = "/StereoCamera/left/image_trigger";
  m_camera_original_trigger_sub = node_handle.subscribe(string("/_original") + image_trigger_str,
    10, &FaultDetector::cameraTriggerOriginalCb, this);
  m_camera_trigger_sub = node_handle.subscribe(image_trigger_str,
    10, &FaultDetector::cameraTriggerCb, this);

  // topics for JPL msgs: system fault messages, see Faults.msg, Arm.msg, Power.msg, PTFaults.msg
  // m_antenna_fault_msg_pub = node_handle.advertise<ow_faults::PTFaults>("/faults/pt_faults_status", 10);
  // m_arm_fault_msg_pub = node_handle.advertise<ow_faults::ArmFaults>("/faults/arm_faults_status", 10);
  m_camera_fault_msg_pub = node_handle.advertise<ow_faults::CamFaults>("/faults/cam_faults_status", 10);
  // m_power_fault_msg_pub = node_handle.advertise<ow_faults::PowerFaults>("/faults/power_faults_status", 10);
  m_system_fault_msg_pub = node_handle.advertise<ow_faults::SystemFaults>("/faults/system_faults_status", 10);

  //  power fault publishers and subs
  m_power_soc_sub = node_handle.subscribe("/power_system_node/state_of_charge",
                                          10,
                                          &FaultDetector::powerSOCListener,
                                          this);
  m_power_temperature_sub = node_handle.subscribe("/power_system_node/battery_temperature",
                                                  10,
                                                  &FaultDetector::powerTemperatureListener,
                                                  this);
}

// void FaultDetector::faultsConfigCb(ow_faults::FaultsConfig& faults, uint32_t level)
// {
//   // This is where we would check to see if faults is different from m_faults
//   // if we wanted to change some state based on that.

//   // Store current set of faults for later use
//   m_faults = faults;
// }

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

void FaultDetector::cameraTriggerOriginalCb(const std_msgs::Empty& msg){
  ow_faults::CamFaults camera_faults_msg;
  m_cam_trigger_on = true;
}

void FaultDetector::cameraTriggerCb(const std_msgs::Empty& msg){
  ow_faults::CamFaults camera_faults_msg;

  if (m_cam_trigger_on){
    m_system_faults_bitset |= isCamExecutionError;
    setComponentFaultsMessage(camera_faults_msg, ComponentFaults::Hardware);
  } else {
    m_system_faults_bitset &= ~isCamExecutionError;
  }

  publishSystemFaultsMessage();
  m_camera_fault_msg_pub.publish(camera_faults_msg);
}

// void FaultDetector::publishAntennaeFaults(const std_msgs::Float64& msg, bool encoder, bool torque, float& m_faultValue, ros::Publisher& m_publisher){
//   std_msgs::Float64 out_msg;

//   if (!(encoder || torque)) {
//     m_faultValue = msg.data;
//   }
//   out_msg.data = m_faultValue;
//   m_publisher.publish(out_msg);
// }

// // Note for torque sensor failure, we are finding whether or not the hardware faults for antenna are being triggered.
// // Given that, this is separate from the torque sensor implemented by Ussama.
// void FaultDetector::antennaPanFaultCb(const std_msgs::Float64& msg){
//   publishAntennaeFaults(msg,
//                         m_faults.ant_pan_encoder_failure,
//                         m_faults.ant_pan_effort_failure,
//                         m_fault_pan_value, m_fault_ant_pan_remapped_pub );
// }

// void FaultDetector::antennaTiltFaultCb(const std_msgs::Float64& msg){
//   publishAntennaeFaults(msg,
//                         m_faults.ant_tilt_encoder_failure,
//                         m_faults.ant_tilt_effort_failure,
//                         m_fault_tilt_value, m_fault_ant_tilt_remapped_pub );
// }

float FaultDetector::getRandomFloatFromRange( float min_val, float max_val){
  return min_val + (max_val - min_val) * (rand() / static_cast<float>(RAND_MAX));
}

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

// void FaultDetector::jointStateCb(const sensor_msgs::JointStateConstPtr& msg)
// {
//   // Populate the map once here.
//   // This assumes the collection of joints will never change.
//   if (m_joint_state_indices.empty()) {
//     for (int j = 0; j < NUM_JOINTS; j ++) {
//       int index = findPositionInGroup(msg->name, joint_names[j]);
//       if (index >= 0)
//         m_joint_state_indices.push_back(index);
//     }
//   }

//   sensor_msgs::JointState output(*msg);

//   ow_faults::SystemFaults system_faults_msg;
//   ow_faults::ArmFaults arm_faults_msg;
//   ow_faults::PTFaults pt_faults_msg;
//   ow_faults::CamFaults camera_faults_msg;

//   ComponentFaults hardwareFault =  ComponentFaults::Hardware;

//   // Set failed sensor values to 0
//   unsigned int index;

//   checkArmFaults();
//   checkAntFaults();
//   checkCamFaults();

//   //pant tilt faults
//   if (m_faults.ant_pan_encoder_failure && findJointIndex(J_ANT_PAN, index)) {
//     output.position[index] = FAULT_ZERO_TELEMETRY;
//   }
//   if (m_faults.ant_pan_effort_failure && findJointIndex(J_ANT_PAN, index)) {
//     output.effort[index]  = FAULT_ZERO_TELEMETRY;
//   }
//   if (m_faults.ant_tilt_encoder_failure && findJointIndex(J_ANT_TILT, index)) {
//     output.position[index]  = FAULT_ZERO_TELEMETRY;
//   }
//   if (m_faults.ant_tilt_effort_failure && findJointIndex(J_ANT_TILT, index)) {
//     output.effort[index]  = FAULT_ZERO_TELEMETRY;
//   }

//   if (m_ant_fault){
//     m_system_faults_bitset |= isPanTiltExecutionError;
//     setComponentFaultsMessage(pt_faults_msg, hardwareFault);
//   } else {
//     m_system_faults_bitset &= ~isPanTiltExecutionError;
//   }

//   //arm faults
//   if (m_faults.shou_yaw_encoder_failure && findJointIndex(J_SHOU_YAW, index)) {
//     output.position[index]  = FAULT_ZERO_TELEMETRY;
//   }
//   if (m_faults.shou_yaw_effort_failure && findJointIndex(J_SHOU_YAW, index)) {
//     output.effort[index]  = FAULT_ZERO_TELEMETRY;
//   }

//   if (m_faults.shou_pitch_encoder_failure && findJointIndex(J_SHOU_PITCH, index)) {
//     output.position[index]  = FAULT_ZERO_TELEMETRY;
//   }
//   if (m_faults.shou_pitch_effort_failure && findJointIndex(J_SHOU_PITCH, index)) {
//     output.effort[index]  = FAULT_ZERO_TELEMETRY;
//   }

//   if (m_faults.prox_pitch_encoder_failure && findJointIndex(J_PROX_PITCH, index)) {
//     output.position[index]  = FAULT_ZERO_TELEMETRY;
//   }
//   if (m_faults.prox_pitch_effort_failure && findJointIndex(J_PROX_PITCH, index)) {
//     output.effort[index]  = FAULT_ZERO_TELEMETRY;
//   }

//   if (m_faults.dist_pitch_encoder_failure && findJointIndex(J_DIST_PITCH, index)) {
//     output.position[index]  = FAULT_ZERO_TELEMETRY;
//   }
//   if (m_faults.dist_pitch_effort_failure && findJointIndex(J_DIST_PITCH, index)) {
//     output.effort[index]  = FAULT_ZERO_TELEMETRY;
//   }

//   if (m_faults.hand_yaw_encoder_failure && findJointIndex(J_HAND_YAW, index)) {
//     output.position[index]  = FAULT_ZERO_TELEMETRY;
//   }
//   if (m_faults.hand_yaw_effort_failure && findJointIndex(J_HAND_YAW, index)) {
//     output.effort[index]  = FAULT_ZERO_TELEMETRY;
//   }

//   if (m_faults.scoop_yaw_encoder_failure && findJointIndex(J_SCOOP_YAW, index)) {
//     output.position[index]  = FAULT_ZERO_TELEMETRY;
//   }
//   if (m_faults.scoop_yaw_effort_failure && findJointIndex(J_SCOOP_YAW, index)) {
//     output.effort[index]  = FAULT_ZERO_TELEMETRY;
//   }

//   if (m_arm_fault) {
//     m_system_faults_bitset |= isArmExecutionError;
//     setComponentFaultsMessage(arm_faults_msg, hardwareFault);
//   } else {
//     m_system_faults_bitset &= ~isArmExecutionError;
//   }

//   if (m_cam_fault){
//     m_system_faults_bitset |= isCamExecutionError;
//     setComponentFaultsMessage(camera_faults_msg, hardwareFault);
//   } else {
//     m_system_faults_bitset &= ~isCamExecutionError;
//   }

//   m_joint_state_pub.publish(output);
//   publishSystemFaultsMessage();

//   m_arm_fault_msg_pub.publish(arm_faults_msg);
//   m_antenna_fault_msg_pub.publish(pt_faults_msg);
//   m_camera_fault_msg_pub.publish(camera_faults_msg);
// }

void FaultDetector::publishSystemFaultsMessage(){
  ow_faults::SystemFaults system_faults_msg;
  setBitsetFaultsMessage(system_faults_msg, m_system_faults_bitset);
  m_system_fault_msg_pub.publish(system_faults_msg);
}

// void FaultDetector::distPitchFtSensorCb(const geometry_msgs::WrenchStamped& msg)
// {
//   if (!m_faults.groups.ft_sensor_faults.enable) {
//     m_dist_pitch_ft_sensor_pub.publish(msg);
//     return;
//   }

//   auto out_msg = msg;

//   if (m_faults.groups.ft_sensor_faults.zero_signal_failure) {
//     out_msg.wrench.force = geometry_msgs::Vector3();
//     out_msg.wrench.torque = geometry_msgs::Vector3();
//   }

//   auto mean = m_faults.groups.ft_sensor_faults.signal_bias_failure;
//   auto stddev = m_faults.groups.ft_sensor_faults.signal_noise_failure;
//   // TODO: consider optimizing this by re-creating the distribution only when
//   // mean and stddev values change
//   auto normal_dist = std::normal_distribution<float>(mean, stddev);
//   out_msg.wrench.force.x += normal_dist(m_random_generator);
//   out_msg.wrench.force.y += normal_dist(m_random_generator);
//   out_msg.wrench.force.z += normal_dist(m_random_generator);
//   out_msg.wrench.torque.x += normal_dist(m_random_generator);
//   out_msg.wrench.torque.y += normal_dist(m_random_generator);
//   out_msg.wrench.torque.z += normal_dist(m_random_generator);

//   m_dist_pitch_ft_sensor_pub.publish(out_msg);
// }

// void FaultDetector::checkArmFaults(){
//   m_arm_fault = (m_faults.shou_yaw_encoder_failure || m_faults.shou_yaw_effort_failure ||
//                 m_faults.shou_pitch_encoder_failure || m_faults.shou_pitch_effort_failure ||
//                 m_faults.prox_pitch_encoder_failure || m_faults.prox_pitch_effort_failure ||
//                 m_faults.dist_pitch_encoder_failure || m_faults.dist_pitch_effort_failure ||
//                 m_faults.hand_yaw_encoder_failure || m_faults.hand_yaw_effort_failure ||
//                 m_faults.scoop_yaw_encoder_failure || m_faults.scoop_yaw_effort_failure);
// }

// void FaultDetector::checkAntFaults(){
//   m_ant_fault = (m_faults.ant_pan_encoder_failure || m_faults.ant_pan_effort_failure ||
//                 m_faults.ant_tilt_encoder_failure || m_faults.ant_tilt_effort_failure);
// }

// void FaultDetector::checkCamFaults(){
//   m_cam_fault = m_faults.camera_left_trigger_failure ;
// }

// template<typename group_t, typename item_t>
// int FaultDetector::findPositionInGroup(const group_t& group, const item_t& item)
// {
//   auto iter = std::find(group.begin(), group.end(), item);
//   if (iter == group.end())
//     return -1;
//   return iter - group.begin();
// }

// bool FaultDetector::findJointIndex(const unsigned int joint, unsigned int& out_index)
// {
//   if(joint >= NUM_JOINTS)
//     return false;

//   out_index = m_joint_state_indices[joint];
//   return true;
// }
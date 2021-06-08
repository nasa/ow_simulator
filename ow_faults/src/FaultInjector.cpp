// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ow_faults/FaultInjector.h"
#include <algorithm>

using namespace std;
using namespace ow_lander;

constexpr std::bitset<10> FaultInjector::isCamExecutionError;
constexpr std::bitset<10> FaultInjector::isPanTiltExecutionError;
constexpr std::bitset<10> FaultInjector::isArmExecutionError;
constexpr std::bitset<10> FaultInjector::isPowerSystemFault;

constexpr std::bitset<3> FaultInjector::islowVoltageError;
constexpr std::bitset<3> FaultInjector::isCapLossError;
constexpr std::bitset<3> FaultInjector::isThermalError;

FaultInjector::FaultInjector(ros::NodeHandle& node_handle)
{
  //  arm pub and subs
  const char* joint_states_str = "joint_states";
  m_joint_state_sub = node_handle.subscribe(string("/_original/") + joint_states_str, 10,
    &FaultInjector::jointStateCb, this);
  m_joint_state_pub = node_handle.advertise<sensor_msgs::JointState>(joint_states_str, 10);

  auto ft_sensor_dist_pitch_str = "ft_sensor_dist_pitch";
  m_dist_pitch_ft_sensor_sub = node_handle.subscribe(string("/_original/") + ft_sensor_dist_pitch_str,
    10, &FaultInjector::distPitchFtSensorCb, this);
  m_dist_pitch_ft_sensor_pub = node_handle.advertise<geometry_msgs::WrenchStamped>(ft_sensor_dist_pitch_str, 10);

  m_camera_trigger_sub = node_handle.subscribe("/_original/StereoCamera/left/image_trigger",
    10, &FaultInjector::cameraTriggerCb, this);
  m_camera_trigger_remapped_pub = node_handle.advertise<std_msgs::Empty>("/StereoCamera/left/image_trigger", 10);

  m_fault_ant_pan_remapped_pub = node_handle.advertise<std_msgs::Float64>("/ant_pan_position_controller/command", 10);
  m_fault_ant_tilt_remapped_pub = node_handle.advertise<std_msgs::Float64>("/ant_tilt_position_controller/command", 10);

  // topics for JPL msgs: system fault messages, see Faults.msg, Arm.msg, Power.msg, PTFaults.msg
  m_antennae_fault_jpl_msg_pub = node_handle.advertise<ow_faults::PTFaults>("/faults/pt_faults_status", 10);
  m_arm_fault_jpl_msg_pub = node_handle.advertise<ow_faults::ArmFaults>("/faults/arm_faults_status", 10);
  m_camera_fault_jpl_msg_pub = node_handle.advertise<ow_faults::CamFaults>("/faults/cam_faults_status", 10);
  m_power_fault_jpl_msg_pub = node_handle.advertise<ow_faults::PowerFaults>("/faults/power_faults_status", 10);
  m_system_fault_jpl_msg_pub = node_handle.advertise<ow_faults::SystemFaults>("/faults/system_faults_status", 10);

  //power fault publishers and subs
  m_power_soc_sub = node_handle.subscribe("/power_system_node/state_of_charge",
                                          10,
                                          &FaultInjector::powerSOCListener,
                                          this);
  m_power_temperature_sub = node_handle.subscribe("/power_system_node/battery_temperature",
                                                  10,
                                                  &FaultInjector::powerTemperatureListener,
                                                  this);

  //antenna fault publishers and subs
  m_fault_ant_pan_sub = node_handle.subscribe("/_original/ant_pan_position_controller/command",
                                              3,
                                              &FaultInjector::antennaePanFaultCb,
                                              this);
  m_fault_ant_tilt_sub = node_handle.subscribe("/_original/ant_tilt_position_controller/command",
                                              3,
                                              &FaultInjector::antennaeTiltFaultCb,
                                              this);

  srand (static_cast <unsigned> (time(0)));
}

void FaultInjector::faultsConfigCb(ow_faults::FaultsConfig& faults, uint32_t level)
{
  // This is where we would check to see if faults is different from m_faults
  // if we wanted to change some state based on that.

  // Store current set of faults for later use
  m_faults = faults;
}

// Creating Fault Messages
template<typename fault_msg>
void FaultInjector::setFaultsMessageHeader(fault_msg& msg){
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
}

template<typename bitsetFaultsMsg, typename bitmask>
void FaultInjector::setBitsetFaultsMessage(bitsetFaultsMsg& msg, bitmask bm) {
  setFaultsMessageHeader(msg);
  msg.value = bm.to_ullong();
}

template<typename fault_msg>
void FaultInjector::setComponentFaultsMessage(fault_msg& msg, ComponentFaults value) {
  setFaultsMessageHeader(msg);
  msg.value = static_cast<uint>(value);
}

void FaultInjector::cameraTriggerCb(const std_msgs::Empty& msg){
  ow_faults::SystemFaults system_faults_msg;
  ow_faults::CamFaults camera_faults_msg;

  if (m_faults.camera_left_trigger_failure) {// if fault
    setComponentFaultsMessage(camera_faults_msg, ComponentFaults::Hardware);
    m_system_faults_bitset |= isCamExecutionError;
  } else { //no fault
    std_msgs::Empty msg;
    m_camera_trigger_remapped_pub.publish(msg);
  }
  setBitsetFaultsMessage(system_faults_msg, m_system_faults_bitset);
  m_camera_fault_jpl_msg_pub.publish(camera_faults_msg);
  m_system_fault_jpl_msg_pub.publish(system_faults_msg);
}

void FaultInjector::publishAntennaeFaults(const std_msgs::Float64& msg, bool encoder, bool torque, float& m_faultValue, ros::Publisher& m_publisher){
  std_msgs::Float64 out_msg;
  ow_faults::PTFaults pt_faults_msg;

  if (!(encoder || torque)) {
    m_faultValue = msg.data;
  } 
  out_msg.data = m_faultValue;
  m_publisher.publish(out_msg);

  if (m_ant_fault) {
    setComponentFaultsMessage(pt_faults_msg, ComponentFaults::Hardware);
  }
  m_antennae_fault_jpl_msg_pub.publish(pt_faults_msg);
}

// Note for torque sensor failure, we are finding whether or not the hardware faults for antenna are being triggered.
// Given that, this is separate from the torque sensor implemented by Ussama.
void FaultInjector::antennaePanFaultCb(const std_msgs::Float64& msg){
  checkAntFaults();
  publishAntennaeFaults(msg,
                        m_faults.ant_pan_encoder_failure,
                        m_faults.ant_pan_effort_failure,
                        m_fault_pan_value, m_fault_ant_pan_remapped_pub );
}

void FaultInjector::antennaeTiltFaultCb(const std_msgs::Float64& msg){
  checkAntFaults();
  publishAntennaeFaults(msg,
                        m_faults.ant_tilt_encoder_failure,
                        m_faults.ant_tilt_effort_failure,
                        m_fault_tilt_value, m_fault_ant_tilt_remapped_pub );
}

float FaultInjector::getRandomFloatFromRange( float min_val, float max_val){
  return min_val + (max_val - min_val) * (rand() / static_cast<float>(RAND_MAX));
}

void FaultInjector::publishPowerSystemFault(){
  ow_faults::PowerFaults power_faults_msg;
  ow_faults::SystemFaults system_faults_msg;
  //update if fault
  if (m_temperature_fault || m_soc_fault) {
    //system
    m_system_faults_bitset |= isPowerSystemFault;
    //power
    setComponentFaultsMessage(power_faults_msg, ComponentFaults::Hardware);
  }
  //publish
  setBitsetFaultsMessage(system_faults_msg, m_system_faults_bitset);
  m_system_fault_jpl_msg_pub.publish(system_faults_msg);
  m_power_fault_jpl_msg_pub.publish(power_faults_msg);

}

void FaultInjector::powerTemperatureListener(const std_msgs::Float64& msg)
{
  m_temperature_fault = msg.data > THERMAL_MAX;
  publishPowerSystemFault();
}

void FaultInjector::powerSOCListener(const std_msgs::Float64& msg)
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

  ComponentFaults hardwareFault =  ComponentFaults::Hardware;
  std::bitset<10> systemFaultsBitmask{};

  // Set failed sensor values to 0
  unsigned int index;

  checkArmFaults();
  checkAntFaults();

  //pant tilt faults
  if (m_faults.ant_pan_encoder_failure && findJointIndex(J_ANT_PAN, index)) {
    output.position[index] = FAULT_ZERO_TELEMETRY;
  }
  if (m_faults.ant_pan_effort_failure && findJointIndex(J_ANT_PAN, index)) {
    output.effort[index]  = FAULT_ZERO_TELEMETRY;
  }
  if (m_faults.ant_tilt_encoder_failure && findJointIndex(J_ANT_TILT, index)) {
    output.position[index]  = FAULT_ZERO_TELEMETRY;
  }
  if (m_faults.ant_tilt_effort_failure && findJointIndex(J_ANT_TILT, index)) {
    output.effort[index]  = FAULT_ZERO_TELEMETRY;
  }

  if (m_ant_fault){
    systemFaultsBitmask |= isPanTiltExecutionError;
  }

  //arm faults
  if (m_faults.shou_yaw_encoder_failure && findJointIndex(J_SHOU_YAW, index)) {
    output.position[index]  = FAULT_ZERO_TELEMETRY;
  }
  if (m_faults.shou_yaw_effort_failure && findJointIndex(J_SHOU_YAW, index)) {
    output.effort[index]  = FAULT_ZERO_TELEMETRY;
  }

  if (m_faults.shou_pitch_encoder_failure && findJointIndex(J_SHOU_PITCH, index)) {
    output.position[index]  = FAULT_ZERO_TELEMETRY;
  }
  if (m_faults.shou_pitch_effort_failure && findJointIndex(J_SHOU_PITCH, index)) {
    output.effort[index]  = FAULT_ZERO_TELEMETRY;
  }

  if (m_faults.prox_pitch_encoder_failure && findJointIndex(J_PROX_PITCH, index)) {
    output.position[index]  = FAULT_ZERO_TELEMETRY;
  }
  if (m_faults.prox_pitch_effort_failure && findJointIndex(J_PROX_PITCH, index)) {
    output.effort[index]  = FAULT_ZERO_TELEMETRY;
  }

  if (m_faults.dist_pitch_encoder_failure && findJointIndex(J_DIST_PITCH, index)) {
    output.position[index]  = FAULT_ZERO_TELEMETRY;
  }
  if (m_faults.dist_pitch_effort_failure && findJointIndex(J_DIST_PITCH, index)) {
    output.effort[index]  = FAULT_ZERO_TELEMETRY;
  }

  if (m_faults.hand_yaw_encoder_failure && findJointIndex(J_HAND_YAW, index)) {
    output.position[index]  = FAULT_ZERO_TELEMETRY;
  }
  if (m_faults.hand_yaw_effort_failure && findJointIndex(J_HAND_YAW, index)) {
    output.effort[index]  = FAULT_ZERO_TELEMETRY;
  }

  if (m_faults.scoop_yaw_encoder_failure && findJointIndex(J_SCOOP_YAW, index)) {
    output.position[index]  = FAULT_ZERO_TELEMETRY;
  }
  if (m_faults.scoop_yaw_effort_failure && findJointIndex(J_SCOOP_YAW, index)) {
    output.effort[index]  = FAULT_ZERO_TELEMETRY;
  }

  if (m_arm_fault) {
    systemFaultsBitmask |= isArmExecutionError;
    setComponentFaultsMessage(arm_faults_msg, hardwareFault);
  }

  setBitsetFaultsMessage(system_faults_msg, systemFaultsBitmask);
  m_system_faults_bitset = systemFaultsBitmask;
  m_joint_state_pub.publish(output);
  // MD: don't publish here, because power faults are not included.
  // m_system_fault_jpl_msg_pub.publish(system_faults_msg);

  m_arm_fault_jpl_msg_pub.publish(arm_faults_msg);
  // m_antennae_fault_jpl_msg_pub.publish(pt_faults_msg);
}

void FaultInjector::distPitchFtSensorCb(const geometry_msgs::WrenchStamped& msg)
{
  if (!m_faults.groups.ft_sensor_faults.enable) {
    m_dist_pitch_ft_sensor_pub.publish(msg);
    return;
  }

  auto out_msg = msg;

  if (m_faults.groups.ft_sensor_faults.zero_signal_failure) {
    out_msg.wrench.force = geometry_msgs::Vector3();
    out_msg.wrench.torque = geometry_msgs::Vector3();
  }

  auto mean = m_faults.groups.ft_sensor_faults.signal_bias_failure;
  auto stddev = m_faults.groups.ft_sensor_faults.signal_noise_failure;
  // TODO: consider optimizing this by re-creating the distribution only when
  // mean and stddev values change
  auto normal_dist = std::normal_distribution<float>(mean, stddev);
  out_msg.wrench.force.x += normal_dist(m_random_generator);
  out_msg.wrench.force.y += normal_dist(m_random_generator);
  out_msg.wrench.force.z += normal_dist(m_random_generator);
  out_msg.wrench.torque.x += normal_dist(m_random_generator);
  out_msg.wrench.torque.y += normal_dist(m_random_generator);
  out_msg.wrench.torque.z += normal_dist(m_random_generator);

  m_dist_pitch_ft_sensor_pub.publish(out_msg);
}

void FaultInjector::checkArmFaults(){
  m_arm_fault = (m_faults.shou_yaw_encoder_failure || m_faults.shou_yaw_effort_failure ||
                m_faults.shou_pitch_encoder_failure || m_faults.shou_pitch_effort_failure ||
                m_faults.prox_pitch_encoder_failure || m_faults.prox_pitch_effort_failure ||
                m_faults.dist_pitch_encoder_failure || m_faults.dist_pitch_effort_failure ||
                m_faults.hand_yaw_encoder_failure || m_faults.hand_yaw_effort_failure ||
                m_faults.scoop_yaw_encoder_failure || m_faults.scoop_yaw_effort_failure);
}

void FaultInjector::checkAntFaults(){
  m_ant_fault = (m_faults.ant_pan_encoder_failure || m_faults.ant_pan_effort_failure ||
                m_faults.ant_tilt_encoder_failure || m_faults.ant_tilt_effort_failure);
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

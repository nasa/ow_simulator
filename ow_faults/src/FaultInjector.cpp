// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ow_faults/FaultInjector.h"
#include <algorithm>

using namespace std;
using namespace ow_lander;

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
  //camera sub and repub for remapped topic
  auto image_trigger_str = "/StereoCamera/left/image_raw";
  m_camera_trigger_sub = node_handle.subscribe(string("/_original") + image_trigger_str,
    10, &FaultInjector::cameraFaultRepublishCb, this);
  m_camera_trigger_remapped_pub = node_handle.advertise<sensor_msgs::Image>(image_trigger_str, 10);

  m_fault_ant_pan_remapped_pub = node_handle.advertise<std_msgs::Float64>("/ant_pan_position_controller/command", 10);
  m_fault_ant_tilt_remapped_pub = node_handle.advertise<std_msgs::Float64>("/ant_tilt_position_controller/command", 10);

  // topics for JPL msgs: system fault messages, see Faults.msg, Arm.msg, Power.msg, PTFaults.msg
  // m_antenna_fault_msg_pub = node_handle.advertise<ow_faults::PTFaults>("/faults/pt_faults_status", 10);
  // m_arm_fault_msg_pub = node_handle.advertise<ow_faults::ArmFaults>("/faults/arm_faults_status", 10);

  //antenna fault publishers and subs
  m_fault_ant_pan_sub = node_handle.subscribe("/_original/ant_pan_position_controller/command",
                                              3,
                                              &FaultInjector::antennaPanFaultCb,
                                              this);
  m_fault_ant_tilt_sub = node_handle.subscribe("/_original/ant_tilt_position_controller/command",
                                              3,
                                              &FaultInjector::antennaTiltFaultCb,
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

void FaultInjector::cameraFaultRepublishCb(const sensor_msgs::Image& msg){
  if (!m_cam_fault) {// if no fault
    m_camera_trigger_remapped_pub.publish(msg);
  } 
}

void FaultInjector::publishAntennaeFaults(const std_msgs::Float64& msg, bool encoder, bool torque, float& m_faultValue, ros::Publisher& m_publisher){
  std_msgs::Float64 out_msg;

  if (!(encoder || torque)) {
    m_faultValue = msg.data;
  }
  out_msg.data = m_faultValue;
  m_publisher.publish(out_msg);
}

// Note for torque sensor failure, we are finding whether or not the hardware faults for antenna are being triggered.
// Given that, this is separate from the torque sensor implemented by Ussama.
void FaultInjector::antennaPanFaultCb(const std_msgs::Float64& msg){
  publishAntennaeFaults(msg,
                        m_faults.ant_pan_encoder_failure,
                        m_faults.ant_pan_effort_failure,
                        m_fault_pan_value, m_fault_ant_pan_remapped_pub );
}

void FaultInjector::antennaTiltFaultCb(const std_msgs::Float64& msg){
  publishAntennaeFaults(msg,
                        m_faults.ant_tilt_encoder_failure,
                        m_faults.ant_tilt_effort_failure,
                        m_fault_tilt_value, m_fault_ant_tilt_remapped_pub );
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

  // Set failed sensor values to 0
  unsigned int index;

  checkArmFaults();
  checkAntFaults();
  checkCamFaults();

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

  // if (m_ant_fault){
  //   m_system_faults_bitset |= isPanTiltExecutionError;
  //   setComponentFaultsMessage(pt_faults_msg, hardwareFault);
  // } else {
  //   m_system_faults_bitset &= ~isPanTiltExecutionError;
  // }

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

  // if (m_arm_fault) {
  //   m_system_faults_bitset |= isArmExecutionError;
  //   setComponentFaultsMessage(arm_faults_msg, hardwareFault);
  // } else {
  //   m_system_faults_bitset &= ~isArmExecutionError;
  // }

  m_joint_state_pub.publish(output);
  // publishSystemFaultsMessage();

  // m_arm_fault_msg_pub.publish(arm_faults_msg);
  // m_antenna_fault_msg_pub.publish(pt_faults_msg);
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

void FaultInjector::checkCamFaults(){
  m_cam_fault = m_faults.camera_left_trigger_failure;
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

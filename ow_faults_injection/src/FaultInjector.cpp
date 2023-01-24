// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ow_faults_injection/FaultInjector.h"
#include <algorithm>

using namespace ow_lander;

using std::string;

FaultInjector::FaultInjector(ros::NodeHandle& nh)
{ 
  string original_str = "/_original";
  //  arm pub and subs
  const char* joint_states_str = "/joint_states";
  m_joint_state_sub = nh.subscribe( original_str + joint_states_str, 
                                    10,
                                    &FaultInjector::jointStateCb, 
                                    this);
  m_joint_state_remapped_pub = nh.advertise<sensor_msgs::JointState>(joint_states_str, 10);

  const char* arm_end_effector_force_torque_str = "/arm_end_effector_force_torque";
  m_arm_end_effector_force_torque_sensor_sub = nh.subscribe( original_str + arm_end_effector_force_torque_str,
                                             10, 
                                             &FaultInjector::ArmEndEffectorForceTorqueSensorCb, 
                                             this);
  m_arm_end_effector_force_torque_sensor_pub = nh.advertise<owl_msgs::ArmEndEffectorForceTorque>(arm_end_effector_force_torque_str, 10);

  //camera sub and repub for remapped topic
  const char* image_raw_str = "/StereoCamera/left/image_raw";
  m_camera_raw_sub = nh.subscribe( original_str + image_raw_str,
                                   10, 
                                   &FaultInjector::cameraFaultRepublishCb, 
                                   this);
  m_camera_trigger_remapped_pub = nh.advertise<sensor_msgs::Image>(image_raw_str, 10);

  srand (static_cast <unsigned> (time(0)));
}

void FaultInjector::faultsConfigCb(ow_faults_injection::FaultsConfig& faults, uint32_t level)
{
  // This is where we would check to see if faults is different from m_faults
  // if we wanted to change some state based on that.

  // Store current set of faults for later use
  m_faults = faults;
}

void FaultInjector::cameraFaultRepublishCb(const sensor_msgs::Image& msg)
{
  if (!m_camera_fault) {// if no fault
    m_camera_trigger_remapped_pub.publish(msg);
  } 
}

void FaultInjector::jointStateCb(const sensor_msgs::JointStateConstPtr& msg)
{
  sensor_msgs::JointState output(*msg);
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

  // checking rqt
  checkCameraFaults();

  m_joint_state_remapped_pub.publish(output);
}

void FaultInjector::ArmEndEffectorForceTorqueSensorCb(const geometry_msgs::WrenchStamped& msg)
{
  if (!m_faults.groups.ft_sensor_faults.enable) {
    m_arm_end_effector_force_torque_sensor_pub.publish(msg);
    return;
  }

  owl_msgs::ArmEndEffectorForceTorque out_msg;
  out_msg.header = msg.header;

  if (m_faults.groups.ft_sensor_faults.zero_signal_failure) {
    out_msg.value = geometry_msgs::Wrench();
  } else {
    out_msg.value.force.x = msg.wrench.force.x;
    out_msg.value.force.y = msg.wrench.force.y;
    out_msg.value.force.z = msg.wrench.force.z;
    out_msg.value.torque.x = msg.wrench.torque.x;
    out_msg.value.torque.y = msg.wrench.torque.y;
    out_msg.value.torque.z = msg.wrench.torque.z;
  }

  auto mean = m_faults.groups.ft_sensor_faults.signal_bias_failure;
  auto stddev = m_faults.groups.ft_sensor_faults.signal_noise_failure;
  // TODO: consider optimizing this by re-creating the distribution only when
  // mean and stddev values change
  auto normal_dist = std::normal_distribution<float>(mean, stddev);
  out_msg.value.force.x += normal_dist(m_random_generator);
  out_msg.value.force.y += normal_dist(m_random_generator);
  out_msg.value.force.z += normal_dist(m_random_generator);
  out_msg.value.torque.x += normal_dist(m_random_generator);
  out_msg.value.torque.y += normal_dist(m_random_generator);
  out_msg.value.torque.z += normal_dist(m_random_generator);

  m_arm_end_effector_force_torque_sensor_pub.publish(out_msg);
}

void FaultInjector::checkCameraFaults()
{
  m_camera_fault = m_faults.camera_left_trigger_failure;
}

int FaultInjector::findPositionInGroup(const std::vector<string>& group, const string& item)
{
  auto iter = std::find(group.begin(), group.end(), item);
  return (iter == group.end()) ? -1 :  iter - group.begin();
}

bool FaultInjector::findJointIndex(unsigned int joint, unsigned int& out_index)
{
  if(joint >= m_joint_state_indices.size())
    return false;

  out_index = m_joint_state_indices[joint];
  return true;
}

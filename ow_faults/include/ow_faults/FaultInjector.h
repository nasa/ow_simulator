// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef FaultInjector_h
#define FaultInjector_h

#include <bitset>
#include <ctime>
#include <cstdlib>
#include <ros/ros.h>
#include <cstdint>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <ow_faults/FaultsConfig.h>
#include "ow_faults/SystemFaults.h"
#include "ow_faults/ArmFaults.h"
#include "ow_faults/PowerFaults.h"
#include "ow_faults/PTFaults.h"
#include "ow_faults/CamFaults.h"
#include <ow_lander/lander_joints.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <unordered_map>
#include <random>


// This class injects simple message faults that don't need to be simulated
// at their source. Modified topics are prefixed with "/faults". This could be
// accomplished on the original topics, but, for example, in the case of
// /joint_states, this would require forking and modifying joint_state_publisher.
// Creating a modified version of the original message is simpler, and it
// leaves the original message intact in case we want to use it as part of a
// placeholder fault estimator.
class FaultInjector
{

public:
  FaultInjector(ros::NodeHandle& node_handle);
  ~FaultInjector(){}

  void faultsConfigCb(ow_faults::FaultsConfig& faults, uint32_t level);

  //arm
  static constexpr float FAULT_ZERO_TELEMETRY = 0.0;

private:
  
  ////////// functions
  void publishSystemFaultsMessage();

  //Arm functions
  // Output /faults/joint_states, a modified version of /joint_states, injecting
  // simple message faults that don't need to be simulated at their source.
  void jointStateCb(const sensor_msgs::JointStateConstPtr& msg);
  // Output /faults/joint_states, a modified version of /joint_states, injecting
  // simple message faults that don't need to be simulated at their source.
  void distPitchFtSensorCb(const geometry_msgs::WrenchStamped& msg);
  // Find an item in an std::vector or other find-able data structure, and
  // return its index. Return -1 if not found.
  template<typename group_t, typename item_t>
  int findPositionInGroup(const group_t& group, const item_t& item);
  // Get index from m_joint_index_map. If found, modify out_index and return
  // true. Otherwise, return false.
  bool findJointIndex(const unsigned int joint, unsigned int& out_index);

  //camera function
  void cameraTriggerRepublishCb(const std_msgs::Empty& msg);

  // Antennae functions
  void antennaPanFaultCb(const std_msgs::Float64& msg);
  void antennaTiltFaultCb(const std_msgs::Float64& msg);
  void publishAntennaeFaults(const std_msgs::Float64& msg, bool encoder, 
                             bool torque, float& m_faultValue, ros::Publisher& m_publisher);

  //checking rqt faults
  void checkArmFaults();
  void checkAntFaults();
  void checkCamFaults();

  // publishers and subscribers

  // arm faults
  ros::Subscriber m_joint_state_sub;
  ros::Publisher m_joint_state_pub;

  // ft sensor
  ros::Subscriber m_dist_pitch_ft_sensor_sub;
  ros::Publisher m_dist_pitch_ft_sensor_pub;

  // camera
  ros::Subscriber m_camera_trigger_sub;
  ros::Publisher m_camera_trigger_remapped_pub;

  //antenna 
  ros::Subscriber m_fault_ant_pan_sub;
  ros::Subscriber m_fault_ant_tilt_sub;
  ros::Publisher m_fault_ant_pan_remapped_pub;
  ros::Publisher m_fault_ant_tilt_remapped_pub;

  // jpl message publishers
  // ros::Publisher m_antenna_fault_msg_pub;
  // ros::Publisher m_arm_fault_msg_pub;

  ////////// vars
  //system
  std::bitset<10> m_system_faults_bitset{};

  //general component faults
  bool m_arm_fault;
  bool m_ant_fault;
  bool m_cam_fault = false;

  //arm joint faults
  ow_faults::FaultsConfig m_faults;

  // antenna vars
  float m_fault_pan_value;
  float m_fault_tilt_value;

  // Map ow_lander::joint_t enum values to indices in JointState messages
  std::vector<unsigned int> m_joint_state_indices;
  std::mt19937 m_random_generator; // Utilize a Mersenne Twister pesduo random generation
};

#endif

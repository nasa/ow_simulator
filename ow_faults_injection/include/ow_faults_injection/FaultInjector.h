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
#include <ow_faults_injection/FaultsConfig.h>
#include <ow_faults_detection/JointStatesFlag.h>
#include <std_msgs/Empty.h>
#include <ow_lander/lander_joints.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
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
  ~FaultInjector() = default;

  void faultsConfigCb(ow_faults_injection::FaultsConfig& faults, uint32_t level);

private:

  // Functions
  
  void publishSystemFaultsMessage();

  // Arm functions
  
  // Output /faults/joint_states, a modified version of /joint_states, injecting
  // simple message faults that don't need to be simulated at their source.
  void jointStateCb(const sensor_msgs::JointStateConstPtr& msg);
  
  // Output /faults/joint_states, a modified version of /joint_states, injecting
  // simple message faults that don't need to be simulated at their source.
  void distPitchFtSensorCb(const geometry_msgs::WrenchStamped& msg);
  
  // Find an item in an std::vector or other find-able data structure, and
  // return its index. Return -1 if not found.
  int findPositionInGroup(const std::vector<std::string>& group,
                          const std::string& item);
  
  // Get index from m_joint_index_map. If found, modify out_index and return
  // true. Otherwise, return false.
  bool findJointIndex(unsigned int joint, unsigned int& out_index);

  // Camera functions
  void cameraFaultRepublishCb(const sensor_msgs::Image& msg);
  void checkCamFaults();

  // Publishers and subscribers

  // Arm
  ros::Subscriber m_joint_state_sub;
  ros::Publisher m_joint_state_remapped_pub;

  // Force/Torque sensor
  ros::Subscriber m_dist_pitch_ft_sensor_sub;
  ros::Publisher m_dist_pitch_ft_sensor_pub;

  // Camera
  ros::Subscriber m_camera_raw_sub;
  ros::Publisher m_camera_trigger_remapped_pub;

  // Antenna
  ros::Subscriber m_fault_ant_pan_sub;
  ros::Subscriber m_fault_ant_tilt_sub;
  ros::Publisher m_ant_pan_remapped_pub;
  ros::Publisher m_ant_tilt_remapped_pub;

  // Variables
  
  // System
  std::bitset<10> m_system_faults_bitset{};

  // General component faults
  bool m_cam_fault = false;

  // Arm joint faults
  ow_faults_injection::FaultsConfig m_faults;

  // Antenna
  float m_fault_pan_value;
  float m_fault_tilt_value;

  // Map ow_lander::joint_t enum values to indices in JointState messages
  std::vector<unsigned int> m_joint_state_indices;

  // Utilize Mersenne Twister pseudo-random generation
  std::mt19937 m_random_generator;
};

#endif

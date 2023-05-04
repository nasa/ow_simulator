// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

#ifndef FaultDetector_h
#define FaultDetector_h

#include <bitset>
#include <ctime>
#include <cstdlib>
#include <ros/ros.h>
#include <cstdint>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <ow_faults_detection/JointStatesFlag.h>
#include <owl_msgs/ArmFaultsStatus.h>
#include <owl_msgs/BatteryStateOfCharge.h>
#include <owl_msgs/BatteryTemperature.h>
#include <owl_msgs/CameraFaultsStatus.h>
#include <owl_msgs/PanTiltFaultsStatus.h>
#include <owl_msgs/PowerFaultsStatus.h>
#include <owl_msgs/SystemFaultsStatus.h>
#include <ow_lander/lander_joints.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <control_msgs/JointControllerState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/Image.h>

// This class detects system faults and publishes the relevant fault
// information to a series of topics.
class FaultDetector
{
public:
  FaultDetector(ros::NodeHandle& nh);
  ~FaultDetector() = default;
  
  FaultDetector (const FaultDetector&) = delete;
  FaultDetector& operator= (const FaultDetector&) = delete;

  static constexpr float POWER_THERMAL_MAX = 70.0;
  static constexpr float POWER_SOC_MIN = 0.1;
  static constexpr float POWER_SOC_MAX_DIFF = 0.05;
  
private:
  // COMPONENT FUNCTIONS
  
  // Arm
  void jointStatesFlagCb(const ow_faults_detection::JointStatesFlagConstPtr& msg);
  bool isFlagSet(uint joint, const std::vector<uint8_t>& flags);
  
  // Find an item in an std::vector or other find-able data structure, and
  // return its index. Return -1 if not found.
  template<typename group_t, typename item_t>
  int findPositionInGroup(const group_t& group, const item_t& item);
  
  // Get index from m_joint_index_map. If found, modify out_index and return
  // true. Otherwise, return false.
  bool findJointIndex(const unsigned int joint, unsigned int& out_index);
  
  // Camera
  void cameraTriggerCb(const std_msgs::Empty& msg);
  void cameraRawCb(const sensor_msgs::Image& msg);

  // Power
  void powerSOCListener(const owl_msgs::BatteryStateOfCharge& msg);
  void powerTemperatureListener(const owl_msgs::BatteryTemperature& msg);

  // OWLAT MESSAGE FUNCTIONS AND PUBLISHERS
  template<typename fault_msg>
  void setFaultsMessageHeader(fault_msg& msg);
  template<typename pub_t, typename msg_t, typename flags_t>
  void publishFaultsMessage(pub_t& faults_pub, msg_t faults_msg, flags_t faults_flags);

  // PUBLISHERS AND SUBSCRIBERS
  
  // faults topic publishers
  ros::Publisher m_arm_faults_msg_pub;
  ros::Publisher m_antenna_faults_msg_pub;
  ros::Publisher m_camera_faults_msg_pub;
  ros::Publisher m_power_faults_msg_pub;
  ros::Publisher m_system_faults_msg_pub;

  // faults topic subscribers;
  ros::Subscriber m_joint_states_sub; // for arm and antenna
  ros::Subscriber m_camera_original_trigger_sub;
  ros::Subscriber m_camera_raw_sub;
  ros::Subscriber m_power_soc_sub;
  ros::Subscriber m_power_temperature_sub;

  // faults bitflags
  uint64_t m_arm_faults_flags     = owl_msgs::ArmFaultsStatus::NONE;
  uint64_t m_antenna_faults_flags = owl_msgs::PanTiltFaultsStatus::NONE;
  uint64_t m_camera_faults_flags  = owl_msgs::CameraFaultsStatus::NONE;
  uint64_t m_power_faults_flags   = owl_msgs::PowerFaultsStatus::NONE;
  uint64_t m_system_faults_flags  = owl_msgs::SystemFaultsStatus::NONE;

  // component variables
  std::vector<unsigned int> m_joint_state_indices;
  bool m_camera_data_pending = false;
  float m_last_soc = std::numeric_limits<float>::quiet_NaN();
};

#endif

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
#include "ow_faults_detection/SystemFaults.h"
#include "ow_faults_detection/ArmFaults.h"
#include "ow_faults_detection/PowerFaults.h"
#include "ow_faults_detection/PTFaults.h"
#include "ow_faults_detection/CamFaults.h"
#include <ow_lander/lander_joints.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <control_msgs/JointControllerState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/Image.h>

// This class detects system faults and publishes the relevant fault
// information to a series of topics.  Fault topics are prefixed with
// "/faults".
class FaultDetector
{
public:
  FaultDetector(ros::NodeHandle& nh);
  ~FaultDetector(){}
  
  FaultDetector (const FaultDetector&) = delete;
  FaultDetector& operator= (const FaultDetector&) = delete;

  enum class ComponentFaults : uint {
    Hardware = 1, 
    JointLimit = 2,
    TrajectoryGeneration = 2,
    Collision = 3, 
    Estop = 4, 
    PositionLimit = 5, 
    TorqueLimit = 6, 
    VelocityLimit = 7, 
    NoForceData = 8
    };

  //system
  static constexpr std::bitset<10> isSystem{                0b00'0000'0001 };
  static constexpr std::bitset<10> isArmGoalError{          0b00'0000'0010 };
  static constexpr std::bitset<10> isArmExecutionError{     0b00'0000'0100 };
  static constexpr std::bitset<10> isTaskGoalError{         0b00'0000'1000 };
  static constexpr std::bitset<10> isCamGoalError{          0b00'0001'0000 };
  static constexpr std::bitset<10> isCamExecutionError{     0b00'0010'0000 };
  static constexpr std::bitset<10> isPanTiltGoalError{      0b00'0100'0000 };
  static constexpr std::bitset<10> isPanTiltExecutionError{ 0b00'1000'0000 };
  static constexpr std::bitset<10> isLanderExecutionError{  0b01'0000'0000 };
  static constexpr std::bitset<10> isPowerSystemFault{      0b10'0000'0000 };

  //power
  static constexpr std::bitset<3> islowVoltageError{ 0b001 };
  static constexpr std::bitset<3> isCapLossError{    0b010 };
  static constexpr std::bitset<3> isThermalError{    0b100 };

  static constexpr float THERMAL_MAX = 70;
  static constexpr float SOC_MIN = 0.1;
  static constexpr float SOC_MAX_DIFF = 0.05;
  
  //arm
  static constexpr float FAULT_ZERO_TELEMETRY = 0.0;

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

  // Antenna
  void antPublishFaultMessages();
  
  // Camera
  void camerTriggerCb(const std_msgs::Empty& msg);
  void cameraRawCb(const sensor_msgs::Image& msg);
  void cameraTriggerPublishCb(const ros::TimerEvent& t);

  // Power
  void publishPowerSystemFault();
  void powerSOCListener(const std_msgs::Float64& msg);
  void powerTemperatureListener(const std_msgs::Float64& msg);

  // OWLAT MESSAGE FUNCTIONS AND PUBLISHERS
  void publishSystemFaultsMessage();
  template<typename fault_msg>
  void setFaultsMessageHeader(fault_msg& msg);
  template<typename bitsetFaultsMsg, typename bitmask>
  void setBitsetFaultsMessage(bitsetFaultsMsg& msg, bitmask systemFaultsBitmask);
  template<typename fault_msg>
  void setComponentFaultsMessage(fault_msg& msg, ComponentFaults value);


  // PUBLISHERS AND SUBSCRIBERS
  
  // faults topic publishers
  ros::Publisher m_arm_fault_msg_pub;
  ros::Publisher m_antenna_fault_msg_pub;
  ros::Publisher m_camera_fault_msg_pub;
  ros::Publisher m_power_fault_msg_pub;
  ros::Publisher m_system_fault_msg_pub;

  // Arm and antenna
  ros::Subscriber m_joint_states_sub;

  // Camera
  ros::Timer m_camera_trigger_timer;
  ros::Subscriber m_camera_original_trigger_sub;
  ros::Subscriber m_camera_raw_sub;

  // Power
  ros::Subscriber m_power_soc_sub;
  ros::Subscriber m_power_temperature_sub;

  // VARIABLES
  
  // System 
  std::bitset<10> m_system_faults_bitset{};
  std::vector<unsigned int> m_joint_state_indices;
  
  // Antenna
  bool m_pan_fault;
  bool m_tilt_fault;
  
  // Camera
  ros::Time m_cam_raw_time;
  ros::Time m_cam_trigger_time;
  
  // Power
  float m_last_SOC = std::numeric_limits<float>::quiet_NaN();
  bool m_soc_fault = false;
  bool m_temperature_fault = false;
};

#endif

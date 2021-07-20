// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef FaultDetector_h
#define FaultDetector_h

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


// This class injects simple message faults that don't need to be simulated
// at their source. Modified topics are prefixed with "/faults". This could be
// accomplished on the original topics, but, for example, in the case of
// /joint_states, this would require forking and modifying joint_state_publisher.
// Creating a modified version of the original message is simpler, and it
// leaves the original message intact in case we want to use it as part of a
// placeholder fault estimator.
class FaultDetector
{

public:
  FaultDetector(ros::NodeHandle& node_handle);
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

  static constexpr float THERMAL_MAX = 50;
  static constexpr float SOC_MIN = 0.1;
  static constexpr float SOC_MAX_DIFF = 0.05;
  
private:

  // //camera function
  void cameraTriggerOriginalCb(const std_msgs::Empty& msg);
  void cameraTriggerCb(const std_msgs::Empty& msg);
  void cameraTriggerPublishCb(const ros::TimerEvent& t);

  // // power functions
  float getRandomFloatFromRange(float min_val, float max_val);
  void publishPowerSystemFault();
  void powerSOCListener(const std_msgs::Float64& msg);
  void powerTemperatureListener(const std_msgs::Float64& msg);

  // //Setting message values
  void publishSystemFaultsMessage();
  template<typename fault_msg>
  void setFaultsMessageHeader(fault_msg& msg);
  template<typename bitsetFaultsMsg, typename bitmask>
  void setBitsetFaultsMessage(bitsetFaultsMsg& msg, bitmask systemFaultsBitmask);
  template<typename fault_msg>
  void setComponentFaultsMessage(fault_msg& msg, ComponentFaults value);
 
  // // camera
  ros::Timer m_camera_trigger_timer;
  ros::Subscriber m_camera_original_trigger_sub;
  ros::Subscriber m_camera_trigger_sub;

  // //power
  ros::Subscriber m_power_soc_sub;
  ros::Subscriber m_power_temperature_sub;
  ros::Publisher m_power_fault_trigger_pub;

  // // jpl message publishers
  ros::Publisher m_camera_fault_msg_pub;
  ros::Publisher m_power_fault_msg_pub;
  ros::Publisher m_system_fault_msg_pub;


  // ////////// vars
  // //system
  std::bitset<10> m_system_faults_bitset{};

  // //general component faults
  bool m_cam_trigger_on = false;
  bool m_soc_fault = false;
  bool m_temperature_fault = false;
  ros::Time m_cam_trigger_time;
  ros::Time m_cam_og_trigger_time;

  // //power vars
  float m_last_SOC = std::numeric_limits<float>::quiet_NaN();

};

#endif

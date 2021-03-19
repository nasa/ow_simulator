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
#include <ow_faults/FaultsConfig.h>
#include "ow_faults/SystemFaults.h"
#include "ow_faults/ArmFaults.h"
#include "ow_faults/PowerFaults.h"
#include "ow_faults/PTFaults.h"
#include <ow_lander/lander_joints.h>
#include <sensor_msgs/JointState.h>
#include <unordered_map>


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
  FaultInjector(ros::NodeHandle node_handle);
  ~FaultInjector(){}

  void faultsConfigCb(ow_faults::FaultsConfig& faults, uint32_t level);

  static constexpr float FAULT_ZERO_TELEMETRY = 0.0;

  enum Nominal { None=0 };
  
  enum class ComponentFaults : uint {
    // general
    Hardware = 1, 
    //pt
    JointLimit = 2,
    //arm 
    TrajectoryGeneration = 2,
    Collision = 3, 
    Estop = 4, 
    PositionLimit = 5, 
    TorqueLimit = 6, 
    VelocityLimit = 7, 
    NoForceData = 8
    };

	static constexpr std::bitset<10> isSystem{		0b00'0000'0001 };
	static constexpr std::bitset<10> isArmGoalError{		0b00'0000'0010 };
	static constexpr std::bitset<10> isArmExecutionError{		0b00'0000'0100 };
	static constexpr std::bitset<10> isTaskGoalError{	0b00'000'1000 };
	static constexpr std::bitset<10> isCamGoalError{	0b00'0001'0000 };
	static constexpr std::bitset<10> isCamExecutionError{	0b00'0010'0000 };
	static constexpr std::bitset<10> isPanTiltGoalError{		0b00'0100'0000 };
	static constexpr std::bitset<10> isPanTiltExecutionError{	0b00'1000'0000 };
	static constexpr std::bitset<10> isLanderExecutionError{	0b01'0000'0000 };
	static constexpr std::bitset<10> isPowerSystemFault{	0b10'0000'0000 };
  
private:
  float powerTemperatureOverloadValue;
  

  std_msgs::Float64 m_realPanMsg;
  std_msgs::Float64 m_realTiltMsg;
  float m_faultPanValue;
  float m_faultTiltValue;

  // renders new temperature when thermal power fault is re-triggered
  void setPowerTemperatureFaultValue(bool getTempBool);

  // Antennae functions
  void antennaePanFaultCb(const std_msgs::Float64& msg);
  void antennaeTiltFaultCb(const std_msgs::Float64& msg);
  void publishAntennaeFaults( std_msgs::Float64& msg, float& faultValue , ros::Publisher m_publisher);
  void handleAllAntFaults();

  // Output /faults/joint_states, a modified version of /joint_states, injecting
  // simple message faults that don't need to be simulated at their source.
  void jointStateCb(const sensor_msgs::JointStateConstPtr& msg);

  //Setting the correct values for faults messages via function overloading

  template<typename fault_msg>
  void setFaultsMessage( fault_msg& msg);
  void setSystemFaultsMessage(ow_faults::SystemFaults& msg, std::bitset<10> systemFaultsBitmask);

  template<typename fault_msg>
  void setComponentFaultsMessage(fault_msg& msg, ComponentFaults value);

  //checking faults
  void checkArmFaults();
  void checkAntFaults();

  // Find an item in an std::vector or other find-able data structure, and
  // return its index. Return -1 if not found.
  template<typename group_t, typename item_t>
  int findPositionInGroup(const group_t& group, const item_t& item);

  // Get index from m_joint_index_map. If found, modify out_index and return
  // true. Otherwise, return false.
  bool findJointIndex(const unsigned int joint, unsigned int& out_index);

  ow_faults::FaultsConfig m_faults;

  //component faults
  bool m_armFault;
  bool m_antFault;

  // arm faults
  ros::Subscriber m_joint_state_sub;
  ros::Publisher m_joint_state_pub;

  // temporary placeholder publishers until power feautre is finished
  ros::Publisher m_fault_power_state_of_charge_pub;
  ros::Publisher m_fault_power_temp_pub;

  // publishers for sending ros messages for component failures
  ros::Publisher m_fault_status_pub;
  ros::Publisher m_arm_fault_status_pub;
  ros::Publisher m_power_fault_status_pub;
  ros::Publisher m_antennae_fault_status_pub;

  //antenna pub and subs
  ros::Subscriber m_fault_ant_pan_sub;
  ros::Subscriber m_fault_ant_tilt_sub;
  ros::Publisher m_fault_ant_pan_pub;
  ros::Publisher m_fault_ant_tilt_pub;

  // Map ow_lander::joint_t enum values to indices in JointState messages
  std::vector<unsigned int> m_joint_state_indices;
};

#endif

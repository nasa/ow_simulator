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
  FaultInjector(ros::NodeHandle node_handle);
  ~FaultInjector(){}

  void faultsConfigCb(ow_faults::FaultsConfig& faults, uint32_t level);

  static constexpr float FAULT_ZERO_TELEMETRY = 0.0;

  enum Nominal { None=0 };
  
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
	static constexpr std::bitset<10> isSystem{		            0b00'0000'0001 };
	static constexpr std::bitset<10> isArmGoalError{          0b00'0000'0010 };
	static constexpr std::bitset<10> isArmExecutionError{     0b00'0000'0100 };
	static constexpr std::bitset<10> isTaskGoalError{	        0b00'0000'1000 };
	static constexpr std::bitset<10> isCamGoalError{	        0b00'0001'0000 };
	static constexpr std::bitset<10> isCamExecutionError{	    0b00'0010'0000 };
	static constexpr std::bitset<10> isPanTiltGoalError{	  	0b00'0100'0000 };
	static constexpr std::bitset<10> isPanTiltExecutionError{	0b00'1000'0000 };
	static constexpr std::bitset<10> isLanderExecutionError{	0b01'0000'0000 };
	static constexpr std::bitset<10> isPowerSystemFault{    	0b10'0000'0000 };
  
  //power
  static constexpr std::bitset<3> islowVoltageError{ 0b001 };
	static constexpr std::bitset<3> isCapLossError{  	 0b010 };
	static constexpr std::bitset<3> isThermalError{	   0b100 };

private:
  
  ////////// functions
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

  // power functions
  float getRandomFloatFromRange(float min_val, float max_val);
  void publishPowerSystemFault(bool fault);
  void powerSOCListener(const std_msgs::Float64& msg);
  void powerTemperatureListener(const std_msgs::Float64& msg);

  // Antennae functions
  void antennaePanFaultCb(const std_msgs::Float64& msg);
  void antennaeTiltFaultCb(const std_msgs::Float64& msg);
  void publishAntennaeFaults(const std_msgs::Float64& msg, bool encoder, bool torque, float& m_faultValue, ros::Publisher& m_publisher);

  //Setting message values
  template<typename fault_msg>
  void setFaultsMessageHeader(fault_msg& msg);
  template<typename bitsetFaultsMsg, typename bitmask>
  void setBitsetFaultsMessage(bitsetFaultsMsg& msg, bitmask systemFaultsBitmask);
  template<typename fault_msg>
  void setComponentFaultsMessage(fault_msg& msg, ComponentFaults value);
 
  //checking rqt faults
  void checkArmFaults();
  void checkAntFaults();

  ///////publishers and subsscribers
  // arm faults
  ros::Subscriber m_joint_state_sub;
  ros::Publisher m_joint_state_pub;

  //power
  ros::Subscriber m_power_soc_sub;
  ros::Subscriber m_power_temperature_sub;
  ros::Publisher m_power_fault_trigger_pub;

  // ft sensor
  ros::Subscriber m_dist_pitch_ft_sensor_sub;
  ros::Publisher m_dist_pitch_ft_sensor_pub;

  //antenna 
  ros::Subscriber m_fault_ant_pan_sub;
  ros::Subscriber m_fault_ant_tilt_sub;
  ros::Publisher m_fault_ant_pan_remapped_pub;
  ros::Publisher m_fault_ant_tilt_remapped_pub;

  // jpl message publishers
  ros::Publisher m_system_fault_jpl_msg_pub;
  ros::Publisher m_arm_fault_jpl_msg_pub;
  ros::Publisher m_power_fault_jpl_msg_pub;
  ros::Publisher m_antennae_fault_jpl_msg_pub;

  ////////// vars
  //system
  std::bitset<10> systemFaultsBitset;

  //general component faults
  bool m_armFault;
  bool m_antFault;

  //arm joint faults
  ow_faults::FaultsConfig m_faults;

  //power vars
  float m_originalSOC;

  // antenna vars
  std_msgs::Float64 m_realPanMsg;
  std_msgs::Float64 m_realTiltMsg;
  float m_faultPanValue;
  float m_faultTiltValue;

  // Map ow_lander::joint_t enum values to indices in JointState messages
  std::vector<unsigned int> m_joint_state_indices;
  std::mt19937 m_random_generator; // Utilize a Mersenne Twister pesduo random generation
};

#endif

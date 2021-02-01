// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef FaultInjector_h
#define FaultInjector_h

#include <python3.6m/Python.h>
#include <stdio.h>

#include <ctime>
#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ow_faults/FaultsConfig.h>
#include "ow_faults/SystemFaults.h"
#include "ow_faults/ArmFaults.h"
#include "ow_faults/PowerFaults.h"
#include <ow_lander/lander_joints.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
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

  enum Nominal { None=0 };

  enum ComponentFaults {
    // general
    Hardware=1, 
    //arm 
    TrajectoryGeneration=2, 
    Collision=3, 
    Estop=4, 
    PositionLimit=5, 
    TorqueLimit=6, 
    VelocityLimit=7, 
    NoForceData=8};

  enum SystemFaults {
    System=1, 
    ArmGoalError=2, 
    ArmExecutionError=4,
    TaskGoalError=8, 
    CamGoalError=16, 
    CamExecutionError=32, 
    PtGoalError=64, 
    PtExecutionError=128, 
    LanderExecutionError = 256};

private:
  float powerTemperatureOverloadValue;
  
  // renders new temperature when thermal power fault is re-triggered
  void setPowerTemperatureFaultValue(bool getTempBool);

  // Output /faults/joint_states, a modified version of /joint_states, injecting
  // simple message faults that don't need to be simulated at their source.
  void jointStateCb(const sensor_msgs::JointStateConstPtr& msg);
  
  // void stopArmMovement();
  // void armFailureCb(const ow_faults::SystemFaults& msg);

  //Setting the correct values for system faults and arm faults messages
  void setSytemFaultsMessage(ow_faults::SystemFaults& msg, int value);
  void setArmFaultsMessage(ow_faults::ArmFaults& msg, int value);
  void setPowerFaultsMessage(ow_faults::PowerFaults& msg, int value);

  // Find an item in an std::vector or other find-able data structure, and
  // return its index. Return -1 if not found.
  template<typename group_t, typename item_t>
  int findPositionInGroup(const group_t& group, const item_t& item);

  // Get index from m_joint_index_map. If found, modify out_index and return
  // true. Otherwise, return false.
  bool findJointIndex(const unsigned int joint, unsigned int& out_index);

  ow_faults::FaultsConfig m_faults;

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

  // ros::Subscriber m_arm_state_sub;
  // ros::Publisher m_fault_arm_plan_pub;

  // Map ow_lander::joint_t enum values to indices in JointState messages
  std::vector<unsigned int> m_joint_state_indices;
};


#endif

// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef TrajectoryAsyncExecuterCppClass_h
#define TrajectoryAsyncExecuterCppClass_h

#include "ow_faults/FaultInjector.h"
#include <python3.6m/Python.h>
#include <stdio.h>

#include <ctime>
#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ow_faults/FaultsConfig.h>
#include "ow_faults/SystemFaults.h"
#include "ow_faults/ArmFaults.h"
// #include "ow_faults/PowerFaults.h"
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
class TrajectoryAsyncExecuterCppClass
{
public:
  TrajectoryAsyncExecuterCppClass(ros::NodeHandle node_handle);
  ~TrajectoryAsyncExecuterCppClass(){}

  
private:
 
  // Output /faults/joint_states, a modified version of /joint_states, injecting
  // simple message faults that don't need to be simulated at their source.
  bool _connected = false;
//   type _goal_time_tolerance = someTime = .1;
//   type _client = NULL;

  void stopArmMovement();
  void armFailureCb(const ow_faults::SystemFaults& msg);
 
  ros::Subscriber m_arm_state_sub;
  ros::Publisher m_fault_arm_plan_pub;

};


#endif

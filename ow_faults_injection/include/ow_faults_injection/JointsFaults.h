// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef JOINTS_FAULTS_H
#define JOINTS_FAULTS_H

#include <ros/ros.h>
#include <ow_lander/lander_joints.h>
#include <ros/subscribe_options.h>
#include <ros/callback_queue.h>
#include <gazebo/common/Plugin.hh>
#include <ow_faults_detection/JointStatesFlag.h>
#include <sensor_msgs/JointState.h>

struct JointFaultInfo
{
  const std::string fault;
  bool activated;
  double friction;
  unsigned int landerJoint;

  JointFaultInfo(const std::string& fault_, unsigned int lander_joint, bool activated_ = false, double friction_ = 0.0)
  : fault(fault_ + "_joint_locked_failure"),
    activated(activated_), 
    friction(friction_),
    landerJoint(lander_joint)
  {
  }
};


class JointsFaults : public gazebo::ModelPlugin
{
public:
  JointsFaults();
  ~JointsFaults();

  virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr /* _sdf */);

private:
  void onUpdate();

  void initFlagMessage();
  void injectFault(const std::string& joint_name, JointFaultInfo& jfi);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& msg);
  int findPositionInGroup(const std::vector<std::string>& group, const std::string& item);
  // Get index from m_joint_index_map. If found, modify out_index and return
  // true. Otherwise, return false.
  bool findJointIndex(unsigned int joint, unsigned int& out_index);

  static constexpr double MAX_FRICTION = 3000.0;
  const bool SET_FLAG = true;

  gazebo::physics::ModelPtr m_model;
  std::map<std::string, JointFaultInfo> m_JointsFaultsMap;
  gazebo::event::ConnectionPtr m_updateConnection;
  gazebo::event::ConnectionPtr m_on_update_connection;

  std::unique_ptr<ros::NodeHandle> m_node_handle;
  ros::CallbackQueue m_callback_queue;

  ros::Publisher m_joint_state_flags_pub;

  std::vector<unsigned int> m_joint_state_indices;
  ow_faults_detection::JointStatesFlag m_flag_msg;

};


#endif  // JOINTS_FAULTS_H

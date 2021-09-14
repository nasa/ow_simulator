// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef JOINTS_FAULTS_H
#define JOINTS_FAULTS_H

#include <gazebo/common/Plugin.hh>

struct JointFaultInfo
{
  const std::string fault;
  bool activated;
  double friction;

  JointFaultInfo(const std::string& fault_, bool activated_ = false, double friction_ = 0.0)
  : fault(fault_), activated(activated_), friction(friction_)
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

  // void injectFault(const std::string& joint_fault, bool& fault_activated,  bool& other_active, const std::string& joint_name,
  //                  double lower_limit, double upper_limit);

  gazebo::physics::ModelPtr m_model;
  double m_antennaTiltLowerLimit;
  double m_antennaTiltUpperLimit;
  double m_antennaPanLowerLimit;
  double m_antennaPanUpperLimit;
  bool m_antennaTiltEffortFaultActivated;
  bool m_antennaPanEffortFaultActivated;
  bool m_antennaTiltEncFaultActivated;
  bool m_antennaPanEncFaultActivated;

  void injectFault(const std::string& joint_name, JointFaultInfo& jfi);

  static constexpr double MAX_FRICTION = 3000.0;

  gazebo::physics::ModelPtr m_model;
  std::map<std::string, JointFaultInfo> m_JointsFaultsMap;
  gazebo::event::ConnectionPtr m_updateConnection;
};


#endif  // JOINTS_FAULTS_H

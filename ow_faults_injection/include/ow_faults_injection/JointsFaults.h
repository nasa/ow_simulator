// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef JOINTS_FAULTS_H
#define JOINTS_FAULTS_H

#include <gazebo/common/Plugin.hh>

struct JointFaultInfo
{
  const std::string effortFault;
  const std::string encoderFault;
  bool activated;
  double friction;

  JointFaultInfo(const std::string& fault_, bool activated_ = false, double friction_ = 0.0)
  : effortFault(fault_ + "_effort_failure"), 
    encoderFault(fault_ + "_encoder_failure"), 
    activated(activated_), 
    friction(friction_)
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

  double m_antennaTiltLowerLimit;
  double m_antennaTiltUpperLimit;
  double m_antennaPanLowerLimit;
  double m_antennaPanUpperLimit;
  bool m_antennaTiltEffortFaultActivated;
  bool m_antennaPanEffortFaultActivated;
  bool m_antennaTiltEncFaultActivated;
  bool m_antennaPanEncFaultActivated;

  void injectArmFault(const std::string& joint_name, JointFaultInfo& jfi);
  void injectAntFault(const std::string& joint_name, JointFaultInfo& jfi);

  static constexpr double MAX_FRICTION = 3000.0;

  gazebo::physics::ModelPtr m_model;
  std::map<std::string, JointFaultInfo> m_JointsArmFaultsMap;
  std::map<std::string, JointFaultInfo> m_JointsAntFaultsMap;
  gazebo::event::ConnectionPtr m_updateConnection;
};


#endif  // JOINTS_FAULTS_H

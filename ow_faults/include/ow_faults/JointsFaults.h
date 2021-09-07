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
  double lower;
  double upper;

  JointFaultInfo(const std::string& fault_, bool activated_ = false, double lower_ = 0.0, double upper_ = 0.0)
  : fault(fault_), activated(activated_), lower(lower_), upper(upper_)
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

  void injectFault(const std::string& joint_fault, bool& fault_activated, const std::string& joint_name,
                   double lower_limit, double upper_limit);

  gazebo::physics::ModelPtr m_model;
  double m_antennaTiltLowerLimit;
  double m_antennaTiltUpperLimit;
  double m_antennaPanLowerLimit;
  double m_antennaPanUpperLimit;
  bool m_antennaTiltFaultActivated;
  bool m_antennaPanFaultActivated;

  std::map<std::string, JointFaultInfo> m_JointsFaultsMap;

  // Connection to the update event
  gazebo::event::ConnectionPtr m_updateConnection;
};


#endif  // JOINTS_FAULTS_H

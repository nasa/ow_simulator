// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef LinkForcePlugin_h
#define LinkForcePlugin_h

#include <gazebo/common/Plugin.hh>

namespace gazebo
{
class JointsFaults : public ModelPlugin
{
public:
  JointsFaults();
  ~JointsFaults();

  virtual void Load(physics::ModelPtr model, sdf::ElementPtr /* _sdf */);

private:
  void OnUpdate();

  physics::ModelPtr m_model;
  double m_AntennaTiltLowerLimit;
  double m_AntennaTiltUpperLimit;
  double m_AntennaPanLowerLimit;
  double m_AntennaPanUpperLimit;
  bool m_AntennaTiltFaultActivated;
  bool m_AntennaPanFaultActivated;

  void injectFault(const std::string& joint_fault, bool& fault_activated, const std::string& joint_name,
                   double lower_limit, double upper_limit);

  // Connection to the update event
  event::ConnectionPtr m_updateConnection;
};

}  // namespace gazebo

#endif

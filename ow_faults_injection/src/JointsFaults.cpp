// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ow_faults_injection/JointsFaults.h"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <ros/ros.h>

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(JointsFaults)

JointsFaults::JointsFaults() :
  ModelPlugin()
{
}

JointsFaults::~JointsFaults()
{
}

void JointsFaults::Load(physics::ModelPtr model, sdf::ElementPtr /* sdf */)
{
  m_model = model;

  auto j_ant_tilt = m_model->GetJoint("j_ant_tilt");
  m_antennaTiltLowerLimit = j_ant_tilt->LowerLimit(0);
  m_antennaTiltUpperLimit = j_ant_tilt->UpperLimit(0);

  auto j_ant_pan = m_model->GetJoint("j_ant_pan");
  m_antennaPanLowerLimit = j_ant_pan->LowerLimit(0);
  m_antennaPanUpperLimit = j_ant_pan->UpperLimit(0);

  // Listen to the update event. This event is broadcast every sim iteration.
  // If result goes out of scope updates will stop, so it is assigned to a member variable.
  m_updateConnection = event::Events::ConnectBeforePhysicsUpdate(std::bind(&JointsFaults::onUpdate, this));

  gzlog << "JointsFaultsPlugin - successfully loaded!" << endl;
}

void JointsFaults::onUpdate()
{
  injectFault("ant_tilt_effort_failure", m_antennaTiltFaultActivated, "j_ant_tilt",
            m_antennaTiltLowerLimit, m_antennaTiltUpperLimit);

  injectFault("ant_pan_effort_failure", m_antennaPanFaultActivated, "j_ant_pan",
            m_antennaPanLowerLimit, m_antennaPanUpperLimit);

  // injectFault("ant_tilt_encoder_failure", m_antennaTiltFaultActivated, "j_ant_tilt",
  //           m_antennaTiltLowerLimit, m_antennaTiltUpperLimit);

  // injectFault("ant_pan_encoder_failure", m_antennaPanFaultActivated, "j_ant_pan",
  //           m_antennaPanLowerLimit, m_antennaPanUpperLimit);
}

void JointsFaults::injectFault(const std::string& joint_fault, bool& fault_activated,
                               const std::string& joint_name, double lower_limit, double upper_limit)
{
  bool fault_enabled;
  ros::param::param("/faults/" + joint_fault, fault_enabled, false);
  std::cout << "enables " << fault_enabled << std::endl;

  if (!fault_activated && fault_enabled)
  {
    ROS_INFO_STREAM(joint_fault << " activated!");
    fault_activated = true;
    // lock the joint to current position
    auto j = m_model->GetJoint(joint_name);
    auto p = j->Position(0);
    j->SetLowerLimit(0, p);
    j->SetUpperLimit(0, p);
  }
  else if (fault_activated && !fault_enabled)
  {
    ROS_INFO_STREAM(joint_fault << " de-activated!");
    fault_activated = false;
    // restore the joint limits
    auto j = m_model->GetJoint(joint_name);
    j->SetLowerLimit(0, lower_limit);
    j->SetUpperLimit(0, upper_limit);
  }
}
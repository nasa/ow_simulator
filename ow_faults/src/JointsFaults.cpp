// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ow_faults/JointsFaults.h"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <ros/ros.h>

using namespace std;
using namespace gazebo;
using namespace gazebo::event;

constexpr double JointsFaults::MAX_FRICTION;

GZ_REGISTER_MODEL_PLUGIN(JointsFaults)

JointsFaults::JointsFaults() :
  ModelPlugin()
{
  m_JointsFaultsMap = {
    {"j_shou_yaw", JointFaultInfo("/faults/shou_yaw_effort_failure")},
    {"j_shou_pitch", JointFaultInfo("/faults/shou_pitch_effort_failure")},
    {"j_prox_pitch", JointFaultInfo("/faults/prox_pitch_effort_failure")},
    {"j_dist_pitch", JointFaultInfo("/faults/dist_pitch_effort_failure")},
    {"j_hand_yaw", JointFaultInfo("/faults/hand_yaw_effort_failure")},
    {"j_scoop_yaw", JointFaultInfo("/faults/scoop_yaw_effort_failure")},
    {"j_ant_pan", JointFaultInfo("/faults/ant_pan_effort_failure")},
    {"j_ant_tilt", JointFaultInfo("/faults/ant_tilt_effort_failure")}
  };
}

JointsFaults::~JointsFaults()
{
}

void JointsFaults::Load(physics::ModelPtr model, sdf::ElementPtr /* sdf */)
{
  m_model = model;
  gzlog << "JointsFaultsPlugin - successfully loaded!" << endl;
  m_updateConnection = Events::ConnectBeforePhysicsUpdate(bind(&JointsFaults::onUpdate, this));
}

void JointsFaults::onUpdate()
{
  for (auto& kv : m_JointsFaultsMap)
    injectFault(kv.first, kv.second);
}

void JointsFaults::injectFault(const string& joint_name, JointFaultInfo& jfi)
{
  auto fault_enabled = false;
  auto success = ros::param::getCached(jfi.fault, fault_enabled);

  if (!success)
    return;

  if (!jfi.activated && fault_enabled)
  {
    ROS_INFO_STREAM(jfi.fault << " activated!");
    jfi.activated = true;
    // lock the joint to current position
    auto j = m_model->GetJoint(joint_name);
    jfi.friction = j->GetParam("friction", 0);
    j->SetParam("friction", 0, MAX_FRICTION);
  }
  else if (jfi.activated && !fault_enabled)
  {
    ROS_INFO_STREAM(jfi.fault << " de-activated!");
    jfi.activated = false;
    // restore the joint limits
    auto j = m_model->GetJoint(joint_name);
    j->SetParam("friction", 0, jfi.friction);
  }
}
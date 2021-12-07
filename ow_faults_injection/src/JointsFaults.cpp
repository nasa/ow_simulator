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

constexpr double JointsFaults::MAX_FRICTION;

GZ_REGISTER_MODEL_PLUGIN(JointsFaults)

JointsFaults::JointsFaults() :
  ModelPlugin()
{
  m_node_handle = make_unique<ros::NodeHandle>("Joint_Faults_Flag");
  m_node_handle->setCallbackQueue(&m_callback_queue);

  const char* joint_states_str = "/joint_states";
  m_joint_state_flags_pub = m_node_handle->advertise<ow_faults_detection::JointStatesFlag>(string("/flags") + joint_states_str, 10);

  m_JointsFaultsMap = {
    {"j_shou_yaw", JointFaultInfo("shou_yaw", ow_lander::J_SHOU_YAW)},
    {"j_shou_pitch", JointFaultInfo("shou_pitch", ow_lander::J_SHOU_PITCH)},
    {"j_prox_pitch", JointFaultInfo("prox_pitch", ow_lander::J_PROX_PITCH)},
    {"j_dist_pitch", JointFaultInfo("dist_pitch", ow_lander::J_DIST_PITCH)},
    {"j_hand_yaw", JointFaultInfo("hand_yaw", ow_lander::J_HAND_YAW)},
    {"j_scoop_yaw", JointFaultInfo("scoop_yaw", ow_lander::J_SCOOP_YAW)},
    {"j_ant_pan", JointFaultInfo("ant_pan", ow_lander::J_ANT_PAN)},
    {"j_ant_tilt", JointFaultInfo("ant_tilt", ow_lander::J_ANT_TILT)} };

   m_on_update_connection = event::Events::ConnectPostRender([this]() {
    if (m_node_handle->ok())
      m_callback_queue.callAvailable();
  });

  initFlagMessage();

}

JointsFaults::~JointsFaults()
{
}

void JointsFaults::Load(physics::ModelPtr model, sdf::ElementPtr /* sdf */)
{
  m_model = model;

  // Listen to the update event. This event is broadcast every sim iteration.
  // If result goes out of scope updates will stop, so it is assigned to a member variable.
  m_updateConnection = event::Events::ConnectBeforePhysicsUpdate(std::bind(&JointsFaults::onUpdate, this));

  gzlog << "JointsFaultsPlugin - successfully loaded!" << endl;
}

void JointsFaults::onUpdate()
{
  for (auto& kv : m_JointsFaultsMap)
    injectFault(kv.first, kv.second);

  m_joint_state_flags_pub.publish(m_flag_msg);
}

void JointsFaults::injectFault(const std::string& joint_name, JointFaultInfo& jfi)
{
  bool fault_enabled = false;
  bool success = ros::param::getCached("/faults/" + jfi.fault, fault_enabled);

  if (!success)
    return;

  // Set failed sensor values to 0
  unsigned int index;

  findJointIndex(jfi.landerJoint, index);

  if (!jfi.activated && fault_enabled) {
      ROS_INFO_STREAM(joint_name << " joint locked!");
      jfi.activated = true;
      m_flag_msg.flags[index] = SET_FLAG;
      // lock the joint to current position
      auto j = m_model->GetJoint(joint_name);
      jfi.friction = j->GetParam("friction", 0);
      j->SetParam("friction", 0, MAX_FRICTION);
  }
  else if (jfi.activated && !fault_enabled) {
      ROS_INFO_STREAM(joint_name << " joint unlocked!");
      jfi.activated = false;
      m_flag_msg.flags[index] = false;
      // restore the joint limits
      auto j = m_model->GetJoint(joint_name);
      j->SetParam("friction", 0, jfi.friction);
  }
}

void JointsFaults::initFlagMessage()
{
  m_flag_msg.name = {"j_ant_pan", "j_ant_tilt", "j_dist_pitch", "j_grinder", "j_hand_yaw", "j_prox_pitch", "j_scoop_yaw",
  "j_shou_pitch", "j_shou_yaw"};

  m_flag_msg.flags = {false, false, false, false, false, false, false, false, false};

  // Populate the map once here.
  // This assumes the collection of joints will never change.
  if (m_joint_state_indices.empty()) {
    for (int j = 0; j < ow_lander::NUM_JOINTS; j ++) {
      int index = findPositionInGroup(m_flag_msg.name, ow_lander::joint_names[j]);
      if (index >= 0)
        m_joint_state_indices.push_back(index);
    }
  }
}

int JointsFaults::findPositionInGroup(const std::vector<string>& group, const string& item)
{
  auto iter = std::find(group.begin(), group.end(), item);
  return (iter == group.end()) ? -1 :  iter - group.begin();
}

bool JointsFaults::findJointIndex(unsigned int joint, unsigned int& out_index)
{
  if(joint >= m_joint_state_indices.size())
    return false;

  out_index = m_joint_state_indices[joint];
  return true;
}

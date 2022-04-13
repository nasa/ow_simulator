#include <DigStateMachine.h>

#include <RegolithSpawner.h>

using namespace ow_regolith;

using tf::Quaternion, tf::Vector3, tf::tfDot, tf::quatRotate;

const Vector3 DigStateMachine::SCOOP_DOWNWARD = Vector3(0.0, 0.0, 1.0);
const Vector3 DigStateMachine::WORLD_DOWNWARD = Vector3(0.0, 0.0, -1.0);

DigStateMachine::DigStateMachine(ros::NodeHandle *node_handle,
                                 RegolithSpawner *spawner)
  : m_node_handle(node_handle),
    m_spawner(spawner),
    m_not_digging(new NotDiggingState(this)),
    m_sinking(new SinkingState(this)),
    m_plowing(new PlowingState(this)),
    m_retracting(new RetractingState(this))
{
  setState(m_not_digging.get());
}

void DigStateMachine::setState(DigState *s) {
  if (m_current == s)
    return; // do nothing if new state is already current
  if (m_current) {
    m_current->exit();
    // ROS_INFO("Exitting state %s", m_current->getName().c_str());
  }
  m_current = s;
  m_current->enter();
  // ROS_INFO("Entering state %s", m_current->getName().c_str());
}

void DigStateMachine::handleTerrainModified()
{
  m_current->terrainModified();
}

void DigStateMachine::handleScoopPoseUpdate(Quaternion new_orientation)
{
  // compute downward projection of scoop for states to access
  Vector3 scoop_bottom(quatRotate(new_orientation, SCOOP_DOWNWARD));
  m_downward_projection = tfDot(scoop_bottom, WORLD_DOWNWARD);

  m_current->scoopPoseUpdate();
}

void DigStateMachine::NotDiggingState::terrainModified()
{
  if (m_context->m_downward_projection > m_context->THRESHOLD_ENTER)
    m_context->setState(m_context->m_sinking.get());
}

void DigStateMachine::SinkingState::scoopPoseUpdate()
{
  if (m_context->m_downward_projection > m_context->THRESHOLD_TROUGH)
    m_context->setState(m_context->m_plowing.get());
}

void DigStateMachine::PlowingState::scoopPoseUpdate()
{
  if (m_context->m_downward_projection < m_context->THRESHOLD_TROUGH)
    m_context->setState(m_context->m_retracting.get());
}

void DigStateMachine::RetractingState::scoopPoseUpdate()
{
  if (m_context->m_downward_projection < m_context->THRESHOLD_EXIT)
    m_context->setState(m_context->m_not_digging.get());
}

void DigStateMachine::RetractingState::enter() {
  m_timeout.start();
  // psuedo forces no longer needed to keep particles in scoop in this dig phase
  m_context->m_spawner->clearAllPsuedoForces();
}

void DigStateMachine::RetractingState::exit() {
  m_timeout.stop();
  // notify spawner dig activity is complete, so it can reset volume
  m_context->m_spawner->resetTrackedVolume();
}

void DigStateMachine::RetractingState::onTimeout(const ros::TimerEvent&) {
  m_context->setState(m_context->m_not_digging.get());
}

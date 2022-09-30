#include <DigStateMachine.h>

#include <RegolithSpawner.h>

using namespace ow_regolith;

using std::shared_ptr, std::make_unique;

using tf::Quaternion, tf::Vector3, tf::tfDot, tf::quatRotate;

using ros::Duration;

                                                                // degrees
const tfScalar DigStateMachine::THRESHOLD_SINK    = tfCos(tfRadians(90.0));
const tfScalar DigStateMachine::THRESHOLD_PLOW    = tfCos(tfRadians(10.0));
const tfScalar DigStateMachine::THRESHOLD_RETRACT = tfCos(tfRadians(80.0));

                                                      // seconds
const Duration DigStateMachine::TIMEOUT_SINK    = Duration(12.0);
const Duration DigStateMachine::TIMEOUT_PLOW    = Duration(10.0);
const Duration DigStateMachine::TIMEOUT_RETRACT = Duration(5.0);

const Vector3 DigStateMachine::SCOOP_DOWNWARD = Vector3(0.0, 0.0, 1.0);
const Vector3 DigStateMachine::WORLD_DOWNWARD = Vector3(0.0, 0.0, -1.0);

DigStateMachine::DigStateMachine(shared_ptr<ros::NodeHandle> node_handle,
                                 RegolithSpawner *const spawner)
  : m_node_handle(node_handle),
    m_spawner(spawner),
    m_current(nullptr),
    m_not_digging(make_unique<NotDiggingState>(this)),
    m_sinking(make_unique<SinkingState>(this)),
    m_plowing(make_unique<PlowingState>(this)),
    m_retracting(make_unique<RetractingState>(this))
{
  setState(m_not_digging.get());
}

void DigStateMachine::setState(DigState *const s) {
  if (m_current == s)
    return; // do nothing if new state is already current
  if (m_current) {
    m_current->exit();
    ROS_DEBUG("Exitting state %s", m_current->getName().c_str());
  }
  m_current = s;
  m_current->enter();
  ROS_DEBUG("Entering state %s", m_current->getName().c_str());
}

void DigStateMachine::handleTerrainModified()
{
  m_current->terrainModified();
}

void DigStateMachine::handleScoopPoseUpdate(const Quaternion &new_orientation)
{
  // compute downward projection of scoop for states to access
  const Vector3 scoop_bottom(quatRotate(new_orientation, SCOOP_DOWNWARD));
  m_downward_projection = tfDot(scoop_bottom, WORLD_DOWNWARD);
  m_current->scoopPoseUpdate();
}

void DigStateMachine::NotDiggingState::terrainModified()
{
  // check if scoop is sinking into the terrain tip first
  if (m_context->m_downward_projection > m_context->THRESHOLD_SINK &&
      m_context->m_downward_projection < m_context->THRESHOLD_PLOW)
    m_context->setState(m_context->m_sinking.get());
}

void DigStateMachine::NotDiggingState::enter()
{
  // dig activity is complete call
  m_context->m_spawner->resetTrackedVolume();
  // for a nominal dig clearAllPsuedoForces will be redudant, but calling it
  // here ensures forces will not persist in the case of off-nominal digs
  m_context->m_spawner->clearAllPsuedoForces();
}

void DigStateMachine::SinkingState::scoopPoseUpdate()
{
  if (m_context->m_downward_projection > m_context->THRESHOLD_PLOW)
    m_context->setState(m_context->m_plowing.get());
}

void DigStateMachine::SinkingState::onTimeout(const ros::TimerEvent&)
{
  m_context->setState(m_context->m_not_digging.get());
}

void DigStateMachine::PlowingState::scoopPoseUpdate()
{
  if (m_context->m_downward_projection < m_context->THRESHOLD_PLOW)
    m_context->setState(m_context->m_retracting.get());
}

void DigStateMachine::PlowingState::onTimeout(const ros::TimerEvent&)
{
  m_context->setState(m_context->m_not_digging.get());
}

// RETRACTING STATE
void DigStateMachine::RetractingState::scoopPoseUpdate()
{
  if (m_context->m_downward_projection < m_context->THRESHOLD_RETRACT)
    m_context->setState(m_context->m_not_digging.get());
}

void DigStateMachine::RetractingState::enter()
{
  DigState::enter();
  // psuedo forces no longer needed to keep particles in scoop in this dig phase
  m_context->m_spawner->clearAllPsuedoForces();
}

void DigStateMachine::RetractingState::onTimeout(const ros::TimerEvent&)
{
  m_context->setState(m_context->m_not_digging.get());
}

// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <DigStateMachine.h>

using namespace ow_dynamic_terrain;

using std::make_unique;

using tf::Pose, tf::Vector3, tf::tfDot, tf::quatRotate;

using ros::Duration;

                                                    // degrees
const tfScalar DigStateMachine::THRESHOLD_SINK    = tfCos(tfRadians(90.0));
const tfScalar DigStateMachine::THRESHOLD_PLOW    = tfCos(tfRadians(10.0));
const tfScalar DigStateMachine::THRESHOLD_RETRACT = tfCos(tfRadians(35.0));

const tfScalar DigStateMachine::THRESHOLD_FORWARD_MOVE = tfCos(tfRadians(5.0));
const tfScalar DigStateMachine::THRESHOLD_SPEED        = 1e-4; // meters/sec

                                                  // seconds
const Duration DigStateMachine::TIMEOUT_SINK    = Duration(12.0);
const Duration DigStateMachine::TIMEOUT_PLOW    = Duration(10.0);
const Duration DigStateMachine::TIMEOUT_RETRACT = Duration(5.0);

const Vector3 DigStateMachine::SCOOP_DOWNWARD = Vector3(0.0,  0.0,  1.0);
const Vector3 DigStateMachine::SCOOP_FORWARD  = Vector3(1.0,  0.0,  0.0);
const Vector3 DigStateMachine::WORLD_DOWNWARD = Vector3(0.0,  0.0, -1.0);

DigStateMachine::DigStateMachine(ros::NodeHandle *node_handle,
                                 StateTransitionCallback cb)
  : m_node_handle(node_handle),
    m_transition_cb(cb),
    m_current(nullptr),
    m_not_digging(make_unique<NotDiggingState>(this)),
    m_sinking(make_unique<SinkingState>(this)),
    m_plowing(make_unique<PlowingState>(this)),
    m_retracting(make_unique<RetractingState>(this))
{
  setState(m_not_digging.get());
}

void DigStateMachine::setState(DigState *const s) {
  if (m_current == s) {
    return; // do nothing if new state is already current
  }
  if (m_current) {
    m_current->exit();
  }
  m_current = s;
  m_current->enter();
  m_transition_cb(m_current->m_id);
}

void DigStateMachine::handleScoopPoseUpdate(const Pose &new_pose)
{
  double dt = (ros::Time::now() - m_prev_position_update_time).toSec();
  // check whether scoop state variables have been updated at least once
  // dt is sometimes zero because link_states updates at the physics rate
  if (!m_prev_position_update_time.isZero() && dt > 0.0) {
    // transform scoop bottom normal to world coordinates
    Vector3 scoop_bottom(quatRotate(new_pose.getRotation(), SCOOP_DOWNWARD));
    // project scoop bottom's normal onto the z-axis to use as a measure
    // of how level the scoop is
    m_scoop_state.bottom_projection = tfDot(scoop_bottom, WORLD_DOWNWARD);
    m_scoop_state.forward = quatRotate(new_pose.getRotation(), SCOOP_FORWARD);
    // compute velocity and use to determine whether scoop is moving forward
    Vector3 velocity = (new_pose.getOrigin() - m_prev_scoop_position) / dt;
    tfScalar forward_move_projection = tfDot(m_scoop_state.forward,
                                             velocity.normalize());
    m_scoop_state.moving_forward = velocity.length() > THRESHOLD_SPEED &&
                             forward_move_projection > THRESHOLD_FORWARD_MOVE;
    // notify current state that a new scoop information is available
    m_current->scoopStateUpdated();
  }
  m_prev_scoop_position = new_pose.getOrigin();
  m_prev_position_update_time = ros::Time::now();
}

// NOT DIGGING STATE
void DigStateMachine::NotDiggingState::scoopStateUpdated()
{
  if (m_context->m_scoop_state.bottom_projection > m_context->THRESHOLD_SINK
      && m_context->m_scoop_state.moving_forward
      && m_context->m_scoop_state.forward.z() < 0.0) {
    m_context->setState(m_context->m_sinking.get());
  }
}

// SINKING STATE
void DigStateMachine::SinkingState::scoopStateUpdated()
{
  if (m_context->m_scoop_state.bottom_projection > m_context->THRESHOLD_PLOW
      && m_context->m_scoop_state.moving_forward) {
    m_context->setState(m_context->m_plowing.get());
  }
}
void DigStateMachine::SinkingState::onTimeout(const ros::TimerEvent&)
{
  m_context->setState(m_context->m_not_digging.get());
}

// PLOWING STATE
void DigStateMachine::PlowingState::scoopStateUpdated()
{
  if (m_context->m_scoop_state.bottom_projection < m_context->THRESHOLD_PLOW
      && m_context->m_scoop_state.moving_forward) {
    m_context->setState(m_context->m_retracting.get());
  }
}
void DigStateMachine::PlowingState::onTimeout(const ros::TimerEvent&)
{
  m_context->setState(m_context->m_not_digging.get());
}

// RETRACTING STATE
void DigStateMachine::RetractingState::scoopStateUpdated()
{
  if (m_context->m_scoop_state.bottom_projection < m_context->THRESHOLD_RETRACT
      && !m_context->m_scoop_state.moving_forward) {
    m_context->setState(m_context->m_not_digging.get());
  }
}
void DigStateMachine::RetractingState::onTimeout(const ros::TimerEvent&)
{
  m_context->setState(m_context->m_not_digging.get());
}

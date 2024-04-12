// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef DIG_STATE_MACHINE_H
#define DIG_STATE_MACHINE_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <string>
#include <memory>
#include <optional>
#include <functional>

namespace ow_dynamic_terrain {

enum DigStateId {
  NOT_DIGGING,
  SINKING,
  PLOWING,
  RETRACTING
};

using StateTransitionCallback = std::function<void(DigStateId)>;

class DigStateMachine
{

// Defines a finite state machine, associated states, and allowed transitions
// for the purpose of identifying dig operations performed by the lander scoop.

public:
  DigStateMachine(ros::NodeHandle *node_handle, StateTransitionCallback cb);
  ~DigStateMachine() = default;

  DigStateMachine() = delete;
  DigStateMachine(const DigStateMachine&) = delete;
  DigStateMachine& operator=(const DigStateMachine&) = delete;

  // external events propogate to states via these functions
  void handleScoopPoseUpdate(const tf::Pose &new_pose);

private:

  // external interface pointers
  ros::NodeHandle *m_node_handle;

  // call whenever a state transition occurs
  StateTransitionCallback m_transition_cb;

  // historic state variables to compute scoop velocity
  tf::Vector3 m_prev_scoop_position;
  ros::Time m_prev_position_update_time;

  // collections of variables DigStates reference for state transition logic
  struct {
    // Dot product between scoop bottom's normal and the world's downward
    // unit vector
    tfScalar bottom_projection;
    // The scoop's forward facing normal in world coordinates
    tf::Vector3 forward;
    // True if scoop's velocity is nearly in the same direction as forward
    bool moving_forward;
  } m_scoop_state;

  // A collection of dot product thresholds that define transition points along
  // the downward projection curve where start transitions occur
  static const tfScalar THRESHOLD_SINK;
  static const tfScalar THRESHOLD_PLOW;
  static const tfScalar THRESHOLD_RETRACT;

  // Maximum allowed dot product between normalized movement and the scoop
  // forward vector that can be considered a forward movement
  static const tfScalar THRESHOLD_FORWARD_MOVE;
  // scoop speed threshold; below is consider stationary and above is moving
  static const tfScalar THRESHOLD_SPEED;

  // maximum time a state can be inhabited before it transitions to NOT_DIGGING
  static const ros::Duration TIMEOUT_SINK;
  static const ros::Duration TIMEOUT_PLOW;
  static const ros::Duration TIMEOUT_RETRACT;

  // unit vectors describing scoop and world orientation
  static const tf::Vector3 SCOOP_DOWNWARD;
  static const tf::Vector3 SCOOP_FORWARD;
  static const tf::Vector3 WORLD_DOWNWARD;

  // Defines an abstract base class for DigStates. Supports (state) enter and
  // exit methods and two events methods relevant to digging operations.
  class DigState
  {
  public:
    DigState(DigStateId id, DigStateMachine *const c)
      : m_id(id), m_context(c),
        m_timeout(std::nullopt) // timeout will not be used
    { };
    DigState(DigStateId id, DigStateMachine *const c,
             const ros::Duration &interval)
      : m_id(id), m_context(c),
        m_timeout(
          c->m_node_handle->createTimer(
            interval, &DigState::onTimeout, this, true, false
          )
        )
    { };
    virtual ~DigState() = default;

    DigState() = delete;
    DigState(const DigState&) = delete;
    DigState& operator=(const DigState&) = delete;

    const DigStateId m_id;

    // external events derived DigStates will handle or ignore
    virtual void scoopStateUpdated() { };

    // called when a start becomes active
    virtual void enter() {
      if (m_timeout) m_timeout->start();
    };
    // called when a start become inactive
    virtual void exit() {
      if (m_timeout) m_timeout->stop();
    };

    virtual void onTimeout(const ros::TimerEvent&) { };

  protected:
    DigStateMachine *const m_context;

  private:
    std::optional<ros::Timer> m_timeout;
  };

  // The NotDigging state indicates digging is not happening. Transitions to the
  // Sinking state when a terrain modification occurs and the scoop is in a
  // proper orientation to initiate a dig.
  class NotDiggingState : public DigState
  {
  public:
    NotDiggingState(DigStateMachine *c)
      : DigState(DigStateId::NOT_DIGGING, c) { };
    void scoopStateUpdated() override;
  };

  // The Sinking state indicates the scoop is digging downward into terrain.
  // Transitions to the Plowing state when the scoop pitches upward to be
  // level with the terrain, or transitions to NotDigging when it times out.
  class SinkingState : public DigState
  {
  public:
    SinkingState(DigStateMachine *c)
      : DigState(DigStateId::SINKING, c, TIMEOUT_SINK) { };
    void scoopStateUpdated() override;
    void onTimeout(const ros::TimerEvent&) override;
  };

  // The Plowing state indicates the scoop is plowing the terrain along the
  // the horizontal. Transitions to the Retracting state when the scoop tip
  // pitches upward, or transitions into NotDigging when it times out.
  class PlowingState : public DigState
  {
  public:
    PlowingState(DigStateMachine *c)
      : DigState(DigStateId::PLOWING, c, TIMEOUT_PLOW) { };
    void scoopStateUpdated() override;
    void onTimeout(const ros::TimerEvent&) override;
  };

  // The Retracting state indicates the scoop is pitching upward and exiting
  // the terrain. Transitions to the NotDigging state when the scoop rotates
  // into a non-digging orientation, or when it times out.
  class RetractingState : public DigState
  {
  public:
    RetractingState(DigStateMachine *c)
      : DigState(DigStateId::RETRACTING, c, TIMEOUT_RETRACT) { };
    void scoopStateUpdated() override;
    void onTimeout(const ros::TimerEvent&) override;
  };

  // instantiations of all states supported by the state machine
  const std::unique_ptr<NotDiggingState> m_not_digging;
  const std::unique_ptr<SinkingState> m_sinking;
  const std::unique_ptr<PlowingState> m_plowing;
  const std::unique_ptr<RetractingState> m_retracting;

  // currently active DigState
  DigState *m_current;

  void setState(DigState *const s);

}; // class DigStateMachine

} // namespace ow_regolith

#endif // DIG_STATE_MACHINE_H

// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef DIG_FSM_H
#define DIG_FSM_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <string>
#include <memory>

namespace ow_regolith {

class RegolithSpawner;

// Defines a finite state machine and associated states that estimate the phase
// of a digging operation performed with the lander scoop.
class DigStateMachine
{
public:
  DigStateMachine(std::shared_ptr<ros::NodeHandle> node_handle,
                  RegolithSpawner *const spawner);
  ~DigStateMachine() = default;

  DigStateMachine() = delete;
  DigStateMachine(const DigStateMachine&) = delete;
  DigStateMachine& operator=(const DigStateMachine&) = delete;

  // external events propogate to states via these functions
  void handleTerrainModified();
  void handleScoopPoseUpdate(const tf::Quaternion &new_orientation);

  bool isDigging() const {
    return m_current != m_not_digging.get();
  };

  bool isRetracting() const {
    return m_current == m_retracting.get();
  }

private:
  // external interface pointers
  std::shared_ptr<ros::NodeHandle> m_node_handle;
  RegolithSpawner *const m_spawner;

  // stores the dot product between the world's downward unit vector and the
  // scoop's downward unit vector
  tfScalar m_downward_projection;

  // points along the downward projection curve where state changes occur
  static const tfScalar THRESHOLD_SINK;
  static const tfScalar THRESHOLD_PLOW;
  static const tfScalar THRESHOLD_RETRACT;

  // maximum time a state can exist before it transitions to NOT_DIGGING
  static const ros::Duration TIMEOUT_SINK;
  static const ros::Duration TIMEOUT_PLOW;
  static const ros::Duration TIMEOUT_RETRACT;

  // unit vectors describing scoop and world orientation
  static const tf::Vector3 SCOOP_DOWNWARD;
  static const tf::Vector3 WORLD_DOWNWARD;

  // Defines an abstract base class for DigStates. Supports (state) enter and exit
  // methods and two events methods relevant to digging operations.
  class DigState
  {
  public:
    DigState(const std::string &name, DigStateMachine *const c)
      : m_name(name), m_context(c) { };
    virtual ~DigState() = default;

    DigState() = delete;
    DigState(const DigState&) = delete;
    DigState& operator=(const DigState&) = delete;

    const std::string& getName() const {return m_name;}

    // external events DigStates will handle or ignore
    virtual void terrainModified() { };
    virtual void scoopPoseUpdate() { };

    virtual void enter() { };
    virtual void exit() { };

  protected:
    DigStateMachine *const m_context;
  private:
    const std::string m_name;
  };

  // Defines a supplemental class for anything that inherits from DigState.
  // Inheriting Timeout will enable the DigState to define a timeout and what
  // happens when the timeout triggers.
  class Timeout
  {
  public:
    Timeout(const ros::Duration &interval, ros::NodeHandle *const nh) {
      // create a oneshot timer that does not autostart
      m_timer = nh->createTimer(interval, &Timeout::onTimeout, this, true, false);
    };
    virtual ~Timeout() = default;

    Timeout() = delete;
    Timeout(const Timeout&) = delete;
    Timeout& operator=(const Timeout&) = delete;

  protected:
    virtual void onTimeout(const ros::TimerEvent&) = 0;

    void startTimer() {m_timer.start();};
    void stopTimer() {m_timer.stop();};
  private:
    ros::Timer m_timer;
  };

  // The NotDigging state indicates digging is not happening. Transitions to the
  // Sinking state when a terrain modification occurs and the scoop is in a
  // proper orientation to initiate a dig.
  class NotDiggingState : public DigState
  {
  public:
    NotDiggingState(DigStateMachine *c) : DigState("NotDigging", c) { };
    void terrainModified() override;
    void enter() override;
  };

  // The Sinking state indicates the scoop is digging downward into terrain.
  // Transitions to the Plowing state when the scoop pitches upward to be
  // level with the terrain, or transitions to NotDigging when it times out.
  class SinkingState : public DigState, Timeout
  {
  public:
    SinkingState(DigStateMachine *c)
      : DigState("Sinking", c),
        Timeout(TIMEOUT_SINK, c->m_node_handle.get()) { };
    void scoopPoseUpdate() override;
    void enter() override;
    void exit() override;
    void onTimeout(const ros::TimerEvent&) override;
  };

  // The Plowing state indicates the scoop is plowing the terrian along the
  // the horizontal. Transitions to the Retracting state when the scoop tip
  // pitches upward, or transitions into NotDigging when it times out.
  class PlowingState : public DigState, Timeout
  {
  public:
    PlowingState(DigStateMachine *c)
      : DigState("Plowing", c),
        Timeout(TIMEOUT_PLOW, c->m_node_handle.get()) { };
    void scoopPoseUpdate() override;
    void enter() override;
    void exit() override;
    void onTimeout(const ros::TimerEvent&) override;
  };

  // The Retracting state indicates the scoop is pitching upward and exitting
  // the terrain. Transitions to the NotDigging state when the scoop rotates
  // into a non-digging orientation, or when it times out.
  class RetractingState : public DigState, public Timeout
  {
  public:
    RetractingState(DigStateMachine *c)
      : DigState("Retracting", c),
        Timeout(TIMEOUT_RETRACT, c->m_node_handle.get()) { };
    void scoopPoseUpdate() override;
    void enter() override;
    void exit() override;
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

#endif // DIG_FMS_H

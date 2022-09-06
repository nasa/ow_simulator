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

  // Defines an abstract base class for DigStates. Supports (state) enter and
  // exit methods and two events methods relevant to digging operations.
  class DigState
  {
  public:
    DigState(const std::string &name, DigStateMachine *const c)
      : m_name(name), m_context(c),
        m_timeout(std::nullopt) // timeout will not be used
    { };
    DigState(const std::string &name, DigStateMachine *const c,
             const ros::Duration &interval)
      : m_name(name), m_context(c),
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

    const std::string& getName() const {return m_name;}

    // external events derived DigStates will handle or ignore
    virtual void terrainModified() { };
    virtual void scoopPoseUpdate() { };

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
    const std::string m_name;
    std::optional<ros::Timer> m_timeout;
  };

  // The NotDigging state indicates digging is not happening. Transitions to the
  // Sinking state when a terrain modification occurs and the scoop is in a
  // proper orientation to initiate a dig.
  class NotDiggingState : public DigState
  {
  public:
    NotDiggingState(DigStateMachine *c)
      : DigState("NotDigging", c) { };
    void terrainModified() override;
    void enter() override;
  };

  // The Sinking state indicates the scoop is digging downward into terrain.
  // Transitions to the Plowing state when the scoop pitches upward to be
  // level with the terrain, or transitions to NotDigging when it times out.
  class SinkingState : public DigState
  {
  public:
    SinkingState(DigStateMachine *c)
      : DigState("Sinking", c, TIMEOUT_SINK) { };
    void scoopPoseUpdate() override;
    void onTimeout(const ros::TimerEvent&) override;
  };

  // The Plowing state indicates the scoop is plowing the terrian along the
  // the horizontal. Transitions to the Retracting state when the scoop tip
  // pitches upward, or transitions into NotDigging when it times out.
  class PlowingState : public DigState
  {
  public:
    PlowingState(DigStateMachine *c)
      : DigState("Plowing", c, TIMEOUT_PLOW) { };
    void scoopPoseUpdate() override;
    void onTimeout(const ros::TimerEvent&) override;
  };

  // The Retracting state indicates the scoop is pitching upward and exitting
  // the terrain. Transitions to the NotDigging state when the scoop rotates
  // into a non-digging orientation, or when it times out.
  class RetractingState : public DigState
  {
  public:
    RetractingState(DigStateMachine *c)
      : DigState("Retracting", c, TIMEOUT_RETRACT) { };
    void scoopPoseUpdate() override;
    void enter() override;
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

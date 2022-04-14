// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Defines a finite state machine and associated states that estimate the phase
// of a digging operation performed with the lander scoop.

#ifndef DIG_FSM_H
#define DIG_FSM_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <string>
#include <memory>

namespace ow_regolith {

class DigStateMachine;

class DigState
{
public:
  DigState(std::string name, DigStateMachine *c)
    : m_name(name), m_context(c) { };
  virtual ~DigState() = default;

  DigState() = delete;
  DigState(const DigState&) = delete;
  DigState& operator=(const DigState&) = delete;

  inline std::string getName() const {return m_name;}

  // external events DigStates will handle or ignore
  virtual void terrainModified() { };
  virtual void scoopPoseUpdate() { };

  virtual void enter() { };
  virtual void exit() { };

protected:
  DigStateMachine *m_context;
private:
  std::string m_name;
};

class RegolithSpawner;

class DigStateMachine
{
public:
  DigStateMachine(std::shared_ptr<ros::NodeHandle> node_handle,
                  RegolithSpawner *spawner);
  ~DigStateMachine() = default;

  DigStateMachine() = delete;
  DigStateMachine(const DigStateMachine&) = delete;
  DigStateMachine& operator=(const DigStateMachine&) = delete;

  // external events propogate to states via these functions
  void handleTerrainModified();
  void handleScoopPoseUpdate(tf::Quaternion new_orientation);

  inline bool isDigging() const {
    return m_current != m_not_digging.get();
  };

  inline bool isRetracting() const {
    return m_current == m_retracting.get();
  }

private:
  void setState(DigState *s);

  // currently active DigState
  DigState *m_current;

  // external interface pointers
  std::shared_ptr<ros::NodeHandle> m_node_handle;
  RegolithSpawner *m_spawner;

  // stores the dot product between the world's downward unit vector and the
  // scoop's downward unit vector
  float m_downward_projection;

  // points along the downward projection curve where state changes occur
  static constexpr auto THRESHOLD_SINK    = 0.0f;
  static constexpr auto THRESHOLD_PLOW    = 0.9f;
  static constexpr auto THRESHOLD_RETRACT = 0.2f;

  // unit vectors describing scoop and world orientation
  static const tf::Vector3 SCOOP_DOWNWARD;
  static const tf::Vector3 WORLD_DOWNWARD;

  // each DigState subclass marks a unique phase in the digging process
  // as nested classes each is given friend access to DigStateMachine
  class NotDiggingState : public DigState
  {
  public:
    NotDiggingState(DigStateMachine *c) : DigState("NotDigging", c) { };
    void terrainModified() override;
  };

  class SinkingState : public DigState
  {
  public:
    SinkingState(DigStateMachine *c) : DigState("Sinking", c) { };
    void scoopPoseUpdate() override;
  };

  class PlowingState : public DigState
  {
  public:
    PlowingState(DigStateMachine *c) : DigState("Plowing", c) { };
    void scoopPoseUpdate() override;
  };

  class RetractingState : public DigState
  {
  public:
    RetractingState(DigStateMachine *c) : DigState("Retracting", c) {
      constexpr auto STATE_TIMEOUT = 5.0f;
      m_timeout = m_context->m_node_handle->createTimer(
        ros::Duration(STATE_TIMEOUT),
        &RetractingState::onTimeout,
        this,
        true, // oneshot
        false // autostart
      );
    }
    void scoopPoseUpdate() override;
    void enter() override;
    void exit() override;
    void onTimeout(const ros::TimerEvent&);
  private:
    ros::Timer m_timeout;
  };

  std::unique_ptr<NotDiggingState> m_not_digging;
  std::unique_ptr<SinkingState> m_sinking;
  std::unique_ptr<PlowingState> m_plowing;
  std::unique_ptr<RetractingState> m_retracting;
};

} // namespaced ow_regolith

#endif // DIG_FMS_H

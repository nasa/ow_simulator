// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef REGOLITH_SPAWNER_H
#define REGOLITH_SPAWNER_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <vector>

#include <ModelPool.h>
#include <DigStateMachine.h>

#include <gazebo_msgs/LinkStates.h>

#include <ow_dynamic_terrain/modified_terrain_diff.h>

#include <ow_regolith/SpawnRegolith.h>
#include <ow_regolith/RemoveRegolith.h>
#include <ow_regolith/Contacts.h>

namespace ow_regolith {

// RegolithSpawner is a ROS node that detects when digging by the scoop
// end-effector occurs and spawns a model in the scoop to represent collected
// material. The node will also remove models it has spawned as they collide
// with the terrain or in response to a service call.
class RegolithSpawner
{
public:
  RegolithSpawner(const std::string &node_name);
  ~RegolithSpawner() = default;

  RegolithSpawner()  = delete;
  RegolithSpawner(const RegolithSpawner&) = delete;
  RegolithSpawner& operator=(const RegolithSpawner&) = delete;

  // loads regolith SDF and computes its mass
  // NOTE: this must be called before any other functions
  bool initialize();

  // reset tracked volume to 0
  void resetTrackedVolume();

  // clear any psuedo forces acting on regolith
  void clearAllPsuedoForces();

  // service callback for spawnRegolithInScoop
  bool spawnRegolithSrv(ow_regolith::SpawnRegolithRequest &request,
                        ow_regolith::SpawnRegolithResponse &response);

  // service callback for removeAllRegolithModels
  bool removeRegolithSrv(ow_regolith::RemoveRegolithRequest &request,
                         ow_regolith::RemoveRegolithResponse &response);

  // saves the orientation of scoop to a member variable
  void onLinkStatesMsg(const gazebo_msgs::LinkStates::ConstPtr &msg);

  void onTerrainContact(const ow_regolith::Contacts::ConstPtr &msg);

  // computes the volume displaced from a modified terrain diff image and
  // and spawns reoglith if it surpasses the spawn threshold
  void onModDiffVisualMsg(const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg);

private:
  // ROS interfaces
  std::shared_ptr<ros::NodeHandle> m_node_handle;
  ros::ServiceServer m_srv_spawn_regolith;
  ros::ServiceServer m_srv_remove_all_regolith;
  ros::Subscriber m_sub_link_states;
  ros::Subscriber m_sub_terrain_contact;
  ros::Subscriber m_sub_mod_diff_visual;

  // spawns, removes, and applies forces to spawned models
  std::unique_ptr<ModelPool> m_model_pool;
  // estimates the state of a dig from scoop orientation and terrain mods
  std::unique_ptr<DigStateMachine> m_dig_state;

  // magnitude of force that keeps regolith in the scoop while digging
  float m_psuedo_force_mag;
  // regolith will spawn once this amount of volume is displaced
  double m_spawn_threshold;
  // sum of volume displaced since previous reoglith spawning
  double m_volume_displaced;
  // list of spawn positions relative to scoop link
  std::vector<tf::Vector3> m_spawn_offsets;
  std::vector<tf::Vector3>::const_iterator m_spawn_offset_selector;
  // orientation of scoop in Gazebo world
  tf::Quaternion m_scoop_orientation;
};

} // namespace ow_regolith

#endif // REGOLITH_SPAWNER_H

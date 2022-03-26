// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// RegolithSpawner is a ROS node that detects when digging by the scoop
// end-effector occurs and spawns a model in the scoop to represent collected
// material. The node will also clean-up models it has spawned when a
// deliver/discard action occurs.

#ifndef REGOLITH_SPAWNER_H
#define REGOLITH_SPAWNER_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <ServiceClientFacade.h>
#include <ModelPool.h>

#include <gazebo_msgs/LinkStates.h>

#include <ow_dynamic_terrain/modified_terrain_diff.h>

#include <ow_lander/DigLinearActionResult.h>
#include <ow_lander/DigCircularActionResult.h>

#include <ow_regolith/SpawnRegolith.h>
#include <ow_regolith/RemoveRegolith.h>
#include <ow_regolith/Contacts.h>

namespace ow_regolith {

class RegolithSpawner
{
public:
  RegolithSpawner()  = delete;
  ~RegolithSpawner() = default;
  RegolithSpawner(const RegolithSpawner&) = delete;
  RegolithSpawner& operator=(const RegolithSpawner&) = delete;

  RegolithSpawner(const std::string &node_name);

  // loads regolith SDF and computes its mass
  // NOTE: this must be called before any other functions
  bool initialize();

  // clear all forces acting on particles and reset tracked volume
  void reset();

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

  // both call clearAllPsuedoForces at the end of a dig and reset the tracked
  // volume to zero
  void onDigLinearResultMsg(const ow_lander::DigLinearActionResult::ConstPtr &msg);
  void onDigCircularResultMsg(const ow_lander::DigCircularActionResult::ConstPtr &msg);

private:
  // ROS interfaces
  std::shared_ptr<ros::NodeHandle> m_node_handle;

  ros::ServiceServer m_srv_spawn_regolith;
  ros::ServiceServer m_srv_remove_all_regolith;

  ros::Subscriber m_sub_link_states;
  ros::Subscriber m_sub_terrain_contact;

  ros::Subscriber m_sub_mod_diff_visual;
  ros::Subscriber m_sub_dig_linear_result;
  ros::Subscriber m_sub_dig_circular_result;

  // model pool
  std::unique_ptr<ModelPool> m_pool;

  // sum of volume displaced since previous reoglith spawning
  double m_volume_displaced;
  tf::Point m_volume_center;
  // orientation of scoop in Gazebo
  tf::Quaternion m_scoop_orientation;

  // magnitude of force that keeps regolith in the scoop while digging
  float m_psuedo_force_mag;

  // regolith will spawn once this amount of volume is displaced
  double m_spawn_threshold;
};

} // namespace ow_regolith

#endif // REGOLITH_SPAWNER_H

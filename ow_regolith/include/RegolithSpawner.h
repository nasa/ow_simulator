// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef REGOLITH_SPAWNER_H
#define REGOLITH_SPAWNER_H

#include <ros/ros.h>
#include <tf/tf.h>

#include "gazebo_msgs/LinkStates.h"
#include "ow_dynamic_terrain/modified_terrain_diff.h"
#include "ow_lander/DigLinearActionResult.h"
#include "ow_lander/DigCircularActionResult.h"
#include "ow_lander/DeliverActionResult.h"

class RegolithSpawner
{
public:
  RegolithSpawner()  = delete;
  ~RegolithSpawner() = default;
  RegolithSpawner(const RegolithSpawner&) = delete;
  RegolithSpawner& operator=(const RegolithSpawner&) = default;

  RegolithSpawner(ros::NodeHandle* nh);

  // loads regolith SDF and computes its mass
  // NOTE: this must be called before any other functions
  bool initialize();

  // spawn the regolith model just above the tip of the scoop and apply a force
  // that keeps it in the scoop during the remainder of scooping operation
  bool spawnRegolithInScoop(bool with_pushback);

  // clears all artificial forces still being applied to regolith models
  void clearAllPsuedoForces();

  // deletes all regolith models
  void removeAllRegolithModels();

  // saves the orientation of scoop to a member variable
  void onLinkStatesMsg(const gazebo_msgs::LinkStates::ConstPtr &msg);

  // computes the volume displaced from a modified terrain diff image and
  // and spawns reoglith if it surpasses the spawn threshold
  void onModDiffVisualMsg(const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg);

  // both call clearAllPsuedoForces at the end of a dig and reset the tracked
  // volume to zero
  void onDigLinearResultMsg(const ow_lander::DigLinearActionResult::ConstPtr &msg);
  void onDigCircularResultMsg(const ow_lander::DigCircularActionResult::ConstPtr &msg);

  // calls deleteAllRegolithModels
  void onDeliverResultMsg(const ow_lander::DeliverActionResult::ConstPtr &msg);

private:
  // ROS interfaces
  std::unique_ptr<ros::NodeHandle> m_node_handle;
  ros::ServiceClient m_gz_spawn_model;
  ros::ServiceClient m_gz_delete_model;
  ros::ServiceClient m_gz_apply_wrench;
  ros::ServiceClient m_gz_clear_wrench;
  ros::Subscriber m_link_states;
  ros::Subscriber m_mod_diff_visual;
  ros::Subscriber m_dig_linear_result;
  ros::Subscriber m_dig_circular_result;
  ros::Subscriber m_deliver_result;

  // sum of volume displaced since previous reoglith spawning
  double m_volume_displaced;
  // orientation of scoop in Gazebo
  tf::Quaternion m_scoop_orientation;

  // regolith will spawn once this amount of volume is displaced
  double m_spawn_threshold;
  // Gazebo link name of the scoop particles will spawn in
  std::string m_scoop_link_name;
  // a vector that describes the forward direction of the scoop
  tf::Vector3 m_scoop_forward;
  // an offset relative to the scoop's frame where regolith will be spawned
  tf::Vector3 m_scoop_spawn_offset;

  // regolith model that spawns in the scoop when digging occurs
  std::string m_model_uri;
  std::string m_model_sdf;
  std::string m_model_link_name;
  // magnitude of the force that pushes each model into the back of the scoop
  float m_psuedo_force_mag;

  // keeps track of all regolith models and links present in the simulation
  struct Regolith {
    std::string model_name;
    std::string body_name;
  };
  std::vector<Regolith> m_active_models;
};

#endif // REGOLITH_SPAWNER_H

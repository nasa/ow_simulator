// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef REGOLITH_SPAWNER_H
#define REGOLITH_SPAWNER_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "ow_dynamic_terrain/modified_terrain_diff.h"
#include "ow_lander/DigLinearActionResult.h"
#include "ow_lander/DigCircularActionResult.h"

class RegolithSpawner
{
public:
  RegolithSpawner() = delete;

  RegolithSpawner(ros::NodeHandle* nh);

  // loads regolith SDF and computes its mass
  // must be called before the other functions
  bool initialize(void); // may block excecution

  // spawn the regolith model just above the tip of the scoop and apply a force
  // that keeps it in the scoop during the remainder of scooping operation
  bool spawnRegolithInScoop(tf::Vector3 pushback_direction = tf::Vector3());

  void clearAllPsuedoForces(void);

  // computes the volume displaced from a modified terrain diff image and
  // and spawns reoglith if it surpasses the spawn threshold
  void terrainVisualModCb(
    const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg
  );

  void digLinearResultCb(const ow_lander::DigLinearActionResult::ConstPtr &msg);

  void digCircularResultCb(const ow_lander::DigCircularActionResult::ConstPtr &msg);

private:
  // sum of volume displaced since last call to spawnRegolithInScoop
  double m_volume_displaced;
  // volume threshold for spawning regolith
  double m_spawn_threshold;
  // where terrain modification previously occurred
  tf::Vector3 m_previous_center;

  // regolith model that spawns in the scoop when digging occurs
  std::string m_model_uri;
  std::string m_model_sdf;
  std::string m_model_linkname;
  // magnitude of the force that pushes each model into the back of the scoop.
  float m_psuedo_force_mag;

  // keep track of all regolith models and links present in the simulation
  struct Regolith {
    std::string model_name;
    std::string body_name;
    bool applying_force;
  };
  std::vector<Regolith> m_active_regolith;

  // ROS interfaces
  std::unique_ptr<ros::NodeHandle> m_node_handle;
  ros::ServiceClient m_gz_spawn_model;
  ros::ServiceClient m_gz_delete_model;
  ros::ServiceClient m_gz_apply_wrench;
  ros::ServiceClient m_gz_clear_wrench;
  ros::Subscriber m_modify_terrain_visual;
  ros::Subscriber m_dig_linear_result;
  ros::Subscriber m_dig_circular_result;
};

#endif // REGOLITH_SPAWNER_H
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

#include <gazebo_msgs/LinkStates.h>

#include <ow_dynamic_terrain/modified_terrain_diff.h>

#include <ow_lander/DigLinearActionResult.h>
#include <ow_lander/DigCircularActionResult.h>
#include <ow_lander/DeliverActionResult.h>
#include <ow_lander/DiscardActionResult.h>

#include <ow_regolith/SpawnRegolith.h>
#include <ow_regolith/RemoveAllRegolith.h>
#include <ow_regolith/TerrainContact.h>

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

  // spawn the regolith model just above the tip of the scoop and apply a force
  // that keeps it in the scoop during the remainder of scooping operation
  bool spawnRegolith(tf::Point position, std::string reference_frame);

  // adds a force aligned towards the back of the scoop for indefinite duration
  bool applyScoopPushback(std::string body_name);

  // clears all artificial forces still being applied to regolith models
  bool clearAllPsuedoForces();

  // deletes all regolith models
  bool removeAllRegolithModels();

  // service callback for spawnRegolithInScoop
  bool spawnRegolithSrv(ow_regolith::SpawnRegolithRequest &request,
                        ow_regolith::SpawnRegolithResponse &response);

  // service callback for removeAllRegolithModels
  bool removeAllRegolithSrv(ow_regolith::RemoveAllRegolithRequest &request,
                            ow_regolith::RemoveAllRegolithResponse &response);

  // saves the orientation of scoop to a member variable
  void onLinkStatesMsg(const gazebo_msgs::LinkStates::ConstPtr &msg);

  void onTerrainContact(const TerrainContact::ConstPtr &msg);

  // computes the volume displaced from a modified terrain diff image and
  // and spawns reoglith if it surpasses the spawn threshold
  void onModDiffVisualMsg(const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg);

  // both call clearAllPsuedoForces at the end of a dig and reset the tracked
  // volume to zero
  void onDigLinearResultMsg(const ow_lander::DigLinearActionResult::ConstPtr &msg);
  void onDigCircularResultMsg(const ow_lander::DigCircularActionResult::ConstPtr &msg);

private:

  inline bool isRegolith(const std::string &name) {
    return name.rfind("regolith_") == 0;
  }

  void resetTrackedVolume();

  bool removeRegolithModel(const std::string &name);

  // ROS interfaces
  std::shared_ptr<ros::NodeHandle> m_node_handle;

  ServiceClientFacade m_gz_spawn_model, m_gz_delete_model,
                      m_gz_apply_wrench, m_gz_clear_wrench;

  ros::ServiceServer m_srv_spawn_regolith;
  ros::ServiceServer m_srv_remove_all_regolith;

  ros::Subscriber m_sub_link_states;
  ros::Subscriber m_sub_terrain_contact;

  ros::Subscriber m_mod_diff_visual;
  ros::Subscriber m_dig_linear_result;
  ros::Subscriber m_dig_circular_result;

  // sum of volume displaced since previous reoglith spawning
  double m_volume_displaced;
  tf::Point m_volume_center;
  // orientation of scoop in Gazebo
  tf::Quaternion m_scoop_orientation;

  // regolith will spawn once this amount of volume is displaced
  double m_spawn_threshold;

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

} // namespace ow_regolith

#endif // REGOLITH_SPAWNER_H

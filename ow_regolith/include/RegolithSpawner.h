// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef REGOLITH_SPAWNER_H
#define REGOLITH_SPAWNER_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <ServiceClientFacade.h>

#include <gazebo_msgs/LinkStates.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/BodyRequest.h>

#include <ow_dynamic_terrain/modified_terrain_diff.h>

#include <ow_lander/DigLinearActionResult.h>
#include <ow_lander/DigCircularActionResult.h>
#include <ow_lander/DeliverActionResult.h>
#include <ow_lander/DiscardActionResult.h>

#include <ow_regolith/SpawnRegolithInScoop.h>
#include <ow_regolith/RemoveAllRegolith.h>

namespace ow_regolith {

class RegolithSpawner
{
public:
  RegolithSpawner()  = delete;
  ~RegolithSpawner() = default;
  RegolithSpawner(const RegolithSpawner&) = delete;
  RegolithSpawner& operator=(const RegolithSpawner&) = delete;

  RegolithSpawner(std::string node_name);

  // loads regolith SDF and computes its mass
  // NOTE: this must be called before any other functions
  bool initialize();

  // spawn the regolith model just above the tip of the scoop and apply a force
  // that keeps it in the scoop during the remainder of scooping operation
  bool spawnRegolithInScoop(bool with_pushback);

  // clears all artificial forces still being applied to regolith models
  bool clearAllPsuedoForces();

  // deletes all regolith models
  bool removeAllRegolithModels();

  // service callback for spawnRegolithInScoop
  bool spawnRegolithInScoopSrv(ow_regolith::SpawnRegolithInScoopRequest &request,
                               ow_regolith::SpawnRegolithInScoopResponse &response);

  // service callback for removeAllRegolithModels
  bool removeAllRegolithSrv(ow_regolith::RemoveAllRegolithRequest &request,
                            ow_regolith::RemoveAllRegolithResponse &response);

  // saves the orientation of scoop to a member variable
  void onLinkStatesMsg(const gazebo_msgs::LinkStates::ConstPtr &msg);

  // computes the volume displaced from a modified terrain diff image and
  // and spawns reoglith if it surpasses the spawn threshold
  void onModDiffVisualMsg(const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg);

  // both call clearAllPsuedoForces at the end of a dig and reset the tracked
  // volume to zero
  void onDigLinearResultMsg(const ow_lander::DigLinearActionResult::ConstPtr &msg);
  void onDigCircularResultMsg(const ow_lander::DigCircularActionResult::ConstPtr &msg);

  // both call removeAllRegolithModels
  void onDeliverResultMsg(const ow_lander::DeliverActionResult::ConstPtr &msg);
  void onDiscardResultMsg(const ow_lander::DiscardActionResult::ConstPtr &msg);

private:
  // ROS interfaces
  std::shared_ptr<ros::NodeHandle> m_node_handle;

  ServiceClientFacade<gazebo_msgs::SpawnModel>      m_gz_spawn_model;
  ServiceClientFacade<gazebo_msgs::DeleteModel>     m_gz_delete_model;
  ServiceClientFacade<gazebo_msgs::ApplyBodyWrench> m_gz_apply_wrench;
  ServiceClientFacade<gazebo_msgs::BodyRequest>     m_gz_clear_wrench;

  ros::ServiceServer m_spawn_regolith_in_scoop;
  ros::ServiceServer m_remove_all_regolith;

  ros::Subscriber m_link_states;
  ros::Subscriber m_mod_diff_visual;
  ros::Subscriber m_dig_linear_result;
  ros::Subscriber m_dig_circular_result;
  ros::Subscriber m_deliver_result;
  ros::Subscriber m_discard_result;

  // sum of volume displaced since previous reoglith spawning
  double m_volume_displaced;
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

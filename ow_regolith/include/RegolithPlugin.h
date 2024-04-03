// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef REGOLITH_SPAWNER_H
#define REGOLITH_SPAWNER_H

#include <vector>
#include <memory>

#include "ros/ros.h"

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"

#include "ow_materials/MaterialDatabase.h"
#include "ow_materials/material_mixing.h"
#include "ow_materials/BulkExcavation.h"

#include "ow_dynamic_terrain/scoop_dig_phase.h"

#include "ow_regolith/SpawnRegolith.h"
#include "ow_regolith/RemoveRegolith.h"
#include "ow_regolith/Contacts.h"

#include "ParticlePool.h"
#include "SingleThreadedTaskQueue.h"

namespace ow_regolith {

class RegolithPlugin : public gazebo::ModelPlugin
{

// RegolithPlugin is a ROS node that detects when digging by the scoop
// end-effector occurs and spawns a model in the scoop to represent collected
// material. The node will also remove models it has spawned as they collide
// with the terrain or in response to a service call.

public:
  RegolithPlugin();
  ~RegolithPlugin() = default;

  RegolithPlugin(const RegolithPlugin&)            = delete;
  RegolithPlugin& operator=(const RegolithPlugin&) = delete;

  void Load(gazebo::physics::ModelPtr world, sdf::ElementPtr sdf) override;

  // reset displaced bulk so that it is empty
  void resetDisplacedBulk();

  // service callback for spawnRegolithInScoop
  bool spawnRegolithSrv(ow_regolith::SpawnRegolithRequest &request,
                        ow_regolith::SpawnRegolithResponse &response);

  // service callback for removeAllRegolithModels
  bool removeRegolithSrv(ow_regolith::RemoveRegolithRequest &request,
                         ow_regolith::RemoveRegolithResponse &response);

  void onTerrainContact(const ow_regolith::Contacts::ConstPtr &msg);

  // computes the volume displaced from a modified terrain diff image and
  // and spawns reoglith if it surpasses the spawn threshold
  void onBulkExcavationVisualMsg(
    const ow_materials::BulkExcavation::ConstPtr &msg);

  void onDigPhaseMsg(const ow_dynamic_terrain::scoop_dig_phase::ConstPtr &msg);

private:
  void processBulkExcavation(ow_materials::BulkExcavation *bulk);

  // when called, broadcasts a BulkExcavation of ingested material
  void onConsolidateIngestedTimeout(const ros::TimerEvent&);

  // make necessary calls to reset the member ROS Timer
  void resetConsolidatedIngestedTimeout();

  // the following members support ingest bulk consolidation of ingested and
  // displaced material
  ros::Publisher m_pub_material_ingested;
  ros::Timer m_consolidate_ingested_timeout;
  ow_materials::Bulk m_bulk_ingested;
  ow_materials::Bulk m_bulk_displaced;

  std::unique_ptr<ros::NodeHandle> m_node_handle;

  ros::ServiceServer m_srv_spawn_regolith;
  ros::ServiceServer m_srv_remove_all_regolith;
  ros::Subscriber m_sub_terrain_contact;
  ros::Subscriber m_sub_bulk_excavation;
  ros::Subscriber m_sub_dig_phase;

  gazebo::physics::LinkPtr m_scoop_link;

  // If true, the scoop is performing a dig motion. Based on the value of
  // the dig phase message.
  bool m_scoop_is_digging = false;
  // If true, a push back force will be applied to spawned particles to keep
  // them in the scoop. Determined based on the dig phase.
  bool m_pushback_force_required = false;
  // sequence number of mod diff visual message
  std::uint32_t m_next_expected_seq = 0u;
  // spawns, removes, and applies forces to spawned models
  ParticlePool m_pool;
  // allows for look up of material properties
  ow_materials::MaterialDatabase m_material_db;
  // Forces processBulkExcavation to occur synchronously even if message
  // arrivals overlap.
  SingleThreadedTaskQueue<ow_materials::BulkExcavation> m_task_queue;

  // Speed that will be given to regolith spawned by terrain interaction in the
  // opposite direction of scoop movement in the x-y plane.
  double m_regolith_spawn_velocity;
  // Tracks the rate at which spawn occur so spawns don't overlap
  ros::Time m_time_of_last_central_spawn;
  // The time it takes for a particle to exit the spawn volume under the
  // the spawn velocity.
  ros::Duration m_spawn_overlap_interval;

  // regolith will spawn once this amount of volume is displaced
  double m_spawn_threshold;
  // list of spawn positions relative to scoop link
  std::vector<ignition::math::Vector3d> m_spawn_offsets;
  std::vector<ignition::math::Vector3d>::const_iterator m_spawn_offset_selector;
};

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RegolithPlugin)

} // namespace ow_regolith

#endif // REGOLITH_SPAWNER_H

// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MODEL_POOL_H
#define MODEL_POOL_H

#include <string>
#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "ignition/math/Vector3.hh"
#include "gazebo/physics/physics.hh"

#include "ow_materials/material_mixing.h"

namespace ow_regolith
{

class ParticlePool
{

  // ParticlePool enables rapid spawning and removal of multiple instances of a
  // model in gazebo. It can also apply forces to the models it has spawned.

public:
  ParticlePool();
  ~ParticlePool() = default;

  ParticlePool(const ParticlePool&)            = delete;
  ParticlePool& operator=(const ParticlePool&) = delete;

  bool initialize(const std::string name, gazebo::physics::WorldPtr world,
                  const std::string &model_uri, ros::NodeHandle *nh);

  float getModelMass() const {return 0.031f;}

  // spawn a model
  std::string spawn(const ignition::math::Vector3d &position,
                    const std::string &reference_frame,
                    const ow_materials::Bulk &bulk);

  // remove models within the pool by link name
  void remove(const std::vector<std::string> &link_names, bool ingested);

  // remove all models in pool
  void clear();

  // applies of force to the specified link
  bool applyMaintainedForce(const std::string &link_name,
                            const ignition::math::Vector3d &force);

  // clears all forces acting on all active models
  void clearAllForces();

private:
  inline bool isInitialized() {
    return m_world != nullptr;
  }

  // bool removeModel(gazebo_msgs::DeleteModel &msg);
  void getModelFromWorld();

  void onUpdate() const;

  void onConsolidateIngestedTimeout(const ros::TimerEvent&);

  // make necessary calls to reset the member ROS Timer
  void resetConsolidatedIngestedTimeout();

  ros::Timer m_consolidate_ingested_timeout;

  ow_materials::Bulk m_ingested_bulk;

  ros::Publisher m_pub_material_ingested;

  gazebo::event::ConnectionPtr m_update_event;

  gazebo::physics::WorldPtr m_world;

  // name of Gazebo model that contains all regolith links
  std::string m_name;

  // regolith model that spawns in the scoop when digging occurs
  sdf::SDF m_model_sdf;

  struct Particle {
    Particle(ow_materials::Bulk bulk)
      : force_applied(false), force(ignition::math::Vector3d::Zero) {
      // do nothing
    }
    Particle() = delete;
    Particle(const Particle&) = delete;
    Particle& operator=(const Particle&) = delete;

    const ow_materials::Bulk bulk;
    bool force_applied;
    ignition::math::Vector3d force;
  };

  std::unordered_map<std::string, Particle> m_active_models;
};

} // namespace ow_regolith

#endif // MODEL_POOL_H

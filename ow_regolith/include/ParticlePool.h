// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MODEL_POOL_H
#define MODEL_POOL_H

#include <string>
#include <vector>
#include <unordered_map>

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
  ParticlePool() = default;
  ~ParticlePool() = default;

  ParticlePool(const ParticlePool&)            = delete;
  ParticlePool& operator=(const ParticlePool&) = delete;

  bool initialize(const std::string name, gazebo::physics::WorldPtr world,
                  const std::string &model_uri);

  // spawn a model
  std::string spawn(const ignition::math::Vector3d &position,
                    const std::string &reference_frame,
                    const ow_materials::Bulk &bulk,
                    const double mass = 0.03);

  // remove models in pool by link name
  void remove(const std::vector<std::string> &link_names);

  // remove models in pool by link name and return their mixed bulk
  ow_materials::Bulk removeAndConsolidate(
    const std::vector<std::string> &link_names);

  // remove all models in pool
  void clear();

  // applies of force to the specified link
  bool applyMaintainedForce(const std::string &link_name,
                            const ignition::math::Vector3d &force);

  // clears all forces acting on all active models
  void clearAllForces();

private:
  inline bool isInitialized() {
    return m_initialized;
  }

  void onUpdate() const;

  bool m_initialized = false;

  gazebo::event::ConnectionPtr m_update_event;

  gazebo::physics::WorldPtr m_world;

  // name of Gazebo model that contains all regolith links
  std::string m_name;

  // regolith model that spawns in the scoop when digging occurs
  sdf::SDF m_sdf;
  // associated parameter and element pointers to customized spawned particles
  sdf::ParamPtr  m_sdf_name;
  sdf::ElementPtr m_sdf_pose;
  sdf::ElementPtr m_sdf_mass;
  sdf::ElementPtr m_sdf_ixx;
  sdf::ElementPtr m_sdf_iyy;
  sdf::ElementPtr m_sdf_izz;
  sdf::ElementPtr m_sdf_radius;

  struct Particle {
    Particle(ow_materials::Bulk bulk_)
      : bulk(bulk_), force_applied(false), force(ignition::math::Vector3d::Zero)
    {
      // do nothing
    }
    Particle() = delete;
    Particle(const Particle&) = delete;
    Particle& operator=(const Particle&) = delete;

    const ow_materials::Bulk bulk;
    bool force_applied;
    ignition::math::Vector3d force;
  };

  using PoolType = std::unordered_map<std::string, Particle>;

  void removeParticle(const PoolType::iterator it);

   PoolType m_active_models;
};

} // namespace ow_regolith

#endif // MODEL_POOL_H

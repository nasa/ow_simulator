// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <sstream>

#include "sdf_utility.h"
#include "ParticlePool.h"

using namespace ow_regolith;
using namespace sdf_utility;

using std::string, std::vector, std::stringstream, std::endl;

using ignition::math::Vector3d, ignition::math::Quaterniond,
      ignition::math::Pose3d;

const string NOT_INITIALIZED_ERR{
  "ParticlePool has not yet been initialized!"
};

bool ParticlePool::initialize(const string name,
                              gazebo::physics::WorldPtr world,
                              const string &model_uri)
{
  // prepare world to contain a particle pool
  if (!world) {
    gzerr << "Invalid world pointer" << endl;
    return false;
  }
  m_world = world;
  m_name = name;

  // acquire SDF data from URI path
  string sdf_string;
  if (!getSdfFromUri(model_uri, sdf_string)) {
    gzerr << "Failed to load particle SDF file" << endl;
    return false;
  }
  parseSdf(sdf_string, m_sdf);

  m_update_event = gazebo::event::Events::ConnectBeforePhysicsUpdate(
    std::bind(&ParticlePool::onUpdate, this)
  );

  // get sdf attribute pointer for name setting
  auto model_element = m_sdf.Root()->GetElement("model");
  if (!model_element) {
    gzerr << "Regolith SDF is missing model element." << endl;
    return false;
  }
  m_sdf_name = model_element->GetAttribute("name");
  if (!m_sdf_name) {
    gzerr << "Regolith SDF is missing name element." << endl;
    return false;
  }
  // get sdf element pointer for pose setting
  m_sdf_pose = model_element->GetElement("pose");
  if (!m_sdf_pose) {
    gzerr << "Regolith SDF is missing pose element." << endl;
    return false;
  }
  // get mass and inertia pointers for mass and moment of inertia setting
  auto link_element = model_element->GetElement("link");
  if (!link_element) {
    gzerr << "Regolith SDF is missing link element." << endl;
    return false;
  }
  auto inertial_element = link_element->GetElement("inertial");
  if (!inertial_element) {
    gzerr << "Regolith SDF is missing inertial element." << endl;
    return false;
  }
  m_sdf_mass = inertial_element->GetElement("mass");
  if (!m_sdf_mass) {
    gzerr << "Regolith SDF is missing mass element." << endl;
    return false;
  }

  m_initialized = true;
  return true;
}

string ParticlePool::spawn(const Vector3d &position,
                           const string &reference_frame,
                           const ow_materials::Bulk &bulk,
                           const double mass)
{
  if (!isInitialized()) {
    gzerr << NOT_INITIALIZED_ERR << endl;
    return "";
  }
  // ensure each created particle has a unique name
  static unsigned int spawn_count = 0u;
  stringstream ss;
  ss << "regolith_" << spawn_count++;
  const auto name = ss.str();
  // set model name
  m_sdf_name->Set(name);
  // set model pose
  if (reference_frame == "" || reference_frame == "world") {
    m_sdf_pose->Set(Pose3d(position, Quaterniond::Identity));
  } else {
    // find world pose relative to requested reference frame
    const auto reference = m_world->EntityByName(reference_frame);
    if (!reference) {
      gzerr << "Failed to find the entity that corresponds to the requested "
               "reference frame, " << reference_frame << "." << endl;
      return "";
    }
    m_sdf_pose->Set(
      reference->WorldPose() * Pose3d(position, Quaterniond::Identity)
    );
  }
  m_sdf_mass->Set(mass);
  // insert into world and track as an active model
  // NOTE: Models inserted with InsertModelSDF may take several simulation
  //  cycles or more before appearing in the world's model list. For this reason
  //  immediately a pointer to the inserted model is tricky and not attempted.
  //  Models are instead always referenced by their name.
  m_world->InsertModelSDF(m_sdf);
  auto inserted = m_active_models.try_emplace(name, bulk);
  if (!inserted.second) {
    gzerr << "Insertion of a new particle into the pool failed. This should "
             "never happen!" << endl;
  }
  return name;
}

void ParticlePool::remove(const vector<string> &model_names)
{
  if (!isInitialized()) {
    gzerr << NOT_INITIALIZED_ERR << endl;
    return;
  }
  for (const auto name : model_names) {
    auto particle_it = m_active_models.find(name);
    if (particle_it == m_active_models.end()) {
      gzwarn << "Particle " << name << " not present. Nothing removed." << endl;
      continue;
    }
    removeParticle(particle_it);
  }
}

ow_materials::Bulk ParticlePool::removeAndConsolidate(
  const vector<string> &model_names)
{
  if (!isInitialized()) {
    gzerr << NOT_INITIALIZED_ERR << endl;
    return ow_materials::Bulk();
  }
  ow_materials::Bulk consolidated;
  for (const auto name : model_names) {
    auto particle_it = m_active_models.find(name);
    if (particle_it == m_active_models.end()) {
      gzwarn << "Particle " << name << " not present. Nothing removed." << endl;
      continue;
    }
    consolidated.mix(particle_it->second.bulk);
    removeParticle(particle_it);
  }
  return consolidated;
}

void ParticlePool::removeParticle(const PoolType::iterator it)
{
  m_world->RemoveModel(it->first);
  m_active_models.erase(it);
}

void ParticlePool::clear() {
  if (!isInitialized()) {
    gzerr << NOT_INITIALIZED_ERR << endl;
    return;
  }
  for (auto const &particle : m_active_models) {
    m_world->RemoveModel(particle.first);
  }
  m_active_models.clear();
}

bool ParticlePool::applyMaintainedForce(const string &model_name,
                                        const Vector3d &force) {
  if (!isInitialized()) {
    gzerr << NOT_INITIALIZED_ERR << endl;
    return false;
  }
  try {
    auto &particle = m_active_models.at(model_name);
    particle.force_applied = true;
    particle.force = force;
  } catch (std::out_of_range err) {
    gzerr << "Attempted to apply force to link that does not exist." << endl;
    return false;
  }
  return true;
}

void ParticlePool::clearAllForces()
{
  if (!isInitialized()) {
    gzerr << NOT_INITIALIZED_ERR << endl;
    return;
  }
  for (auto &particle : m_active_models) {
    particle.second.force_applied = false;
  }
}

void ParticlePool::onUpdate() const
{
  for (auto const &particle : m_active_models) {
    if (particle.second.force_applied) {
      auto model = m_world->ModelByName(particle.first);
      if (!model) {
        // NOTE: Models take several updates before they appear in the world's
        // model list. As a result, not finding the model should not result in a
        // warning/error.
        continue;
      }
      auto link = model->GetChildLink("link");
      if (!link) {
        gzwarn << "Regolith model " << particle.first
               << " is has no link." << endl;
        continue;
      }
      link->AddLinkForce(particle.second.force);
    }
  }
}

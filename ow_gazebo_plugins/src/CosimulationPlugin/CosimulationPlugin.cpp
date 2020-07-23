// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// TODO:
//    1) Add in discrete element domain relaxation step before cosimulation
//       begins (if needed).
//    2) Transform force and torque to Gazebo's reference frame (if needed).
//    3) Interpolate between consecutive scoop poses before each call to 
//       Cosimulator::update to ensure smooth movement.
//    4) Implement CropHeightmap
//    5) Read in parameters in a more OceanWATERS-like way (i.e rosparam yaml?)

#include "CosimulationPlugin.h"

#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>

#include <cmath>

using namespace gazebo;

// Generate a mesh with a +Z-facing geometry that matches the heigthmap and
// all other sides match the box boundaries
static common::Mesh CropHeightmap(rendering::Heightmap* heightmap, 
    ignition::math::OrientedBoxd& box)
{
  gzwarn << "CropHeightmap - STUBBED" << std::endl;

  return common::Mesh();
}

// Convenience function for the Load function SDF parsing
template <class T>
static T parseElement(sdf::ElementPtr sdf, const std::string &name) {
  if (!sdf->HasElement(name)) {
    gzthrow ("CosimulationPlugin::Load - you must specify a " << name << " element.");
  }
  return sdf->Get<T>(name);
}

CosimulationPlugin::CosimulationPlugin() 
: ModelPlugin(), m_started(false)
{
}

CosimulationPlugin::~CosimulationPlugin()
{
}

void CosimulationPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // SDF parameters
  std::string link_model;
  double link_youngs_modulus, link_poisson_ratio, link_restitution_coef, 
      link_friction_static, link_friction_rolling;
  double part_youngs_modulus, part_poisson_ratio, part_restitution_coef, 
      part_friction_static, part_friction_rolling, part_density, part_radius;
  ignition::math::Vector3d workspace_dimensions;
  ignition::math::Pose3d workspace_pose; 

  try {
    // parse link parameter and save reference
    std::string link_name = parseElement<std::string>(sdf, "link");
    m_link = model->GetLink(link_name);
    if(!m_link) {
      gzerr << "CosimulationPlugin::Load - specified link is invalid." << std::endl;
      return;
    }
    // NOTE: link_model is assumed to be the same as the links mesh in Gazebo
    // Simplified meshes can be used here to decrease cosimulation expense
    link_model = parseElement<std::string>(sdf, "link_model");
    // parse link property parameters
    link_youngs_modulus = parseElement<double>(sdf, "link_youngs_modulus");
    link_poisson_ratio = parseElement<double>(sdf, "link_poisson_ratio");
    link_restitution_coef = parseElement<double>(sdf, "link_restitution_coef");
    link_friction_static = parseElement<double>(sdf, "link_static_friction_coef");
    link_friction_rolling = parseElement<double>(sdf, "link_rolling_friction_coef");
    // parse particle property parameters
    part_youngs_modulus = parseElement<double>(sdf, "particle_youngs_modulus");
    part_poisson_ratio = parseElement<double>(sdf, "particle_poisson_ratio");
    part_restitution_coef = parseElement<double>(sdf, "particle_restitution_coef");
    part_friction_static = parseElement<double>(sdf, "particle_static_friction_coef");
    part_friction_rolling = parseElement<double>(sdf, "particle_rolling_friction_coef");
    part_density = parseElement<double>(sdf, "particle_density");
    part_radius = parseElement<double>(sdf, "particle_radius");
    // workspace parameters
    workspace_dimensions = parseElement<ignition::math::Vector3d>(sdf, "workspace_dimensions");
    workspace_pose = parseElement<ignition::math::Pose3d>(sdf, "workspace_pose");  
    // Cosimulation timestep parameter
    m_timestep = parseElement<double>(sdf, "timestep");
  } catch (common::Exception err) {
    // TODO: print this exception to gzerr
    return;
  }

  gzmsg << "CosimulationPlugin::Load - Initializing cosimulation..." << std::endl;
  // pass material properties from the SDF configuration
  m_cosim.setToolProperties(link_youngs_modulus, link_poisson_ratio,
      link_restitution_coef, link_friction_static, link_friction_rolling);
  m_cosim.setParticleProperties(part_youngs_modulus, part_poisson_ratio, 
      part_restitution_coef, part_friction_static, part_friction_rolling, 
      part_density, part_radius);
  // get world pointer
  m_world = physics::get_world();
  if (!m_world) {
    gzerr << "CosimulationPlugin::Load - world pointer is NULL" << std::endl;
    return;
  }
  // set gravity to be the same as Gazebo's
  m_cosim.setGravity(m_world->Gravity());
  // pass model file path to cosimulator
  m_cosim.setToolMesh(link_model);
  // build workspace bounding box
  m_workspace_box.Size(workspace_dimensions);
  m_workspace_box.Pose(workspace_pose);
  // get heightmap and pass it to the cosimulation as a cropped heightmap
  rendering::ScenePtr scene = rendering::get_scene();
  if (!scene) {
    gzerr << "CosimulationPlugin::Load - scene pointer is NULL" << std::endl;
    return;
  }
  rendering::Heightmap* heightmap = scene->GetHeightmap();
  if (!heightmap) {
    gzerr << "CosimulationPlugin::Load - heightmap pointer is NULL" << std::endl;
    return;  
  }
  m_cosim.setParticleFillGeometry(CropHeightmap(heightmap, m_workspace_box));

  m_cosim.initialize();

  m_phys_connect = event::Events::ConnectBeforePhysicsUpdate(
      std::bind(&CosimulationPlugin::OnUpdate, this));
}

void CosimulationPlugin::OnUpdate(void)
{
  // NOTE: So long as region's height is larger than the link, using the link's
  //       CoG here works fine.
  ignition::math::Pose3d link_pose = m_link->WorldCoGPose();

  if (m_workspace_box.Contains(link_pose.Pos()) == false) {
    // ignore this call if link is not within workspace yet  
    return;
  } else if (m_started == false) {
    // record time when link first enters workspace
    m_started = true;
    m_starttime = m_world->SimTime().Double();
  }

  double gz_time = m_world->SimTime().Double();
  // linear and angular impulse for accumulating forces and torques
  ignition::math::Vector3d lin_impulse;
  ignition::math::Vector3d ang_impulse;
  // track total cosimulation time for all loops
  double elapsed = 0.0;
  // catch cosimulation up to Gazebo simulation
  // FIXME: m_starttime should be used in here for comparing cosim time and gz_time
  while(m_cosim.getSimTime() <= gz_time) {
    // cosim update function output variables
    ignition::math::Vector3d out_force;
    ignition::math::Vector3d out_torque;
    // select timestep
    // NOTE: remainder calculation allows both simulations to sync their clocks
    double remainder = gz_time - (m_cosim.getSimTime() + m_timestep);
    if (remainder == 0.0) {
      break;
    }
    double timestep = std::min(remainder, m_timestep);
    // advance cosimulation physics
    m_cosim.update(link_pose, timestep, out_force, out_torque);
    // accumulate forces and torques as linear and angular impulse
    lin_impulse += out_force * timestep;
    ang_impulse += out_torque * timestep;
    elapsed += timestep;
  }
  // calculate time averaged force and torque from impulses
  if (elapsed <= 0.0) {
    gzerr << "CosimulationPlugin::OnUpdate - Cosimulation elapsed time <= 0" << std::endl;
    return;
  }
  auto avg_force = lin_impulse / elapsed;
  auto avg_torque = ang_impulse / elapsed;
  // pass force and torque back to Gazebo
  m_link->AddRelativeForce(avg_force);
  m_link->AddRelativeTorque(avg_torque);
}

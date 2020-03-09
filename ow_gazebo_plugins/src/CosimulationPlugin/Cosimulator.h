// __BEGIN_LICENSE__
// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

// TODO:
//   1) Add a mesh data structure to define the fill volume of discrete elements
//   2) Add a mesh data structure to for the tool
//   3) Add all necessary external physics simulation data structures and calls

#ifndef Cosimulator_h
#define Cosimulator_h

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <gazebo/common/common.hh>

class Cosimulator
{
public:
  Cosimulator();

  // fill geometry with discrete elements, place tool in sim, and initialize sim
  void initialize(void);

  // relax DE geometry
  // can be ran in the background to shorten total simulation time
  void relax(const int &max_it, const int &max_speed);
  bool isRelaxed(void) {
    return m_relaxed;
  }

  // moves tool in cosimulation and iterates by a single timestep
  // outputs force and torque acted on scoop during timestep
  void update(const ignition::math::Pose3d &tool_pose, double timestep,
      ignition::math::Vector3d &out_force, 
      ignition::math::Vector3d &out_torque);

  // return in-simulation time in seconds
  double getSimTime(void);

  void setParticleFillGeometry(gazebo::common::Mesh mesh);
  void setToolMesh(const std::string& path_to_mesh);

  void setParticleProperties(
      double youngs_modulus,
      double poisson_ratio,
      double restitution_coef,
      double friction_static,
      double friction_rolling,
      double density,
      double radius);
  void setToolProperties(
      double youngs_modulus,
      double poisson_ratio,
      double restitution_coef,
      double friction_static,
      double friction_rolling);

  void setGravity(const ignition::math::Vector3d &gravity_vector) {
    m_gravity = gravity_vector;
  }

private:
  // simulation parameters
  ignition::math::Vector3d m_gravity;
  double m_timestep;
  bool m_relaxed;
  // tool material properties
  double m_tool_youngs_modulus;
  double m_tool_poisson_ratio;
  double m_tool_restitution_coef;
  double m_tool_friction_static;
  double m_tool_friction_rolling;
  // particle material and bulk properties
  double m_particle_youngs_modulus;
  double m_particle_poisson_ratio;
  double m_particle_restitution_coef;
  double m_particle_friction_static;
  double m_particle_friction_rolling;
  double m_particle_radius;
  double m_particle_density;

};

#endif // Cosimulator_h

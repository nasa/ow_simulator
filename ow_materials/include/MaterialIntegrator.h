// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIAL_INTEGRATOR_H
#define MATERIAL_INTEGRATOR_H

#include <memory>

#include <ros/ros.h>

#include <ow_dynamic_terrain/modified_terrain_diff.h>

namespace ow_materials
{

class MaterialIntegrator
{
public:
  MaterialIntegrator(std::shared_ptr<ros::NodeHandle> node_handle,
                     const gazebo::physics::ModelPtr& model);
  ~MaterialIntegrator() = default;

  MaterialIntegrator() = delete;
  MaterialIntegrator(const MaterialIntegrator&) = delete;
  MaterialIntegrator& operator=(const MaterialIntegrator&) = delete;

private:
  std::shared_ptr<ros::NodeHandle> m_node_handle;
  ros::Subscriber m_sub_visual_modification;
  ros::Subscriber m_sub_collision_modification;

  gazebo::physics::ModelPtr m_model;
  gazebo::physics::HeigtMapShapePtr m_terrain_shape;

  void onModification(
    const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg);

}

}

#endif // MATERIAL_INTEGRATOR_H

 // The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIAL_DISTRIBUTION_PLUGIN_H
#define MATERIAL_DISTRIBUTION_PLUGIN_H

#include <memory>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>

#include <AxisAlignedGrid.h>
#include <MaterialDatabase.h>

namespace ow_materials
{

class MaterialDistributionPlugin : public gazebo::ModelPlugin
{
  public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  private:
  gazebo::physics::ModelPtr m_model;

  std::unique_ptr<MaterialDatabase> m_material_db;

  std::unique_ptr<AxisAlignedGrid<MaterialBlend>> m_grid;

};

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MaterialDistributionPlugin)
}

#endif // MATERIAL_DISTRIBUTION_PLUGIN_H

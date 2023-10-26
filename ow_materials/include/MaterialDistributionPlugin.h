// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIAL_DISTRIBUTION_PLUGIN_H
#define MATERIAL_DISTRIBUTION_PLUGIN_H

#include <memory>

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>

#include <AxisAlignedGrid.h>
#include <MaterialDatabase.h>
#include <MaterialIntegrator.h>

namespace ow_materials
{

class MaterialDistributionPlugin : public gazebo::ModelPlugin
{

  // A Gazebo model plugin that simulates varying materials properties through
  // the subsurface of a heightmap. Requires concurrent operation with
  // ow_dynamic_terrain to read-in modification events that occur to the
  // heightmap and from that data synthesize bulk material properties.

public:
  MaterialDistributionPlugin() = default;
  ~MaterialDistributionPlugin() = default;

  MaterialDistributionPlugin(const MaterialDistributionPlugin&) = delete;
  MaterialDistributionPlugin& operator=(
    const MaterialDistributionPlugin&) = delete;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  void getHeightmapAlbedo();

  void handleVisualBulk(MaterialBlend const &blend);

  void handleCollisionBulk(MaterialBlend const &blend);

  Color interpolateColor(MaterialBlend const &blend) const;

private:
  void populateGrid(Ogre::Image albedo, Ogre::Terrain *terrain);

  std::unique_ptr<ros::NodeHandle> m_node_handle;

  std::unique_ptr<gazebo::event::ConnectionPtr> m_temp_render_connection;

  ros::Publisher m_grid_pub;

  std::unique_ptr<MaterialDatabase> m_material_db;
  std::unique_ptr<AxisAlignedGrid<MaterialBlend>> m_grid;

  std::unique_ptr<MaterialIntegrator> m_visual_integrator;
  std::unique_ptr<MaterialIntegrator> m_collision_integrator;

};

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MaterialDistributionPlugin)
}

#endif // MATERIAL_DISTRIBUTION_PLUGIN_H
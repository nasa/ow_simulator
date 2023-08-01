// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIAL_INTEGRATOR_H
#define MATERIAL_INTEGRATOR_H

#include <functional>

#include <ros/ros.h>

#include <AxisAlignedGrid.h>
#include <Materials.h>

#include <ow_dynamic_terrain/modified_terrain_diff.h>

namespace ow_materials
{

using HandleBulkCallback = std::function<void(MaterialBlend const &)>;

class MaterialIntegrator
{
public:
  MaterialIntegrator(ros::NodeHandle *node_handle,
                    const std::string &modification_topic,
                    AxisAlignedGrid<MaterialBlend> const *grid,
                    HandleBulkCallback handle_bulk_cb);
  ~MaterialIntegrator() = default;

  MaterialIntegrator() = delete;
  MaterialIntegrator(const MaterialIntegrator&) = delete;
  MaterialIntegrator& operator=(const MaterialIntegrator&) = delete;

private:
  const HandleBulkCallback m_handle_bulk_cb;

  ros::NodeHandle *m_node_handle;

  AxisAlignedGrid<MaterialBlend> const *m_grid;

  ros::Subscriber m_sub_modification_diff;

  void onModification(
    const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg);

};

}

#endif // MATERIAL_INTEGRATOR_H

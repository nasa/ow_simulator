// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIAL_INTEGRATOR_H
#define MATERIAL_INTEGRATOR_H

#include <functional>
#include <memory>
#include <mutex>

#include <boost/asio/thread_pool.hpp>

#include <ros/ros.h>

#include <AxisAlignedGrid.h>
#include <Materials.h>

#include <ow_dynamic_terrain/modified_terrain_diff.h>


namespace ow_materials
{

using HandleBulkCallback = std::function<void(MaterialBlend const &)>;

using ColorizerCallback = std::function<Color(MaterialBlend const &)>;

class MaterialIntegrator
{

  // Listens for modification events from ow_dynamic_terrain and computes a
  // bulk material blend by correlating the location of those events with the
  // content of the material grid.

public:
  MaterialIntegrator(ros::NodeHandle *node_handle,
                     AxisAlignedGrid<MaterialBlend> const *grid,
                     const std::string &modification_topic,
                     const std::string &dug_points_topic,
                     HandleBulkCallback handle_bulk_cb,
                     ColorizerCallback colorizer_cb);
  ~MaterialIntegrator() = default;

  MaterialIntegrator() = delete;
  MaterialIntegrator(const MaterialIntegrator&) = delete;
  MaterialIntegrator& operator=(const MaterialIntegrator&) = delete;

private:
  ros::NodeHandle *m_node_handle;

  ros::Subscriber m_sub_modification_diff;

  ros::Publisher m_dug_points_pub;

  // when enough time has passed since the last onModification call, this
  // triggers logging of statistics about blend operation performance
  ros::Timer m_stat_log_timeout;

  HandleBulkCallback m_handle_bulk_cb;

  ColorizerCallback m_colorizer_cb;

  AxisAlignedGrid<MaterialBlend> const *m_grid;

  std::uint32_t m_next_expected_seq = 0u;

  // used to compute performance statistics on batches of modifications
  std::atomic<std::size_t> m_batch_total_modifications = 0u;
  std::atomic<std::int64_t> m_batch_accumulated_processor_time = 0L;

  // used to asynchronously parallelize calls to integrate
  boost::asio::thread_pool m_threads;

  void onStatLogTimeout(const ros::TimerEvent&);

  void resetStatLogTimeout();

  void onModification(
    const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg);

  void integrate(
    const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg);

};

}

#endif // MATERIAL_INTEGRATOR_H

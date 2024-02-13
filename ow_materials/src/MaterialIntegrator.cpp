// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <cmath>
#include <algorithm>
#include <thread>

#include "gazebo/gazebo.hh"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/mat.hpp"

#include "point_cloud_util.h"

#include "MaterialIntegrator.h"

using namespace ow_materials;

using std::uint32_t, std::size_t, std::max;

const auto STAT_LOG_TIMEOUT = ros::Duration(1.0); // second

const uint MINIMUM_INTEGRATE_THREADS = 4u;

MaterialIntegrator::MaterialIntegrator(
  ros::NodeHandle *node_handle, AxisAlignedGrid<Blend> const *grid,
  const std::string &modification_topic, const std::string &dug_points_topic,
  HandleBulkCallback handle_bulk_cb, ColorizerCallback colorizer_cb)
  : m_node_handle(node_handle), m_grid(grid),
    m_handle_bulk_cb(handle_bulk_cb), m_colorizer_cb(colorizer_cb),
    m_stat_log_timeout(
      node_handle->createTimer(STAT_LOG_TIMEOUT,
        &MaterialIntegrator::onStatLogTimeout, this, true, false)
    ),
    m_threads(
      max(MINIMUM_INTEGRATE_THREADS, std::thread::hardware_concurrency())
    )
{
  m_sub_modification_diff = m_node_handle->subscribe(modification_topic, 1,
    &MaterialIntegrator::onModificationMsg, this);
  m_dug_points_pub = m_node_handle
    ->advertise<sensor_msgs::PointCloud2>(dug_points_topic, 10);
}

void MaterialIntegrator::onStatLogTimeout(const ros::TimerEvent&)
{
  // compute averages
  auto average_time = (m_batch_accumulated_processor_time * 1.0e-6) // ns to ms
                      / m_batch_total_modifications;
  auto average_merges = m_batch_accumulated_merges
                        / m_batch_total_modifications;

  gzlog << "Batch of modifications from topic "
          << m_sub_modification_diff.getTopic() << " completed.\n"
        << "Total number of terrain modifications in batch was "
          << m_batch_total_modifications << ".\n"
        << "Average processor time per modification was "
          << average_time << " ms.\n"
        << "Average material blend merges per modification was "
          << average_merges << "." << std::endl;

  // reset batch statistics for next batch of modifications
  m_batch_total_modifications = 0u;
  m_batch_accumulated_processor_time = 0L;
  m_batch_accumulated_merges = 0u;
}

void MaterialIntegrator::resetStatLogTimeout()
{
  m_stat_log_timeout.stop();
  m_stat_log_timeout.setPeriod(STAT_LOG_TIMEOUT);
  m_stat_log_timeout.start();
}

void MaterialIntegrator::onModificationMsg(
  const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg)
{
  resetStatLogTimeout();

  if (msg->header.seq != m_next_expected_seq) {
    ROS_WARN_STREAM(
      "Modification message on topic " << m_sub_modification_diff.getTopic()
      << " was dropped! At least " << (msg->header.seq - m_next_expected_seq)
      << " message(s) may have been missed."
    );
  }
  m_next_expected_seq = msg->header.seq + 1;

  boost::asio::post(m_threads,
    boost::bind(&MaterialIntegrator::integrate, this, msg));
}


void MaterialIntegrator::integrate(
  const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg)
{
  ++m_batch_total_modifications;

  auto start_time = ros::WallTime::now();

  auto diff_handle = cv_bridge::CvImageConstPtr();
  auto result_handle = cv_bridge::CvImageConstPtr();

  try {
    diff_handle = cv_bridge::toCvShare(msg->diff, msg);
    result_handle = cv_bridge::toCvShare(msg->result, msg);
  } catch (cv_bridge::Exception& e) {
    gzlog << "cv_bridge exception occurred while reading modification "
             "differential message: " << e.what() << std::endl;
    return;
  }

  const auto rows = diff_handle->image.rows;
  const auto cols = diff_handle->image.cols;

  const auto pixel_height = msg->height / rows;
  const auto pixel_width = msg->width / cols;

  Blend bulk_blend;
  pcl::PointCloud<pcl::PointXYZRGB> points;

  // The following code makes these assumptions:
  //   a) Each intersected voxel contributes the entirety of its blend to the
  //      final bulk blend. In other words, partial voxel intersections along
  //      the boundary acquire the entire voxel volume.
  //   b) Each blend in the material grid is already normalized.
  //   c) All voxels are of the same volume.

  // TODO: make float-double usage uniform where it makes sense to

  // iterate over entire the differential image and blend voxel grid values that
  // lie within the volume between old heightmap values and new heightmap values
  for (size_t i = 0; i != cols; ++i) {
    for (size_t j = 0; j != rows; ++j) {
      // change in height at pixel (x, y)
      auto dz = diff_handle->image.at<float>(i, j);
      if (dz >= 0.0) { // ignore non-modified or raised terrain
        continue;
      }
      // compute world-space coordinates of this pixel's center
      const float x = (msg->position.x - msg->width / 2)
                        + static_cast<float>(j) / rows * msg->width;
      // +i and +y are in opposite directions, so flip the signs
      const float y = (msg->position.y + msg->height / 2)
                        - static_cast<float>(i) / cols * msg->height;
      const auto pixel_center = GridPositionType2D{x + pixel_width / 2,
                                                   y + pixel_height / 2};
      // determine world-space z values at the top and bottom of the z-column
      const auto z_bottom = result_handle->image.at<float>(i, j);
      const auto z_top = z_bottom - dz; // dz is negative
      m_grid->runForEachInColumn(
        pixel_center, z_bottom, z_top,
        [&bulk_blend, &points]
        (Blend const &b, GridPositionType center) {
          if (b.isEmpty()) return;
          bulk_blend.merge(b);
          // WORKAROUND for OW-1194, TF has an incorrect transform for
          //            base_link (specific for atacama_y1a)
          center -= GridPositionType(-1.0, 0.0, 0.37);
          points.emplace_back();
          points.back().x = static_cast<float>(center.X());
          points.back().y = static_cast<float>(center.Y());
          points.back().z = static_cast<float>(center.Z());
        }
      );
    }
  }

  if (points.size() == 0) {
    return;
  }

  bulk_blend.normalize();

  Color bulk_color = m_colorizer_cb(bulk_blend);
  uint32_t temp_rgb = static_cast<uint32_t>(bulk_color.r) << 16
                    | static_cast<uint32_t>(bulk_color.g) << 8
                    | static_cast<uint32_t>(bulk_color.b);
  float bulk_rgb = *reinterpret_cast<float*>(&temp_rgb);
  for (auto &p : points) {
    p.rgb = bulk_rgb;
  }

  m_batch_accumulated_processor_time
    += (ros::WallTime::now() - start_time).toNSec();
  m_batch_accumulated_merges += points.size();

  publishPointCloud(&m_dug_points_pub, points);

  m_handle_bulk_cb(bulk_blend, static_cast<uint32_t>(points.size()));
}


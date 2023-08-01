// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <MaterialIntegrator.h>

#include <gazebo/gazebo.hh>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

using namespace ow_materials;

MaterialIntegrator::MaterialIntegrator(ros::NodeHandle *node_handle,
                                       const std::string &modification_topic,
                                       AxisAlignedGrid<MaterialBlend> const *grid,
                                       HandleBulkCallback handle_bulk_cb)
  : m_node_handle(node_handle), m_grid(grid), m_handle_bulk_cb(handle_bulk_cb)
{
  m_sub_modification_diff = m_node_handle->subscribe(modification_topic, 10,
    &MaterialIntegrator::onModification, this);
}

void MaterialIntegrator::onModification(
  const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg)
{
  using namespace cv_bridge;

  auto diff_handle = CvImageConstPtr();
  auto result_handle = CvImageConstPtr();

  try {
    diff_handle = toCvShare(msg->diff, msg);
    result_handle = toCvShare(msg->result, msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception occurred while reading modification "
              "differential message: %s", e.what());
    return;
  }

  const auto rows = diff_handle->image.rows;
  const auto cols = diff_handle->image.cols;
  const auto pixel_height = msg->height / rows;
  const auto pixel_width = msg->width / cols;
  const auto pixel_area = pixel_height * pixel_height;

  // float volume_displaced = 0.0;

  MaterialBlend bulk_blend;
  uint merges = 0;

  // The following code makes these assumptions:
  //   a) Each voxel the modification touches acquires the entirety of that
  //      voxel. In other words, volume intersections are not factored in.
  //   b) Each blend in the material grid is already normalized.
  //   c) All voxels are of the same volume.

  // estimate the total volume displaced using a Riemann sum over the image
  for (auto y = 0; y < rows; ++y) {
    for (auto x = 0; x < cols; ++x) {
      auto diff_px = diff_handle->image.at<float>(y, x);
      if (diff_px < 0) {
        const auto volume = diff_px * pixel_area;
        // compute location in grid
        const float xx = (msg->position.x - msg->width / 2)
                          + x / rows * msg->height;
        const float yy = (msg->position.y - msg->height / 2)
                          + y / cols * msg->width;
        const float zz = msg->position.z;
        // FIXME: implicit conversion to double parameters
        if (m_grid->containsPoint(xx, yy, zz)) {
          auto blend = m_grid->getCellValueAtPoint(xx, yy, zz);
          bulk_blend.merge(blend);
          ++merges;
        }
      }
    }
  }

  // gzlog << "merges = " << merges << std::endl;

  if (merges == 0) {
    return;
  }

  // normalize the bulk by the number of blends that were merged into it
  bulk_blend.normalize();

  if (!bulk_blend.isNormalized()) {
    gzwarn << "Bulk is not normalized" << std::endl;
  }

  m_handle_bulk_cb(bulk_blend);
}


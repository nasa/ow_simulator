// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <MaterialIntegrator.h>

#include <cmath>
#include <algorithm>

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

  // TODO: use a binding rectangle/box to early return if there is no overlap
  //       between it and the axis-aligned box

  MaterialBlend bulk_blend;
  uint merges = 0;

  // The following code makes these assumptions:
  //   a) Each voxel the modification touches acquires the entirety of that
  //      voxel. In other words, volume intersections are not factored in.
  //   b) Each blend in the material grid is already normalized.
  //   c) All voxels are of the same volume.

  // estimate the total volume displaced using a Riemann sum over the image
  // FIXME: make float-double usage uniform
  for (auto y = 0; y < rows; ++y) {
    for (auto x = 0; x < cols; ++x) {
      // change in height at pixel (x, y)
      auto dz = diff_handle->image.at<float>(y, x);
      if (dz < 0.0) {
        // const float zz = msg->position.z;
        // const auto volume = diff_px * pixel_area;
        // compute location in grid
        const float xx = (msg->position.x - msg->width / 2)
                          + x / rows * msg->height;
        const float yy = (msg->position.y - msg->height / 2)
                          + y / cols * msg->width;
        // starting layer position; next loop will iterate up to the original z
        auto z_result = result_handle->image.at<float>(y, x);
        // implicit float truncation
        // FIXME: float / double
        // if dz is less than the cell side length, it is still counted as
        // one full cell
        const std::size_t layers = std::min(
          std::size_t(1),
          static_cast<std::size_t>(dz / m_grid->getCellLength())
        );
        for (std::size_t i = 0; i != layers; ++i) {
          const auto zz = z_result + i * m_grid->getCellLength();
          if (m_grid->containsPoint(xx, yy, zz)) {
            const auto blend = m_grid->getCellValueAtPoint(xx, yy, zz);
            bulk_blend.merge(blend);
            ++merges;
          }
        }
      }
    }
  }

  gzlog << "merges = " << merges << std::endl;

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


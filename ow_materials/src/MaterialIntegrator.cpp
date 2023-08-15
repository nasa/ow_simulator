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

/// DEBUG
#include <pcl_visualize.h>

using namespace ow_materials;

MaterialIntegrator::MaterialIntegrator(ros::NodeHandle *node_handle,
                                       const std::string &modification_topic,
                                       AxisAlignedGrid<MaterialBlend> const *grid,
                                       HandleBulkCallback handle_bulk_cb,
                                       ColorizerCallback colorizer_cb)
  : m_node_handle(node_handle), m_grid(grid), m_handle_bulk_cb(handle_bulk_cb),
    m_colorizer_cb(colorizer_cb)
{
  m_sub_modification_diff = m_node_handle->subscribe(modification_topic, 10,
    &MaterialIntegrator::onModification, this);
}

void MaterialIntegrator::onModification(
  const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg)
{
  gzlog << "MaterialIntegrator::onModification called" << std::endl;
  using namespace cv_bridge;
  using PositionType = AxisAlignedGrid<MaterialBlend>::PositionType;

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
  pcl::PointCloud<pcl::PointXYZRGB> points;
  uint merges = 0;

  // The following code makes these assumptions:
  //   a) Each voxel the modification touches acquires the entirety of that
  //      voxel. In other words, volume intersections are not factored in.
  //   b) Each blend in the material grid is already normalized.
  //   c) All voxels are of the same volume.

  // estimate the total volume displaced using a Riemann sum over the image
  // FIXME: make float-double usage uniform
  for (std::size_t i = 0; i != cols; ++i) {
    for (std::size_t j = 0; j != rows; ++j) {
      // change in height at pixel (x, y)
      auto dz = diff_handle->image.at<float>(i, j);
      if (dz < 0.0) {
        // const float zz = msg->position.z;
        // const auto volume = diff_px * pixel_area;
        // compute location in grid
        const float x = (msg->position.x - msg->width / 2)
                          + static_cast<float>(i) / cols * msg->width;
        // +j and +y are in opposite directions, so flip signs
        const float y = (msg->position.y + msg->height / 2)
                          - static_cast<float>(j) / rows * msg->height;
        // starting layer position; next loop will iterate up to the original z
        auto z_result = result_handle->image.at<float>(i, j);
        auto box_min = PositionType(x, y, z_result);
        auto box_max = PositionType(x + pixel_width,
                                     y + pixel_height,
                                     z_result + dz);
        // gzlog << "box_min = (" << box_min.X() << "," << box_min.Y() << "," << box_min.Z() << ")\n";
        // gzlog << "box_max = (" << box_max.X() << "," << box_max.Y() << "," << box_max.Z() << ")\n";
        m_grid->runForEachInRectangle(box_min, box_max,
          // FIXME: do this without static cast
          static_cast<std::function<void(MaterialBlend, PositionType)>>(
          [&bulk_blend, &points, &merges, this]
          (MaterialBlend b, PositionType center) {
            bulk_blend.merge(b);
            ++merges;
            const Color c = m_colorizer_cb(b);
            points.emplace_back(
              static_cast<std::uint8_t>(c.r), // truncates double to int (<256)
              static_cast<std::uint8_t>(c.g),
              static_cast<std::uint8_t>(c.b)
            );

            // WORKAROUND for OW-1194, TF has an incorrect transform for
            //            base_link (specific for atacama_y1a)
            center -= PositionType(-1.0, 0.0, 0.37);

            points.back().x = static_cast<float>(center.X());
            points.back().y = static_cast<float>(center.Y());
            points.back().z = static_cast<float>(center.Z());
          }
        ));
      }
    }
  }

  gzlog << "point count = " << points.size() << "\n";
  gzlog << "points[0] = (" << points[0].x << "," << points[0].y << "," << points[0].z << ")\n";
  gzlog << "points[0] color = (" << (uint)points[0].r << "," << (uint)points[0].g << "," << (uint)points[0].b << ")\n";

  gzlog << "merges = " << merges << std::endl;

  if (merges == 0) {
    return;
  }

  // DEBUG
  static ros::Publisher DEBUG_points_publisher = m_node_handle->advertise<sensor_msgs::PointCloud2>(
    "/ow_materials/dug_points2", 10
  );
  publish_points_as_cloud(&DEBUG_points_publisher, points);

  // normalize the bulk by the number of blends that were merged into it
  bulk_blend.normalize();

  if (!bulk_blend.isNormalized()) {
    gzwarn << "Bulk is not normalized" << std::endl;
  }

  m_handle_bulk_cb(bulk_blend);
}


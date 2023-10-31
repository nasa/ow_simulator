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

#include <point_cloud_util.h>

using namespace ow_materials;

using std::uint32_t, std::size_t;

MaterialIntegrator::MaterialIntegrator(
  ros::NodeHandle *node_handle, AxisAlignedGrid<MaterialBlend> const *grid,
  const std::string &modification_topic, const std::string &dug_points_topic,
  HandleBulkCallback handle_bulk_cb, ColorizerCallback colorizer_cb)
  : m_node_handle(node_handle), m_grid(grid),
    m_handle_bulk_cb(handle_bulk_cb), m_colorizer_cb(colorizer_cb)
{
  m_sub_modification_diff = m_node_handle->subscribe(modification_topic, 10,
    &MaterialIntegrator::onModification, this);
  m_dug_points_pub = m_node_handle
    ->advertise<sensor_msgs::PointCloud2>(dug_points_topic, 10);
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
    gzlog << "cv_bridge exception occurred while reading modification "
             "differential message: " << e.what() << std::endl;
    return;
  }

  const auto rows = diff_handle->image.rows;
  const auto cols = diff_handle->image.cols;
  const auto pixel_height = msg->height / rows;
  const auto pixel_width = msg->width / cols;

  // SAVED CODE for integration with ow_regolith
  // const auto pixel_area = pixel_height * pixel_height;
  // float volume_displaced = 0.0;

  MaterialBlend bulk_blend;
  pcl::PointCloud<pcl::PointXYZRGB> points;

  // The following code makes these assumptions:
  //   a) Each voxel the modification touches acquires the entirety of that
  //      voxel. In other words, volume intersections are not factored in.
  //   b) Each blend in the material grid is already normalized.
  //   c) All voxels are of the same volume.

  // TODO: make float-double usage uniform

  // estimate the total volume displaced using a Riemann sum over the image
  for (size_t i = 0; i != cols; ++i) {
    for (size_t j = 0; j != rows; ++j) {
      // change in height at pixel (x, y)
      auto dz = diff_handle->image.at<float>(i, j);
      if (dz < 0.0) {
        // SAVED CODE for integration with ow_regolith
        // const auto volume = diff_px * pixel_area;
        // compute location in grid
        const float x = (msg->position.x - msg->width / 2)
                          + static_cast<float>(i) / cols * msg->width;
        // +j and +y are in opposite directions, so flip signs
        const float y = (msg->position.y + msg->height / 2)
                          - static_cast<float>(j) / rows * msg->height;
        // starting layer position; next loop will iterate up to the original z
        auto z_result = result_handle->image.at<float>(i, j);
        auto box_min = GridPositionType(x, y, z_result);
        auto box_max = GridPositionType(x + pixel_width,
                                        y + pixel_height,
                                        z_result - dz); // dz is always negative
        // SAVED for debugging
        // gzlog << "box_min = (" << box_min.X() << "," << box_min.Y() << "," << box_min.Z() << ")\n";
        // gzlog << "box_max = (" << box_max.X() << "," << box_max.Y() << "," << box_max.Z() << ")\n";
        m_grid->runForEachInAxisAlignedBox(box_min, box_max,
          [&bulk_blend, &points]
          (MaterialBlend const &b, GridPositionType center) {
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
  }

  if (points.size() == 0) {
    return;
  }

  // normalize the bulk by the number of blends that were merged into it
  bulk_blend.normalize();

  Color bulk_color = m_colorizer_cb(bulk_blend);
  uint32_t temp_rgb = static_cast<uint32_t>(bulk_color.r) << 16
                    | static_cast<uint32_t>(bulk_color.g) << 8
                    | static_cast<uint32_t>(bulk_color.b);
  float bulk_rgb = *reinterpret_cast<float*>(&temp_rgb);
  for (auto &p : points) {
    p.rgb = bulk_rgb;
  }

  // SAVED for debugging
  // gzlog << "point count = " << points.size() << "\n";
  // gzlog << "points[0] = (" << points[0].x << "," << points[0].y << "," << points[0].z << ")\n";
  // gzlog << "points[0] color = (" << (uint)points[0].r << ","
  //                                << (uint)points[0].g << ","
  //                                << (uint)points[0].b << ")\n";

  // DEBUG
  gzlog << "number of merges = " << points.size() << std::endl;

  publishPointCloud(&m_dug_points_pub, points);

  m_handle_bulk_cb(bulk_blend);
}


// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef PCL_VISUALIZE_H
#define PCL_VISUALIZE_H

#include <vector>
#include <algorithm>

#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

void publish_points_as_cloud(ros::Publisher *pub,
    // const std::vector<pcl::PointXYZ> &points) {
    const std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> &points) {
  pcl::PointCloud<pcl::PointXYZ> msg;
  msg.header.frame_id = "world";
  pcl_conversions::toPCL(ros::Time::now(), msg.header.stamp);
  msg.height = 1;
  msg.width = points.size();
  // msg.points.resize(msg.points.size());
  // std::copy(msg.points.begin(), points.begin(), points.end());
  // msg.points = points;
  for (auto const &p : points)
    msg.points.push_back(p);

  sensor_msgs::PointCloud2 ros_msg;

  pcl::toROSMsg(msg, ros_msg);

  pub->publish(ros_msg);
}

#endif // PCL_VISUALIZE_H

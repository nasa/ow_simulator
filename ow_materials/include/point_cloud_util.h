// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef POINT_CLOUD_UTIL_H
#define POINT_CLOUD_UTIL_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

void publishPointCloud(ros::Publisher *pub,
                       pcl::PointCloud<pcl::PointXYZRGB> &cloud);

#endif // POINT_CLOUD_UTIL_H

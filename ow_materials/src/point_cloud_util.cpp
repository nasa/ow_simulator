// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "point_cloud_util.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"

void publishPointCloud(ros::Publisher *pub,
                       pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                       const std::string frame_id)
{
  cloud.header.frame_id = frame_id;
  pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  pub->publish(msg);
}

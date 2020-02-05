// OW power system ROS node - listen and publish.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__


//ROS
#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char* argv[]) {
  
  ros::init(argc,argvf,"power_system_node");
  ros::NodeHandle nhandle ("power_system_node");

  // ROS Loop. Note that once this loop starts,
  // this function (and node) is terminated with an interrupt.

  ros::Rate rate(1); // 1 Hz, tailor as needed
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO ("Power system node running");
  }

  return 0;
}

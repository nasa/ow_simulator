// OW power system ROS node - listen and publish.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__


//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>

int main(int argc, char* argv[]) {
  
  ros::init(argc,argv,"power_system_node");
  ros::NodeHandle n ("power_system_node");

  //Construct our State of Charge (SOC) publisher
  ros::Publisher SOC_pub = n.advertise<std_msgs::Float64>("state_of_charge",1000);

  // ROS Loop. Note that once this loop starts,
  // this function (and node) is terminated with an interrupt.
  ros::Rate rate(1); // 1 Hz, tailor as needed
  while (ros::ok()) {
    //individual soc_msg to be published by SOC_pub
    std_msgs::Float64 soc_msg;

    soc_msg.data=0.0; //zero for testing purposes, will be calculated later

    //publish current SOC
    SOC_pub.publish(soc_msg);
    
    ros::spinOnce();
    rate.sleep();
    ROS_INFO ("Power system node running");
  }

  return 0;
}

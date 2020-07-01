// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// OW power system ROS node - publishes 0 as a placeholder for once Battery models are linked.

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>

void computeSOC(const std_msgs::Float64& power){}

int main(int argc, char* argv[]) {
  
  ros::init(argc,argv,"power_system_node");
  ros::NodeHandle n ("power_system_node");

  //Construct our power draw listener
  //TODO: Check out latching
  ros::Subscriber power_listener = n.subscribe("power_draw", 1000, computeSOC);

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

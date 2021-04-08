#ifndef __POWER_SYSTEM_NODE_H__
#define __POWER_SYSTEM_NODE_H__

#include <chrono>
#include <ros/ros.h>

#include "PrognoserFactory.h"

class PowerSystemNode
{
public:
  PowerSystemNode();

  int Run(int argc, char* argv[]);

private:
  void powerCallback(const std_msgs::Float64::ConstPtr& msg);

private:
  ros::NodeHandle m_nh; // Node Handle Initialization
  ros::Publisher m_SOC_pub; // State of Charge Publisher
  ros::Publisher m_RUL_pub; // Remaining Useful Life Publisher
  ros::Publisher m_temp_bat_pub; // Battery Temperature Publisher
  ros::Subscriber m_mech_power_sub; // Mechanical Power Subscriber

  double m_battery_lifetime = 2738.0; // Estimate of battery lifetime (seconds)

  std::chrono::high_resolution_clock::time_point m_init_time; // Timestep initialization

  std::unique_ptr<PCOE::Prognoser> m_prognoser; // Prognoser initialization
};

#endif

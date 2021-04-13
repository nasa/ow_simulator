#ifndef __POWER_SYSTEM_NODE_H__
#define __POWER_SYSTEM_NODE_H__

#include <ros/ros.h>
#include <chrono>

#include "PrognoserFactory.h"
// #include "ow_faults/FaultInjector.h"
#include <ow_faults/SystemFaults.h>

class PowerSystemNode
{
public:
  void Run();

  int LOW_VOLTAGE = 1;
  int CAP_LOSS = 2;
  int THERMAL_FAULT = 4;

private:
  bool m_lowVoltageFault = false;
  bool m_capLossFault = false;
  bool m_thermalFault = false;
  
  void powerCallback(const std_msgs::Float64::ConstPtr& msg);
  void powerFaultsCallback(const ow_faults::SystemFaults::ConstPtr& msg);
  double checkForFaults(double originalValue);

  ros::NodeHandle m_nh;                          // Node Handle Initialization
  ros::Publisher m_state_of_charge_pub;          // State of Charge Publisher
  ros::Publisher m_remaining_useful_life_pub;    // Remaining Useful Life Publisher
  ros::Publisher m_battery_temperature_pub;      // Battery Temperature Publisher
  ros::Subscriber m_mechanical_power_sub;        // Mechanical Power Subscriber
  ros::Subscriber m_power_fault_sub;             // Faults Power Subscriber

  std::unique_ptr<PCOE::Prognoser> m_prognoser;  // Prognoser initialization

  std::chrono::time_point<std::chrono::system_clock> m_init_time;

  // declare set constants .. hardcoded for now.
  // TODO: read from config file
  static constexpr double m_initial_power = 0.0;        // This is probably always zero
  static constexpr double m_initial_temperature = 20.0; // 20.0 deg. C
  static constexpr double m_initial_voltage = 4.1;      // Volts
  static constexpr double m_min_temperature = 17.5;     // minimum temp = 17.5 deg. C
  static constexpr double m_max_temperature = 21.5;     // maximum temp = 21.5 deg. C
  static constexpr double m_battery_lifetime = 2738.0;  // Estimate of battery lifetime (seconds)
  static constexpr double m_base_voltage = 3.2;         // [V] estimate
  static constexpr double m_voltage_range = 0.1;        // [V]

  // The index use to access temperature information.
  // This might change to median SOC or RUL index or fixed percentile
  static constexpr int TEMPERATURE_INDEX = 1;
};

#endif

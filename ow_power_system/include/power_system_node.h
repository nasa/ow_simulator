#ifndef __POWER_SYSTEM_NODE_H__
#define __POWER_SYSTEM_NODE_H__

#include <vector>
#include <chrono>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <PrognoserFactory.h>

class PowerSystemNode
{
public:
  PowerSystemNode();
  bool Initialize();
  void Run();

private:
  std::vector<std::map<MessageId, Datum<double>>> loadPowerProfile(const std::string& filename);
  void jointStatesCb(const sensor_msgs::JointStateConstPtr& msg);
  double generateTemperatureEstimate();
  double generateVoltageEstimate();
  void injectFaults(double& power, double& temperature, double& voltage);
  std::map<PCOE::MessageId, PCOE::Datum<double>> composePrognoserData(double power, double voltage, double temperature);
  void powerCb(double electrical_power);

  ros::NodeHandle m_nh;                        // Node Handle Initialization
  ros::Publisher m_state_of_charge_pub;        // State of Charge Publisher
  ros::Publisher m_remaining_useful_life_pub;  // Remaining Useful Life Publisher
  ros::Publisher m_battery_temperature_pub;    // Battery Temperature Publisher
  ros::Subscriber m_joint_states_sub;          // Mechanical Power Subscriber

  static constexpr int m_moving_average_window = 10;
  std::vector<double> m_power_values;
  size_t m_power_values_index = 0;

  std::unique_ptr<PCOE::Prognoser> m_prognoser;  // Prognoser initialization

  std::chrono::time_point<std::chrono::system_clock> m_init_time;

  // main system configuration
  double m_initial_power = 0.0;         // This is probably always zero
  double m_initial_temperature = 20.0;  // 20.0 deg. C
  double m_initial_voltage = 4.1;       // Volts
  double m_min_temperature = 17.5;      // minimum temp = 17.5 deg. C
  double m_max_temperature = 21.5;      // maximum temp = 21.5 deg. C
  double m_battery_lifetime = 2738.0;   // Estimate of battery lifetime (seconds)
  double m_base_voltage = 3.2;          // [V] estimate
  double m_voltage_range = 0.1;         // [V]
  double m_efficiency = 0.9;            // default 90% efficiency

  std::mt19937 m_random_generator;  // Utilize a Mersenne Twister pesduo random generation
  std::uniform_real_distribution<double> m_temperature_dist;
};

#endif

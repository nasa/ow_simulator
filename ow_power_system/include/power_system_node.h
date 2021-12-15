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
  bool loadSystemConfig();
  std::vector<std::map<MessageId, Datum<double>>> loadPowerProfile(const std::string& filename);
  bool loadFaultPowerProfiles();
  bool initPrognoser();
  bool initTopics();
  void jointStatesCb(const sensor_msgs::JointStateConstPtr& msg);
  double generateTemperatureEstimate();
  double generateVoltageEstimate();
  void injectFault(const std::string& power_fault_name, bool& fault_activated,
                   const std::vector<std::map<PCOE::MessageId, PCOE::Datum<double>>>& sequence, size_t& sequence_index,
                   double& power, double& voltage, double& temperature);
  void injectFaults(double& power, double& temperature, double& voltage);
  std::map<PCOE::MessageId, PCOE::Datum<double>> composePrognoserData(double power, double voltage, double temperature);
  void parseEoD_Event(const ProgEvent& eod_event,
                      std_msgs::Float64& soc_msg, std_msgs::Int16& rul_msg, std_msgs::Float64& battery_temperature_msg);
  void powerCb(double electrical_power);

  ros::NodeHandle m_nh;                        // Node Handle Initialization
  ros::Publisher m_mechanical_power_raw_pub;   // Mechanical Power Raw
  ros::Publisher m_mechanical_power_avg_pub;   // Mechanical Power Averaged
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

  bool m_low_state_of_charge_power_failure_activated = false;
  std::vector<std::map<PCOE::MessageId, PCOE::Datum<double>>> m_low_state_of_charge_power_failure_sequence;
  size_t m_low_state_of_charge_power_failure_sequence_index = 0;

  bool m_instantaneous_capacity_loss_power_failure_activated = false;
  std::vector<std::map<PCOE::MessageId, PCOE::Datum<double>>> m_instantaneous_capacity_loss_power_failure_sequence;
  size_t m_instantaneous_capacity_loss_power_failure_sequence_index = 0;

  bool m_thermal_power_failure_activated = false;
  std::vector<std::map<PCOE::MessageId, PCOE::Datum<double>>> m_thermal_power_failure_sequence;
  size_t m_thermal_power_failure_sequence_index = 0;

  double m_power_node_processing_rate = 0.5;   // [Hz] hard code processing rate for now
  bool m_processing_power_batch = false;       // flag that indicates that the prognoser is handling current batch
  bool m_trigger_processing_new_power_batch = false;
  double m_unprocessed_mechanical_power = 0.0;
  double m_mechanical_power_to_be_processed = 0.0;
};

#endif

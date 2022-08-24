#ifndef __POWER_SYSTEM_NODE_H__
#define __POWER_SYSTEM_NODE_H__

#include <vector>
#include <map>
#include <chrono>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <PrognoserFactory.h>

using PrognoserMap = std::map<PCOE::MessageId, PCOE::Datum<double>>;
using PrognoserVector = std::vector<PrognoserMap>;

class PowerSystemNode
{
public:
  PowerSystemNode();
  bool Initialize();
  void Run();

private:
  bool loadSystemConfig();
  PrognoserVector loadPowerProfile(const std::string& filename, std::string custom_file);
  bool loadCustomFaultPowerProfile(std::string path, std::string custom_file);
  bool initPrognoser();
  bool initTopics();
  void jointStatesCb(const sensor_msgs::JointStateConstPtr& msg);
  double generateTemperatureEstimate();
  double generateVoltageEstimate();
  void injectFault(const std::string& power_fault_name,
                   bool& fault_activated,
                   double& power,
                   double& voltage,
                   double& temperature);
  void injectCustomFault(bool& fault_activated,
                         const PrognoserVector& sequence,
                         size_t& index,
                         double& wattage,
                         double& voltage,
                         double& temperature);
  void injectFaults(double& power, double& temperature, double& voltage);
  PrognoserMap composePrognoserData(double power,
                                    double voltage,
                                    double temperature);
  void parseEoD_Event(const ProgEvent& eod_event,
                      std_msgs::Float64& soc_msg,
                      std_msgs::Int16& rul_msg,
                      std_msgs::Float64& battery_temperature_msg);
  void runPrognoser(double electrical_power);

  ros::NodeHandle m_nh;                        // Node Handle Initialization
  ros::Publisher m_mechanical_power_raw_pub;   // Mechanical Power Raw
  ros::Publisher m_mechanical_power_avg_pub;   // Mechanical Power Averaged
  ros::Publisher m_state_of_charge_pub;        // State of Charge Publisher
  ros::Publisher m_remaining_useful_life_pub;  // Remaining Useful Life Publisher
  ros::Publisher m_battery_temperature_pub;    // Battery Temperature Publisher
  ros::Subscriber m_joint_states_sub;          // Mechanical Power Subscriber

  int m_moving_average_window = 25;
  std::vector<double> m_power_values;
  size_t m_power_values_index = 0;

  std::unique_ptr<PCOE::Prognoser> m_prognoser;  // Prognoser initialization

  std::chrono::time_point<std::chrono::system_clock> m_init_time;

  // Main system configuration: these values are overriden by values
  // in ../config/system.cfg.

  double m_initial_power = 0.0;         // This is probably always zero
  double m_initial_temperature = 20.0;  // 20.0 deg. C
  double m_initial_voltage = 4.1;       // Volts
  double m_min_temperature = 17.5;      // minimum temp = 17.5 deg. C
  double m_max_temperature = 21.5;      // maximum temp = 21.5 deg. C
  double m_battery_lifetime = 2738.0;   // Estimate of battery lifetime (seconds)
  double m_base_voltage = 3.2;          // [V] estimate
  double m_voltage_range = 0.1;         // [V]
  double m_efficiency = 0.9;            // default 90% efficiency
  double m_gsap_rate_hz = 0.5;          // GSAP's cycle time

  // Baseline value for power drawn by continuously-running systems.
  // This initial value is overriden by the system config.
  double m_baseline_wattage = 1.0;

  // HACK ALERT.  The prognoser produced erratic/erroneous output when
  // given too high a power input.  This made-up value protects
  // against this, but is a temporary hack until a circuit breaker
  // model is added to the power system, and/or the multi-pack battery
  // model is implemented and can handle any envisioned power draw.
  // This initial value is overriden by the system config.
  //
  double m_max_gsap_input_watts = 30;

  // Number of lines in power fault profiles to skip, in order to
  // synchonize the consumption of the profile with GSAP's cycle rate
  // (above).  This computation is left to the user for now, though
  // note that the default values are not expected to change as of
  // Release 9.  This initial value is overriden by the system config.
  int m_profile_increment = 2;

  // End main system configuration.

  // Utilize a Mersenne Twister pseudo-random generation.
  std::mt19937 m_random_generator;

  std::uniform_real_distribution<double> m_temperature_dist;
  
  bool m_high_power_draw_activated = false;
  bool m_custom_power_fault_activated = false;
  PrognoserVector m_custom_power_fault_sequence;
  size_t m_custom_power_fault_sequence_index = 0;

  // Flag that indicates that the prognoser is handling current batch.
  bool m_processing_power_batch = false;

  bool m_trigger_processing_new_power_batch = false;
  double m_unprocessed_mechanical_power = 0.0;
  double m_mechanical_power_to_be_processed = 0.0;
};

#endif

// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// This is the header file for the PowerSystemNode class, which handles
// the simulation of a single cell within a PowerSystemPack. It stores the
// data used as inputs to a GSAP asynchronous prognoser and generates the next
// input set (including any value modifications from faults or other functions) 
// to be sent to GSAP.

#ifndef __POWER_SYSTEM_NODE_H__
#define __POWER_SYSTEM_NODE_H__

#include <vector>
#include <map>
#include <chrono>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <owl_msgs/BatteryRemainingUsefulLife.h>
#include <owl_msgs/BatteryStateOfCharge.h>
#include <owl_msgs/BatteryTemperature.h>
#include <PrognoserFactory.h>

class PowerSystemNode
{
public:
  PowerSystemNode() = default;
  ~PowerSystemNode() = default;
  PowerSystemNode(const PowerSystemNode&) = default;
  PowerSystemNode& operator=(const PowerSystemNode&) = default;
  bool Initialize();
  void RunOnce();
  void GetPowerStats(double &time, double &power, double &volts, double &tmp);
  double GetRawMechanicalPower();
  double GetAvgMechanicalPower();
  void SetHighPowerDraw(double draw);
  void SetCustomPowerDraw(double draw);
  void SetCustomVoltageFault(double volts);
  void SetCustomTemperatureFault(double tmp);
private:
  bool loadSystemConfig();
  bool initCallback();
  void jointStatesCb(const sensor_msgs::JointStateConstPtr& msg);
  double generateTemperatureEstimate();
  double generateVoltageEstimate();
  void applyValueMods(double& power, double& voltage, double& temperature);

  void runPrognoser(double electrical_power);

  ros::NodeHandle m_nh;                        // Node Handle Initialization
  ros::Subscriber m_joint_states_sub;          // Mechanical Power Subscriber

  int m_moving_average_window = 25;
  std::vector<double> m_power_values;
  size_t m_power_values_index = 0;

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

  double m_current_timestamp = 0.0;

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

  // The three main statistics relating to battery model health:
  // Power, Voltage, & Temperature.
  // Used by the main function to get the health of all models.
  double m_wattage_estimate;
  double m_voltage_estimate;
  double m_temperature_estimate;

  // The stored values for modifying input to GSAP.
  double m_added_hpd = 0.0;
  double m_added_cpd = 0.0;
  double m_voltage_modifier = 0.0;
  double m_temperature_modifier = 0.0;

  // End main system configuration.

  // Utilize a Mersenne Twister pseudo-random generation.
  std::mt19937 m_random_generator;

  std::uniform_real_distribution<double> m_temperature_dist;

  // Flag that indicates that the prognoser is handling current batch.
  bool m_processing_power_batch = false;

  bool m_trigger_processing_new_power_batch = false;
  double m_unprocessed_mechanical_power = 0.0;
  double m_mechanical_power_to_be_processed = 0.0;

  double m_mechanical_power_raw = 0.0;
  double m_mechanical_power_avg = 0.0;
};

#endif

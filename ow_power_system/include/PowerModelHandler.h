// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// This is the header file for the PowerModelHandler class, which handles
// the simulation of a single model within a PowerSystemNode. It stores the
// data used as inputs to a GSAP asynchronous prognoser and generates the next
// input set (including any value modifications from faults or other functions) 
// to be sent to GSAP.

#ifndef __POWER_MODEL_HANDLER_H__
#define __POWER_MODEL_HANDLER_H__

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

// Struct that contains the information used for EoD predictions.
struct ModelInfo {
  double timestamp;
  double wattage;
  double voltage;
  double temperature;
};

class PowerModelHandler
{
public:
  PowerModelHandler() = default;
  ~PowerModelHandler() = default;
  PowerModelHandler(const PowerModelHandler&) = delete;
  PowerModelHandler& operator=(const PowerModelHandler&) = delete;
  bool Initialize();
  double RunOnce();
  void GetPowerStats(ModelInfo &model);
  void applyMechanicalPower(double mechanical_power);
  void SetHighPowerDraw(double draw);
  void SetCustomPowerDraw(double draw);
  void SetCustomVoltageFault(double volts);
  void SetCustomTemperatureFault(double tmp);
private:
  bool loadSystemConfig();
  double generateTemperatureEstimate();
  double generateVoltageEstimate();
  void applyValueMods(double& power, double& voltage, double& temperature);

  double runPrognoser(double electrical_power);

  std::chrono::time_point<std::chrono::system_clock> m_init_time;

  // Main system configuration: these values are overriden by values
  // in ../config/system.cfg.

  double m_min_temperature;     // minimum temp = 17.5 deg. C
  double m_max_temperature;     // maximum temp = 21.5 deg. C
  double m_battery_lifetime;    // Estimate of battery lifetime (seconds)
  double m_base_voltage;        // [V] estimate
  double m_voltage_range;       // [V]
  double m_efficiency;          // default 90% efficiency
  double m_baseline_wattage;    // Base power drawn by continuously-running systems.

  double m_current_timestamp = 0.0;

  // HACK ALERT.  The prognoser produced erratic/erroneous output when
  // given too high a power input.  This made-up value protects
  // against this, but is a temporary hack until a circuit breaker
  // model is added to the power system, and/or the multi-cell battery
  // model is implemented and can handle any envisioned power draw.
  // This initial value is overriden by the system config.
  //
  double m_max_gsap_input_watts = 20;

  // The expected time interval between publications. It is essentially the
  // rate at which the main loop executes. It is overridden by the value in
  // system.cfg.
  int m_time_interval = 2;

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
};

#endif

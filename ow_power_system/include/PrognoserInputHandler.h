// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// This is the header file for the PrognoserInputHandler class, which controls
// the inputs to a single prognoser within a PowerSystemNode. It stores the
// data used as inputs to a GSAP asynchronous prognoser and generates the next
// input set (including any value modifications from faults or other functions)
// to be sent to GSAP.

#ifndef __PROGNOSER_INPUT_HANDLER_H__
#define __PROGNOSER_INPUT_HANDLER_H__

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

// Struct that contains the input information used for EoD predictions.
struct InputInfo {
  double timestamp;
  double wattage;
  double voltage;
  double temperature;
};

class PrognoserInputHandler
{
public:
  PrognoserInputHandler() = default;
  ~PrognoserInputHandler() = default;
  PrognoserInputHandler(const PrognoserInputHandler&) = delete;
  PrognoserInputHandler& operator=(const PrognoserInputHandler&) = delete;
  bool initialize();
  bool cyclePrognoserInputs();
  void getPowerStats(InputInfo &inputs);
  void setPowerLoad(double power_load);
  void setHighPowerDraw(double draw);
  void setCustomPowerDraw(double draw);
private:
  bool loadSystemConfig();
  double generateTemperatureEstimate();
  double generateVoltageEstimate();
  void applyValueMods(double& power, double& voltage, double& temperature);

  double m_init_time = 0;

  // Main system configuration: these values are overriden by values
  // in ../config/system.cfg.

  double m_min_temperature;     // minimum temp = 17.5 deg. C
  double m_max_temperature;     // maximum temp = 21.5 deg. C
  double m_battery_lifetime;    // Estimate of battery lifetime (seconds)
  double m_base_voltage;        // [V] estimate
  double m_voltage_range;       // [V]
  double m_baseline_wattage;    // Base power drawn by continuously-running systems.
  double m_current_timestamp = 0.0;

  // HACK ALERT.  The prognoser produced erratic/erroneous output when
  // given too high a power input.  This made-up value protects
  // against this, but is a temporary hack until a circuit breaker
  // model is added to the power system, and/or the multi-cell battery
  // model is implemented and can handle any envisioned power draw.
  double m_max_gsap_input_watts;

  // The expected time interval between publications. It is essentially the
  // inverse of the rate at which the main loop executes.
  double m_time_interval;

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

  double m_power_load = 0.0;
};

#endif

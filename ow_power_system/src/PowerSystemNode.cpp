// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// See PowerSystemNode.h for a summary of the purpose of this file.

#include <numeric>
#include <fstream>
#include <math.h>
#include <algorithm>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <ow_lander/lander_joints.h>
#include "PowerSystemNode.h"

using namespace std;
using namespace std::chrono;
using namespace std_msgs;

/*
 * Called on creating a PowerSystemNode object. Sets up class variables
 * and initializes various other things.
 */
bool PowerSystemNode::Initialize()
{
  if (!loadSystemConfig()) {
    ROS_ERROR("Failed to load PowerSystemNode system config.");
    return false;
  }

  m_init_time = system_clock::now(); // Taken from initPrognoser()

  return true;
}

/*
 * Called from Initialize(). Reads the system.cfg file within ow_power_system
 * and stores its information in relevant class variables.
 */
bool PowerSystemNode::loadSystemConfig()
{
  auto system_config_path = ros::package::getPath("ow_power_system")
    + "/config/system.cfg";
  auto system_config = ConfigMap(system_config_path);
  m_base_voltage = system_config.getDouble("base_voltage");
  m_voltage_range = system_config.getDouble("voltage_range");
  m_min_temperature = system_config.getDouble("min_temperature");
  m_max_temperature = system_config.getDouble("max_temperature");
  m_battery_lifetime = system_config.getDouble("battery_lifetime");
  m_efficiency = system_config.getDouble("efficiency");
  m_temperature_dist = uniform_real_distribution<double>(m_min_temperature,
                                                         m_max_temperature);
  m_baseline_wattage = system_config.getDouble("baseline_power");
  m_max_gsap_input_watts = system_config.getDouble("max_gsap_power_input");
  m_time_interval = system_config.getInt32("time_interval");

  return true;
}

void PowerSystemNode::applyMechanicalPower(double mechanical_power)
{
  m_unprocessed_mechanical_power = mechanical_power;

  if (!m_processing_power_batch)
  {
    m_mechanical_power_to_be_processed = m_unprocessed_mechanical_power;
    m_unprocessed_mechanical_power = 0.0;   // reset the accumulator
    m_trigger_processing_new_power_batch = true;
  }
}

/*
 * Returns a random value between the lower and upper bounds of the random
 * generator, to mimic a sensor's temperature measurement.
 */
double PowerSystemNode::generateTemperatureEstimate()
{
  return m_temperature_dist(m_random_generator);
}

/*
 * Calculates a voltage value with some variance to be used as a data input. Voltage
 * values will slowly drop over time.
 */
double PowerSystemNode::generateVoltageEstimate()
{
  // Create voltage estimate with pseudorandom noise generator - needs to decrease over time
  double timelapse = duration<double>(system_clock::now() - m_init_time).count();               // [s]
  double min_V = m_base_voltage + (m_battery_lifetime - timelapse) / m_battery_lifetime * 0.8;  // [V]
  double max_V = min_V + m_voltage_range;                                                       // [V]

  // If voltage limits dip below baseline, set to baseline values
  if (min_V < m_base_voltage)
  {
    min_V = m_base_voltage;
  }

  // Voltage estimate based on pseudorandom noise and moving range
  uniform_real_distribution<double> voltage_dist(min_V, max_V);
  return voltage_dist(m_random_generator);
}

/*
 * Applies any modifiers to the input power/voltage/temperature values.
 * Currently the only source of these modifiers is power faults, but could be
 * expanded in the future.
 */
void PowerSystemNode::applyValueMods(double& power, double& voltage, double& temperature)
{
  power += m_added_hpd;
  power += m_added_cpd;

  voltage += m_voltage_modifier;

  temperature += m_temperature_modifier;

  // Reset the fault values in anticipation of the next cycle.
  // If the next cycle does not modify them, injection is assumed complete.
  m_added_hpd = 0.0;
  m_added_cpd = 0.0;
  m_voltage_modifier = 0.0;
  m_temperature_modifier = 0.0;
}

/*
 * Compiles the input values of power/voltage/temperature to be sent into GSAP
 * by the PowerSystemPack each cycle.
 */
double PowerSystemNode::runPrognoser(double electrical_power)
{
  // Temperature estimate based on pseudorandom noise and fixed range
  m_current_timestamp += m_time_interval;
  m_temperature_estimate = generateTemperatureEstimate();
  m_voltage_estimate = generateVoltageEstimate();
  m_wattage_estimate = electrical_power + m_baseline_wattage;
  applyValueMods(m_wattage_estimate, m_voltage_estimate, m_temperature_estimate);

  if (m_wattage_estimate > m_max_gsap_input_watts) {
    // Excessive power draw computed. Return the high draw amount to warn user, 
    // but cap the draw at the max allowed value.
    double temp = m_wattage_estimate;
    m_wattage_estimate = m_max_gsap_input_watts;
    return temp;
  }
  return -1;
}

/*
 * Function called by PowerSystemPack to update input power/voltage/temp.
 */
double PowerSystemNode::RunOnce()
{
  double excessiveDraw = -1;
  if (m_trigger_processing_new_power_batch)
  {
    m_trigger_processing_new_power_batch = false;
    m_processing_power_batch = true;
    excessiveDraw = runPrognoser(m_mechanical_power_to_be_processed / m_efficiency);
    m_processing_power_batch = false;
  }
  // Returns -1 if no excessive draw was calculated, else returns
  // the excessive draw amount for warning statement in the pack.
  return excessiveDraw;
}

/*
 * Getter function that obtains all the input data currently stored in this
 * PowerSystemNode.
 */
void PowerSystemNode::GetPowerStats(double &time, double &power, double &volts, double &tmp)
{
  time = m_current_timestamp;
  power = m_wattage_estimate;
  volts = m_voltage_estimate;
  tmp = m_temperature_estimate;
}

void PowerSystemNode::SetHighPowerDraw(double draw)
{
  m_added_hpd = draw;
}

void PowerSystemNode::SetCustomPowerDraw(double draw)
{
  m_added_cpd = draw;
}

void PowerSystemNode::SetCustomVoltageFault(double volts)
{
  m_voltage_modifier = volts;
}

void PowerSystemNode::SetCustomTemperatureFault(double tmp)
{
  m_temperature_modifier = tmp;
}
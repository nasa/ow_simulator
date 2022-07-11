// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <numeric>
#include <fstream>
#include <math.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <ow_lander/lander_joints.h>
#include "power_system_node.h"

using namespace std;
using namespace std::chrono;
using namespace std_msgs;

const string FAULT_NAME_HPD           = "high_power_draw";
const string FAULT_NAME_HPD_ACTIVATE  = "activate_high_power_draw";

// The index use to access temperature information.
// This might change to median SOC or RUL index or fixed percentile.
//
static constexpr int TEMPERATURE_INDEX = 1;

PowerSystemNode::PowerSystemNode()
{ }

bool PowerSystemNode::Initialize()
{
  if (!loadSystemConfig()) {
    ROS_ERROR("Failed to load ow_power_system system config.");
    return false;
  }

  m_power_values.resize(m_moving_average_window);
  std::fill(m_power_values.begin(), m_power_values.end(), 0.0);

  if (!loadFaultPowerProfiles()) {
    ROS_ERROR("Failed to load power fault profiles.");
    return false;
  }

  if (!initPrognoser()) {
    ROS_ERROR("Failed to initialize power system prognoser.");
    return false;
  }

  if (!initTopics()) {
    ROS_ERROR("Failed to initialize power system topics.");
    return false;
  }

  return true;
}

bool PowerSystemNode::loadSystemConfig()
{
  auto system_config_path = ros::package::getPath("ow_power_system")
    + "/config/system.cfg";
  auto system_config = ConfigMap(system_config_path);
  m_initial_power = system_config.getDouble("initial_power");
  m_initial_voltage = system_config.getDouble("initial_voltage");
  m_initial_temperature = system_config.getDouble("initial_temperature");
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
  m_gsap_rate_hz = system_config.getDouble("gsap_rate");
  m_profile_increment = system_config.getInt32("profile_increment");
  m_moving_average_window = system_config.getInt32("power_average_size");
  return true;
}

PrognoserVector PowerSystemNode::loadPowerProfile(const string& filename)
{
  ifstream file(filename);
  if (file.fail())
  {
    cerr << "Unable to open data file" << filename << endl;
  }
  // Skip header line
  file.ignore(numeric_limits<streamsize>::max(), '\n');

  auto now = system_clock::now();

  PrognoserVector result;
  while (file.good())
  {
    PrognoserMap data;
    string line;
    getline(file, line);
    if (line.empty())
      continue;

    stringstream line_stream(line);
    string cell;
    getline(line_stream, cell, ',');
    double file_time = stod(cell);
    auto timestamp = now + milliseconds(static_cast<unsigned>(file_time * 1000));

    getline(line_stream, cell, ',');
    Datum<double> power(stod(cell));
    power.setTime(timestamp);

    getline(line_stream, cell, ',');
    Datum<double> temperature(stod(cell));
    temperature.setTime(timestamp);

    getline(line_stream, cell, ',');
    Datum<double> voltage(stod(cell));
    voltage.setTime(timestamp);

    data.insert({ MessageId::Watts, power });
    data.insert({ MessageId::Centigrade, temperature });
    data.insert({ MessageId::Volts, voltage });

    result.push_back(data);
  }
  return result;
}

bool PowerSystemNode::loadFaultPowerProfiles()
{
  string path;
  
  // DEPRECATED
  // This code is used to get the path to the CSV files containing data for
  // predetermined power faults. The current high power draw fault does not use this.
  //path = ros::package::getPath("ow_power_system") + "/data/YOUR_FAULT_HERE.csv";
  //m_high_power_draw_power_failure_sequence = loadPowerProfile(path);

  return true;
}

bool PowerSystemNode::initPrognoser()
{
  // Create a configuration from a file
  auto prognoser_config_path = ros::package::getPath("ow_power_system") + "/config/prognoser.cfg";
  ConfigMap prognoser_config(prognoser_config_path);

  // Contruct a new prognoser using the prognoser factory. The prognoser
  // will automatically construct an appropriate model, observer and predictor
  // based on the values specified in the config.
  m_prognoser = PrognoserFactory::instance().Create("ModelBasedPrognoser", prognoser_config);

  if (m_prognoser == nullptr)
    return false;

  // Initialize the GSAP prognoser
  auto init_data = composePrognoserData(m_initial_power, m_initial_voltage, m_initial_temperature);
  m_prognoser->step(init_data);

  m_init_time = system_clock::now();

  return true;
}

bool PowerSystemNode::initTopics()
{
  // Construct the PowerSystemNode publishers
  m_mechanical_power_raw_pub = m_nh.advertise<Float64>("mechanical_power/raw", 1);
  m_mechanical_power_avg_pub = m_nh.advertise<Float64>("mechanical_power/average", 1);
  m_state_of_charge_pub = m_nh.advertise<Float64>("power_system_node/state_of_charge", 1);
  m_remaining_useful_life_pub = m_nh.advertise<Int16>("power_system_node/remaining_useful_life", 1);
  m_battery_temperature_pub = m_nh.advertise<Float64>("power_system_node/battery_temperature", 1);
  // Finally subscribe to the joint_states to estimate the mechanical power
  m_joint_states_sub = m_nh.subscribe("/joint_states", 1, &PowerSystemNode::jointStatesCb, this);
  return true;
}

void PowerSystemNode::jointStatesCb(const sensor_msgs::JointStateConstPtr& msg)
{
  auto power_watts = 0.0;  // This includes the arm + antenna
  for (auto i = 0; i < ow_lander::NUM_JOINTS; ++i)
    power_watts += fabs(msg->velocity[i] * msg->effort[i]);

  m_power_values[++m_power_values_index % m_power_values.size()] = power_watts;   // [W]
  auto mean_mechanical_power =
      accumulate(begin(m_power_values), end(m_power_values), 0.0) / m_power_values.size();

  Float64 mechanical_power_raw_msg, mechanical_power_avg_msg;
  mechanical_power_raw_msg.data = power_watts;
  mechanical_power_avg_msg.data = mean_mechanical_power;
  m_mechanical_power_raw_pub.publish(mechanical_power_raw_msg);
  m_mechanical_power_avg_pub.publish(mechanical_power_avg_msg);

  m_unprocessed_mechanical_power = mean_mechanical_power;

  if (!m_processing_power_batch)
  {
    m_mechanical_power_to_be_processed = m_unprocessed_mechanical_power;
    m_unprocessed_mechanical_power = 0.0;   // reset the accumulator
    m_trigger_processing_new_power_batch = true;
  }
}

double PowerSystemNode::generateTemperatureEstimate()
{
  return m_temperature_dist(m_random_generator);
}

double PowerSystemNode::generateVoltageEstimate()
{
  // Create voltage estimate with pseudorandom noise generator - needs to decrease over time
  double timelapse = duration<double>(system_clock::now() - m_init_time).count();               // [s]
  double min_V = m_base_voltage + (m_battery_lifetime - timelapse) / m_battery_lifetime * 0.8;  // [V]
  double max_V = min_V + m_voltage_range;                                                       // [V]

  // If voltage limits dip below baseline, set to baseline values
  if (min_V < m_base_voltage)
    min_V = m_base_voltage;

  // Voltage estimate based on pseudorandom noise and moving range
  uniform_real_distribution<double> voltage_dist(min_V, max_V);
  return voltage_dist(m_random_generator);
}

// NOTE: To use the old CSV format for power faults,
//       the two additional parameters here must be uncommented.
//       If the old CSV format is completely removed, these parameters
//       can also be deleted. Will also require updating header file.
void PowerSystemNode::injectFault (const string& fault_name,
                                   bool& fault_activated,
                                   //const PrognoserVector& sequence,
                                   //size_t& index,
                                   double& wattage,
                                   double& voltage,
                                   double& temperature)
{
  bool fault_enabled = false;
  double hpd_wattage = 0.0;

  // Do nothing unless the specified fault has been injected.
  if (! ros::param::getCached("/faults/" + fault_name, fault_enabled)) {
    return;
  }

  if (!fault_activated && fault_enabled)
  {
    ROS_INFO_STREAM(fault_name << " activated!");
    // DEPRECATED
    //index = 0;
    fault_activated = true;
  }
  else if (fault_activated && !fault_enabled)
  {
    ROS_INFO_STREAM(fault_name << " de-activated!");
    fault_activated = false;
  }

  if (fault_activated && fault_enabled)
  {
    // If the current fault being utilized is high_power_draw,
    // simply update wattage based on the current value of the HPD slider.
    if (fault_name == FAULT_NAME_HPD_ACTIVATE)
    {
      ros::param::getCached("/faults/" + FAULT_NAME_HPD, hpd_wattage);
      wattage += hpd_wattage;
    } else
    {
      // DEPRECATED
      // This code was used to inject the values from the CSV-defined power faults
      // when they are activated. The CSV fault system is not currently used by
      // the high power draw fault. If support is removed, this code can also be
      // fully removed.

      /*
      // TODO: Unspecified how to handle end of fault profile, which is
      // unlikely.  For now, reuse the last entry.
      if (index + m_profile_increment >= sequence.size()) {
        ROS_WARN_STREAM_ONCE
          (fault_name << ": reached end of fault profile, reusing last entry.");
        // Probably unneeded, but makes index explicit.
        index = sequence.size() - 1;
      }
      else index += m_profile_increment;

      auto data = sequence[index];
      wattage += data[MessageId::Watts];
      voltage += data[MessageId::Volts];
      temperature += data[MessageId::Centigrade];
      */
    }
  }
}

void PowerSystemNode::injectFaults(double& power,
				   double& voltage,
				   double& temperature)
{
  // NOTE: To use the old CSV fault format,
  // two additional parameters must be uncommented here
  // and declared in the header file.
  injectFault(FAULT_NAME_HPD_ACTIVATE,
        m_high_power_draw_power_failure_activated,
        //m_high_power_draw_power_failure_sequence,
        //m_high_power_draw_power_failure_sequence_index,
        power, voltage, temperature);
}

PrognoserMap
PowerSystemNode::composePrognoserData(double power,
				      double voltage,
				      double temperature)
{
  return PrognoserMap {
    { MessageId::Watts, Datum<double>{ power } },
    { MessageId::Volts, Datum<double>{ voltage } },
    { MessageId::Centigrade, Datum<double>{ temperature } }
  };
}

void PowerSystemNode::parseEoD_Event(const ProgEvent& eod_event,
				     Float64& soc_msg,
				     Int16& rul_msg,
				     Float64& battery_temperature_msg)
{
  // The time of event is a `UData` structure, which represents a data
  // point while maintaining uncertainty. For the MonteCarlo predictor
  // used by this example, the uncertainty is captured by storing the
  // result of each particle used in the prediction.
  UData eod_time = eod_event.getTOE();
  if (eod_time.uncertainty() != UType::Samples)
  {
    // Log warning and don't update the last value
    ROS_WARN_NAMED("power_system_node", "Unexpected uncertainty type for EoD prediction");
    return;
  }

  // valid prediction
  // Determine the median RUL.
  auto samplesRUL = eod_time.getVec();
  sort(samplesRUL.begin(), samplesRUL.end());
  double eod_median = samplesRUL.at(samplesRUL.size() / 2);
  auto now = MessageClock::now();
  auto now_s = duration_cast<chrono::seconds>(now.time_since_epoch());
  double rul_median = eod_median - now_s.count();
  rul_msg.data = rul_median;

  // Determine the median SOC.
  UData currentSOC = eod_event.getState()[0];
  auto samplesSOC = currentSOC.getVec();
  sort(samplesSOC.begin(), samplesSOC.end());
  double soc_median = samplesSOC.at(samplesSOC.size() / 2);
  soc_msg.data = soc_median;

  // Determine the Battery Temperature
  auto stateSamples = eod_event.getSystemState()[0];
  vector<double> state;
  for (auto sample : stateSamples)
    state.push_back(sample[0]);

  auto& model = dynamic_cast<ModelBasedPrognoser*>(m_prognoser.get())->getModel();
  auto model_output = model.outputEqn(now_s.count(), static_cast<PrognosticsModel::state_type>(state));
  battery_temperature_msg.data = model_output[TEMPERATURE_INDEX];
}

void PowerSystemNode::runPrognoser(double electrical_power)
{
  // Temperature estimate based on pseudorandom noise and fixed range
  double temperature_estimate = generateTemperatureEstimate();
  double voltage_estimate = generateVoltageEstimate();
  double adjusted_wattage = electrical_power + m_baseline_wattage;
  injectFaults(adjusted_wattage, voltage_estimate, temperature_estimate);

  if (adjusted_wattage > m_max_gsap_input_watts) {
    ROS_WARN_STREAM("Power system node computed excessive power input for GSAP, "
                    << adjusted_wattage << "W. Capping GSAP input at "
                    << m_max_gsap_input_watts << "W.");
      adjusted_wattage = m_max_gsap_input_watts;
  }

  auto current_data = composePrognoserData(adjusted_wattage,
                                           voltage_estimate,
                                           temperature_estimate);
  auto prediction = m_prognoser->step(current_data);

  // Individual msgs to be published
  Float64 soc_msg;
  Int16 rul_msg;
  Float64 battery_temperature_msg;

  auto& eod_events = prediction.getEvents();
  if (!eod_events.empty())
  {
    auto eod_event = eod_events.front();
    parseEoD_Event(eod_event, soc_msg, rul_msg, battery_temperature_msg);
  }

  // publish current SOC, RUL, and battery temperature
  m_state_of_charge_pub.publish(soc_msg);
  m_remaining_useful_life_pub.publish(rul_msg);
  m_battery_temperature_pub.publish(battery_temperature_msg);
}

void PowerSystemNode::Run()
{
  ROS_INFO("Power system node running.");

  // For simplicity, we run the power node at the same rate as GSAP.
  ros::Rate rate(m_gsap_rate_hz);

  while (ros::ok())
  {
    ros::spinOnce();

    if (m_trigger_processing_new_power_batch)
    {
      m_trigger_processing_new_power_batch = false;
      m_processing_power_batch = true;
      runPrognoser(m_mechanical_power_to_be_processed / m_efficiency);
      m_processing_power_batch = false;
    }

    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "power_system_node");
  PowerSystemNode psn;
  if (!psn.Initialize())
  {
    ROS_ERROR("Power system node failed to initialize");
    return -1;
  }
  psn.Run();
  return 0;
}

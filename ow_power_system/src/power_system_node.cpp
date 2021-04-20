// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <numeric>
#include <fstream>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <ow_lander/lander_joints.h>
#include "power_system_node.h"

using namespace std;
using namespace std::chrono;

// The index use to access temperature information.
// This might change to median SOC or RUL index or fixed percentile
static constexpr int TEMPERATURE_INDEX = 1;

PowerSystemNode::PowerSystemNode() :
m_power_values(m_moving_average_window, 0)
{
}

bool PowerSystemNode::Initialize()
{
  auto system_config_path = ros::package::getPath("ow_power_system") + "/config/system.cfg";
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

  m_temperature_dist = uniform_real_distribution<double>(m_min_temperature, m_max_temperature);

  return true;
}

vector<map<MessageId, Datum<double>>> PowerSystemNode::loadPowerProfile(const string& filename)
{
    ifstream file(filename);
    if (file.fail()) {
        cerr << "Unable to open data file" << endl;
    }
    // Skip header line
    file.ignore(numeric_limits<streamsize>::max(), '\n');

    auto now = system_clock::now();

    vector<map<MessageId, Datum<double>>> result;
    while (file.good()) {
        map<MessageId, Datum<double>> data;
        string line;
        getline(file, line);
        if (line.empty()) {
            continue;
        }
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

        data.insert({MessageId::Watts, power});
        data.insert({MessageId::Centigrade, temperature});
        data.insert({MessageId::Volts, voltage});

        result.push_back(data);
    }
    return result;
}

void PowerSystemNode::jointStatesCb(const sensor_msgs::JointStateConstPtr& msg)
{
  auto power_watts = 0;  // This includes the arm + antenna
  for (auto i = 0; i < ow_lander::NUM_JOINTS; ++i)
    power_watts = msg->velocity[i] * msg->effort[i];

  m_power_values[++m_power_values_index % m_power_values.size()] = (power_watts);
  auto mean_mechanical_power =
      accumulate(begin(m_power_values), end(m_power_values), 0.0) / m_power_values.size();  // [W]

  powerCb(mean_mechanical_power / m_efficiency);
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
  uniform_real_distribution<double> m_voltage_dist(min_V, max_V);
  return m_voltage_dist(m_random_generator);
}

void PowerSystemNode::injectFaults(double& power, double& voltage, double& temperature)
{
  bool low_state_of_charge_power_failure;
  ros::param::param("/faults/low_state_of_charge_power_failure", low_state_of_charge_power_failure, false);
  if (low_state_of_charge_power_failure)
    ROS_INFO("low_state_of_charge_power_failure active!");

  bool instantaneous_capacity_loss_power_failure;
  ros::param::param("/faults/instantaneous_capacity_loss_power_failure", instantaneous_capacity_loss_power_failure,
                    false);
  if (instantaneous_capacity_loss_power_failure)
    ROS_INFO("instantaneous_capacity_loss_power_failure active!");

  bool thermal_power_failure;
  ros::param::param("/faults/thermal_power_failure", thermal_power_failure, false);
  if (thermal_power_failure)
    ROS_INFO("thermal_power_failure active!");
}

map<MessageId, Datum<double>> PowerSystemNode::composePrognoserData(double power, double voltage, double temperature)
{
  return map<MessageId, Datum<double>>{ { MessageId::Watts, Datum<double>{ power } },
                                        { MessageId::Volts, Datum<double>{ voltage } },
                                        { MessageId::Centigrade, Datum<double>{ temperature } } };
}

void PowerSystemNode::powerCb(double electrical_power)
{
  // Temperature estimate based on pseudorandom noise and fixed range
  double voltage_estimate = generateVoltageEstimate();
  double temperature_estimate = generateTemperatureEstimate();
  injectFaults(electrical_power, voltage_estimate, temperature_estimate);
  auto current_data = composePrognoserData(electrical_power, voltage_estimate, temperature_estimate);
  auto prediction = m_prognoser->step(current_data);

  // Individual msgs to be published
  std_msgs::Float64 soc_msg;
  std_msgs::Int16 rul_msg;
  std_msgs::Float64 battery_temperature_msg;

  // Get the event for battery EoD.
  auto eod_event = prediction.getEvents().front();
  // The time of event is a `UData` structure, which represents a data
  // point while maintaining uncertainty. For the MonteCarlo predictor
  // used by this example, the uncertainty is captured by storing the
  // result of each particle used in the prediction.
  UData eod_time = eod_event.getTOE();
  if (eod_time.uncertainty() != UType::Samples)
  {
    // Log warning and don't update the last value
    ROS_WARN_NAMED("power_system_node", "Unexpected uncertainty type for EoD prediction");
  }
  else  // valid prediction
  {
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

  // publish current SOC, RUL, and battery temperature
  m_state_of_charge_pub.publish(soc_msg);
  m_remaining_useful_life_pub.publish(rul_msg);
  m_battery_temperature_pub.publish(battery_temperature_msg);
}

void PowerSystemNode::Run()
{
  // Create a configuration from a file
  string prognoser_config_path = ros::package::getPath("ow_power_system") + "/config/prognoser.cfg";
  ConfigMap prognoser_config(prognoser_config_path);

  // Contruct a new prognoser using the prognoser factory. The prognoser
  // will automatically construct an appropriate model, observer and predictor
  // based on the values specified in the config.
  m_prognoser = PrognoserFactory::instance().Create("ModelBasedPrognoser", prognoser_config);

  // Initialize the GSAP prognoser
  auto init_data = composePrognoserData(m_initial_power, m_initial_voltage, m_initial_temperature);
  m_prognoser->step(init_data);

  this->m_init_time = system_clock::now();

  // Construct the PowerSystemNode publishers
  m_state_of_charge_pub = m_nh.advertise<std_msgs::Float64>("power_system_node/state_of_charge", 1);
  m_remaining_useful_life_pub = m_nh.advertise<std_msgs::Int16>("power_system_node/remaining_useful_life", 1);
  m_battery_temperature_pub = m_nh.advertise<std_msgs::Float64>("power_system_node/battery_temperature", 1);

  // Finally subscribe to the joint_states to estimate the mechanical power
  m_joint_states_sub = m_nh.subscribe("/joint_states", 1, &PowerSystemNode::jointStatesCb, this);

  ROS_INFO("Power system node running");
  ros::spin();
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
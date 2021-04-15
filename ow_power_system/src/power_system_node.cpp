// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <numeric>
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
m_temperature_dist(m_min_temperature, m_max_temperature)
{
}

void PowerSystemNode::jointStatesCb(const sensor_msgs::JointStateConstPtr& msg)
{
  auto power_watts = 0; // This includes the arm + antenna
  for (auto i = 0; i < ow_lander::NUM_JOINTS; ++i)
    power_watts = msg->velocity[i] * msg->effort[i];

  m_power_values.push_back(power_watts);
  if (m_power_values.size() > m_moving_average_window)
    m_power_values.pop_front(); // remove the oldest extra item
  
  auto mean_mechanical_power =
    accumulate(begin(m_power_values), end(m_power_values), 0.0) / m_power_values.size();    // [W]

  powerCb(mean_mechanical_power / m_efficiency);
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
  double voltage_estimate = m_voltage_dist(m_random_generator);
}

void PowerSystemNode::powerCb(double elecrtical_power)
{
  // Temperature estimate based on pseudorandom noise and fixed range
  double temperature_estimate = m_temperature_dist(m_random_generator);
  double voltage_estimate = generateVoltageEstimate();

  // Initialize the GSAP prognoser
  auto current_data = map<MessageId, Datum<double>>{ { MessageId::Watts, Datum<double>{ elecrtical_power } },
                                                     { MessageId::Centigrade, Datum<double>{ temperature_estimate } },
                                                     { MessageId::Volts, Datum<double>{ voltage_estimate } } };

  // Get a new prediction
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
  string config_path = ros::package::getPath("ow_power_system") + "/config/example.cfg";
  ConfigMap config(config_path);

  // Initialize the GSAP prognoser
  auto init_data = map<MessageId, Datum<double>>{ { MessageId::Watts, Datum<double>{ m_initial_power } },
                                                  { MessageId::Centigrade, Datum<double>{ m_initial_temperature } },
                                                  { MessageId::Volts, Datum<double>{ m_initial_voltage } } };

  // Contruct a new prognoser using the prognoser factory. The prognoser
  // will automatically construct an appropriate model, observer and predictor
  // based on the values specified in the config.
  m_prognoser = PrognoserFactory::instance().Create("ModelBasedPrognoser", config);
  m_prognoser->step(init_data);

  this->m_init_time = system_clock::now();

  // Construct the PowerSystemNode publishers
  m_state_of_charge_pub = m_nh.advertise<std_msgs::Float64>("power_system_node/state_of_charge", 1);
  m_remaining_useful_life_pub = m_nh.advertise<std_msgs::Int16>("power_system_node/remaining_useful_life", 1);
  m_battery_temperature_pub = m_nh.advertise<std_msgs::Float64>("power_system_node/battery_temperature", 1);

  // Finally subscribe to mechanical power topic (Watts)
  m_joint_states_sub = m_nh.subscribe("/joint_states", 1, &PowerSystemNode::jointStatesCb, this);

  ROS_INFO("Power system node running");
  ros::spin();
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "power_system_node");
  PowerSystemNode psn;
  psn.Run();
  return 0;
}
// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// See PowerSystemPack.h for a summary of the purpose of this file.

#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <numeric>
#include <math.h>
#include <algorithm>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <ow_lander/lander_joints.h>

#include "PowerSystemPack.h"
#include "PredictionHandler.h"

using namespace PCOE;

const auto START_TIME       = MessageClock::now();

/*
 * The primary function with the loop that controls each cycle.
 * Initializes everything, then starts looping.
 */
void PowerSystemPack::InitAndRun()
{
  // Read system.cfg for the 'print_debug' flag.
  auto system_config_path = ros::package::getPath("ow_power_system")
    + "/config/system.cfg";
  auto system_config = ConfigMap(system_config_path);
  m_print_debug_val = system_config.getString("print_debug");
  // See explanation of m_print_debug_val in the header file for why this
  // is needed for what is essentially a bool.
  if (m_print_debug_val == "true") 
  {
    m_print_debug = true;
  }
  else
  {
    m_print_debug = false;
  }

  if (!initNodes())
  {
    ROS_ERROR_STREAM("Power system pack failed to initialize nodes");
    return;
  }

  if (!initTopics())
  {
    ROS_ERROR_STREAM("Power system pack failed to initialize topics");
    return;
  }

  // Initialize EoD_events and previous_times.
  for (int i = 0; i < NUM_NODES; i++)
  {
    m_nodes[i].previous_time = 0;
    m_EoD_events[i].remaining_useful_life = -1;
    m_EoD_events[i].state_of_charge = -1;
    m_EoD_events[i].battery_temperature = -1;
  }

  // Construct the prediction handlers.
  auto handlers = std::array<std::unique_ptr<PredictionHandler>, NUM_NODES>();
  for (int i = 0; i < NUM_NODES; i++)
  {
    handlers[i] = std::make_unique<PredictionHandler>(
                    m_EoD_events[i].remaining_useful_life,
                    m_EoD_events[i].state_of_charge,
                    m_EoD_events[i].battery_temperature,
                    m_nodes[i].bus, m_nodes[i].name, i
    );
  }

  // Get the asynchronous prognoser configuration and create a builder with it.
  auto config_path = ros::package::getPath("ow_power_system") + "/config/async_prognoser.cfg";
  ConfigMap config(config_path);

  ModelBasedAsyncPrognoserBuilder builder(std::move(config));
  builder.setModelName("Battery");
  builder.setObserverName("UKF");
  builder.setPredictorName("MC");
  // There is a value in async_prognoser.cfg: "LoadEstimator.Window", which sets
  // the number of values looked back on for the moving average. It is 1 by default
  // (so the most recent prediction value is used for the estimation step), but
  // may benefit from being increased in the future.
  builder.setLoadEstimatorName("MovingAverage");
  // To modify the number of samples & horizon (and thus change performance), see the
  // constant declared in PowerSystemPack.h.
  builder.setConfigParam("Predictor.SampleCount", std::to_string(NUM_SAMPLES));
  builder.setConfigParam("Predictor.Horizon", std::to_string(MAX_HORIZON_SECS));

  // Create the prognosers using the builder that will send predictions using
  // their corresponding message bus.
  std::vector<PCOE::AsyncPrognoser> prognosers;
  for (int i = 0; i < NUM_NODES; i++)
  {
    prognosers.push_back(builder.build(m_nodes[i].bus, m_nodes[i].name, "trajectory"));
  }

  ROS_INFO_STREAM("Power system pack running.");

  // This rate object is used to sync the cycles up to the provided Hz.
  // NOTE: If the cycle takes longer than the provided Hz, nothing bad will
  //       necessarily occur, but the simulation will be out of sync with real
  //       time. Ideally, values like NUM_SAMPLES and MAX_HORIZON_SECS should be set
  //       such that the cycle time is under the provided rate (currently 0.5Hz).
  ros::Rate rate(m_gsap_rate_hz);

  bool firstLoop[NUM_NODES];
  for (int i = 0; i < NUM_NODES; i++)
  {
    firstLoop[i] = true;
  }

  // For debug prints.
  int saved_timestamp = -1;

  // Loop through the PowerSystemNodes to update their values and send them to the bus
  // to get predictions.
  while(ros::ok())
  {
    ros::spinOnce();

    // Set up value modifiers in each node for injection later.
    injectFaults();

    // /* DEBUG PRINT
    if (m_print_debug)
    {
      saved_timestamp = m_nodes[0].model.timestamp;
      ROS_INFO_STREAM("Timestamp " << saved_timestamp << " INPUTS:");
    }
    // */

    for (int i = 0; i < NUM_NODES; i++)
    {
      m_nodes[i].node.RunOnce();
      m_nodes[i].node.GetPowerStats(m_nodes[i].model.timestamp, m_nodes[i].model.wattage,
                                    m_nodes[i].model.voltage, m_nodes[i].model.temperature);

      // /* DEBUG PRINT
      if (m_print_debug && !(m_nodes[i].model.timestamp <= 0))
      {
        ROS_INFO_STREAM("Node " << i << " power: " << m_nodes[i].model.wattage 
                        << ".  volts: " << m_nodes[i].model.voltage
                        << ".  tmp: " << m_nodes[i].model.temperature);
      }
      // */

      // If the timestamp is the same as the previous one (happens during startup),
      // do not publish the data to prevent terminate crashes.
      if (m_nodes[i].previous_time != m_nodes[i].model.timestamp)
      {
        auto timestamp = START_TIME + std::chrono::milliseconds(
          static_cast<unsigned>(m_nodes[i].model.timestamp * 1000));

        // Compile a vector<shared_ptr<DoubleMessage>> and then individually publish
        // each individual component.
        std::vector<std::shared_ptr<DoubleMessage>> data_to_publish;
        double input_power;
        double input_tmp;
        double input_voltage;

        if (firstLoop[i])
        {
          // The very first values sent in should be the init values.
          input_power = m_initial_power;
          input_tmp = m_initial_temperature;
          input_voltage = m_initial_voltage;
          firstLoop[i] = false;
        }
        else
        {
          input_power = m_nodes[i].model.wattage;
          input_tmp = m_nodes[i].model.temperature;
          input_voltage = m_nodes[i].model.voltage;
        }

        // Set up the input data that will be passed through the message bus
        // to GSAP's asynchronous prognosers.
        data_to_publish.push_back(
          std::make_shared<DoubleMessage>(MessageId::Watts, m_nodes[i].name, timestamp, input_power));
        data_to_publish.push_back(
          std::make_shared<DoubleMessage>(MessageId::Centigrade, m_nodes[i].name, timestamp, input_tmp));
        data_to_publish.push_back(
          std::make_shared<DoubleMessage>(MessageId::Volts, m_nodes[i].name, timestamp, input_voltage));

      
        m_nodes[i].previous_time = m_nodes[i].model.timestamp;
        // Publish the data to GSAP's async prognosers, triggering a prediction.
        for (const auto& info : data_to_publish)
        {
          m_nodes[i].bus.publish(info);
        }
      }
    }

    // /* DEBUG PRINT
    if (m_print_debug && !(m_nodes[0].model.timestamp <= 0))
    {
      ROS_INFO_STREAM("Waiting for all...");
    }
    // */

    // Wait for all predictions to complete from all async prognosers.
    for (int i = 0; i < NUM_NODES; i++)
    {
      m_nodes[i].bus.waitAll();
    }

    // DEBUG PRINT
    if (m_print_debug && !(m_nodes[0].model.timestamp <= 0))
    {
      ROS_INFO_STREAM("Waited for all!");
    }

    // /* DEBUG PRINT
    if (m_print_debug)
    {
      ROS_INFO_STREAM("Timestamp " << saved_timestamp << " OUTPUTS:");
      for (int i = 0; i < NUM_NODES; i++)
      {
        // Display output if the time is past the initial startup phase, no matter what
        // (for debugging purposes).
        if ((!(m_EoD_events[i].remaining_useful_life <= 0)) || (m_nodes[i].model.timestamp >= 20))
        {
          ROS_INFO_STREAM("Node " << i << " RUL: " << m_EoD_events[i].remaining_useful_life
                          << ". SOC: " << m_EoD_events[i].state_of_charge << ". TMP: "
                          << m_EoD_events[i].battery_temperature);
        }
      }
      if (!(m_nodes[0].model.timestamp <= 0))
      {
        std::cout << std::endl;
      }
    }
    // */

    // Now that EoD_events is ready, manipulate and publish the relevant values.
    publishPredictions();

    // Sleep for any remaining time in the loop that would cause it to
    // complete before the set rate of GSAP.
    // NOTE: Currently there is no catch for if the loop takes longer than the
    // specified ROS rate. If this occurs, the simulation will lag behind real
    // time.
    rate.sleep();
  }
}

/* 
 * Initializes every PowerSystemNode by calling their respective Initialize()
 * functions.
 */
bool PowerSystemPack::initNodes()
{
  // Initialize the nodes.
  for (int i = 0; i < NUM_NODES; i++)
  {
    m_nodes[i].name = setNodeName(i);
    if (!m_nodes[i].node.Initialize())
    {
        return false;
    }
  }
  return true;
}

/*
 * Initializes all publishers for other components in OceanWATERS to get battery
 * outputs from.
 */
bool PowerSystemPack::initTopics()
{
  // Construct the PowerSystemNode publishers
  m_mechanical_power_raw_pub = m_nh.advertise<std_msgs::Float64>("mechanical_power/raw", 1);
  m_mechanical_power_avg_pub = m_nh.advertise<std_msgs::Float64>("mechanical_power/average", 1);
  m_state_of_charge_pub = m_nh.advertise<owl_msgs::BatteryStateOfCharge>("/battery_state_of_charge", 1);
  m_remaining_useful_life_pub = m_nh.advertise<owl_msgs::BatteryRemainingUsefulLife>("/battery_remaining_useful_life", 1);
  m_battery_temperature_pub = m_nh.advertise<owl_msgs::BatteryTemperature>("/battery_temperature", 1);
  // Finally subscribe to the joint_states to estimate the mechanical power
  m_joint_states_sub = m_nh.subscribe("/joint_states", 1, &PowerSystemPack::jointStatesCb, this);
  return true;
}

/*
 * If a custom fault profile has been designated within the RQT window, this
 * function handles getting the relevant information from the file and activating
 * the fault. Also handles deactivation and any errors that might arise.
 */
void PowerSystemPack::injectCustomFault(bool& fault_activated,
                                        const PrognoserVector& sequence,
                                        size_t& index)
{
  static std::string saved_fault_path = "N/A";
  std::string current_fault_path;
  std::string designated_file;
  static std::string saved_file;
  static bool custom_fault_ready = false;
  static bool custom_warning_displayed = false;
  static bool end_fault_warning_displayed = false;
  bool fault_enabled = false;

  // Get the value of fault_enabled.
  ros::param::getCached("/faults/activate_custom_fault", fault_enabled);

  if (!fault_activated && fault_enabled)
  {
    // Multiple potential points of failure, so alert user the process has started.
    ROS_INFO_STREAM("Attempting custom fault activation...");

    // Get user-entered file directory.
    ros::param::getCached("/faults/custom_fault_profile", designated_file);

    // Append the current fault directory to the stored file path.
    current_fault_path = ros::package::getPath("ow_power_system") + "/profiles/" + designated_file;
    
    // Attempt to load/reload the designated file.
    if (designated_file == saved_file)
    {
      ROS_INFO_STREAM("Reloading " << designated_file << "...");
    }
    else if (saved_fault_path != "N/A")
    {
      ROS_INFO_STREAM("Loading " << designated_file << " and unloading " << saved_file << "...");
    }
    else
    {
      ROS_INFO_STREAM("Loading " << designated_file << "...");
    }
    if (loadCustomFaultPowerProfile(current_fault_path, designated_file))
    {
      ROS_WARN_STREAM_ONCE("Custom power faults may exhibit unexpected results. Caution is advised.");
      if (designated_file == saved_file)
      {
        ROS_INFO_STREAM(designated_file << " reactivated!");
      }
      else
      {
        ROS_INFO_STREAM(designated_file << " activated!");
      }
      custom_fault_ready = true;
      end_fault_warning_displayed = false;
      saved_fault_path = current_fault_path;
      saved_file = designated_file;
      index = 0;
    }
    else
    {
      // Custom fault failed to load correctly.
      custom_fault_ready = false;
    }        
    fault_activated = true;
  }
  else if (fault_activated && !fault_enabled)
  {
    if (custom_fault_ready)
    {
      ROS_INFO_STREAM(saved_file << " deactivated!");
    }
    else
    {
      ROS_INFO_STREAM("Custom fault deactivated!");
    }
    fault_activated = false;
    custom_fault_ready = false;
    custom_warning_displayed = false;
  }

  if (fault_activated && fault_enabled && custom_fault_ready)
  {
    // TODO: There's no specification on how to handle reaching the end
    // of a custom fault profile. For now, simply disable
    // the fault.
    if (index >= sequence.size())
    {
      if (!end_fault_warning_displayed)
      {
        ROS_WARN_STREAM
          (saved_file << ": reached end of fault profile. "
           << "Fault disabled, but will restart if re-enabled.");
        end_fault_warning_displayed = true;
      }
    }
    else
    {
      auto data = sequence[index];

      // Evenly distribute the power draw, voltage, and temperature values from
      // the custom fault profile across each node.
      // Note this behavior may need to be updated
      // in the future (e.g. unsure exactly how voltage and temperature
      // affect predictions, as they do not directly add to the values
      // like power does).
      double wattage = data[MessageId::Watts] / NUM_NODES;
      double voltage = data[MessageId::Volts] / NUM_NODES;
      double temperature = data[MessageId::Centigrade] / NUM_NODES;

      for (int i = 0; i < NUM_NODES; i++)
      {
        m_nodes[i].node.SetCustomPowerDraw(wattage);
        m_nodes[i].node.SetCustomVoltageFault(voltage);
        m_nodes[i].node.SetCustomTemperatureFault(temperature);
      }
      
      index += m_profile_increment;
    }
  }
}

/*
 * Handles defined faults within the RQT window (currently only high power draw).
 */
void PowerSystemPack::injectFault (const std::string& fault_name,
                                   bool& fault_activated)
{
  static bool warning_displayed = false;
  bool fault_enabled = false;
  double hpd_wattage = 0.0;

  // Get the value of fault_enabled.
  ros::param::getCached("/faults/" + fault_name, fault_enabled);

  if (!fault_activated && fault_enabled)
  {
    ROS_INFO_STREAM(fault_name << " activated!");
    fault_activated = true;
  }
  else if (fault_activated && !fault_enabled)
  {
    ROS_INFO_STREAM(fault_name << " deactivated!");
    fault_activated = false;
    warning_displayed = false;
  }

  if (fault_activated && fault_enabled)
  {
    // If the current fault being utilized is high_power_draw,
    // simply update wattage based on the current value of the HPD slider.
    if (fault_name == FAULT_NAME_HPD_ACTIVATE)
    {
      ros::param::getCached("/faults/" + FAULT_NAME_HPD, hpd_wattage);
      double split_wattage = hpd_wattage / NUM_NODES;

      for (int i = 0; i < NUM_NODES; i++)
      {
        m_nodes[i].node.SetHighPowerDraw(split_wattage);
      }
    }

    // NOTE: If other faults are added to the RQT window in the future,
    // this is where their related flags/logic would be checked/used
    // in a similar manner to the above code block with high power draw.
  }
}

/*
 * Basic function that calls the other fault-handling functions.
 */
void PowerSystemPack::injectFaults()
{
  injectFault(FAULT_NAME_HPD_ACTIVATE,
              m_high_power_draw_activated);
  injectCustomFault(m_custom_power_fault_activated,
                    m_custom_power_fault_sequence,
                    m_custom_power_fault_sequence_index);
}

/*
 * Callback function that publishes mechanical power values.
 */
void PowerSystemPack::jointStatesCb(const sensor_msgs::JointStateConstPtr& msg)
{
  // NOTE: This callback function appears to call after the nodes' callback
  //       functions complete, every single time. This is quite convenient since
  //       it depends on values determined from those callbacks, but
  //       I don't know why exactly this is the case. If they should happen
  //       to stop calling in this order, it could potentially cause problems.
  //       (~Liam, Summer 2022)
  
  // Get the mechanical power values from each node.
  double raw_mechanical_values[NUM_NODES];
  double avg_mechanical_values[NUM_NODES];
  double avg_power = 0.0;

  for (int i = 0; i < NUM_NODES; i++)
  {
    raw_mechanical_values[i] = m_nodes[i].node.GetRawMechanicalPower();
    avg_mechanical_values[i] = m_nodes[i].node.GetAvgMechanicalPower();
    avg_power += raw_mechanical_values[i];
  }

  // Publish the mechanical raw and average power values.
  // Raw mechanical power should be averaged out, but the avg_power should
  // already be the same value in every node.
  avg_power = avg_power / NUM_NODES;

  std_msgs::Float64 mechanical_power_raw_msg, mechanical_power_avg_msg;
  mechanical_power_raw_msg.data = avg_power;

  // Since all average mechanical power values should be identical, it doesn't
  // matter which node's value we take.
  mechanical_power_avg_msg.data = avg_mechanical_values[0];
  m_mechanical_power_raw_pub.publish(mechanical_power_raw_msg);
  m_mechanical_power_avg_pub.publish(mechanical_power_avg_msg);
}

/*
 * Function called during custom fault injection that attempts to
 * load the specified file and its data.
 */
PrognoserVector PowerSystemPack::loadPowerProfile(const std::string& filename, std::string custom_file)
{
  std::ifstream file(filename);

  if (file.fail())
  {
    ROS_WARN_STREAM("Could not find a custom file in the 'profiles' directory with name '"
                          << custom_file << "'. Deactivate fault and try again.");
    return PrognoserVector();
  }

  // Skip header line.
  file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  auto now = std::chrono::system_clock::now();

  PrognoserVector result;

  // Line number starts at 2 instead of 1 since the first line is the header.
  int line_number = 2;

  try
  {
    while (file.good())
    {
      PrognoserMap data;
      std::string line;
      getline(file, line);
      if (line.empty())
      {
        continue;
      }

      std::stringstream line_stream(line);
      std::string cell;

      // Confirm the line contains the expected number of columns.
      auto cols = std::count(line.begin(), line.end(), ',') + 1;

      if (cols != CUSTOM_FILE_EXPECTED_COLS)
      {
        throw ERR_CUSTOM_FILE_FORMAT;
      }

      // Get the time index and power values.

      // NOTE: The voltage and temperature values of a PrognoserVector have much
      //       different functions as GSAP inputs than power does. It's not fully
      //       understood what exactly they do yet, but they seem to affect the
      //       estimation step rather than the prediction step.
      getline(line_stream, cell, ',');
      double file_time = std::stod(cell);
      auto timestamp = now + std::chrono::milliseconds(static_cast<unsigned>(file_time * 1000));

      getline(line_stream, cell, ',');
      Datum<double> power(std::stod(cell));
      power.setTime(timestamp);

      getline(line_stream, cell, ',');
      Datum<double> temperature(std::stod(cell));
      temperature.setTime(timestamp);

      getline(line_stream, cell, ',');
      Datum<double> voltage(std::stod(cell));
      voltage.setTime(timestamp);

      data.insert({ MessageId::Watts, power });
      data.insert({ MessageId::Centigrade, temperature});
      data.insert({ MessageId::Volts, voltage });

      result.push_back(data);

      line_number++;
    }
  }
  catch(...) // Many possible different errors could result from reading an improperly formatted file.
  {
    ROS_ERROR_STREAM("Failed to read " << custom_file << ":" << std::endl <<
                     "Improper formatting detected on line " << line_number << "." << std::endl << 
                     "Confirm " << custom_file << " follows the exact format of example_fault.csv before retrying.");
    return PrognoserVector();
  }
  return result;
}

/*
 * Attempts to call loadPowerProfile and returns true or false depending on its success.
 */
bool PowerSystemPack::loadCustomFaultPowerProfile(std::string path, std::string custom_file)
{
  m_custom_power_fault_sequence = loadPowerProfile(path, custom_file);

  // Return false if the sequence was not properly initialized.
  return (m_custom_power_fault_sequence.size() > 0);
}

/*
 * Called at the end of a cycle, this publishes RUL/SoC/battery temperature
 * based on all prognoser predictions obtained.
 */
void PowerSystemPack::publishPredictions()
{
  // Using EoD_events, publish the relevant values.

  int min_rul = -1;
  double min_soc = -1;
  double max_tmp = -1;
  owl_msgs::BatteryRemainingUsefulLife rul_msg;
  owl_msgs::BatteryStateOfCharge soc_msg;
  owl_msgs::BatteryTemperature tmp_msg;

  for (int i = 0; i < NUM_NODES; i++)
  {
    // If RUL is infinity, set it to the maximum horizon value instead.
    if (isinf(m_EoD_events[i].remaining_useful_life))
    {
      m_EoD_events[i].remaining_useful_life = MAX_HORIZON_SECS;
    }

    // Published RUL (remaining useful life) is defined as the minimum RUL of all EoDs.
    if (m_EoD_events[i].remaining_useful_life < min_rul || min_rul == -1)
    {
      min_rul = m_EoD_events[i].remaining_useful_life;
    }

    // Published SoC (state of charge) is defined as the minimum SoC of all EoDs.
    if (m_EoD_events[i].state_of_charge < min_soc || min_soc == -1)
    {
      min_soc = m_EoD_events[i].state_of_charge;
    }
    
    // Published battery temperature is defined as the highest tmp of all EoDs.
    if (m_EoD_events[i].battery_temperature > max_tmp || max_tmp == -1)
    {
      max_tmp = m_EoD_events[i].battery_temperature;
    }
  }

  // If either RUL or SoC have entered the negative, it indicates the battery
  // is in a fail state.
  // While it is possible sometimes for the battery to recover from this,
  // differentiating between the 2 different types of fail states has not been
  // implemented yet.
  // For now, treat all fail states as permanent and flatline all future
  // publications.
  // The timestamp stipulation is to prevent odd values on startup from triggering
  // the fail state.
  if ((min_rul < 0 || min_soc < 0) && (!m_battery_failed) && (m_nodes[0].model.timestamp >= 10))
  {
    ROS_WARN_STREAM("The battery has reached a fail state. Flatlining "
                    << "all future published predictions");
    m_battery_failed = true;
  }

  if (m_battery_failed)
  {
    min_rul = 0;
    min_soc = 0;
    max_tmp = 0;
  }

  // /* DEBUG PRINT
  // Note that the debug printouts do not cap at MAX_HORIZON_SECS, nor do they
  // flatline at 0 when the battery fails.
  if (m_print_debug && ((!(min_rul < 0 || min_soc < 0 || max_tmp < 0)) || (m_nodes[0].model.timestamp >= 20)))
  {
    ROS_INFO_STREAM("min_rul: " << std::to_string(min_rul) <<
                    ", min_soc: " << std::to_string(min_soc) <<
                    ", max_tmp: " << std::to_string(max_tmp));
  }

  // Publish the values for other components.
  rul_msg.value = min_rul;
  soc_msg.value = min_soc;
  tmp_msg.value = max_tmp;

  // Apply the most recent timestamp to each message header.
  // MAY NOT WORK PROPERLY, depending on how ros::Time::now() works.
  auto timestamp = ros::Time::now();
  rul_msg.header.stamp = timestamp;
  soc_msg.header.stamp = timestamp;
  tmp_msg.header.stamp = timestamp;

  m_state_of_charge_pub.publish(soc_msg);
  m_remaining_useful_life_pub.publish(rul_msg);
  m_battery_temperature_pub.publish(tmp_msg);
}

std::string PowerSystemPack::setNodeName(int node_num)
{
  return "Node " + std::to_string(node_num);
}

int main(int argc, char* argv[]) 
{
  ros::init(argc, argv, "power_system_node");

  PowerSystemPack pack;

  pack.InitAndRun();

  return 0;
}

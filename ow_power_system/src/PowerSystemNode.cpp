// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// See PowerSystemNode.h for a summary of the purpose of this file.

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

#include "PowerSystemNode.h"

using namespace PCOE;

const std::string FAULT_NAME_HPD           = "high_power_draw";
const std::string FAULT_NAME_HPD_ACTIVATE  = "activate_high_power_draw";
const int CUSTOM_FILE_EXPECTED_COLS           = 4;

// Error flags.
const int ERR_CUSTOM_FILE_FORMAT              = -1;

/*
 * The primary function with the loop that controls each cycle.
 * Initializes everything, then starts looping.
 */
void PowerSystemNode::initAndRun()
{
  const auto START_TIME       = MessageClock::now();

  // Read system.cfg for various parameters.
  try
  {
    auto system_config_path = ros::package::getPath("ow_power_system")
      + "/config/system.cfg";
    auto system_config = ConfigMap(system_config_path);

    // DEBUG CUSTOMIZATION
    // See explanation of m_print_debug in the header file for why this
    // comparison is needed for what is essentially a bool.
    const std::string TRUE = "true";
    m_print_debug = (system_config.getString("print_debug") == TRUE);

    if (m_print_debug)
    {
      m_timestamp_print_debug = (system_config.getString("timestamp_print_debug") == TRUE);
      m_inputs_print_debug = (system_config.getString("inputs_print_debug") == TRUE);
      m_outputs_print_debug = (system_config.getString("outputs_print_debug") == TRUE);
      m_topics_print_debug = (system_config.getString("topics_print_debug") == TRUE);
      m_mech_power_print_debug = (system_config.getString("mech_power_print_debug") == TRUE);
    }
    // If print_debug was false, then all flags remain false as initialized.

    m_max_horizon_secs = system_config.getInt32("max_horizon");
    m_num_samples = system_config.getInt32("num_samples");
    m_initial_power = system_config.getDouble("initial_power");
    m_initial_voltage = system_config.getDouble("initial_voltage");
    m_initial_temperature = system_config.getDouble("initial_temperature");
    m_initial_soc = system_config.getDouble("initial_soc");
    m_max_gsap_input_watts = system_config.getDouble("max_gsap_power_input");
    m_loop_rate_hz = system_config.getDouble("loop_rate");
    m_spinner_threads = system_config.getInt32("spinner_threads");
  }
  catch(const std::exception& err)
  {
    ROS_ERROR_STREAM("OW_POWER_SYSTEM ERROR: " << err.what() 
                     << " while attempting to read system.cfg in the node!\n"
                     << "Ensure system.cfg is formatted properly before "
                     << "restarting the simulation.");
    return;
  }
  
  if (!initModels())
  {
    ROS_ERROR_STREAM("OW_POWER_SYSTEM ERROR: "
                     << "Failed to initialize power models!");
    return;
  }

  // Set up the time interval & moving average size.
  m_time_interval = 1 / m_loop_rate_hz;
  m_moving_average_window = floor(m_joint_states_rate / m_loop_rate_hz);

  // Set up the moving average vector.
  m_power_values.resize(m_moving_average_window, 0.0);

  initTopics();

  // Initialize EoD_events and previous_times.
  for (int i = 0; i < NUM_MODELS; i++)
  {
    m_power_models[i].previous_time = 0;
    m_EoD_events[i].remaining_useful_life = m_max_horizon_secs;
    m_EoD_events[i].state_of_charge = m_initial_soc;
    m_EoD_events[i].battery_temperature = m_initial_temperature;
  }

  // Construct the prediction handlers.
  auto handlers = std::array<std::unique_ptr<PredictionHandler>, NUM_MODELS>();
  for (int i = 0; i < NUM_MODELS; i++)
  {
    handlers[i] = std::make_unique<PredictionHandler>(
                    m_power_models[i].bus, m_power_models[i].name
    );
  }


  std::vector<PCOE::AsyncPrognoser> prognosers;

  // Get the prognoser configuration and create a builder with it.
  try
  {
    std::string config_path = ros::package::getPath("ow_power_system")
                            + "/config/prognoser.cfg";

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
    builder.setConfigParam("Predictor.SampleCount", std::to_string(m_num_samples));
    builder.setConfigParam("Predictor.Horizon", std::to_string(m_max_horizon_secs));

    // Create the prognosers using the builder that will send predictions using
    // their corresponding message bus.
    for (int i = 0; i < NUM_MODELS; i++)
    {
      prognosers.push_back(builder.build(m_power_models[i].bus,
                                         m_power_models[i].name,
                                         "trajectory"));
    }
  }
  catch(const std::exception& err)
  {
    ROS_ERROR_STREAM("OW_POWER_SYSTEM ERROR: " << err.what()
                     << " while attempting to initialize asynchronous "
                     << "prognosers!\nEnsure /config/async_prognoser.cfg exists "
                     << "and is formatted properly before restarting the "
                     << "simulation.");
    return;
  }

  ROS_INFO_STREAM("Power system node running.");

  // This rate object is used to sync the cycles up to the provided Hz.
  // NOTE: If the cycle takes longer than the provided Hz, nothing bad will
  //       necessarily occur, but the simulation will be out of sync with real
  //       time.
  ros::Rate rate(m_loop_rate_hz);

  // Start the asynchronous spinner, which will call jointStatesCb as the
  // joint_states topic is published (50Hz).
  ros::AsyncSpinner spinner(m_spinner_threads);
  spinner.start();

  // Loop through the PrognoserInputHandlers to update their values and send them to
  // the bus each loop to trigger predictions.
  // This loop should run at 0.5Hz, publishing predictions every 2 seconds via rostopics.
  int start_loops = NUM_MODELS;
  while(ros::ok())
  {
    // Set up value modifiers in each model handler for injection later.
    injectFaults();

    // DEBUG PRINT
    if (m_timestamp_print_debug)
    {
      printTimestamp(m_power_models[0].input_info.timestamp);
    }

    bool draw_warning_displayed = false;

    // Cycle the inputs in each model and send the data sets to the
    // corresponding prognoser via its message bus. The power system is
    // locked during this, preventing callback functions from altering
    // related variables mid-loop.
    m_processing_power_batch = true;
    for (int i = 0; i < NUM_MODELS; i++)
    {
      // Cycle the inputs in each model.
      // If excessive power draw was detected/corrected, display the warning
      // if it hasn't already displayed once this cycle.
      if (m_power_models[i].model.runOnce() && !draw_warning_displayed)
      {
        // Current implementation means if one ModelHandler exceeds the draw
        // limit, every ModelHandler will exceed at the same amount.
        // So it doesn't matter which model triggers the warning as long as
        // it displays once.
        ROS_WARN_STREAM("Power system computed power input above the "
                        << m_max_gsap_input_watts
                        << "W cap for one or more GSAP prognosers.\n"
                        << "Input was set to "
                        << m_max_gsap_input_watts
                        << "W for each model over the limit.");
        // Prevent warning from repeating within the same cycle.
        draw_warning_displayed = true;
      }

      m_power_models[i].model.getPowerStats(m_power_models[i].input_info);
      
      // If the timestamp is the same as the previous one (happens during startup),
      // do not publish the data to prevent crashes.
      if (m_power_models[i].previous_time != m_power_models[i].input_info.timestamp)
      {
        auto timestamp = START_TIME + std::chrono::milliseconds(
          static_cast<unsigned>(m_power_models[i].input_info.timestamp * 1000)
        );

        double input_power, input_tmp, input_voltage;

        if (start_loops > 0)
        {
          // The very first values sent in to each prognoser
          // should be the init values.
          input_power = m_initial_power;
          input_voltage = m_initial_voltage;
          input_tmp = m_initial_temperature;
          start_loops--;
        }
        else
        {
          input_power = m_power_models[i].input_info.wattage;
          input_voltage = m_power_models[i].input_info.voltage;
          input_tmp = m_power_models[i].input_info.temperature;
        }

        // DEBUG PRINT
        if (m_inputs_print_debug && (m_power_models[i].input_info.timestamp > 0))
        {
          printPrognoserInputs(input_power, input_voltage, input_tmp, i);
        }

        // Set up the input data that will be passed through the message bus
        // to GSAP's asynchronous prognosers.
        m_power_models[i].bus.publish(std::make_shared<DoubleMessage>(
          MessageId::Watts, m_power_models[i].name, timestamp, input_power));
        m_power_models[i].bus.publish(std::make_shared<DoubleMessage>(
          MessageId::Centigrade, m_power_models[i].name, timestamp, input_tmp));
        m_power_models[i].bus.publish(std::make_shared<DoubleMessage>(
          MessageId::Volts, m_power_models[i].name, timestamp, input_voltage));

        m_power_models[i].previous_time = m_power_models[i].input_info.timestamp;
      }
    }

    // DEBUG PRINT
    if (m_mech_power_print_debug)
    {
      printMechanicalPower(m_power_watts, m_mean_mechanical_power);
    }

    // Unlock the power system.
    m_processing_power_batch = false;

    // Check if the prediction handlers have EoD predictions ready,
    // and update EoD_events if so.
    for (int i = 0; i < NUM_MODELS; i++)
    {
      if (handlers[i]->getStatus())
      {
        m_EoD_events[i] = handlers[i]->getEoD();
      }
    }

    // Publish the predictions in EoD_events.
    publishPredictions();

    // DEBUG PRINT
    if (m_outputs_print_debug)
    {
      ROS_INFO_STREAM("CURRENT MODEL DATA:");
      for (int i = 0; i < NUM_MODELS; i++)
      {
        printPrognoserOutputs(m_EoD_events[i].remaining_useful_life,
                              m_EoD_events[i].state_of_charge,
                              m_EoD_events[i].battery_temperature,
                              i);
      }
    }

    // DEBUG PRINT FORMATTING
    if (m_print_debug && !(m_power_models[0].input_info.timestamp <= 0))
    {
      std::cout << std::endl;
    }

    // Sleep for any remaining time in the loop that would cause it to
    // complete before the set rate of GSAP.
    // NOTE: Currently there is no catch for if the loop takes longer than the
    // specified ROS rate. If this occurs, the simulation will lag behind real
    // time.
    rate.sleep();
  }

  spinner.stop();
}

/* 
 * Initializes every PrognoserInputHandler by calling their respective Initialize()
 * functions.
 */
bool PowerSystemNode::initModels()
{
  // Initialize the models.
  for (int i = 0; i < NUM_MODELS; i++)
  {
    m_power_models[i].name = setModelName(i);
    if (!m_power_models[i].model.initialize())
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
void PowerSystemNode::initTopics()
{
  // Construct the PrognoserInputHandler publishers
  m_mechanical_power_raw_pub = m_nh.advertise
                                  <std_msgs::Float64>(
                                  "mechanical_power/raw", 1);
  m_mechanical_power_avg_pub = m_nh.advertise
                                  <std_msgs::Float64>(
                                  "mechanical_power/average", 1);
  m_state_of_charge_pub = m_nh.advertise
                                  <owl_msgs::BatteryStateOfCharge>(
                                  "/battery_state_of_charge", 1);
  m_remaining_useful_life_pub = m_nh.advertise
                                  <owl_msgs::BatteryRemainingUsefulLife>(
                                  "/battery_remaining_useful_life", 1);
  m_battery_temperature_pub = m_nh.advertise
                                  <owl_msgs::BatteryTemperature>(
                                  "/battery_temperature", 1);
  // Finally subscribe to the joint_states to estimate the mechanical power
  m_joint_states_sub = m_nh.subscribe(
                        "/joint_states",
                        1,
                        &PowerSystemNode::jointStatesCb,
                        this);
}

/*
 * If a custom fault profile has been designated within the RQT window, this
 * function handles getting the relevant information from the file and activating
 * the fault. Also handles deactivation and any errors that might arise.
 */
void PowerSystemNode::injectCustomFault(bool& fault_activated,
                                        const PrognoserVector& sequence,
                                        size_t& index)
{
  static std::string saved_fault_path = "N/A";
  std::string current_fault_path;
  std::string designated_file;
  static std::string saved_file;
  static bool custom_fault_ready = false;
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
  }

  if (fault_activated && fault_enabled && custom_fault_ready)
  {
    // TODO: There's no specification on how to handle reaching the end
    // of a custom fault profile. For now, simply disable the fault.
    if (index >= sequence.size())
    {
      if (!end_fault_warning_displayed)
      {
        ROS_INFO_STREAM
          (saved_file << ": reached end of fault profile. "
           << "Fault disabled, but will restart if re-enabled.");
        end_fault_warning_displayed = true;
      }
    }
    else
    {
      auto data = sequence[index];

      // Evenly distribute the power draw, voltage, and temperature values from
      // the custom fault profile across each model.
      // NOTE this behavior may need to be updated in the future (e.g. unsure
      // exactly how voltage and temperature affect predictions, as they do not
      // directly add to the values like power does).
      double wattage = data[MessageId::Watts] / NUM_MODELS;
      double voltage = data[MessageId::Volts] / NUM_MODELS;
      double temperature = data[MessageId::Centigrade] / NUM_MODELS;

      for (int i = 0; i < NUM_MODELS; i++)
      {
        m_power_models[i].model.setCustomPowerDraw(wattage);
        m_power_models[i].model.setCustomVoltageFault(voltage);
        m_power_models[i].model.setCustomTemperatureFault(temperature);
      }
      
      index++;
    }
  }
}

/*
 * Handles defined faults within the RQT window (currently only high power draw).
 */
void PowerSystemNode::injectFault (const std::string& fault_name,
                                   bool& fault_activated)
{
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
  }

  if (fault_activated && fault_enabled)
  {
    // If the current fault being utilized is high_power_draw,
    // simply update wattage based on the current value of the HPD slider.
    if (fault_name == FAULT_NAME_HPD_ACTIVATE)
    {
      ros::param::getCached("/faults/" + FAULT_NAME_HPD, hpd_wattage);
      double split_wattage = hpd_wattage / NUM_MODELS;

      for (int i = 0; i < NUM_MODELS; i++)
      {
        m_power_models[i].model.setHighPowerDraw(split_wattage);
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
void PowerSystemNode::injectFaults()
{
  injectFault(FAULT_NAME_HPD_ACTIVATE,
              m_high_power_draw_activated);
  injectCustomFault(m_custom_power_fault_activated,
                    m_custom_power_fault_sequence,
                    m_custom_power_fault_sequence_index);
}

/*
 * Callback function that publishes mechanical power values and sends
 * mechanical power, which is converted into power draw, to each model in the
 * node. The mechanical power is distributed evenly amongst all models.
 */
void PowerSystemNode::jointStatesCb(const sensor_msgs::JointStateConstPtr& msg)
{
  m_power_watts = 0.0;  // This includes the arm + antenna
  for (int i = 0; i < ow_lander::NUM_JOINTS; ++i)
    m_power_watts += fabs(msg->velocity[i] * msg->effort[i]);

  m_power_values[m_power_values_index++ % m_power_values.size()] = m_power_watts;
  m_mean_mechanical_power = accumulate(
      begin(m_power_values), end(m_power_values), 0.0)
      / m_power_values.size();

  // Publish the total raw & average mechanical power.
  std_msgs::Float64 mechanical_power_raw_msg, mechanical_power_avg_msg;
  mechanical_power_raw_msg.data = m_power_watts;
  mechanical_power_avg_msg.data = m_mean_mechanical_power;
  m_mechanical_power_raw_pub.publish(mechanical_power_raw_msg);
  m_mechanical_power_avg_pub.publish(mechanical_power_avg_msg);

  // Send in mechanical power to each model, distributed evenly, to be converted
  // into power draw.
  for (int i = 0; i < NUM_MODELS; i++)
  {
    m_power_models[i].model.applyMechanicalPower(
                                          (m_mean_mechanical_power / NUM_MODELS),
                                          m_processing_power_batch);
  }
}

/*
 * Function called during custom fault injection that attempts to
 * load the specified file and its data.
 */
PrognoserVector PowerSystemNode::loadPowerProfile(const std::string& filename, std::string custom_file)
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
bool PowerSystemNode::loadCustomFaultPowerProfile(std::string path, std::string custom_file)
{
  m_custom_power_fault_sequence = loadPowerProfile(path, custom_file);

  // Return false if the sequence was not properly initialized.
  return (m_custom_power_fault_sequence.size() > 0);
}

/*
 * Called at the end of a cycle, this publishes RUL/SoC/battery temperature
 * based on all prognoser predictions obtained.
 */
void PowerSystemNode::publishPredictions()
{
  // Using EoD_events, publish the relevant values.

  // Start the values at either their maximum possible, or beyond the minimum.
  double min_rul = m_max_horizon_secs;
  double min_soc = 1;
  double max_tmp = -1000;
  owl_msgs::BatteryRemainingUsefulLife rul_msg;
  owl_msgs::BatteryStateOfCharge soc_msg;
  owl_msgs::BatteryTemperature tmp_msg;

  // Calculate the min_rul, min_soc, and max_tmp from the new data.
  for (int i = 0; i < NUM_MODELS; i++)
  {
    // If RUL is infinity, set it to the maximum horizon value instead.
    if (isinf(m_EoD_events[i].remaining_useful_life))
    {
      m_EoD_events[i].remaining_useful_life = m_max_horizon_secs;
    }

    min_rul = std::min(min_rul, m_EoD_events[i].remaining_useful_life);
    min_soc = std::min(min_soc, m_EoD_events[i].state_of_charge);
    max_tmp = std::max(max_tmp, m_EoD_events[i].battery_temperature);
  }

  // DEBUG PRINT
  if (m_topics_print_debug)
  {
    printTopics(min_rul, min_soc, max_tmp);
  }

  // If either RUL or SoC have entered the negative, it indicates the battery
  // is in a fail state.
  // While it is possible sometimes for the battery to recover from this,
  // differentiating between the 2 different types of fail states has not been
  // implemented yet.
  // For now, treat all fail states as permanent and flatline all future
  // publications.
  // The timestamp stipulation is to prevent odd/invalid values during startup
  // from triggering the fail state.
  if ((min_rul < 0 || min_soc < 0)
      && (!m_battery_failed)
      && (m_power_models[0].input_info.timestamp >= (m_time_interval * 5)))
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

  // Publish the values for other components.
  rul_msg.value = min_rul;
  soc_msg.value = min_soc;
  tmp_msg.value = max_tmp;

  // Apply the most recent timestamp to each message header.
  auto timestamp = ros::Time::now();
  rul_msg.header.stamp = timestamp;
  soc_msg.header.stamp = timestamp;
  tmp_msg.header.stamp = timestamp;

  // Publish the data.
  m_state_of_charge_pub.publish(soc_msg);
  m_remaining_useful_life_pub.publish(rul_msg);
  m_battery_temperature_pub.publish(tmp_msg);
}

std::string PowerSystemNode::setModelName(int model_num)
{
  return "model " + std::to_string(model_num);
}

/*
 * Prints the timestamp of each cycle to the terminal.
 * Output: 1 line per cycle.
 */
void PowerSystemNode::printTimestamp(double timestamp)
{
  ROS_INFO_STREAM("Timestamp " << timestamp);
}

/*
 * Prints the input data sent to GSAP's asynchronous prognosers each cycle.
 * Output: NUM_MODELS lines per cycle.
 */
void PowerSystemNode::printPrognoserInputs(double power,
                                           double voltage,
                                           double temperature,
                                           int index)
{
  // Header is simply used for printout consistency between
  // models 0-9 and 10+.
  std::string header = "M";
  if (index < 10)
  {
    header += "0";
  }
  ROS_INFO_STREAM(header << index << " INPUT power: " << std::to_string(power) 
                  << ".  volts: " << std::to_string(voltage)
                  << ".  tmp: " << std::to_string(temperature));
}

/*
 * Prints the information currently stored in all models each cycle.
 * Output: NUM_MODELS lines per cycle.
 */
void PowerSystemNode::printPrognoserOutputs(double rul,
                                            double soc,
                                            double tmp,
                                            int index)
{
  std::string header = "M";
  if (index < 10)
  {
    header += "0";
  }
  ROS_INFO_STREAM(header << index << " RUL: " << std::to_string(rul)
                  << ", SOC: " << std::to_string(soc) << ", TMP: "
                  << std::to_string(tmp));
}

/*
 * Prints the mechanical power calculated during jointStatesCb each cycle.
 * Output: 2 lines per cycle.
 */
void PowerSystemNode::printMechanicalPower(double raw, double mean)
{
  ROS_INFO_STREAM("Most recent raw mechanical power: " << std::to_string(raw));
  ROS_INFO_STREAM("Average of last " << std::to_string(m_moving_average_window)
                  << " raw values: " << std::to_string(mean));
}

/*
 * Prints the information that will be published via rostopics each cycle.
 * Output: 1 line per cycle.
 */
void PowerSystemNode::printTopics(double rul, double soc, double tmp)
{
  ROS_INFO_STREAM("min_rul: " << std::to_string(rul) <<
                  ", min_soc: " << std::to_string(soc) <<
                  ", max_tmp: " << std::to_string(tmp));
}
// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// See PowerSystemNode.h for a summary of the purpose of this file.

#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <ow_lander/lander_joints.h>

#include "PowerSystemNode.h"

using namespace PCOE;

// Fault names
const std::string FAULT_NAME_HPD              = "high_power_draw";
const std::string FAULT_NAME_HPD_ACTIVATE     = "activate_high_power_draw";
const std::string FAULT_NAME_DBN              = "battery_nodes_to_disconnect";
const std::string FAULT_NAME_DBN_ACTIVATE     = "disconnect_battery_nodes";
const std::string FAULT_NAME_LOW_SOC          = "low_state_of_charge";
const std::string FAULT_NAME_ICL              = "instantaneous_capacity_loss";
const std::string FAULT_NAME_THERMAL_FAILURE  = "thermal_failure";
const int CUSTOM_FILE_EXPECTED_COLS           = 2;

// Error flags.
const int ERR_CUSTOM_FILE_FORMAT              = -1;

// Static function declarations.
static std::string formatModelName(int model_num);
static void printTimestamp(double timestamp);
static void printPrognoserInputs(double power,
                                 double voltage,
                                 double temperature,
                                 int index);
static void printPrognoserOutputs(double rul,
                                  double soc,
                                  double tmp,
                                  int index);
static void printMechanicalPower(double raw, double mean, int window);
static void printTopics(double rul, double soc, double tmp);
static void printFaultDisabledWarning();
static void printPowerFaultWarning();

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

    m_enable_power_system = (system_config.getString("enable_power_system") == TRUE);
    m_active_models = system_config.getInt32("active_models");
    m_max_horizon_secs = system_config.getInt32("max_horizon");
    m_num_samples = system_config.getInt32("num_samples");
    m_initial_power = system_config.getDouble("initial_power");
    m_initial_voltage = system_config.getDouble("initial_voltage");
    m_initial_temperature = system_config.getDouble("initial_temperature");
    m_initial_soc = system_config.getDouble("initial_soc");
    m_max_gsap_input_watts = system_config.getDouble("max_gsap_power_input");
    m_loop_rate_hz = system_config.getDouble("loop_rate");
    m_spinner_threads = system_config.getInt32("spinner_threads");
    m_prev_soc = m_initial_soc;
  }
  catch(const std::exception& err)
  {
    ROS_ERROR_STREAM("OW_POWER_SYSTEM ERROR: " << err.what() 
                     << " while attempting to read system.cfg in the node!\n"
                     << "Ensure system.cfg is formatted properly before "
                     << "restarting the simulation.");
    return;
  }

  // Error-catch m_active_models.
  if (m_active_models < 1 || m_active_models > NUM_MODELS)
  {
    ROS_WARN_STREAM("OW_POWER_SYSTEM WARNING: "
                    << "Invalid number of active models specified in "
                    << "system.cfg! Check the value of 'active_models' "
                    << "and ensure it is between 1 & "
                    << std::to_string(NUM_MODELS) << ". Defaulting to 1...");
    m_active_models = 1;
  }
  m_deactivated_models = 0;
  
  if (!initModels())
  {
    ROS_ERROR_STREAM("OW_POWER_SYSTEM ERROR: "
                     << "Failed to initialize power models!");
    return;
  }

  // Set up the time interval & moving average size.
  m_time_interval = 1 / m_loop_rate_hz;
  m_moving_average_window = static_cast<int>(
                              std::floor(m_joint_states_rate / m_loop_rate_hz)
                            );

  initTopics();

  // If the power system is fully disabled, swap to a basic loop instead.
  if (!m_enable_power_system)
  {
    ROS_WARN_STREAM("Power system disabled via system.cfg! Node will publish "
                    << "static values and ignore faults.");
    runDisabledLoop();
    return;
  }

  // Initialize EoD_events and previous_times.
  for (int i = 0; i < NUM_MODELS; i++)
  {
    m_power_models[i].previous_time = 0.0;
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
    for (int i = 0; i < m_active_models; i++)
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

  // Notify the user of the state of the battery simulation.
  if (m_active_models > 1)
  {
    ROS_INFO_STREAM("OW_POWER_SYSTEM NOTE: " << std::to_string(m_active_models)
                    << " models instantiated. This means "
                    << std::to_string(m_active_models)
                    << " prognosers will run battery predictions at once. "
                    << "More complex intra-battery faults become possible to "
                    << "simulate with more prognosers, but CPU load will be "
                    << "much higher with each additional model.");
  }
  else
  {
    ROS_INFO_STREAM("OW_POWER_SYSTEM NOTE: Not simulating full battery. "
                    << "This means only one GSAP model will run predictions to "
                    << "keep performance optimal, but intra-battery faults "
                    << "may not work properly.");
  }

  // This rate object is used to sync the cycles up to the provided Hz.
  // NOTE: If the cycle takes longer than the provided Hz, nothing bad will
  //       necessarily occur, but the simulation will be out of sync with real
  //       time.
  ros::Rate rate(m_loop_rate_hz);

  // Start the asynchronous spinner, which will call jointStatesCb as the
  // joint_states topic is published (50Hz).
  ros::AsyncSpinner spinner(m_spinner_threads);
  try
  {
    spinner.start();
  }
  catch(const std::runtime_error& e)
  {
    ROS_ERROR_STREAM("OW_POWER_SYSTEM ERROR: Encountered a std::runtime_error"
                    << " while starting up asynchronous ROS spinner threads!");
    return;
  }
  


  // Loop through the PrognoserInputHandlers to update their values and send them to
  // the bus each loop to trigger predictions.
  int start_loops = m_active_models;
  while(ros::ok())
  {
    // Set up value modifiers in each model handler for injection later.
    // Skip the first loop to prevent activation/deactivation messages from
    // printing during startup.
    if (m_power_models[0].input_info.timestamp > 0)
    {
      injectFaults();
    }

    // DEBUG PRINT
    if (m_timestamp_print_debug)
    {
      printTimestamp(m_power_models[0].input_info.timestamp);
    }

    // Cycle the inputs in each model and send the data sets to the
    // corresponding prognoser via its message bus.
    // Save the current average mechanical power, in case jointStatesCb runs
    // while processing which would update m_mean_mechanical_power mid-loop.
    double shared_power_load = (m_mean_mechanical_power + m_electrical_power)
                                / (NUM_MODELS - m_deactivated_models);
    for (int i = 0; i < m_active_models; i++)
    {
      // Apply mechanical power, then cycle the inputs in each model.
      // If excessive power draw was detected/corrected, display the warning
      // if it hasn't already displayed once this cycle.
      m_power_models[i].model.setPowerLoad(shared_power_load);
      if (m_power_models[i].model.cyclePrognoserInputs())
      {
        // Current implementation means if one ModelHandler exceeds the draw
        // limit, every ModelHandler will exceed at the same amount.
        // So it doesn't matter which model triggers the warning as long as
        // it displays once.
        ROS_WARN_STREAM_THROTTLE(m_time_interval,
                           "Power system computed power input above the "
                        << m_max_gsap_input_watts
                        << "W cap for one or more GSAP prognosers.\n"
                        << "Input was set to "
                        << m_max_gsap_input_watts
                        << "W for each model over the limit.");
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
      printMechanicalPower(m_power_watts,
                           m_mean_mechanical_power,
                           m_moving_average_window);
    }

    // Check if the prediction handlers have EoD predictions ready,
    // and update EoD_events if so.
    for (int i = 0; i < m_active_models; i++)
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
      for (int i = 0; i < m_active_models; i++)
      {
        printPrognoserOutputs(m_EoD_events[i].remaining_useful_life,
                              m_EoD_events[i].state_of_charge,
                              m_EoD_events[i].battery_temperature,
                              i);
      }
    }

    // DEBUG PRINT FORMATTING
    // Skip a line for formatting purposes as long as a printout is being used.
    if (m_print_debug && m_power_models[0].input_info.timestamp > 0
        && (m_timestamp_print_debug
         || m_inputs_print_debug
         || m_outputs_print_debug
         || m_topics_print_debug
         || m_mech_power_print_debug))
    {
      std::cout << std::endl;
    }

    // Sleep for any remaining time in the loop that would cause it to
    // complete before the set rate. Warn the user if the loop somehow
    // took longer than the expected time interval.
    // This will almost always happen during initialization, so suppress the
    // warning during the first few loops.
    if (!rate.sleep() 
        && (m_power_models[0].input_info.timestamp > m_time_interval * 2))
    {
      ROS_WARN_STREAM("OW_POWER_SYSTEM WARNING: Main power system loop took"
                      << " longer than "
                      << std::to_string(m_time_interval)
                      << "s to complete a cycle.");
    }
  }

  spinner.stop();
}

/*
 * Used if the power system is disabled. Simply publishes static values
 * without using GSAP prognosers or reacting to faults. Mechanical power is
 * still calculated using jointStatesCb for publication.
 */
void PowerSystemNode::runDisabledLoop()
{
  // This rate object is used to sync the cycles up to the provided Hz.
  // NOTE: If the cycle takes longer than the provided Hz, nothing bad will
  //       necessarily occur, but the simulation will be out of sync with real
  //       time. Should essentially never happen with the disabled version of
  //       the loop.
  ros::Rate rate(m_loop_rate_hz);

  // Start the asynchronous spinner, which will call jointStatesCb as the
  // joint_states topic is published (50Hz).
  ros::AsyncSpinner spinner(m_spinner_threads);

  try
  {
    spinner.start();
  }
  catch(const std::runtime_error& e)
  {
    ROS_ERROR_STREAM("OW_POWER_SYSTEM ERROR: Encountered a std::runtime_error"
                    << " while starting up asynchronous ROS spinner threads!");
    return;
  }

  bool skip_first_loop = true;

  while (ros::ok())
  {
    // Create the published messages and set them to their ideal states.
    owl_msgs::BatteryRemainingUsefulLife rul_msg;
    owl_msgs::BatteryStateOfCharge soc_msg;
    owl_msgs::BatteryTemperature tmp_msg;

    rul_msg.value = m_max_horizon_secs;   // Will always publish the initial
    soc_msg.value = m_initial_soc;        // values
    tmp_msg.value = m_initial_temperature;

    // Apply the most recent timestamp to each message header.
    auto timestamp = ros::Time::now();
    rul_msg.header.stamp = timestamp;
    soc_msg.header.stamp = timestamp;
    tmp_msg.header.stamp = timestamp;

    // Publish the data.
    m_state_of_charge_pub.publish(soc_msg);
    m_remaining_useful_life_pub.publish(rul_msg);
    m_battery_temperature_pub.publish(tmp_msg);

    // While faults cannot be injected, this call is simply to allow the system
    // to warn the user if they attempt fault activation in this disabled state.
    // Skip on the first loop to prevent messages from being printed during
    // startup.
    if (skip_first_loop)
    {
      skip_first_loop = false;
    }
    else
    {
      injectFaults();
    }

    // Sleep for any remaining time in the loop that would cause it to
    // complete before the set rate.
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
  for (int i = 0; i < m_active_models; i++)
  {
    m_power_models[i].name = formatModelName(i);
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
    "/joint_states", 1, &PowerSystemNode::jointStatesCb, this
  );
  // Subscribe to electrical power to capture power used by non-arm actions
  m_electrical_power_sub = m_nh.subscribe(
    "/electrical_power", 1, &PowerSystemNode::electricalPowerCb, this
  );
}

/*
 * If a custom fault profile has been designated within the RQT window, this
 * function handles getting the relevant information from the file and activating
 * the fault. Also handles deactivation and any errors that might arise.
 */
void PowerSystemNode::injectCustomFault()
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

  if (!m_custom_power_fault_activated && fault_enabled)
  {
    // Block the fault if the power system is disabled.
    if (!m_enable_power_system)
    {
      printFaultDisabledWarning();
      return;
    }

    // Multiple potential points of failure, so alert user the process has started.
    ROS_INFO_STREAM("Attempting custom fault activation...");

    // Get user-entered file directory.
    ros::param::getCached("/faults/custom_fault_profile", designated_file);

    // Append the current fault directory to the stored file path.
    current_fault_path = ros::package::getPath("ow_power_system")
                                         + "/profiles/" 
                                         + designated_file;
    
    // Attempt to load/reload the designated file.
    if (designated_file == saved_file)
    {
      ROS_INFO_STREAM("Reloading " << designated_file << "...");
    }
    else if (saved_fault_path != "N/A")
    {
      ROS_INFO_STREAM("Loading " << designated_file 
                      << " and unloading " << saved_file << "...");
    }
    else
    {
      ROS_INFO_STREAM("Loading " << designated_file << "...");
    }

    // Attempt to load the profile.
    m_custom_power_fault_sequence = loadPowerProfile(current_fault_path, designated_file);

    if (m_custom_power_fault_sequence.size() > 0)
    {
      ROS_WARN_STREAM_ONCE(
        "Custom power faults may exhibit unexpected results. Caution is advised."
      );
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
      m_custom_power_fault_sequence_index = 0;
    }
    else
    {
      // Custom fault failed to load correctly.
      custom_fault_ready = false;
    }        
    m_custom_power_fault_activated = true;
  }
  else if (m_custom_power_fault_activated && !fault_enabled)
  {
    if (custom_fault_ready)
    {
      ROS_INFO_STREAM(saved_file << " deactivated!");
    }
    else
    {
      ROS_INFO_STREAM("Custom fault deactivated!");
    }
    m_custom_power_fault_activated = false;
    custom_fault_ready = false;
  }

  if (m_custom_power_fault_activated && fault_enabled && custom_fault_ready)
  {
    // Disable the fault upon reaching the end of the sequence.
    if (m_custom_power_fault_sequence_index >= m_custom_power_fault_sequence.size())
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
      auto data = m_custom_power_fault_sequence[m_custom_power_fault_sequence_index];

      // Evenly distribute the power draw from
      // the custom fault profile across each model.
      double wattage = data[MessageId::Watts] 
                        / (NUM_MODELS - m_deactivated_models);

      for (int i = 0; i < m_active_models; i++)
      {
        m_power_models[i].model.setCustomPowerDraw(wattage);
      }
      
      m_custom_power_fault_sequence_index++;
    }
  }
}

/*
 * Handles defined faults within the RQT window (currently only high power draw).
 */
void PowerSystemNode::injectFault(const std::string& fault_name)
{
  bool fault_enabled;
  double hpd_wattage = 0.0;
  int dbn_nodes = 0;

  // Get the value of fault_enabled.
  ros::param::getCached("/faults/" + fault_name, fault_enabled);

  // High power draw case
  if (fault_name == FAULT_NAME_HPD_ACTIVATE)
  {
    // Check if the fault has switched.
    if (!m_high_power_draw_activated && fault_enabled)
    {
      // Block the fault if the power system is disabled.
      if (!m_enable_power_system)
      {
        printFaultDisabledWarning();
        return;
      }

      ROS_INFO_STREAM(fault_name << " activated!");
      m_high_power_draw_activated = true;
    }
    else if (m_high_power_draw_activated && !fault_enabled)
    {
      ROS_INFO_STREAM(fault_name << " deactivated!");
      m_high_power_draw_activated = false;
    }

    // Continual behavior with HPD fault.
    if (m_high_power_draw_activated && fault_enabled)
    {
      // Update wattage based on the current value of the HPD slider.
      ros::param::getCached("/faults/" + FAULT_NAME_HPD, hpd_wattage);
      double split_wattage = hpd_wattage / (NUM_MODELS - m_deactivated_models);

      for (int i = 0; i < m_active_models; i++)
      {
        m_power_models[i].model.setHighPowerDraw(split_wattage);
      }
    }
  }

  // Disconnect battery nodes case
  if (fault_name == FAULT_NAME_DBN_ACTIVATE)
  {
    // Check if the fault has switched.
    if (!m_disconnect_battery_nodes_fault_activated && fault_enabled)
    {
      // Block the fault if the power system is disabled.
      if (!m_enable_power_system)
      {
        printFaultDisabledWarning();
        return;
      }

      ROS_INFO_STREAM(fault_name << " activated!");
      m_disconnect_battery_nodes_fault_activated = true;
    }
    else if (m_disconnect_battery_nodes_fault_activated && !fault_enabled)
    {
      ROS_INFO_STREAM(fault_name << " deactivated. Note that disconnected "
                    << "battery nodes cannot be reconnected, even if the fault "
                    << "is deactivated.");
      m_disconnect_battery_nodes_fault_activated = false;
    }

    // Continual behavior with DBN fault.
    if (m_disconnect_battery_nodes_fault_activated && fault_enabled)
    {
      // Get the number of nodes set by the user.
      ros::param::getCached("/faults/" + FAULT_NAME_DBN, dbn_nodes);

      // Warn the user periodically if they try to reduce the number of
      // deactivated nodes from a previous fault activation.
      if (m_deactivated_models > dbn_nodes)
      {
        ROS_WARN_STREAM_THROTTLE(30, fault_name << " currently set to "
                        << "disconnect " << std::to_string(dbn_nodes)
                        << " out of " << std::to_string(NUM_MODELS)
                        << " nodes when there are already "
                        << std::to_string(m_deactivated_models)
                        << " out of " << std::to_string(NUM_MODELS)
                        << " disconnected. Behavior has not changed.");
        ROS_WARN_STREAM_THROTTLE(30, "This warning will repeat periodically "
                        << "until the value of \"battery_nodes_to_disconnect\" is reset "
                        << std::to_string(m_deactivated_models)
                        << ", updated to disconnect more nodes, "
                        << "or the fault is deactivated.");
      }
      else if (m_deactivated_models < dbn_nodes)
      {
        ROS_INFO_STREAM(fault_name << " updated! "
                      << std::to_string(dbn_nodes)
                      << " out of " << std::to_string(NUM_MODELS) << " models "
                      << "are now disconnected.");
        // Update the number of active and deactivated nodes.
        m_deactivated_models = std::max(m_deactivated_models, dbn_nodes);
        m_active_models = std::min(m_active_models,
                                  (NUM_MODELS - m_deactivated_models));
      }
    }
  }

  // Low state of charge case
  if (fault_name == FAULT_NAME_LOW_SOC)
  {
    // Check if the fault has switched.
    if (!m_low_soc_activated && fault_enabled)
    {
      // Block the fault if the power system is disabled.
      if (!m_enable_power_system)
      {
        printFaultDisabledWarning();
        return;
      }


      ROS_INFO_STREAM(fault_name << " activated!");
      m_low_soc_activated = true;
      // Warn the user about the invalid behavior associated with power faults.
      printPowerFaultWarning();

      // Extra logic if the ICL fault was already activated, to immediately
      // bring SoC down to the threshold.
      if (m_icl_activated && m_prev_soc > POWER_SOC_MIN)
      {
        m_prev_soc = POWER_SOC_MIN;
      }
    }
    else if (m_low_soc_activated && !fault_enabled)
    {
      ROS_INFO_STREAM(fault_name << " deactivated!");
      m_low_soc_activated = false;
    }
    
    // There is no continual behavior with this fault. Its effect is applied in
    // publishPredictions().
  }

  // Instantaneous capacity loss case
  if (fault_name == FAULT_NAME_ICL)
  {
    // Check if the fault has switched.
    if (!m_icl_activated && fault_enabled)
    {
      // Block the fault if the power system is disabled.
      if (!m_enable_power_system)
      {
        printFaultDisabledWarning();
        return;
      }


      ROS_INFO_STREAM(fault_name << " activated!");
      m_icl_activated = true;
      // Warn the user about the invalid behavior associated with power faults.
      printPowerFaultWarning();
    }
    else if (m_icl_activated && !fault_enabled)
    {
      ROS_INFO_STREAM(fault_name << " deactivated!");
      m_icl_activated = false;
    }
    
    // There is no continual behavior with this fault. Its effect is applied in
    // publishPredictions().
  }

  // Thermal failure case
  if (fault_name == FAULT_NAME_THERMAL_FAILURE)
  {
    // Check if the fault has switched.
    if (!m_thermal_failure_activated && fault_enabled)
    {
      // Block the fault if the power system is disabled.
      if (!m_enable_power_system)
      {
        printFaultDisabledWarning();
        return;
      }


      ROS_INFO_STREAM(fault_name << " activated!");
      m_thermal_failure_activated = true;
      // Warn the user about the invalid behavior associated with power faults.
      printPowerFaultWarning();
    }
    else if (m_thermal_failure_activated && !fault_enabled)
    {
      ROS_INFO_STREAM(fault_name << " deactivated!");
      m_thermal_failure_activated = false;
    }
    
    // There is no continual behavior with this fault. Its effect is applied in
    // publishPredictions().
  }
}

/*
 * Basic function that calls the other fault-handling functions.
 */
void PowerSystemNode::injectFaults()
{
  // For every fault, its relevant call must be added here to check if it
  // should remain injected every cycle.
  injectFault(FAULT_NAME_HPD_ACTIVATE);
  injectFault(FAULT_NAME_DBN_ACTIVATE);
  injectFault(FAULT_NAME_LOW_SOC);
  injectFault(FAULT_NAME_ICL);
  injectFault(FAULT_NAME_THERMAL_FAILURE);
  injectCustomFault();
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

  // Remove the oldest value from the queue and add the newest one, though only
  // if the queue has already been populated up to the proper window size.
  double oldest_value = 0;
  if (m_queue_size == m_moving_average_window)
  {
    oldest_value = m_power_values.front();
    m_power_values.pop_front();
    m_power_values.push_back(m_power_watts);
  }
  else
  {
    // Queue is not fully populated yet, so just add the next value.
    m_power_values.push_back(m_power_watts);
    m_queue_size++;
  }

  // Update the sum & average by removing the previous value
  // and adding the new one.
  m_sum_mechanical_power +=  (m_power_watts - oldest_value);
  m_mean_mechanical_power = m_sum_mechanical_power / m_queue_size;

  // Publish raw & average mechanical power.
  std_msgs::Float64 mechanical_power_raw_msg, mechanical_power_avg_msg;
  mechanical_power_raw_msg.data = m_power_watts;
  mechanical_power_avg_msg.data = m_mean_mechanical_power;
  m_mechanical_power_raw_pub.publish(mechanical_power_raw_msg);
  m_mechanical_power_avg_pub.publish(mechanical_power_avg_msg);
}

/*
* Receives electrical power topic and saves locally.
*/
void PowerSystemNode::electricalPowerCb(const std_msgs::Float64 &msg)
{
  m_electrical_power = msg.data;
}

/*
 * Function called during custom fault injection that attempts to
 * load the specified file and its data.
 */
PrognoserVector PowerSystemNode::loadPowerProfile(const std::string& filename,
                                                  std::string custom_file)
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
      getline(line_stream, cell, ',');
      double file_time = std::stod(cell);
      auto timestamp = now + std::chrono::milliseconds(
                              static_cast<unsigned>(file_time * 1000)
                             );

      getline(line_stream, cell, ',');
      Datum<double> power(std::stod(cell));
      power.setTime(timestamp);

      // HACK ALERT: The PrognoserMap expects an entry for wattage, voltage,
      //             and temperature, but voltage & temperature inputs are not
      //             used by the prognosers beyond initialization. As such,
      //             fault profiles don't contain voltage or temperature
      //             columns. However, there still needs to be an entry for
      //             these in the PrognoserMap, so simply set them to 0.
      Datum<double> temperature(0.0);
      temperature.setTime(timestamp);

      Datum<double> voltage(0.0);
      voltage.setTime(timestamp);

      data.insert({ MessageId::Watts, power });
      data.insert({ MessageId::Centigrade, temperature});
      data.insert({ MessageId::Volts, voltage });

      result.push_back(data);

      line_number++;
    }
  }
  catch(...) // Many possible different errors could result
  {          // from reading an improperly formatted file.
    ROS_ERROR_STREAM("Failed to read " << custom_file << ":" << std::endl <<
                     "Improper formatting detected on line " << line_number << 
                     "." << std::endl << "Confirm " << custom_file << 
                     " follows the exact format of example_fault.csv before retrying.");
    return PrognoserVector();
  }
  return result;
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
  for (int i = 0; i < m_active_models; i++)
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
  // HACK ALERT: Because the power loop seems to run at least once during
  //             startup before everything is fully initialized, this
  //             fail state can trigger prematurely when everything is set
  //             to 0. As such, we prevent a fail state from triggering
  //             during the first few cycles of the battery loop by simply
  //             ignoring the battery status during that time.
  if ((min_rul < 0 || min_soc < 0)
      && (!m_battery_failed)
      && (m_power_models[0].input_info.timestamp >= (m_time_interval * 5)))
  {
    ROS_WARN_STREAM("The battery has reached a fail state. Flatlining "
                    << "all future published predictions");
    m_battery_failed = true;
  }

  // Alter the published values based on any forced power faults.
  if (m_low_soc_activated)
  {
    min_soc = POWER_SOC_MIN;
  }
  if (m_icl_activated)
  {
    min_soc = m_prev_soc * (1 - POWER_SOC_MAX_DIFF);
  }
  if (m_thermal_failure_activated)
  {
    max_tmp = POWER_THERMAL_MAX;
  }

  if (m_battery_failed)
  {
    min_rul = 0;
    min_soc = 0;
    max_tmp = 0;
  }

  // Store the current SoC for potential fault usage next cycle.
  m_prev_soc = min_soc;

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

static std::string formatModelName(int model_num)
{
  return "model " + std::to_string(model_num);
}

/*
 * Prints the timestamp of each cycle to the terminal.
 * Output: 1 line per cycle.
 */
static void printTimestamp(double timestamp)
{
  ROS_INFO_STREAM("Timestamp " << timestamp);
}

/*
 * Prints the input data sent to GSAP's asynchronous prognosers each cycle.
 * Output: m_active_models lines per cycle.
 */
static void printPrognoserInputs(double power,
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
 * Output: m_active_models lines per cycle.
 */
static void printPrognoserOutputs(double rul,
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
static void printMechanicalPower(double raw, double mean, int window)
{
  ROS_INFO_STREAM("Most recent raw mechanical power: " << std::to_string(raw));
  ROS_INFO_STREAM("Average of last " << std::to_string(window)
                  << " raw values: " << std::to_string(mean));
}

/*
 * Prints the information that will be published via rostopics each cycle.
 * Output: 1 line per cycle.
 */
static void printTopics(double rul, double soc, double tmp)
{
  ROS_INFO_STREAM("min_rul: " << std::to_string(rul) <<
                  ", min_soc: " << std::to_string(soc) <<
                  ", max_tmp: " << std::to_string(tmp));
}

/*
 * Prints a warning to the user should they attempt to inject a fault while
 * the power system is disabled. Only prints once.
 */
static void printFaultDisabledWarning()
{
  ROS_WARN_STREAM_THROTTLE(60, "Faults cannot be injected while the power system "
                    << "is disabled. Modify ow_power_system/config/system.cfg "
                    << "and restart the simulator to re-enable.");
}

/*
 * Prints a warning to the user when they inject one of the artificial power
 * faults.
 */
static void printPowerFaultWarning()
{
  ROS_WARN_STREAM("Note that artificial power fault injection simply overrides "
                  "GSAP's predictions. Remaining useful life will not be "
                  "affected and the system will continue without any loss of "
                  "battery health after the fault is cleared.");
}

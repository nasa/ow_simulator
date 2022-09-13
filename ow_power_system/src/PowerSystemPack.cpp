// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

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

void PowerSystemPack::InitAndRun()
{
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
  //builder.setLoadEstimatorName("Const");
  builder.setLoadEstimatorName("MovingAverage");

  // Create the prognosers using the builder that will send predictions using
  // their corresponding message bus.
  std::vector<PCOE::AsyncPrognoser> prognosers;
  for (int i = 0; i < NUM_NODES; i++)
  {
    prognosers.push_back(builder.build(m_nodes[i].bus, m_nodes[i].name, "trajectory"));
  }

  ROS_INFO_STREAM("Power system pack running.");

  ros::Rate rate(m_gsap_rate_hz);

  // TEST: SIMPLE VS ASYNC PROG
  double rul_median;
  double soc_median;
  double bat_temp;

  auto simple_config_path = ros::package::getPath("ow_power_system") + "/config/prognoser.cfg";
  ConfigMap simple_config(simple_config_path);
  std::unique_ptr<PCOE::Prognoser> simple_prog = PrognoserFactory::instance().Create("ModelBasedPrognoser", simple_config);

  // Initialize the GSAP prognoser. These values are taken directly from system.cfg,
  // just hard-coded since this is a test.
  auto init_data = PrognoserMap {
    { MessageId::Watts, Datum<double>{ 0.0 } },
    { MessageId::Volts, Datum<double>{ 4.1 } },
    { MessageId::Centigrade, Datum<double>{ 20.0 } }
    };
  simple_prog->step(init_data);
  // \TEST

  bool firstLoop[NUM_NODES];
  for (int i = 0; i < NUM_NODES; i++)
  {
    firstLoop[i] = true;
  }

  // Loop through the PowerSystemNodes to update their values and send them to the bus
  // to get predictions.
  while(ros::ok())
  {
    ros::spinOnce();

    // Set up fault values in each node for injection later.
    injectFaults();

    for (int i = 0; i < NUM_NODES; i++)
    {
      m_nodes[i].node.RunOnce();
      m_nodes[i].node.GetPowerStats(m_nodes[i].model.timestamp, m_nodes[i].model.wattage,
                                    m_nodes[i].model.voltage, m_nodes[i].model.temperature);
      //ROS_INFO_STREAM("Calling RunOnce on node " << i << "..."); TEST

      // TEST: SIMPLE VS ASYNC PROG
      if (i == 0)
      {
        auto current_data = PrognoserMap {
          { MessageId::Watts, Datum<double>{ m_nodes[i].model.wattage } },
          { MessageId::Volts, Datum<double>{ m_nodes[i].model.voltage } },
          { MessageId::Centigrade, Datum<double>{ m_nodes[i].model.temperature } }
          };
        auto prediction = simple_prog->step(current_data);
        auto& eod_events = prediction.getEvents();
        if (!eod_events.empty())
        {
          auto eod_event = eod_events.front();

          UData eod_time = eod_event.getTOE();
          if (eod_time.uncertainty() != UType::Samples)
          {
            // Log warning and don't update the last value
            ROS_WARN_NAMED("power_system_node", "Unexpected uncertainty type for EoD prediction");
            //return;
          }

          // valid prediction
          // Determine the median RUL.
          auto samplesRUL = eod_time.getVec();
          sort(samplesRUL.begin(), samplesRUL.end());
          double eod_median = samplesRUL.at(samplesRUL.size() / 2);
          auto now = MessageClock::now();
          auto now_s = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
          rul_median = eod_median - now_s.count();

          // Determine the median SOC.
          UData currentSOC = eod_event.getState()[0];
          auto samplesSOC = currentSOC.getVec();
          sort(samplesSOC.begin(), samplesSOC.end());
          soc_median = samplesSOC.at(samplesSOC.size() / 2);

          // Determine the Battery Temperature
          auto stateSamples = eod_event.getSystemState()[0];
          std::vector<double> state;
          for (auto sample : stateSamples)
            state.push_back(sample[0]);

          auto& model = dynamic_cast<ModelBasedPrognoser*>(simple_prog.get())->getModel();
          auto model_output = model.outputEqn(now_s.count(), static_cast<PrognosticsModel::state_type>(state));
          bat_temp = model_output[1];

          // EoD values printed later in the loop.
        }
      }
      // \TEST

      // /* DEBUG PRINT
      if (!(m_nodes[i].model.timestamp <= 0))
      {
        ROS_INFO_STREAM("Node " << i << "  time: " << m_nodes[i].model.timestamp
                        << ". power: " << m_nodes[i].model.wattage << ".  volts: "
                        << m_nodes[i].model.voltage << ".  temp: " << m_nodes[i].model.temperature);
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
        double input_temp;
        double input_voltage;

        if (firstLoop[i])
        {
          // The very first values sent in should be the init values.
          input_power = m_initial_power;
          input_temp = m_initial_temperature;
          input_voltage = m_initial_voltage;
          firstLoop[i] = false;
        }
        else
        {
          input_power = m_nodes[i].model.wattage;
          input_temp = m_nodes[i].model.temperature;
          input_voltage = m_nodes[i].model.voltage;
        }

        data_to_publish.push_back(
          std::make_shared<DoubleMessage>(MessageId::Watts, m_nodes[i].name, timestamp, input_power));
        data_to_publish.push_back(
          std::make_shared<DoubleMessage>(MessageId::Centigrade, m_nodes[i].name, timestamp, input_temp));
        data_to_publish.push_back(
          std::make_shared<DoubleMessage>(MessageId::Volts, m_nodes[i].name, timestamp, input_voltage));

      
        m_nodes[i].previous_time = m_nodes[i].model.timestamp;
        for (const auto& info : data_to_publish)
        {
          m_nodes[i].bus.publish(info);
        }
      }
    }

    // /* DEBUG PRINT
    if (!(m_nodes[0].model.timestamp <= 0))
    {
      ROS_INFO_STREAM("Waiting for all...");
    }
    // */

    for (int i = 0; i < NUM_NODES; i++)
    {
      m_nodes[i].bus.waitAll();
    }

    // /* DEBUG PRINT
    if (!(m_nodes[0].model.timestamp <= 0))
    {
      ROS_INFO_STREAM("Waited for all!");
    }
    // */

    // TEST: SIMPLE VS ASYNC PROG
    ROS_INFO_STREAM("SIMPLE PROG RUL: " << std::to_string(rul_median) << ", SOC: "
                    << std::to_string(soc_median) << ", TMP: " << std::to_string(bat_temp));
    // \TEST

    // /* DEBUG PRINT
    for (int i = 0; i < NUM_NODES; i++)
    {
      // Display output if the time is past the initial startup phase, no matter what
      // (for debugging purposes).
      if ((!(m_EoD_events[i].remaining_useful_life <= 0)) || (m_nodes[i].model.timestamp >= 50))
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
    // */

    // Now that EoD_events is ready, manipulate and publish the relevant values.
    publishPredictions();

    rate.sleep();
  }
}

bool PowerSystemPack::initNodes()
{
  // Initialize the nodes.
  for (int i = 0; i < NUM_NODES; i++)
  {
    m_nodes[i].name = setNodeName(i);
    if (!m_nodes[i].node.Initialize(NUM_NODES))
    {
        return false;
    }
  }
  return true;
}

bool PowerSystemPack::initTopics()
{
  // Construct the PowerSystemNode publishers
  m_mechanical_power_raw_pub = m_nh.advertise<std_msgs::Float64>("mechanical_power/raw", 1);
  m_mechanical_power_avg_pub = m_nh.advertise<std_msgs::Float64>("mechanical_power/average", 1);
  m_state_of_charge_pub = m_nh.advertise<std_msgs::Float64>("power_system_node/state_of_charge", 1);
  m_remaining_useful_life_pub = m_nh.advertise<std_msgs::Int16>("power_system_node/remaining_useful_life", 1);
  m_battery_temperature_pub = m_nh.advertise<std_msgs::Float64>("power_system_node/battery_temperature", 1);
  // Finally subscribe to the joint_states to estimate the mechanical power
  m_joint_states_sub = m_nh.subscribe("/joint_states", 1, &PowerSystemPack::jointStatesCb, this);
  return true;
}

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
    // TODO: Unspecified how to handle end of fault profile. For now, simply disable
    // the fault from updating any parameters.
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

      // Pass in an evenly distributed amount of high power draw to each node.
      double wattage = data[MessageId::Watts] / NUM_NODES;

      for (int i = 0; i < NUM_NODES; i++)
      {
        m_nodes[i].node.SetCustomPowerDraw(wattage);
      }
      
      index += m_profile_increment;
    }
  }
}

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
  }
}

void PowerSystemPack::injectFaults()
{
  injectFault(FAULT_NAME_HPD_ACTIVATE,
              m_high_power_draw_activated);
  injectCustomFault(m_custom_power_fault_activated,
                    m_custom_power_fault_sequence,
                    m_custom_power_fault_sequence_index);
}

// TODO: All the fault injection mechanisms need to be updated.

void PowerSystemPack::jointStatesCb(const sensor_msgs::JointStateConstPtr& msg)
{
  /* DEBUG
  ROS_INFO_STREAM("Pack jointStatesCb called!!!");
  */

  // NOTE: This callback function appears to call after the nodes' callback
  //       functions complete, every single time. This is quite convenient since
  //       it depends on values determined from those callbacks, but
  //       I don't know why exactly this is the case. If they should happen
  //       to stop calling in this order, it could potentially cause problems.
  //       ~Liam
  
  // Get the mechanical power values from each node.
  double raw_mechanical_values[NUM_NODES];
  double avg_mechanical_values[NUM_NODES];
  double avg_power = 0.0;

  /* DEBUG
  bool differing_avgs = false;
  */

  for (int i = 0; i < NUM_NODES; i++)
  {
    raw_mechanical_values[i] = m_nodes[i].node.GetRawMechanicalPower();
    avg_mechanical_values[i] = m_nodes[i].node.GetAvgMechanicalPower();
    avg_power += raw_mechanical_values[i];
    /* DEBUG PRINT
    if (i > 0)
    {
      if (avg_mechanical_values[i] != avg_mechanical_values[i - 1])
      {
        ROS_ERROR_STREAM("Average mechanical values differ: Node " <<
                         std::to_string(i - 1) << " AMP is " << avg_mechanical_values[i - 1] <<
                         " while Node " << std::to_string(i) << " AMP is " << avg_mechanical_values[i]);
        // DEBUG
        differing_avgs = true;
        
      }
    }
    */
  }

  // Publish the mechanical raw and average power values.
  // Raw mechanical power should be averaged out, but the avg_power should
  // already be the same value in every node.
  avg_power = avg_power / NUM_NODES;

  std_msgs::Float64 mechanical_power_raw_msg, mechanical_power_avg_msg;
  mechanical_power_raw_msg.data = avg_power;
  /* DEBUG PRINT
  if (!differing_avgs)
  {
    ROS_INFO_STREAM("All avg mechanical values were equal to " <<
                    std::to_string(avg_mechanical_values[0]) << "!");
  }
  */
  /* DEBUG PRINT
  ROS_INFO_STREAM("Raw mechanical power (averaged) is " << std::to_string(avg_power) << "!");
  */
  // Since all average mechanical power values should be identical, it doesn't
  // matter which node's value we take.
  mechanical_power_avg_msg.data = avg_mechanical_values[0];
  m_mechanical_power_raw_pub.publish(mechanical_power_raw_msg);
  m_mechanical_power_avg_pub.publish(mechanical_power_avg_msg);
}

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

      // NOTE: The voltage and temperature values of a PrognoserVector are non-functional
      //       when it comes to fault injection. Only power values are used. As such,
      //       voltage and temperature are initialized to 0 and the fault profile
      //       should not contain them.
      getline(line_stream, cell, ',');
      double file_time = std::stod(cell);
      auto timestamp = now + std::chrono::milliseconds(static_cast<unsigned>(file_time * 1000));

      getline(line_stream, cell, ',');
      Datum<double> power(std::stod(cell));
      power.setTime(timestamp);

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
  catch(...) // Many possible different errors could result from reading an improperly formatted file.
  {
    ROS_ERROR_STREAM("Failed to read " << custom_file << ":" << std::endl <<
                     "Improper formatting detected on line " << line_number << "." << std::endl << 
                     "Confirm " << custom_file << " follows the exact format of example_fault.csv before retrying.");
    return PrognoserVector();
  }
  return result;
}

bool PowerSystemPack::loadCustomFaultPowerProfile(std::string path, std::string custom_file)
{
  m_custom_power_fault_sequence = loadPowerProfile(path, custom_file);

  // Return false if the sequence was not properly initialized.
  return (m_custom_power_fault_sequence.size() > 0);
}

void PowerSystemPack::publishPredictions()
{
  /* DEBUG
  ROS_INFO_STREAM("publishPredictions called!");
  */
  // Using EoD_events, publish the relevant values.

  int min_rul = -1;
  double min_soc = -1;
  double max_tmp = -1;
  std_msgs::Int16 rul_msg;
  std_msgs::Float64 soc_msg;
  std_msgs::Float64 tmp_msg;

  for (int i = 0; i < NUM_NODES; i++)
  {
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
    
    // Published battery temperature is defined as the highest temp of all EoDs.
    if (m_EoD_events[i].battery_temperature > max_tmp || max_tmp == -1)
    {
      max_tmp = m_EoD_events[i].battery_temperature;
    }
  }

  // /* DEBUG PRINT
  if ((!(min_rul < 0 || min_soc < 0 || max_tmp < 0)) || (m_nodes[0].model.timestamp >= 50))
  {
    ROS_INFO_STREAM("min_rul: " << std::to_string(min_rul) <<
                    ", min_soc: " << std::to_string(min_soc) <<
                    ", max_tmp: " << std::to_string(max_tmp));
  }
  // */

  // Publish the values for other components.
  rul_msg.data = min_rul;
  soc_msg.data = min_soc;
  tmp_msg.data = max_tmp;

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

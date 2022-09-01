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
  builder.setLoadEstimatorName("Const");

  // Create the prognosers using the builder that will send predictions using
  // their corresponding message bus.
  std::vector<PCOE::AsyncPrognoser> prognosers;
  for (int i = 0; i < NUM_NODES; i++)
  {
    prognosers.push_back(builder.build(m_nodes[i].bus, m_nodes[i].name, "trajectory"));
  }

  ROS_INFO_STREAM("Power system pack running.");

  ros::Rate rate(m_gsap_rate_hz);

  // Loop through the PowerSystemNodes to update their values and send them to the bus
  // to get predictions.
  while(ros::ok())
  {
    ros::spinOnce();
    for (int i = 0; i < NUM_NODES; i++)
    {
      m_nodes[i].node.RunOnce();
      m_nodes[i].node.GetPowerStats(m_nodes[i].model.timestamp, m_nodes[i].model.wattage,
                                    m_nodes[i].model.voltage, m_nodes[i].model.temperature);

      // /* DEBUG PRINT
      if (!(m_nodes[i].model.timestamp <= 0))
      {
        ROS_INFO_STREAM("Node " << i << "  time: " << m_nodes[i].model.timestamp
                        << ". power: " << m_nodes[i].model.wattage << ".  volts: "
                        << m_nodes[i].model.voltage << ".  temp: " << m_nodes[i].model.temperature);
      }
      // */

      auto timestamp = START_TIME + std::chrono::milliseconds(
        static_cast<unsigned>(m_nodes[i].model.timestamp * 1000));

      // Compile a vector<shared_ptr<DoubleMessage>> and then individually publish
      // each individual component.
      std::vector<std::shared_ptr<DoubleMessage>> data_to_publish;

      data_to_publish.push_back(
        std::make_shared<DoubleMessage>(MessageId::Watts, m_nodes[i].name, timestamp, m_nodes[i].model.wattage));
      data_to_publish.push_back(
        std::make_shared<DoubleMessage>(MessageId::Centigrade, m_nodes[i].name, timestamp, m_nodes[i].model.temperature));
      data_to_publish.push_back(
        std::make_shared<DoubleMessage>(MessageId::Volts, m_nodes[i].name, timestamp, m_nodes[i].model.voltage));

      // If the timestamp is the same as the previous one (happens during startup),
      // do not publish the data to prevent terminate crashes.
      if (m_nodes[i].previous_time != m_nodes[i].model.timestamp)
      {
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

    // /* DEBUG PRINT
    for (int i = 0; i < NUM_NODES; i++)
    {
      if (!(m_EoD_events[i].remaining_useful_life <= 0))
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

void PowerSystemPack::jointStatesCb(const sensor_msgs::JointStateConstPtr& msg)
{
  /* DEBUG
  ROS_INFO_STREAM("jointStatesCb called!!!");
  */
}

void PowerSystemPack::publishPredictions()
{
  /* DEBUG
  ROS_INFO_STREAM("publishPredictions called!");
  */
  // Using EoD_events, publish the relevant values.
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

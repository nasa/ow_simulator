// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <vector>
#include <numeric>
#include <math.h>
#include <algorithm>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <ow_lander/lander_joints.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <PrognoserFactory.h>

#include "Messages/MessageBus.h"
#include "Messages/ProgEventMessage.h"
#include "Messages/ScalarMessage.h"
#include "ModelBasedAsyncPrognoserBuilder.h"

#include "PowerSystemPack.h"
#include "PowerSystemNode.h"

using namespace PCOE;

const auto START_TIME       = MessageClock::now();
//const int NUM_NODES         = 8; // NOTE: Should equal m_num_nodes in PowerSystemPack.h

// The indices use to access temperature information.
// This might change to median SOC or RUL index or fixed percentile.
//
static constexpr int BATTERY_TEMPERATURE_INDEX = 0;
static constexpr int MODEL_TEMPERATURE_INDEX = 1;

double EoD_events[NUM_NODES][3];

// The PredictionHandler class subscribes to the battery EoD event message and
// prints each event as it is received.
class PredictionHandler : public IMessageProcessor 
{
public:
  /**
   * Constructs a new prediction handler that subscribes to battery EoD
   * predictions for the specified source and on the specified message
   * bus.
   **/
  PredictionHandler(MessageBus& bus, const std::string& src, int node_num) : bus(bus)
  {
    bus.subscribe(this, src, MessageId::BatteryEod);
    identifier = src;
    node_number = node_num;
  }

  /**
   * Unsubscribes the prediction handler from the message bus.
   **/
  ~PredictionHandler()
  {
    bus.unsubscribe(this);
  }

  /**
   * The message bus will call this function each time the predictor publishes
   * a new battery EoD prediction.
   **/
  void processMessage(const std::shared_ptr<Message>& message) override
  {
    using namespace std::chrono;
    // The prediction handler only ever subscribes to the BatteryEoD message
    // id, which should always be a ProgEventMessage, so this should always
    // succeed.
    auto prediction_msg = dynamic_cast<ProgEventMessage*>(message.get());
    if (prediction_msg == nullptr)
    {
      ROS_ERROR("Failed to cast prediction message to expected type");
      return;
    }

    // Get the event for battery EoD
    auto eod_event = prediction_msg->getValue();

    // The time of event is a `UData` structure, which represents a data
    // point while maintaining uncertainty. For the MonteCarlo predictor
    // used by this example, the uncertainty is captured by storing the
    // result of each particle used in the prediction.
    UData eod_time = eod_event.getTOE();
    if (eod_time.uncertainty() != UType::Samples)
    {
      ROS_ERROR("Unexpected uncertainty type for EoD prediction");
      return;
    }

    // valid prediction
    // Determine the median RUL.
    /*auto samplesRUL = eod_time.getVec();
    sort(samplesRUL.begin(), samplesRUL.end());
    double eod_median = samplesRUL.at(samplesRUL.size() / 2);*/
    double eod_median = findMedian(eod_time.getVec());
    auto now = MessageClock::now();
    auto now_s = duration_cast<std::chrono::seconds>(now.time_since_epoch());
    double rul_median = eod_median - now_s.count();
    //rul_msg.data = rul_median;

    // Determine the median SOC.
    UData currentSOC = eod_event.getState()[0];
    /*auto samplesSOC = currentSOC.getVec();
    sort(samplesSOC.begin(), samplesSOC.end());
    double soc_median = samplesSOC.at(samplesSOC.size() / 2);*/
    double soc_median = findMedian(currentSOC.getVec());
    //soc_msg.data = soc_median;

    // Determine the Battery Temperature
    auto stateSamples = eod_event.getSystemState()[BATTERY_TEMPERATURE_INDEX];
    std::vector<double> temperature_state;
    for (auto sample : stateSamples)
    {
      temperature_state.push_back(sample[BATTERY_TEMPERATURE_INDEX]);
    }

    // HACK ALERT:
    // The asynchronous prognoser does not have a function to get its model like
    // the simple prognoser does. As such, one can instantiate a simple prognoser
    // and get its model; it should be identical. However, a getter function would
    // be much more ideal and more efficient.
    // Construct a simple prognoser to get the battery model.
    auto prognoser_config_path = ros::package::getPath("ow_power_system") + "/config/prognoser.cfg";
    ConfigMap prognoser_config(prognoser_config_path);
    std::unique_ptr<PCOE::Prognoser> temp_prog =
      PrognoserFactory::instance().Create("ModelBasedPrognoser", prognoser_config);

    auto& model = dynamic_cast<ModelBasedPrognoser*>(temp_prog.get())->getModel();

    auto model_output = model.outputEqn(now_s.count(), static_cast<PrognosticsModel::state_type>(temperature_state));

    // Store the newly obtained data.
    EoD_events[node_number][0] = rul_median;
    EoD_events[node_number][1] = soc_median;
    EoD_events[node_number][2] = model_output[MODEL_TEMPERATURE_INDEX];
  }

private:
  MessageBus& bus;
  std::string identifier;
  int node_number;

  double findMedian(std::vector<double> samples)
  {
    std::nth_element(samples.begin(), (samples.begin() + (samples.size() / 2)),
                     samples.end());
    return samples.at(samples.size() / 2);
    /* TEST CODE
    if (samples.size() % 2 == 0)
    {
      // Even size set, median is the average of the middle 2 values.
      double first_val = samples.at(samples.size() / 2);
      std::nth_element(samples.begin(), (samples.begin() + (samples.size() / 2) + 1),
                       samples.end());
      std::cout << "even findM eod_median: " << std::to_string((first_val + 
                    samples.at((samples.size() / 2) + 1)) / 2) << std::endl;
      return ((first_val + samples.at((samples.size() / 2) + 1)) / 2);
    }
    else
    {
      // Odd size set, median is the middle value.
      std::cout << "odd findM eod_median:  " << std::to_string(samples.at(samples.size() / 2)) << std::endl;
      return samples.at(samples.size() / 2);
    }
    */
  }
};

PowerSystemPack::PowerSystemPack()
{ }

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
    m_previous_times[i] = 0;
    for (int j = 0 ; j < 3; j++)
    {
      EoD_events[i][j] = -1;
    }
  }

  // Construct the prediction handlers.
  PredictionHandler handler_0(m_bus[0], m_node_names[0], 0);
  PredictionHandler handler_1(m_bus[1], m_node_names[1], 1);
  PredictionHandler handler_2(m_bus[2], m_node_names[2], 2);
  PredictionHandler handler_3(m_bus[3], m_node_names[3], 3);
  PredictionHandler handler_4(m_bus[4], m_node_names[4], 4);
  PredictionHandler handler_5(m_bus[5], m_node_names[5], 5);
  PredictionHandler handler_6(m_bus[6], m_node_names[6], 6);
  PredictionHandler handler_7(m_bus[7], m_node_names[7], 7);

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
  PCOE::AsyncPrognoser prognoser_0 = builder.build(m_bus[0], m_node_names[0], "trajectory");
  PCOE::AsyncPrognoser prognoser_1 = builder.build(m_bus[1], m_node_names[1], "trajectory");
  PCOE::AsyncPrognoser prognoser_2 = builder.build(m_bus[2], m_node_names[2], "trajectory");
  PCOE::AsyncPrognoser prognoser_3 = builder.build(m_bus[3], m_node_names[3], "trajectory");
  PCOE::AsyncPrognoser prognoser_4 = builder.build(m_bus[4], m_node_names[4], "trajectory");
  PCOE::AsyncPrognoser prognoser_5 = builder.build(m_bus[5], m_node_names[5], "trajectory");
  PCOE::AsyncPrognoser prognoser_6 = builder.build(m_bus[6], m_node_names[6], "trajectory");
  PCOE::AsyncPrognoser prognoser_7 = builder.build(m_bus[7], m_node_names[7], "trajectory");

  ROS_INFO_STREAM("Power system pack running.");

  ros::Rate rate(m_gsap_rate_hz);

  // Loop through the PowerSystemNodes to update their values and send them to the bus
  // to get predictions.
  while(ros::ok())
  {
    ros::spinOnce();
    for (int i = 0; i < NUM_NODES; i++)
    {
      //ROS_INFO_STREAM("Calling RunOnce on node " << i << "..."); TEST
      m_nodes[i].RunOnce();
      m_nodes[i].GetPowerStats(m_models[i]);

      // /* DEBUG PRINT
      if (!(m_models[i][0] <= 0))
      {
        ROS_INFO_STREAM("Node " << i << "  time: " << m_models[i][0]
                        << ". power: " << m_models[i][1] << ".  volts: "
                        << m_models[i][2] << ".  temp: " << m_models[i][3]);
      }
      // */

      auto timestamp = START_TIME + std::chrono::milliseconds(static_cast<unsigned>(m_models[i][0] * 1000));

      // Compile a vector<shared_ptr<DoubleMessage>> and then individually publish
      // each individual component.
      std::vector<std::shared_ptr<DoubleMessage>> data_to_publish;

      data_to_publish.push_back(
        std::make_shared<DoubleMessage>(MessageId::Watts, m_node_names[i], timestamp, m_models[i][1]));
      data_to_publish.push_back(
        std::make_shared<DoubleMessage>(MessageId::Centigrade, m_node_names[i], timestamp, m_models[i][3]));
      data_to_publish.push_back(
        std::make_shared<DoubleMessage>(MessageId::Volts, m_node_names[i], timestamp, m_models[i][2]));

      //std::this_thread::sleep_until(timestamp);

      // If the timestamp is the same as the previous one (happens during startup),
      // do not publish the data to prevent terminate crashes.
      if (m_previous_times[i] != m_models[i][0])
      {
        m_previous_times[i] = m_models[i][0];
        for (const auto& info : data_to_publish)
        {
          m_bus[i].publish(info);
          //ROS_INFO_STREAM("Published info: " << info); TEST
        }

      }
    }

    // /* DEBUG PRINT
    if (!(m_models[0][0] <= 0))
    {
      ROS_INFO_STREAM("Waiting for all...");
    }
    // */
    for (int i = 0; i < NUM_NODES; i++)
    {
      m_bus[i].waitAll();
    }
    // /* DEBUG PRINT
    if (!(m_models[0][0] <= 0))
    {
      ROS_INFO_STREAM("Waited for all!");
    }
    // */

    // /* DEBUG PRINT
    for (int i = 0; i < NUM_NODES; i++)
    {
      if (!(EoD_events[i][0] <= 0))
      {
        ROS_INFO_STREAM("Node " << i << " RUL: " << EoD_events[i][0]
                        << ". SOC: " << EoD_events[i][1] << ". TMP: "
                        << EoD_events[i][2]);
      }
    }
    if (!(m_models[0][0] <= 0))
    {
      std::cout << std::endl;
    }
    // */
    // Now that EoD_events is ready, manipulate and publish the relevant values.
    publishPredictions();

    // NEXT STEP: Each override function should have updated the EoD values
    // within the EoD_events matrix. We need to convert these different values
    // into one of each to publish. What about the mechanical_power values that
    // are also published in PowerSystemNode? How do we handle those?
  }
}

bool PowerSystemPack::initNodes()
{
  // Initialize the nodes.
  for (int i = 0; i < NUM_NODES; i++)
  {
    m_node_names[i] = "Node " + std::to_string(i);
    if (!m_nodes[i].Initialize(NUM_NODES))
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

int main(int argc, char* argv[]) 
{
  ros::init(argc, argv, "power_system_node");

  PowerSystemPack pack;

  pack.InitAndRun();

  return 0;

  //std::cout << "READ FILE!" << std::endl; TEST

  //std::cout << "READ CONFIGURATION!" << std::endl; TEST

  // The handler is the first thing that subscribes to the message bus. Its
  // constructor tells the bus that it wants to know about any predictions
  // that are produced for the thing identified by `node_names[x]`.

  //std::cout << "CREATED HANDLERS!" << std::endl; TEST

  //std::cout << "CREATED BUILDER!" << std::endl; TEST

  //std::cout << "CONSTRUCTED BUILDERS!" << std::endl; TEST

  // Store the timestamp/wattage/voltage/temperature of each series model.

  // Run the power nodes at the same rate as GSAP.

  // Loop the PowerSystemNodes to update their values and send them to the bus.
  
}

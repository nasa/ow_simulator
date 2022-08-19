// TODO (JW): By its nature, this file is designed to be copied. It should
// be explicitely (un)licensed as public domain or CC0.
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

// The index use to access temperature information.
// This might change to median SOC or RUL index or fixed percentile.
//
static constexpr int TEMPERATURE_INDEX = 1;

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
  PredictionHandler(MessageBus& bus, const std::string& src) : bus(bus)
  {
    bus.subscribe(this, src, MessageId::BatteryEod);
    identifier = src;
    node_number = src[5] - '0'; // converts node identifier value to int
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
      std::cerr << "Failed to cast prediction message to expected type" << std::endl;
      std::exit(1);
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
      std::cerr << "Unexpected uncertainty type for EoD prediction" << std::endl;
      return std::exit(1);
    }

    // valid prediction
    // Determine the median RUL.
    auto samplesRUL = eod_time.getVec();
    sort(samplesRUL.begin(), samplesRUL.end());
    double eod_median = samplesRUL.at(samplesRUL.size() / 2);
    auto now = MessageClock::now();
    auto now_s = duration_cast<std::chrono::seconds>(now.time_since_epoch());
    double rul_median = eod_median - now_s.count();
    //rul_msg.data = rul_median;

    // Determine the median SOC.
    UData currentSOC = eod_event.getState()[0];
    auto samplesSOC = currentSOC.getVec();
    sort(samplesSOC.begin(), samplesSOC.end());
    double soc_median = samplesSOC.at(samplesSOC.size() / 2);
    //soc_msg.data = soc_median;

    // Determine the Battery Temperature
    auto stateSamples = eod_event.getSystemState()[0];
    std::vector<double> state;
    for (auto sample : stateSamples)
    {
      state.push_back(sample[0]);
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

    auto model_output = model.outputEqn(now_s.count(), static_cast<PrognosticsModel::state_type>(state));

    // Store the newly obtained data.
    EoD_events[node_number][0] = rul_median;
    EoD_events[node_number][1] = soc_median;
    EoD_events[node_number][2] = model_output[TEMPERATURE_INDEX];
  }

private:
  MessageBus& bus;
  std::string identifier = "Node X";
  int node_number = -1;
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
    previous_times[i] = 0;
    for (int j = 0 ; j < 3; j++)
    {
      EoD_events[i][j] = -1;
    }
  }

  // Construct the prediction handlers.
  PredictionHandler handler_0(bus[0], node_names[0]);
  PredictionHandler handler_1(bus[1], node_names[1]);
  PredictionHandler handler_2(bus[2], node_names[2]);
  PredictionHandler handler_3(bus[3], node_names[3]);
  PredictionHandler handler_4(bus[4], node_names[4]);
  PredictionHandler handler_5(bus[5], node_names[5]);
  PredictionHandler handler_6(bus[6], node_names[6]);
  PredictionHandler handler_7(bus[7], node_names[7]);

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
  PCOE::AsyncPrognoser prognoser_0 = builder.build(bus[0], node_names[0], "trajectory");
  PCOE::AsyncPrognoser prognoser_1 = builder.build(bus[1], node_names[1], "trajectory");
  PCOE::AsyncPrognoser prognoser_2 = builder.build(bus[2], node_names[2], "trajectory");
  PCOE::AsyncPrognoser prognoser_3 = builder.build(bus[3], node_names[3], "trajectory");
  PCOE::AsyncPrognoser prognoser_4 = builder.build(bus[4], node_names[4], "trajectory");
  PCOE::AsyncPrognoser prognoser_5 = builder.build(bus[5], node_names[5], "trajectory");
  PCOE::AsyncPrognoser prognoser_6 = builder.build(bus[6], node_names[6], "trajectory");
  PCOE::AsyncPrognoser prognoser_7 = builder.build(bus[7], node_names[7], "trajectory");

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
      nodes[i].RunOnce();
      nodes[i].GetPowerStats(models[i]);

      // /* DEBUG PRINT
      if (!(models[i][0] <= 0))
      {
        ROS_INFO_STREAM("Node " << i << "  time: " << models[i][0]
                        << ". power: " << models[i][1] << ".  volts: "
                        << models[i][2] << ".  temp: " << models[i][3]);
      }
      // */

      auto timestamp = START_TIME + std::chrono::milliseconds(static_cast<unsigned>(models[i][0] * 1000));

      // Compile a vector<shared_ptr<DoubleMessage>> and then individually publish
      // each individual component.
      std::vector<std::shared_ptr<DoubleMessage>> data_to_publish;

      data_to_publish.push_back(
        std::make_shared<DoubleMessage>(MessageId::Watts, node_names[i], timestamp, models[i][1]));
      data_to_publish.push_back(
        std::make_shared<DoubleMessage>(MessageId::Centigrade, node_names[i], timestamp, models[i][3]));
      data_to_publish.push_back(
        std::make_shared<DoubleMessage>(MessageId::Volts, node_names[i], timestamp, models[i][2]));

      std::this_thread::sleep_until(timestamp);

      // If the timestamp is the same as the previous one (happens during startup),
      // do not publish the data to prevent terminate crashes.
      if (previous_times[i] != models[i][0])
      {
        previous_times[i] = models[i][0];
        for (const auto& info : data_to_publish)
        {
          bus[i].publish(info);
          //ROS_INFO_STREAM("Published info: " << info); TEST
        }

      }
    }

    // /* DEBUG PRINT
    if (!(models[0][0] <= 0))
    {
      ROS_INFO_STREAM("Waiting for all...");
    }
    // */
    for (int i = 0; i < NUM_NODES; i++)
    {
      bus[i].waitAll();
    }
    // /* DEBUG PRINT
    if (!(models[0][0] <= 0))
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
    if (!(models[0][0] <= 0))
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
    node_names[i] = "Node " + std::to_string(i);
    if (!nodes[i].Initialize(NUM_NODES))
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
  
  // Publish the mechanical raw and average power values.
  // The callback function is still present within the individual nodes, where
  // it is used to calculate m_mechanical_power_to_be_processed.
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
    if (EoD_events[i][0] < min_rul || min_rul == -1)
    {
      min_rul = EoD_events[i][0];
    }

    // Published SoC (state of charge) is defined as the minimum SoC of all EoDs.
    if (EoD_events[i][1] < min_soc || min_soc == -1)
    {
      min_soc = EoD_events[i][1];
    }
    
    // Published battery temperature is defined as the highest temp of all EoDs.
    if (EoD_events[i][2] > max_tmp || max_tmp == -1)
    {
      max_tmp = EoD_events[i][2];
    }
  }

  // /* DEBUG PRINT
  ROS_INFO_STREAM("min_rul: " << std::to_string(min_rul) <<
                  ", min_soc: " << std::to_string(min_soc) <<
                  ", max_tmp: " << std::to_string(max_tmp));
  // */

  // Publish the values for other components.
  rul_msg.data = min_rul;
  soc_msg.data = min_soc;
  tmp_msg.data = max_tmp;

  m_state_of_charge_pub.publish(soc_msg);
  m_remaining_useful_life_pub.publish(rul_msg);
  m_battery_temperature_pub.publish(tmp_msg);
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

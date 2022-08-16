// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

#include <numeric>
#include <fstream>
#include <math.h>
#include <algorithm>
#include <vector>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <ow_lander/lander_joints.h>
#include "PowerSystemNode.h"

#include "Messages/MessageBus.h"
#include "Messages/ProgEventMessage.h"
#include "Messages/ScalarMessage.h"
#include "ModelBasedAsyncPrognoserBuilder.h"

using namespace PCOE;

const double ROS_RATE       = 0.5;
const int NUM_NODES         = 8;
const auto START_TIME       = MessageClock::now();

// TEST
class PredictionHandler : public IMessageProcessor
{
public:
  // Constructor
  PredictionHandler(MessageBus& bus, const std::string& src) : bus(bus)
  {
    bus.subscribe(this, src, MessageId::BatteryEod);
    identifier = src;
  }

  // Destructor
  ~PredictionHandler() {
    bus.unsubscribe(this);
  }

  void processMessage(const std::shared_ptr<Message>& message) override 
  {
      // TEST
      ROS_ERROR_STREAM("OVERRIDE FUNCTION CALLED FOR '" << identifier << "!");

      using namespace std::chrono;
      // The prediction printer only ever subscribes to the BatteryEoD message
      // id, which should always be a ProgEventMessage, so this should always
      // succeed.
      auto prediction_msg = dynamic_cast<ProgEventMessage*>(message.get());
      if (prediction_msg == nullptr) {
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
      if (eod_time.uncertainty() != UType::Samples) {
          std::cerr << "Unexpected uncertainty type for EoD prediction" << std::endl;
          return std::exit(1);
      }

      // For this example, we will print the median EoD, which we get by
      // retrieving the raw samples, sorting them, and taking the middle
      // value.
      auto samples = eod_time.getVec();
      std::sort(samples.begin(), samples.end());
      double eod_median = samples.at(samples.size() / 2);

      // Finally, we print the number of milliseconds until EoD
      auto eod_dur = std::chrono::seconds(static_cast<unsigned long>(eod_median));
      auto now =  MessageClock::now();
      auto now_s = duration_cast<std::chrono::seconds>(now.time_since_epoch());
      std::cout << "Predicted median EoD: " << eod_dur.count() << " s (T- "
                << (eod_dur-now_s).count() << " s)" << std::endl;
  }
private:
  MessageBus& bus;
  std::string identifier;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "power_system_node");
  /*PowerSystemNode psn;
  if (!psn.Initialize())
  {
    ROS_ERROR("Power system node failed to initialize");
    return -1;
  }
  psn.Run();
  return 0;*/

  // Store the timestamp/wattage/voltage/temperature of each series model within a 2D array.
  double models[NUM_NODES][4];

  // Create/initialize PowerSystemNodes and their related string identifiers.
  PowerSystemNode nodes[NUM_NODES];
  std::string node_names[NUM_NODES];
  for(int i = 0; i < NUM_NODES; i++)
  {
    node_names[i] = "Node " + std::to_string(i);
    ROS_INFO_STREAM(node_names[i]);
    if (!nodes[i].Initialize())
    {
      ROS_ERROR_STREAM("Power system node " << i << " failed to initialize");
      return -1;
    }
  }

  ros::Rate rate(ROS_RATE); // PRIVATE VAR, NEED TO FIX LATER

  // Load the config file.
  auto prognoser_config_path = ros::package::getPath("ow_power_system") + "/config/prognoser.cfg";
  ConfigMap prognoser_config(prognoser_config_path);

  // TEST
  MessageBus bus;//[NUM_NODES];
  // Do I need to declare this NUM_NODES times?
  PredictionHandler handler(bus, node_names[0]);

  PCOE::ModelBasedAsyncPrognoserBuilder builder(std::move(prognoser_config));
  builder.setModelName("Battery");
  builder.setObserverName("UKF");
  builder.setPredictorName("MC");
  builder.setLoadEstimatorName("Const");

  /*PredictionHandler handler1(bus[1], node_names[1]);
  PredictionHandler handler2(bus[2], node_names[2]);
  PredictionHandler handler3(bus[3], node_names[3]);
  PredictionHandler handler4(bus[4], node_names[4]);
  PredictionHandler handler5(bus[5], node_names[5]);
  PredictionHandler handler6(bus[6], node_names[6]);
  PredictionHandler handler7(bus[7], node_names[7]);*/

  auto prognoser = builder.build(bus, node_names[0], "trajectory");

  double power[NUM_NODES];
  double voltage[NUM_NODES];
  double temperature[NUM_NODES];
  PrognoserMap current_data;

  while(ros::ok())
  {
    ros::spinOnce();
    for (int i = 0; i < 1; i++)
    {
      ROS_INFO_STREAM("Calling RunOnce on node " << i << "...");
      nodes[i].RunOnce();
      nodes[i].GetPowerStats(models[i]);
      ROS_INFO_STREAM("Node " << i << "  time: " << models[i][0]);
      ROS_INFO_STREAM("Node " << i << " power: " << models[i][1]);
      ROS_INFO_STREAM("Node " << i << " volts: " << models[i][2]);
      ROS_INFO_STREAM("Node " << i << "  temp: " << models[i][3]);
      power[i] = models[i][1];
      voltage[i] = models[i][2];
      temperature[i] = models[i][3];

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

      // TEST
      /*current_data = PrognoserMap 
      {
        { MessageId::Watts, Datum<double>{ models[i][1] } },
        { MessageId::Volts, Datum<double>{ models[i][2] } },
        { MessageId::Centigrade, Datum<double>{ models[i][3] } }
      };*/

      std::this_thread::sleep_until(timestamp);

      for (const auto& info : data_to_publish)
      {
        bus.publish(info);
        ROS_INFO_STREAM("Published some info!");
      }
      ROS_INFO_STREAM("Waiting for all...");
      bus.waitAll();
      ROS_INFO_STREAM("Waited for all...");

    }

    // Perhaps the GSAP call should go here instead? Take all the information in
    // models[][] and send it in one call? Can GSAP recognize that?
    // The GSAP module would have to be initialized here in this function instead of within each PowerSystemNode.cpp.
    // Figure out what is needed to instantiate it here.

    // NOTES:
    // Try to revamp the code to use the MessageBus tomorrow. Maybe you can address the problem using that?

    /*vector<PrognoserMap> node_stats;

    PrognoserMap current_data;

    for (int i = 0; i < NUM_NODES; i++)
    {
      current_data = PrognoserMap {
        { MessageId::Watts, Datum<double>{ power[i] } },
        { MessageId::Volts, Datum<double>{ voltage[i] } },
        { MessageId::Centigrade, Datum<double>{ temperature[i] } }
      };

      node_stats.push_back(current_data);
    }*/

    rate.sleep();
  }
  // \TEST
  return 0;
}

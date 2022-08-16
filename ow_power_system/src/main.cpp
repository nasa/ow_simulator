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

#include "PowerSystemNode.h"

using namespace PCOE;

const double ROS_RATE       = 0.5;
const int NUM_NODES         = 8;
const auto START_TIME       = MessageClock::now();

// The index use to access temperature information.
// This might change to median SOC or RUL index or fixed percentile.
//
static constexpr int TEMPERATURE_INDEX = 1;

// This function is a quick and dirty reader for the example data files.
// For production-ready applications, a complete and well-tested CSV library
// should be used.
std::vector<std::vector<std::shared_ptr<DoubleMessage>>> read_file(const std::string& filename,
                                                                   const std::string& src) {
  using namespace std::chrono;
  std::ifstream file(filename);
  if (file.fail()) {
    std::cerr << "Unable to open data file" << std::endl;
  }
  // Skip header line
  file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  auto now = MessageClock::now();

  std::vector<std::vector<std::shared_ptr<DoubleMessage>>> result;
  while (file.good()) {
    std::vector<std::shared_ptr<DoubleMessage>> data;
    std::string line;
    std::getline(file, line);
    if (line.empty()) {
        continue;
    }
    std::stringstream line_stream(line);

    std::string cell;
    std::getline(line_stream, cell, ',');
    double file_time = std::stod(cell);
    auto timestamp = now + milliseconds(static_cast<unsigned>(file_time * 1000));

    std::getline(line_stream, cell, ',');
    double power = std::stod(cell);

    std::getline(line_stream, cell, ',');
    double temperature = std::stod(cell);

    std::getline(line_stream, cell, ',');
    double voltage = std::stod(cell);

    data.push_back(std::make_shared<DoubleMessage>(MessageId::Watts, src, timestamp, power));
    data.push_back(
      std::make_shared<DoubleMessage>(MessageId::Centigrade, src, timestamp, temperature));
    data.push_back(std::make_shared<DoubleMessage>(MessageId::Volts, src, timestamp, voltage));
    result.push_back(data);
  }
  return result;
}

// The PredictionPrinter class subscribes to the battery EoD event message and
// prints each event as it is received.
class PredictionPrinter : public IMessageProcessor {
public:
  /**
   * Constructs a new prediction printer that subscribes to battery EoD
   * predictions for the specified source and on the specified message
   * bus.
   **/
  PredictionPrinter(MessageBus& bus, const std::string& src) : bus(bus) {
    bus.subscribe(this, src, MessageId::BatteryEod);
    identifier = src;
  }

  /**
   * Unsubscribes the prediction printer from the message bus.
   **/
  ~PredictionPrinter() {
    bus.unsubscribe(this);
  }

  /**
   * The message bus will call this function each time the predictor publishes
   * a new battery EoD prediction.
   **/
  void processMessage(const std::shared_ptr<Message>& message) override {
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

    // TEST
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
      state.push_back(sample[0]);

    // TEST/HACK ALERT
    // Construct a simple prognoser to get the battery model.
    auto prognoser_config_path = ros::package::getPath("ow_power_system") + "/config/prognoser.cfg";
    ConfigMap prognoser_config(prognoser_config_path);
    std::unique_ptr<PCOE::Prognoser> temp_prog =
      PrognoserFactory::instance().Create("ModelBasedPrognoser", prognoser_config);

    auto& model = dynamic_cast<ModelBasedPrognoser*>(temp_prog.get())->getModel();

    //auto& model = dynamic_cast<ModelBasedPrognoser*>(m_prognoser.get())->getModel();
    auto model_output = model.outputEqn(now_s.count(), static_cast<PrognosticsModel::state_type>(state));
    //battery_temperature_msg.data = model_output[TEMPERATURE_INDEX];

    ROS_INFO_STREAM(identifier);
    ROS_INFO_STREAM("RUL DATA: " << std::to_string(rul_median));
    ROS_INFO_STREAM("SOC DATA: " << std::to_string(soc_median));
    ROS_INFO_STREAM("TMP DATA: " << std::to_string(model_output[TEMPERATURE_INDEX])
                     << std::endl << std::endl << std::endl);

    /*
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
    */
  }

private:
  MessageBus& bus;
  std::string identifier = "Node X";
};

// This example sets up a predictor to predict battery EoD using a Monte Carlo
// predictor and an Unscented Kalman Filter.
int main(int argc, char* argv[]) 
{
  ros::init(argc, argv, "power_system_node");
  // The source string is a unique identifier for each thing that GSAP is
  // monitoring. This could be a batter serial number or any other unique
  // identifier for each component.

  // Initialize the nodes.
  std::string node_names[NUM_NODES];
  PowerSystemNode nodes[NUM_NODES];
  for (int i = 0; i < NUM_NODES; i++) //SWITCH TO NUM_NODES LATER
  {
    node_names[i] = "Node " + std::to_string(i);
    if (!nodes[i].Initialize())
    {
        ROS_ERROR_STREAM("Power system node failed to initialize");
        return -1;
    }
  }

  

  /*std::string src = "Node 1";

  // Read battery data from a file.
  auto data_path = ros::package::getPath("ow_power_system") + "/profiles/data_const_load.csv";
  auto data = read_file(data_path, src);*/

  //std::cout << "READ FILE!" << std::endl; TEST

  // Read the configuration from a file.
  auto config_path = ros::package::getPath("ow_power_system") + "/config/async_prognoser.cfg";
  ConfigMap config(config_path);

  //std::cout << "READ CONFIGURATION!" << std::endl; TEST

  
  // The message bus is the core of the asynchronous architecture. It
  // maintains a list of listeners who are listening for specific messages
  // and alerts those listeners when a message they are interested in is
  // received.
  MessageBus bus[NUM_NODES];

  // The printer is the first thing that subscribes to the message bus. Its
  // constructor tells the bus that it wants to know about any predictions
  // that are produced for the thing identified by `node_names[x]`.
  PredictionPrinter printer_0(bus[0], node_names[0]);
  PredictionPrinter printer_1(bus[1], node_names[1]);
  PredictionPrinter printer_2(bus[2], node_names[2]);
  PredictionPrinter printer_3(bus[3], node_names[3]);
  PredictionPrinter printer_4(bus[4], node_names[4]);
  PredictionPrinter printer_5(bus[5], node_names[5]);
  PredictionPrinter printer_6(bus[6], node_names[6]);
  PredictionPrinter printer_7(bus[7], node_names[7]);

  //std::cout << "CREATED PRINTERS!" << std::endl; TEST

  // The builder uses configuration information and other methods to determine
  // the correct set of objects needed to perform prognostics.
  ModelBasedAsyncPrognoserBuilder builder(std::move(config));
  builder.setModelName("Battery");
  builder.setObserverName("UKF");
  builder.setPredictorName("MC");
  builder.setLoadEstimatorName("Const");

  //std::cout << "CREATED BUILDER!" << std::endl; TEST

  // The build function call constructs all of the necessary objects to
  // perform prognostics using the specified parameters. These objects
  // are constructed and subscribed to the correct message types on the
  // message bus so that the user only needs to publish data and subscribe
  // to results.
  PCOE::AsyncPrognoser prognoser_0 = builder.build(bus[0], node_names[0], "trajectory");
  PCOE::AsyncPrognoser prognoser_1 = builder.build(bus[1], node_names[1], "trajectory");
  PCOE::AsyncPrognoser prognoser_2 = builder.build(bus[2], node_names[2], "trajectory");
  PCOE::AsyncPrognoser prognoser_3 = builder.build(bus[3], node_names[3], "trajectory");
  PCOE::AsyncPrognoser prognoser_4 = builder.build(bus[4], node_names[4], "trajectory");
  PCOE::AsyncPrognoser prognoser_5 = builder.build(bus[5], node_names[5], "trajectory");
  PCOE::AsyncPrognoser prognoser_6 = builder.build(bus[6], node_names[6], "trajectory");
  PCOE::AsyncPrognoser prognoser_7 = builder.build(bus[7], node_names[7], "trajectory");

  //std::cout << "CONSTRUCTED BUILDERS!" << std::endl; TEST

  // Store the timestamp/wattage/voltage/temperature of each series model.
  double models[NUM_NODES][4];
  double previous_times[NUM_NODES];
  for (int i = 0; i < NUM_NODES; i++)
  {
    previous_times[i] = 0;
  }

  // Run the power nodes at the same rate as GSAP.
  ros::Rate rate(ROS_RATE);

  // Loop the PowerSystemNodes to update their values and send them to the bus.
  while(ros::ok())
  {
    ros::spinOnce();
    for (int i = 0; i < NUM_NODES; i++) //SWITCH TO NUM_NODES LATER
    {
      //ROS_INFO_STREAM("Calling RunOnce on node " << i << "..."); TEST
      nodes[i].RunOnce();
      nodes[i].GetPowerStats(models[i]);
      ROS_INFO_STREAM("Node " << i << "  time: " << models[i][0]); // TEST
      ROS_INFO_STREAM("Node " << i << " power: " << models[i][1]); // TEST
      ROS_INFO_STREAM("Node " << i << " volts: " << models[i][2]); // TEST
      ROS_INFO_STREAM("Node " << i << "  temp: " << models[i][3]); // TEST

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
          bus[i].publish(info); // MAY NEED TO UPDATE WHEN MULTIPLE NODES ARE CALLED.
          //ROS_INFO_STREAM("Published info: " << info); TEST
        }

      }
    }

    ROS_INFO_STREAM("Waiting for all...");
    for (int i = 0; i < NUM_NODES; i++)
    {
      bus[i].waitAll();
    }
    ROS_INFO_STREAM("Waited for all...");
  }

  /*
  // For each line of data in the example file, run a single prediction step.
  for (const auto& line : data) {
      // Sleep until the timestamp specified by the file. While the main
      // thread is sleeping, worker threads owned by the message bus are
      // processing messages and the prediction printer may be printing
      // the results.
      std::this_thread::sleep_until(line.front()->getTimestamp());

      // Publish all of the data in the line. This will trigger the components
      // contructed by the builder to run a prediction, ultimately triggering
      // the prediction printer to print the result.
      std::cout << "Publishing sensor data" << std::endl;
      for (const auto& msg : line) {
        std::cout << msg << std::endl;
        bus.publish(msg);
      }
  }

  // Before exiting, wait for the bus to finish processing all messages to
  // make sure we see all predictions.
  bus.waitAll();
  return 0;
  */
}

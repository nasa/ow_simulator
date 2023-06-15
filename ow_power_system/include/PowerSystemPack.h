// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// This is the header file for the PowerSystemPack class, which handles
// the creation and managing of several PowerSystemNode objects (each
// representing a single cell in a battery). It gathers the data used as 
// input to GSAP's asynchronous prognosers from each PowerSystemNode, sends
// it off, stores GSAP's predictions, and publishes them after manipulations.

#ifndef __POWER_SYSTEM_PACK_H__
#define __POWER_SYSTEM_PACK_H__

#include <vector>
#include <map>
#include <chrono>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <owl_msgs/BatteryRemainingUsefulLife.h>
#include <owl_msgs/BatteryStateOfCharge.h>
#include <owl_msgs/BatteryTemperature.h>
#include <PrognoserFactory.h>
#include "PowerSystemNode.h"

#include "Messages/MessageBus.h"
#include "Messages/ProgEventMessage.h"
#include "Messages/ScalarMessage.h"
#include "ModelBasedAsyncPrognoserBuilder.h"

using PrognoserMap = std::map<PCOE::MessageId, PCOE::Datum<double>>;
using PrognoserVector = std::vector<PrognoserMap>;

// NOTE: This is required as a compile-time constant, so it cannot be placed in
//       a .cfg file. The simulation must be re-built if it is changed.
// This is the number of parallel nodes that will be simulated at once. The
// expected amount for the battery pack is 24.
const int NUM_NODES = 24;

// Struct that contains the information used for EoD predictions.
struct ModelInfo {
  double timestamp;
  double wattage;
  double voltage;
  double temperature;
};

// Struct that groups the variables/classes used to handle PowerSystemNodes.
struct PowerNode {
  std::string name;
  PowerSystemNode node;
  MessageBus bus;
  ModelInfo model;
  double previous_time;
};

// Struct that groups the end-of-discharge (EoD) prediction values.
struct EoDValues {
  double remaining_useful_life;
  double state_of_charge;
  double battery_temperature;
};

class PowerSystemPack
{
public:
  PowerSystemPack() = default;
  ~PowerSystemPack() = default;
  PowerSystemPack(const PowerSystemPack&) = delete;
  PowerSystemPack& operator=(const PowerSystemPack&) = delete;
  void InitAndRun();
private:
  bool initNodes();
  bool initTopics();
  void injectCustomFault(bool& fault_activated,
                         const PrognoserVector& sequence,
                         size_t& index);
  void injectFault(const std::string& power_fault_name,
                   bool& fault_activated);
  void injectFaults();
  void jointStatesCb(const sensor_msgs::JointStateConstPtr& msg);
  PrognoserVector loadPowerProfile(const std::string& filename, std::string custom_file);
  bool loadCustomFaultPowerProfile(std::string path, std::string custom_file);
  void publishPredictions();
  std::string setNodeName(int node_num);

  ros::NodeHandle m_nh;                        // Node Handle Initialization
  ros::Publisher m_mechanical_power_raw_pub;   // Mechanical Power Raw
  ros::Publisher m_mechanical_power_avg_pub;   // Mechanical Power Averaged
  ros::Publisher m_state_of_charge_pub;        // State of Charge Publisher
  ros::Publisher m_remaining_useful_life_pub;  // Remaining Useful Life Publisher
  ros::Publisher m_battery_temperature_pub;    // Battery Temperature Publisher
  ros::Subscriber m_joint_states_sub;          // Mechanical Power Subscriber

  // Flags that determine if debug output is printed regarding battery status
  // during runtime.
  // NOTE: There's no function to get boolean variables from a config, so
  // these bools have to be set via string comparisons. Could benefit from an
  // update if such a function is added in the future.
  bool m_print_debug;
  bool m_timestamp_print_debug;
  bool m_inputs_print_debug;
  bool m_outputs_print_debug;
  bool m_topics_print_debug;
  bool m_mech_power_print_debug;

  // system.cfg variables:

  // HACK ALERT.  The prognoser produced erratic/erroneous output when
  // given too high a power input.  The value assigned to this in system.cfg,
  // protects against this by capping the power input, but it is a temporary
  // hack until a circuit breaker model is added to the power system, and/or 
  // the multi-pack battery model is implemented and can handle any envisioned
  // power draw.
  double m_max_gsap_input_watts;

  // The maximum RUL estimation output from the Monte Carlo prediction process.
  // If the prediction hits this value, it stops immediately and returns infinity
  // (which is processed later into the max value instead).
  // Lower values mean faster performance in the event a RUL prediction would
  // exceed the horizon value, but it also means the prognoser can't return RUL
  // values higher than this.
  int m_max_horizon_secs;

  // The number of samples each node creates during the Monte Carlo prediction
  // process. Lower values mean faster performance, but lower accuracy (needs
  // testing for confirmation).
  int m_num_samples;

  // GSAP's cycle time.
  double m_gsap_rate_hz;

  // The expected time interval between publications. It is essentially the
  // rate at which the main loop executes. It is overridden by the value in
  // system.cfg.
  int m_time_interval;

  // The initial power/temperature/voltage readings used as the start values for
  // the GSAP prognosers.
  double m_initial_power;
  double m_initial_temperature;
  double m_initial_voltage;
  double m_initial_soc;
  
  // End system.cfg variables.


  // Flag determining if the battery has reached a fail state.
  bool m_battery_failed = false;

  // Create a PowerNode struct, containing all the necessary information for a
  // node to operate, for each node.
  PowerNode m_nodes[NUM_NODES];

  // The matrix used to store EoD events.
  EoDValues m_EoD_events[NUM_NODES];

  // Vector w/ supporting variables that stores the moving average of the
  // past mechanical power values.
  const int m_moving_average_window = 25;
  std::vector<double> m_power_values;
  size_t m_power_values_index = 0;

  bool m_high_power_draw_activated = false;
  bool m_custom_power_fault_activated = false;
  PrognoserVector m_custom_power_fault_sequence;
  size_t m_custom_power_fault_sequence_index = 0;

  // Track the prognosers waiting for new data.
  bool m_waiting_buses[NUM_NODES];
};

#endif

// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef __POWER_SYSTEM_PACK_H__
#define __POWER_SYSTEM_PACK_H__

#include <vector>
#include <map>
#include <chrono>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <PrognoserFactory.h>
#include "PowerSystemNode.h"

#include "Messages/MessageBus.h"
#include "Messages/ProgEventMessage.h"
#include "Messages/ScalarMessage.h"
#include "ModelBasedAsyncPrognoserBuilder.h"

using PrognoserMap = std::map<PCOE::MessageId, PCOE::Datum<double>>;
using PrognoserVector = std::vector<PrognoserMap>;

// Change this value to modify the number of parallel battery nodes in the pack.
// NOTE: The expected number of nodes (24) appears to take much longer than 2
//       seconds to process. Currently unknown what the cause of this is, though
//       it is likely related to GSAP's time required to process predictions and
//       so it may not be addressable here.
const int NUM_NODES = 8;

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
  PowerSystemPack(const PowerSystemPack&) = default;
  PowerSystemPack& operator=(const PowerSystemPack&) = default;
  void InitAndRun();
private:
  bool initNodes();
  bool initTopics();
  void jointStatesCb(const sensor_msgs::JointStateConstPtr& msg);
  void publishPredictions();
  std::string setNodeName(int node_num);

  ros::NodeHandle m_nh;                        // Node Handle Initialization
  ros::Publisher m_mechanical_power_raw_pub;   // Mechanical Power Raw
  ros::Publisher m_mechanical_power_avg_pub;   // Mechanical Power Averaged
  ros::Publisher m_state_of_charge_pub;        // State of Charge Publisher
  ros::Publisher m_remaining_useful_life_pub;  // Remaining Useful Life Publisher
  ros::Publisher m_battery_temperature_pub;    // Battery Temperature Publisher
  ros::Subscriber m_joint_states_sub;          // Mechanical Power Subscriber

  // Create a PowerNode struct, containing all the necessary information for a
  // node to operate, for each node.
  PowerNode m_nodes[NUM_NODES];

  // The matrix used to store EoD events.
  EoDValues m_EoD_events[NUM_NODES];

  int m_moving_average_window = 25;
  std::vector<double> m_power_values;
  size_t m_power_values_index = 0;

  std::chrono::time_point<std::chrono::system_clock> m_init_time;

  double m_current_timestamp = 0.0;

  // GSAP's cycle time
  double m_gsap_rate_hz = 0.5;

  // Baseline value for power drawn by continuously-running systems.
  // This initial value is overriden by the system config.
  double m_baseline_wattage = 1.0;

  // HACK ALERT.  The prognoser produced erratic/erroneous output when
  // given too high a power input.  This made-up value protects
  // against this, but is a temporary hack until a circuit breaker
  // model is added to the power system, and/or the multi-pack battery
  // model is implemented and can handle any envisioned power draw.
  // This initial value is overriden by the system config.
  //
  double m_max_gsap_input_watts = 30;

  // Number of lines in power fault profiles to skip, in order to
  // synchonize the consumption of the profile with GSAP's cycle rate
  // (above).  This computation is left to the user for now, though
  // note that the default values are not expected to change as of
  // Release 9.  This initial value is overriden by the system config.
  int m_profile_increment = 2;

  // End main system configuration.

  // Utilize a Mersenne Twister pseudo-random generation.
  std::mt19937 m_random_generator;

  std::uniform_real_distribution<double> m_temperature_dist;
  
  bool m_high_power_draw_activated = false;
  bool m_custom_power_fault_activated = false;
  PrognoserVector m_custom_power_fault_sequence;
  size_t m_custom_power_fault_sequence_index = 0;

  // Flag that indicates that the prognoser is handling current batch.
  bool m_processing_power_batch = false;

  bool m_trigger_processing_new_power_batch = false;
  double m_unprocessed_mechanical_power = 0.0;
  double m_mechanical_power_to_be_processed = 0.0;
};

#endif

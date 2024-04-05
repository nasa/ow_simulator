// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// This is the header file for the PowerSystemNode class, which handles
// the creation and managing of several PrognoserInputHandler objects (each
// simulating inputs to part of a battery). It gathers the data from each object
// and sends it to GSAP's asynchronous prognosers, receives GSAP's predictions,
// and publishes them after manipulations.

#ifndef __POWER_SYSTEM_NODE_H__
#define __POWER_SYSTEM_NODE_H__

#include <vector>
#include <map>
#include <chrono>
#include <random>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <owl_msgs/BatteryRemainingUsefulLife.h>
#include <owl_msgs/BatteryStateOfCharge.h>
#include <owl_msgs/BatteryTemperature.h>
#include <PrognoserFactory.h>
#include "PrognoserInputHandler.h"
#include "PredictionHandler.h"

#include "Messages/MessageBus.h"
#include "Messages/ProgEventMessage.h"
#include "Messages/ScalarMessage.h"
#include "ModelBasedAsyncPrognoserBuilder.h"

using PrognoserMap = std::map<PCOE::MessageId, PCOE::Datum<double>>;
using PrognoserVector = std::vector<PrognoserMap>;

// NOTE: This is required as a compile-time constant, so it cannot be placed in
//       a .cfg file. The simulation must be re-built if it is changed.
// This is the number of parallel inputs and outputs that will be simulated at
// once. The expected amount of cells in the battery pack is 24. The 6S1P model
// simulates 6 cells at once, so only 4 models are needed.
constexpr int NUM_MODELS = 4;

// Power fault thresholds, for their related faults.
// They are slightly beyond the actual thresholds to prevent rounding errors.
static constexpr float POWER_THERMAL_MAX = 70.0;
static constexpr float POWER_SOC_MIN = 0.095;
static constexpr float POWER_SOC_MAX_DIFF = 0.052;

// Struct that groups the variables/classes used to handle PrognoserInputHandlers.
struct PowerModel {
  std::string name;
  PrognoserInputHandler model;
  MessageBus bus;
  InputInfo input_info;
  double previous_time;
};

class PowerSystemNode
{
public:
  PowerSystemNode() = default;
  ~PowerSystemNode() = default;
  PowerSystemNode(const PowerSystemNode&) = delete;
  PowerSystemNode& operator=(const PowerSystemNode&) = delete;
  void initAndRun();
private:
  void runDisabledLoop();
  bool initModels();
  void initTopics();
  void injectCustomFault();
  void injectFault(const std::string& power_fault_name);
  void injectFaults();
  void jointStatesCb(const sensor_msgs::JointStateConstPtr& msg);
  void electricalPowerCb(const std_msgs::Float64& msg);
  double electricalPowerLights() const;
  PrognoserVector loadPowerProfile(const std::string& filename,
                                   std::string custom_file);
  void publishPredictions();

  ros::NodeHandle m_nh;                        // Node Handle Initialization
  ros::Publisher m_mechanical_power_raw_pub;   // Mechanical Power Raw
  ros::Publisher m_mechanical_power_avg_pub;   // Mechanical Power Averaged
  ros::Publisher m_state_of_charge_pub;        // State of Charge Publisher
  ros::Publisher m_remaining_useful_life_pub;  // Remaining Useful Life Publisher
  ros::Publisher m_battery_temperature_pub;    // Battery Temperature Publisher
  ros::Subscriber m_joint_states_sub;          // Mechanical Power Subscriber
  ros::Subscriber m_electrical_power_sub;

  // Flags that determine if debug output is printed regarding battery status
  // during runtime.
  // NOTE: There's no function to get boolean variables from a config, so
  // these bools have to be set via string comparisons. Could benefit from an
  // update if such a function is added in the future.
  bool m_print_debug = false;
  bool m_timestamp_print_debug = false;
  bool m_inputs_print_debug = false;
  bool m_outputs_print_debug = false;
  bool m_topics_print_debug = false;
  bool m_mech_power_print_debug = false;


  // system.cfg variables:

  // Whether or not the power system is active to begin with. Can be disabled
  // to maximize performance for those who do not need battery predictions.
  bool m_enable_power_system;

  // The number of currently simulated models. Default 1, but can be modified
  // mid-simulation by intra-battery faults (or set to a different starting
  // value in system.cfg).
  int m_active_models;

  // HACK ALERT.  The prognoser produces erratic/erroneous output when
  // given too high a power input.  The value assigned to this in system.cfg
  // protects against this by capping the power input, but it is a temporary
  // hack until a circuit breaker model is added to the power system and/or 
  // the multi-cell battery model is implemented and can handle any envisioned
  // power draw.
  double m_max_gsap_input_watts;

  // The maximum RUL estimation output from the Monte Carlo prediction process.
  // If the prediction hits this value, it stops immediately and returns infinity
  // (which is processed later into the max value instead).
  int m_max_horizon_secs;

  // The number of samples each model creates during the Monte Carlo prediction
  // process. Lower values mean faster prediction returns, but lower accuracy
  // (needs testing for confirmation).
  int m_num_samples;

  // Number of threads to use for the asynchronous ROS spinning in the main
  // loop.
  int m_spinner_threads;

  // The cycle time of the main system loop.
  double m_loop_rate_hz;

  // The initial power/temperature/voltage readings used as the start values for
  // the GSAP prognosers.
  double m_initial_power;
  double m_initial_temperature;
  double m_initial_voltage;
  double m_initial_soc;
  
  double m_wattage_coefficient_lights;

  // End system.cfg variables.

  // The number of models deactivated via faults. Used separately from
  // m_active_models to allow the system to distribute power draw as if
  // there were more models running at once (i.e. when full battery simulation
  // is disabled).
  int m_deactivated_models;

  // The rate at which /joint_states is expected to publish. If this is changed,
  // this variable will need to change as well.
  const int m_joint_states_rate = 50;

  // Vector w/ supporting variables that stores the moving average of the
  // past mechanical power values.
  int m_moving_average_window;

  // The expected time interval between publications. It is equal to the
  // reciprocal of the main loop rate.
  double m_time_interval;

  // Flag determining if the battery has reached a fail state.
  bool m_battery_failed = false;

  // Create a PowerModel struct, containing all the necessary information for a
  // model to operate, for each model.
  PowerModel m_power_models[NUM_MODELS];

  // The matrix used to store EoD events.
  EoDValues m_EoD_events[NUM_MODELS];

  // The most recent published SoC, for use in ICL fault injection.
  double m_prev_soc;

  // Fault flags.
  bool m_high_power_draw_activated = false;
  bool m_custom_power_fault_activated = false;
  bool m_disconnect_battery_nodes_fault_activated = false;
  PrognoserVector m_custom_power_fault_sequence;
  size_t m_custom_power_fault_sequence_index = 0;

  bool m_low_soc_activated = false;
  bool m_icl_activated = false;
  bool m_thermal_failure_activated = false;

  double m_mean_mechanical_power = 0.0;
  double m_electrical_power = 0.0;

  // Used for mechanical power debug printouts.
  double m_power_watts;
  double m_sum_mechanical_power = 0.0;
  int m_queue_size = 0;
  std::deque<double> m_power_values;
};

#endif

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
// HACK ALERT (May 2023): High amounts of nodes seems to cause GSAP to eventually
//                        return highly negative and invalid predictions well ahead
//                        of an expected battery failure. On my setup, this happens
//                        around the ~300s mark with 16 nodes, but did not occur at all
//                        over a 45-minute test at 15 nodes or less.
//                        No idea what the issue is yet.
const int NUM_NODES = 24;

const std::string FAULT_NAME_HPD           = "high_power_draw";
const std::string FAULT_NAME_HPD_ACTIVATE  = "activate_high_power_draw";
const int CUSTOM_FILE_EXPECTED_COLS           = 4;

// Error flags.
const int ERR_CUSTOM_FILE_FORMAT              = -1;


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
  void injectCustomFault(bool& fault_activated,
                         const PrognoserVector& sequence,
                         size_t& index);
  void injectFault(const std::string& power_fault_name,
                   bool& fault_activated);
  void injectFaults();
  void jointStatesCb(const sensor_msgs::JointStateConstPtr& msg);
  PrognoserVector loadPowerProfile(const std::string& filename, std::string custom_file);
  bool loadCustomFaultPowerProfile(std::string path, std::string custom_file);
  void publishPredictions(bool update);
  std::string setNodeName(int node_num);

  ros::NodeHandle m_nh;                        // Node Handle Initialization
  ros::Publisher m_mechanical_power_raw_pub;   // Mechanical Power Raw
  ros::Publisher m_mechanical_power_avg_pub;   // Mechanical Power Averaged
  ros::Publisher m_state_of_charge_pub;        // State of Charge Publisher
  ros::Publisher m_remaining_useful_life_pub;  // Remaining Useful Life Publisher
  ros::Publisher m_battery_temperature_pub;    // Battery Temperature Publisher
  ros::Subscriber m_joint_states_sub;          // Mechanical Power Subscriber

  // Flag that determines if debug output is printed regarding battery status
  // during runtime. Overridden by system.cfg on startup.
  // NOTE: This is a string because of limitations with ConfigMap. There's no
  // function to get boolean variables from a config. Could benefit from an
  // update if such a function is added in the future.
  std::string m_print_debug_val = "false";
  bool m_print_debug = false;

  // The maximum RUL estimation output from the Monte Carlo prediction process.
  // Lower values mean faster performance in the event a RUL prediction would
  // exceed the horizon value, but it also means the prognoser can't return RUL
  // values higher than this.
  // The current default value is 10000 (overridden by system.cfg's value), but 
  // this is slow in situations where the prognoser predicts a value that high
  // (or higher than that). Further testing and discussion is needed to 
  // determine the ideal value for this.
  int m_max_horizon_secs = 10000;
  
  // Change this value to modify the number of samples each node creates during the
  // Monte Carlo prediction process. Lower values mean faster performance, but lower
  // accuracy (needs testing for confirmation).
  // The default value is 100 (overridden by system.cfg), but this is very slow 
  // and infeasible for the simulation.
  // My computer reached similar speeds to the original simple prognoser at a sample
  // value of 25, but this may vary from computer to computer, and it also varies
  // depending on the value of the RUL prediction. Higher RULs take significantly longer
  // to return. ~Liam
  int m_num_samples = 100;

  // Flag determining if the battery has reached a fail state.
  bool m_battery_failed = false;

  // Create a PowerNode struct, containing all the necessary information for a
  // node to operate, for each node.
  PowerNode m_nodes[NUM_NODES];

  // The matrix used to store EoD events.
  EoDValues m_EoD_events[NUM_NODES];

  // GSAP's cycle time
  double m_gsap_rate_hz = 0.5;

  // Number of lines in power fault profiles to skip, in order to
  // synchonize the consumption of the profile with GSAP's cycle rate
  // (above).  This computation is left to the user for now, though
  // note that the default values are not expected to change as of
  // Release 9.  This initial value is overriden by the system config.
  int m_profile_increment = 2;

  // The initial power/temperature/voltage readings used as the start values for
  // the GSAP prognosers. Overwritten by the values in system.cfg.
  double m_initial_power = 0.0;
  double m_initial_temperature = 20.0;
  double m_initial_voltage = 4.1;
  double m_initial_soc = 0.95;

  // End main system configuration.
  
  bool m_high_power_draw_activated = false;
  bool m_custom_power_fault_activated = false;
  PrognoserVector m_custom_power_fault_sequence;
  size_t m_custom_power_fault_sequence_index = 0;

  // Track the prognosers waiting for new data.
  bool m_waiting_buses[NUM_NODES];

  // Values to be re-published during intervals where GSAP has not returned
  // predictions yet.
  int m_prev_rul;
  double m_prev_soc;
  double m_prev_tmp;
};

#endif

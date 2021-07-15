// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ow_faults_detection/FaultDetector.h"
#include <dynamic_reconfigure/server.h>


using namespace std;


int main(int argc, char **argv) {
  ros::init(argc, argv, "fault_detection");
  ros::NodeHandle node_handle_detector;

  FaultDetector fault_detector(node_handle_detector);

  ros::spin();
  return 0;
}

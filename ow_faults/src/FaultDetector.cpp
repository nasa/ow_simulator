// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ow_faults/FaultDetector.h"
#include <algorithm>

using namespace std;
// using namespace ow_lander;

FaultDetector::FaultDetector(ros::NodeHandle& node_handle)
{
  srand (static_cast <unsigned> (time(0)));
  m_pub = node_handle.advertise<ow_faults::SystemFaults>("/test", 10);
  cout << "yo" << endl;
}

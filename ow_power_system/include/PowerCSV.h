//PowerCSV.h

/*The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.*/

#ifndef POWERCSV_H
#define POWERCSV_H

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

namespace powerCSV {
  std::vector<std::vector<double>> loadCSV(const std::string& filename);
}

#endif

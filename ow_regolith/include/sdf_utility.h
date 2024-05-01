// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef SDF_UTILITY_H
#define SDF_UTILITY_H

#include <sdf/sdf.hh>

#include <string>

namespace sdf_utility {

// load SDF from an URI address and return its contents
bool getSdfFromUri(const std::string &uri, std::string &out_sdf_text);

// parse SDF contents into an SDF object
void parseSdf(const std::string &sdf_text, sdf::SDF &out_sdf);

bool getModelLink(const sdf::SDF &sdf, sdf::ElementPtr &out_link);

}

#endif // SDF_UTILITY_H

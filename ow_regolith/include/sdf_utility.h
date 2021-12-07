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
sdf::SDFPtr parseSdf(const std::string &sdf_text);

bool getModelLink(const sdf::SDFPtr &sdf, sdf::ElementPtr &out_link);

// get the name of the first link of the model
bool getModelLinkName(const sdf::SDFPtr &sdf, std::string &out_name);

// get the mass of the first link of the model
bool getModelLinkMass(const sdf::SDFPtr &sdf, float &out_mass);

}

#endif // SDF_UTILITY_H

// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <gazebo/common/common.hh>

#include <sdf_utility.h>

using namespace sdf;
using namespace sdf_utility;

using std::string, std::ifstream, std::stringstream, std::endl;

bool sdf_utility::getSdfFromUri(const string &uri, string &out_sdf_text) 
{
  auto file_path = gazebo::common::ModelDatabase::Instance()
    ->GetModelFile(uri);
  ifstream file(file_path);
  if (!file) {
    gzerr << "SDF file "<< uri.c_str()
          << " could not be opened! Error code: " << strerror(errno) << endl;
    return false;
  }
  stringstream ss;
  ss << file.rdbuf();
  out_sdf_text = ss.str();
  return true;
}

void sdf_utility::parseSdf(const string &sdf_text, SDF &out_sdf)
{
  // SDFPtr sdf_data(new SDF());
  out_sdf.SetFromString(sdf_text);
}

bool sdf_utility::getModelLink(const SDF &sdf, ElementPtr &out_link) {
  auto model = sdf.Root()->GetElementImpl("model");
  if (!model) {
    gzerr << "Unable to acquire model element from SDF" << endl;
    return false;
  }
  auto link = model->GetElementImpl("link");
  if (!link) {
    gzerr << "Unable to acquire link element from SDF" << endl;
    return false;
  }
  out_link = link;
  return true;
}

// bool sdf_utility::getModelLinkName(const SDF &sdf, string &out_name)
// {
//   // parse SDF stirng for all links
//   ElementPtr link;
//   if (!getModelLink(sdf, link)) {
//     return false;
//   }
//   auto link_name = link->GetAttribute("name");
//   if (!link_name) {
//     gzerr << "link element has no name attribute" << endl;
//     return false;
//   }
//   out_name = link_name->GetAsString();
//   return true;
// }

// bool sdf_utility::getModelLinkMass(const SDF &sdf, float &out_mass)
// {
//   ElementPtr link;
//   if (!getModelLink(sdf, link))
//     return false;
//   // parse SDF string for all links
//   auto inertial = link->GetElementImpl("inertial");
//   if (!inertial) {
//     gzerr << "Unable to acquire inertial element from SDF" << endl;
//     return false;
//   }
//   auto mass = inertial->GetElementImpl("mass");
//   if (!mass) {
//     gzerr << "Unable to acquire mass element from SDF" << endl;
//     return false;
//   }
//   float link_mass;
//   if (!mass->GetValue()->Get(link_mass)) {
//     gzerr << "Mass of link is in incorrect format" << endl;
//     return false;
//   }
//   out_mass = link_mass;
//   return true;
// }

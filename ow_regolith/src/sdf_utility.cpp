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

using std::string, std::ifstream, std::stringstream;

bool sdf_utility::getSdfFromUri(const string &uri, string &out_sdf_text) 
{
  auto file_path = gazebo::common::ModelDatabase::Instance()
    ->GetModelFile(uri);
  ifstream file(file_path);
  if (!file) {
    ROS_ERROR("SDF file %s could not be opened! Error code: %s", 
      uri.c_str(), strerror(errno));
    return false;
  }
  stringstream ss;
  ss << file.rdbuf();
  out_sdf_text = ss.str();
  return true;
}

SDFPtr sdf_utility::parseSdf(const string &sdf_text)
{
  SDFPtr sdf_data(new SDF());
  sdf_data->SetFromString(sdf_text);
  return sdf_data;
}

bool sdf_utility::getModelLink(const SDFPtr &sdf, ElementPtr &out_link) {
  auto root  = sdf->Root();
  auto model = root->GetElementImpl("model");
  if (!model) {
    ROS_ERROR("Unable to acquire model element from SDF");
    return false;
  }
  auto link = model->GetElementImpl("link");
  if (!link) {
    ROS_ERROR("Unable to acquire link element from SDF");
    return false;
  }
  out_link = link;
  return true;
}

bool sdf_utility::getModelLinkName(const SDFPtr &sdf, string &out_name)
{
  // parse SDF stirng for all links
  ElementPtr link;
  if (!getModelLink(sdf, link))
    return false;
  auto link_name = link->GetAttribute("name");
  if (!link_name) {
    ROS_ERROR("link element has no name attribute");
    return false;
  } 
  out_name = link_name->GetAsString();
  return true;
}

bool sdf_utility::getModelLinkMass(const SDFPtr &sdf, float &out_mass)
{
  ElementPtr link;
  if (!getModelLink(sdf, link))
    return false;
  // parse SDF string for all links
  auto inertial = link->GetElementImpl("inertial");
  if (!inertial) {
    ROS_ERROR("Unable to acquire inertial element from SDF");
    return false;
  }
  auto mass = inertial->GetElementImpl("mass");
  if (!mass) {
    ROS_ERROR("Unable to acquire mass element from SDF");
    return false;
  }
  float link_mass;
  if (!mass->GetValue()->Get(link_mass)) {
    ROS_ERROR("Mass of link is in incorrect format");
    return false;
  }
  out_mass = link_mass;
  return true;
}

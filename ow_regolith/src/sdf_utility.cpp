// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <fstream>
#include <sstream>

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

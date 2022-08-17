// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <ostream>
#include <sstream>
#include <initializer_list>
#include <vector>
#include <set>
#include <limits>

#include <ros/ros.h>

#include <populate_materials.h>

using std::string, std::begin, std::endl, std::initializer_list,
      std::stringstream, std::vector, std::set, std::numeric_limits,
      std::runtime_error;

using namespace ow_materials;

const string PARAM_MATERIAL_NAME           = "name";
const string PARAM_MATERIAL_OCCURRENCE     = "occurrence";
const string PARAM_MATERIAL_SCIENCE_VALUE  = "science_value";
const string PARAM_MATERIAL_DENSITY        = "density";

const string PARAM_MATERIAL_COLOR          = "color";
const string PARAM_MATERIAL_COLOR_RED      = "r";
const string PARAM_MATERIAL_COLOR_GREEN    = "g";
const string PARAM_MATERIAL_COLOR_BLUE     = "b";

static string join(initializer_list<string> args) {
  stringstream joined;
  for (auto x = begin(args); x != end(args) - 1; ++x)
    joined << *x << "/";
  joined << *(end(args) - 1);
  return joined.str();
}

template <typename T>
static T get_param(const string &param_name) {
  if (!ros::param::has(param_name))
    throw MaterialConfigError("Required material parameter is missing!");
  T value;
  ros::param::get(param_name, value);
  return value;
}

// specialize for unsupported rosparam data types
template <>
uint8_t get_param<uint8_t>(const string &param_name) {
  auto value = get_param<int>(param_name);
  if (   value < numeric_limits<uint8_t>::min()
      || value > numeric_limits<uint8_t>::max()) {
    throw MaterialConfigError("Material parameter is out of acceptable range!");
  }
  return static_cast<uint8_t>(value);
}

void populate_material_database(MaterialDatabase *db_ptr, string ns) {
  vector<string> param_names;
  if (!ros::param::getParamNames(param_names))
    throw runtime_error("Failed to communicate with parameter server.");

  set<string> mat_names;
  for (const auto &pn : param_names) {
    // gzlog << "param_name = " << pn << endl;
    if (pn.compare(0, ns.size(), ns) == 0) {
      auto stripped = pn.substr(ns.size() + 1);
      auto mat_name = stripped.substr(0, stripped.find('/'));
      mat_names.insert(mat_name);
    }
  }

  for (auto const &mat : mat_names) {
    db_ptr->addMaterial(
      {
        mat,
        get_param<float>(join({ns, mat, PARAM_MATERIAL_OCCURRENCE})),
        get_param<float>(join({ns, mat, PARAM_MATERIAL_SCIENCE_VALUE})),
        get_param<double>(join({ns, mat, PARAM_MATERIAL_DENSITY})),
        {
          get_param<uint8_t>(join({ns, mat, PARAM_MATERIAL_COLOR,
            PARAM_MATERIAL_COLOR_RED})),
          get_param<uint8_t>(join({ns, mat, PARAM_MATERIAL_COLOR,
            PARAM_MATERIAL_COLOR_GREEN})),
          get_param<uint8_t>(join({ns, mat, PARAM_MATERIAL_COLOR,
            PARAM_MATERIAL_COLOR_BLUE}))
        }
      }
    );
  }
}

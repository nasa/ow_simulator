// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <sstream>
#include <initializer_list>
#include <vector>
#include <set>
#include <limits>

#include <ros/ros.h>

#include <populate_materials.h>

using std::string, std::begin, std::end, std::initializer_list, std::set,
      std::vector, std::stringstream, std::numeric_limits, std::runtime_error;

using namespace ow_materials;

static string join(const initializer_list<string> &args) {
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

void populate_material_database(MaterialDatabase *db_ptr, const string &ns) {
  // search for material names in namespace
  vector<string> param_names;
  if (!ros::param::getParamNames(param_names))
    throw runtime_error("Failed to communicate with parameter server!");
  set<string> mat_names;
  for (const auto &pn : param_names) {
    if (pn.compare(0, ns.size(), ns) == 0) {
      auto stripped = pn.substr(ns.size() + 1);
      auto mat_name = stripped.substr(0, stripped.find('/'));
      mat_names.insert(mat_name);
    }
  }
  // build and add all materials to database
  for (const auto &mat : mat_names) {
    db_ptr->addMaterial(
      {
        mat,
        get_param<float>(join({ns, mat, "occurrence"})),
        get_param<float>(join({ns, mat, "science_value"})),
        get_param<double>(join({ns, mat, "density"})),
        { // Color struct
          get_param<uint8_t>(join({ns, mat, "color", "r"})),
          get_param<uint8_t>(join({ns, mat, "color", "g"})),
          get_param<uint8_t>(join({ns, mat, "color", "b"}))
        }
      }
    );
  }
}

// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <sstream>
#include <initializer_list>
#include <vector>
#include <set>
#include <limits>

#include "ros/ros.h"

#include "gazebo/common/common.hh"

#include "MaterialDatabase.h"

using namespace ow_materials;

using std::begin, std::string, std::end, std::initializer_list, std::set,
      std::vector, std::stringstream, std::numeric_limits, std::runtime_error,
      std::endl, std::pair, std::make_pair;

bool MaterialDatabase::addMaterial(const Material &mat)
{
  static MaterialID id = Material::id_min;
  if (id == Material::id_max) {
    gzerr << "Database already contains maximum number of materials" << endl;
    return false;
  }
  if (!m_names.insert({mat.name, id}).second) {
    gzerr << "Attempted to add a material to the database that shares a name "
          << "with a material that is already present. All materials must have "
          << "a unique name." << endl;
  }
  if (!m_database.insert({id, mat}).second) {
    gzerr << "Attempted to add a pre-existing material ID to the database. "
             "This should never happen!!" << endl;
    return false;
  }
  ++id;
  return true;
}

// The following functions help generate a database from ROS params

static string join(const initializer_list<string> &args) {
  stringstream joined;

  for (auto x = begin(args); x != end(args) - 1; ++x) {
    joined << *x << "/";
  }
  joined << *(end(args) - 1);
  return joined.str();
}

template <typename T>
static T get_param(const string &param_name) {
  if (!ros::param::has(param_name)) {
    throw MaterialConfigError("Required material parameter is missing!");
  }
  T value;
  ros::param::get(param_name, value);
  return value;
}

// specialize for unsupported rosparam data types
// DEPRECATED: Color r, g, and b are no longer uint8_t, but will keep this
//             specialization in case it's needed for another material parameter
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
  if (!ros::param::getParamNames(param_names)) {
    throw MaterialConfigError("Failed to communicate with parameter server!");
  }
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
    const bool added = db_ptr->addMaterial(
      {
        mat,
        get_param<float>(join({ns, mat, "occurrence"})),
        get_param<float>(join({ns, mat, "science_value"})),
        get_param<double>(join({ns, mat, "density"})),
        Color{
          get_param<float>(join({ns, mat, "visualize_color", "r"})),
          get_param<float>(join({ns, mat, "visualize_color", "g"})),
          get_param<float>(join({ns, mat, "visualize_color", "b"}))
        }
      }
    );
    if (!added) {
      throw MaterialConfigError("Failed to added a material to the database!");
    }
  }
}

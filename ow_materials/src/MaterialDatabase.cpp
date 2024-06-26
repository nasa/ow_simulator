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
      std::vector, std::stringstream, std::numeric_limits, std::endl,
      std::out_of_range;

bool MaterialDatabase::addMaterial(const Material &mat)
{
  if (m_last_id_added == Material::id_max) {
    gzerr << "Database already contains maximum number of materials." << endl;
    return false;
  }
  if (!m_names.insert({mat.name, m_last_id_added}).second) {
    gzerr << "Attempted to add a material to the database that shares a name "
          << "with a material that is already present. All materials must have "
          << "a unique name." << endl;
    return false;
  }
  if (!m_database.insert({m_last_id_added, mat}).second) {
    gzerr << "Attempted to add a pre-existing material ID to the database. "
             "This should never happen!!" << endl;
    return false;
  }
  ++m_last_id_added;
  return true;
}

const Material &MaterialDatabase::getMaterial(MaterialID id) const
{
  try {
    return m_database.at(id);
  } catch (out_of_range &) {
    stringstream ss;
    ss << "Database does not contain material with ID "
       << static_cast<std::uint32_t>(id) << ".";
    throw MaterialRangeError(ss.str());
  }
};

MaterialID MaterialDatabase::getMaterialIdFromName(const string &name) const
{
  try {
    return m_names.at(name);
  } catch (out_of_range &) {
    stringstream ss;
    ss << "Database does not contain material with name " << name << ".";
    throw MaterialRangeError(ss.str());
  }
};

// The following functions help generate a database from ROS params

static string join(const initializer_list<string> &args)
{
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

void MaterialDatabase::populate_from_rosparams(const string &ns)
{
  m_database.clear();
  m_names.clear();
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
  if (mat_names.size() == 0) {
    throw MaterialConfigError("A minimum of 1 material is required even if "
                              "the sim_multimaterial flag is false.");
  }
  // build and add all materials to database
  for (const auto &mat : mat_names) {
    const bool added = addMaterial(
      {
        mat,
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

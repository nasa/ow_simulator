// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <MaterialDatabase.h>

#include <gazebo/gazebo.hh>

using std::numeric_limits, std::endl;

using namespace ow_materials;

using namespace gazebo;

MaterialDatabase::MaterialDatabase(const std::string &yaml_path) {
  gzlog << "opening file " << yaml_path << endl;
}

const Material &MaterialDatabase::getMaterial(MaterialID id) const {
  return m_database.at(id);
};

void MaterialDatabase::addMaterial(Material mat) {
  static MaterialID id = Material::id_min;
  if (id == Material::id_max) {
    // TODO Error: Maximum number of unique materials has already been created.
    return;
  }
  if (!m_database.insert({id, mat}).second) {
    // TODO Error: An unknown error has occurred. This should never happen.
    return;
  }
  ++id;
}

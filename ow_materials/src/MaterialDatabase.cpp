// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <MaterialDatabase.h>

using namespace ow_materials;

void MaterialDatabase::addMaterial(Material mat) {
  static MaterialID id = Material::id_min;
  if (id == Material::id_max) {
    // Unsure whether this should use gazebo or ROS logging, so both error
    // message are left as TODOs for now
    // TODO Error: Maximum number of unique materials has already been created.
    return;
  }
  if (!m_database.insert({id, mat}).second) {
    // TODO Error: An unknown error has occurred. This should never happen.
    return;
  }
  ++id;
}

size_t MaterialDatabase::size() {
  return m_database.size();
}

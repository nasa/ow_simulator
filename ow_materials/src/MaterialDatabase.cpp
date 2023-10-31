// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <gazebo/common/common.hh>

#include <MaterialDatabase.h>

using namespace ow_materials;

using std::endl, std::vector, std::pair, std::make_pair;

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


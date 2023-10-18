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
  if (!m_database.insert({id, mat}).second) {
    gzerr << "Attempted to add a pre-existing material ID to the database. "
             "This should never happen." << endl;
    return false;
  }
  ++id;
  return true;
}

vector<pair<MaterialID, Color>> MaterialDatabase::getColorBasis() const
{
  vector<pair<MaterialID, Color>> basis;
  for (auto const &x : m_database)
    basis.push_back(make_pair(x.first, x.second.color));
  return basis;
}

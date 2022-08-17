// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIAL_DATABASE_H
#define MATERIAL_DATABASE_H

#include <string>
#include <ostream>
#include <unordered_map>

#include <Material.h>

namespace ow_materials
{

class MaterialDatabase
{
public:
  MaterialDatabase(const std::string &yaml_path);
  ~MaterialDatabase() = default;

  MaterialDatabase() = default;
  MaterialDatabase(const MaterialDatabase&) = delete;
  MaterialDatabase& operator=(const MaterialDatabase&) = delete;

  const Material &getMaterial(MaterialID id) const;

  void addMaterial(Material mat);

  inline size_t count() {
    return m_database.size();
  }

private:

  std::unordered_map<MaterialID, Material> m_database;

};

}

#endif // MATERIAL_DATABASE_H

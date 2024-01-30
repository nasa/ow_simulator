// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIAL_DATABASE_H
#define MATERIAL_DATABASE_H

#include <string>
#include <unordered_map>
#include <stdexcept>

#include "Material.h"

namespace ow_materials
{

class MaterialDatabase
{

  // A catalogue of defined materials. Each material is referenced by a unique
  // ID so that the data that describes a material only has to be stored in
  // one location, but can be referenced elsewhere.

public:
  MaterialDatabase() = default;
  ~MaterialDatabase() = default;

  MaterialDatabase(const MaterialDatabase&) = delete;
  MaterialDatabase& operator=(const MaterialDatabase&) = delete;

  bool addMaterial(const Material &mat);

  inline size_t size() {
    return m_database.size();
  }

  // may throw MaterialRangeError
  const Material &getMaterial(MaterialID id) const;

  // may throw MaterialRangeError
  MaterialID getMaterialIdFromName(const std::string &name) const;

  // may throw MaterialConfigError
  void populate_from_rosparams(const std::string &ns);

private:

  MaterialID m_last_id_added = Material::id_min;

  std::unordered_map<MaterialID, Material> m_database;

  std::unordered_map<std::string, MaterialID> m_names;

};

// thrown when material configuration is not correctly formatted
class MaterialConfigError : public std::runtime_error
{
public:
  MaterialConfigError(const std::string &what_arg)
    : std::runtime_error(what_arg) { };
};

// thrown when database access fails
class MaterialRangeError : public std::out_of_range
{
public:
  MaterialRangeError(const std::string &what_arg)
    : std::out_of_range(what_arg) { };
};

}

#endif // MATERIAL_DATABASE_H

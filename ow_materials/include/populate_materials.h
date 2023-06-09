// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <string>
#include <stdexcept>

#include <MaterialDatabase.h>

// thrown when material configuration is not correctly formatted
class MaterialConfigError : public std::runtime_error
{
public:
  MaterialConfigError(const std::string &what_arg)
    : std::runtime_error(what_arg) { };
};

void populate_material_database(ow_materials::MaterialDatabase *db_ptr,
                                const std::string &ns);

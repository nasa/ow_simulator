// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIAL_UTILS_H
#define MATERIAL_UTILS_H

#include "MaterialDatabase.h"
#include "material_mixing.h"

// Utility functions for working with Blends and Bulks that require access to
// a material database.

namespace ow_materials {

// may throw MaterialRangeError
double computeBulkMass(Bulk const &bulk, MaterialDatabase const &db);

}

#endif // MATERIAL_UTILS_H

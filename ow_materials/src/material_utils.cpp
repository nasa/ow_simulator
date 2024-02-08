// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "material_utils.h"

using namespace ow_materials;

double ow_materials::computeBulkMass(Bulk const &bulk,
                                     MaterialDatabase const &db)
{
  double mass = 0.0;
  for (auto const &c : bulk.getComposition()) {
    auto material = db.getMaterial(c.first);
    double fraction = static_cast<double>(c.second);
    mass += material.density * fraction * bulk.getVolume();
  }
  return mass;
}

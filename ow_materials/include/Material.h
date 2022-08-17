// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIAL_H
#define MATERIAL_H

#include <limits>

namespace ow_materials
{

using MaterialID = unsigned char;

struct Color
{
  uint8_t r, g, b;
};

struct Material
{
  constexpr static MaterialID id_min = std::numeric_limits<MaterialID>::min();
  constexpr static MaterialID id_max = std::numeric_limits<MaterialID>::max();

  std::string name;
  float occurrence;
  float science_value;

  double density;

  Color color;

  // TODO:
  
  //  1. add appearance parameters
  // double cohesion;
  // double friction_angle;

  //  2. add terramechanics parameters

};

}

#endif // MATERIAL_H

// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIAL_H
#define MATERIAL_H

#include <limits>
#include <unordered_map>

namespace ow_materials
{

using MaterialID = uint8_t;

struct Color
{
  double r, g, b;
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
  // float abledo;
  //  2. add terramechanics parameters
  // double cohesion;
  // double friction_angle;

};

// STUB: The requirements for this class are unclear at this stage, will expand
//       on later when they are better defined.
// Possible Improvements:
//   1. Shared contiguous memory for all instances (locality of reference)
//   2. Cache commonly reference values, like blend hardness (computation)
//   3. Encapsulation of blend map
//   4. Blend maintenance functions like `normalize`
struct MaterialBlend {
  std::unordered_map<MaterialID, float> m_blend;
};

}

#endif // MATERIAL_H

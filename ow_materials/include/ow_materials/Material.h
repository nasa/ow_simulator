// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIAL_H
#define MATERIAL_H

#include <limits>

namespace ow_materials
{

struct Color
{
  Color(float red, float green, float blue) : r(red), g(green), b(blue) { };

  float r, g, b;

  inline bool operator==(const Color &other) {
    return (r == other.r && g == other.g && b == other.b);
  };
  inline bool operator!=(const Color &other) {
    return !(*this == other);
  };
};

using MaterialID = std::uint8_t;

struct Material
{
  constexpr static MaterialID id_min = std::numeric_limits<MaterialID>::min();
  constexpr static MaterialID id_max = std::numeric_limits<MaterialID>::max();

  std::string name;
  float occurrence;
  float science_value;

  double density;

  Color visualize_color;

  // TODO:
  // double cohesion;
  // double friction_angle;

};

} // namespace ow_materials

#endif // MATERIAL_H

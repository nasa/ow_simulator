// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <numeric>
#include <cmath>

#include <Materials.h>

using namespace ow_materials;

void MaterialBlend::merge(MaterialBlend const &other) {
  // Merge this blend with another blend by assigning the summation of their
  // concentrations per material. A material missing from the current
  // concentration will be added and assigned the same value from the external
  // blend.
  m_blend = std::accumulate(other.m_blend.begin(), other.m_blend.end(), m_blend,
    [](BlendType &m, BlendType::value_type const &p) {
      return (m[p.first] += p.second, m);
    }
  );
}

void MaterialBlend::normalize() {
  const auto denominator = sumConcentrations();
  divideElementWise(denominator);
}

void MaterialBlend::divideElementWise(float denominator) {
  for (auto &b : m_blend) {
    b.second /= denominator;
  }
}

bool MaterialBlend::isNormalized() {
  return std::fabs(1.0 - sumConcentrations())
    <= std::numeric_limits<float>::epsilon();
}

float MaterialBlend::sumConcentrations() {
  auto sum = std::accumulate(m_blend.begin(), m_blend.end(), 0.0,
    [](float value, BlendType::value_type const &p) {
      return value + p.second;
    }
  );
  return sum;
}

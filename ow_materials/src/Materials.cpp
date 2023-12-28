// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <numeric>
#include <cmath>

#include "Materials.h"

using std::vector;

using namespace ow_materials;

MaterialBlend::MaterialBlend(vector<MaterialConcentration> const &composition)
{
  m_blend.clear();
  for (const auto &c : composition) {
    m_blend[c.id] = c.proportion;
  }
}

bool MaterialBlend::isNormalized() const
{
  return std::fabs(1.0 - sumConcentrations())
    <= std::numeric_limits<float>::epsilon();
}

void MaterialBlend::normalize()
{
  const auto denominator = sumConcentrations();
  divideElementWise(denominator);
}

void MaterialBlend::merge(MaterialBlend const &other, float ratio_of_whole)
{
  // Merge this blend with another blend by summing their proportions per
  // material. A material missing from the current proportion will be added
  // and assigned the same value from the external blend.
  m_blend = std::accumulate(other.m_blend.begin(), other.m_blend.end(), m_blend,
    [ratio_of_whole](BlendType &m, BlendType::value_type const &p) {
      return (m[p.first] += p.second * ratio_of_whole, m);
    }
  );
}

void MaterialBlend::add(MaterialID id, float proportion)
{
  if (proportion <= 0.0f) {
    return;
  }
  m_blend[id] = proportion;
}

void MaterialBlend::clear()
{
  m_blend.clear();
}

void MaterialBlend::divideElementWise(float denominator)
{
  for (auto &b : m_blend) {
    b.second /= denominator;
  }
}

float MaterialBlend::sumConcentrations() const
{
  return std::accumulate(m_blend.begin(), m_blend.end(), 0.0,
    [](float value, BlendType::value_type const &p) {
      return value + p.second;
    }
  );
}

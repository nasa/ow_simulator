// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <numeric>
#include <cmath>
#include <vector>

#include "material_mixing.h"

using std::vector, std::max;

using namespace ow_materials;

Blend::Blend(vector<MaterialConcentration> const &composition)
{
  m_composition.clear();
  for (const auto &c : composition) {
    m_composition[c.id] = c.proportion;
  }
}

Blend::Blend(CompositionType const &composition)
{
  m_composition = composition;
}

bool Blend::isNormalized() const
{
  return std::fabs(1.0 - sumConcentrations())
    <= std::numeric_limits<float>::epsilon();
}

void Blend::normalize()
{
  const auto denominator = sumConcentrations();
  divideElementWise(denominator);
}

void Blend::merge(Blend const &other, float ratio_of_whole)
{
  // Merge this blend with another blend by summing their proportions per
  // material. A material missing from the current proportion will be added
  // and assigned the same value from the external blend.
  m_composition = std::accumulate(
    other.m_composition.begin(), other.m_composition.end(), m_composition,
    [ratio_of_whole](CompositionType &m, CompositionType::value_type const &p) {
      return (m[p.first] += p.second * ratio_of_whole, m);
    }
  );
}

void Blend::add(MaterialID id, float proportion)
{
  if (proportion <= 0.0f) {
    return;
  }
  m_composition[id] = proportion;
}

void Blend::divideElementWise(float denominator)
{
  for (auto &b : m_composition) {
    b.second /= denominator;
  }
}

float Blend::sumConcentrations() const
{
  return std::accumulate(m_composition.begin(), m_composition.end(), 0.0,
    [](float value, CompositionType::value_type const &p) {
      return value + p.second;
    }
  );
}

Bulk::Bulk(Blend const &blend, double volume)
  : m_blend(blend), m_volume(volume)
{
  initialize();
}

Bulk::Bulk(BulkExcavation const &bulk)
  : m_blend(bulk.composition), m_volume(bulk.volume)
{
  initialize();
}

void Bulk::mix(Bulk const &other)
{
  if (other.m_volume == 0.0) {
    // nothing to mix, stay the same
    return;
  }
  if (m_volume == 0.0) {
    // nothing locally to mix, take on other's values
    m_blend = other.m_blend;
    m_volume = other.m_volume;
    return;
  }
  // The ratio of the volume of the other's volume to this one's volume yields a
  // scaling factor to scale the other's composition by while merging it into
  // this one's composition.
  float ratio_of_whole = static_cast<float>(other.m_volume / m_volume);
  m_blend.merge(other.m_blend, ratio_of_whole);
  m_blend.normalize();
  m_volume += other.m_volume;
}

double Bulk::reduce(double volume_deducted)
{
  auto prev_volume = m_volume;
  m_volume = max(0.0, m_volume - volume_deducted);
  if (m_volume == 0.0) {
    m_blend.clear();
  }
  return prev_volume - m_volume;
}

void Bulk::initialize()
{
  // volume is never below zero
  m_volume = max(0.0, m_volume);
  // volume is zero iff blend is empty
  if (m_blend.isEmpty() || m_volume == 0.0) {
    clear();
    return;
  }
  // blend must be normalized
  m_blend.normalize();
}

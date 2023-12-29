// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIAL_MIXING_H
#define MATERIAL_MIXING_H

#include <unordered_map>

#include "ow_materials/MaterialConcentration.h"
#include "ow_materials/BulkExcavation.h"

#include "Material.h"

namespace ow_materials {

using CompositionType = std::unordered_map<MaterialID, float>;

class Blend {

  // A blend of different materials at concentrations that add up to unity.

  // Possible improvements to Blend:
  //   1. Shared contiguous memory for all instances (locality of reference)
  //   2. Cache commonly reference values, like blend hardness (computation)

public:
  Blend()                        = default;
  Blend(const Blend&)            = default;
  Blend& operator=(const Blend&) = default;
  ~Blend()                       = default;

  Blend(std::vector<MaterialConcentration> const &composition);

  inline CompositionType const &getComposition() const {
    return m_composition;
  }

  inline CompositionType &getComposition() {
    return m_composition;
  }

  inline bool isEmpty() const {
    return m_composition.empty();
  }

  inline void clear() {
    m_composition.clear();
  }

  bool isNormalized() const;

  void normalize();

  void merge(Blend const &other, float ratio_of_whole = 1.0f);

  void add(MaterialID id, float concentration);


private:
  void divideElementWise(float denominator);

  float sumConcentrations() const;

  CompositionType m_composition;

};

class Bulk {

  // An extension of Blend that makes the class analogous to a physical
  // clump of dirt by tracking volume in addition to relative concentrations.
  // Normalization of the Blend component is handled internally, so this class
  // will always represent something physical; namely, a perfectly mixed
  // collection of materials that occupies some volume.

public:
  Bulk(const Bulk&)            = default;
  Bulk& operator=(const Bulk&) = default;
  ~Bulk()                      = default;

  Bulk() : m_blend(), m_volume(0.0) {};
  Bulk(Blend const &blend, double volume);
  Bulk(BulkExcavation const &bulk);

  // Analogous to Blend::merge. In this case, mix is a more accurate name since
  // the inclusion of a volume means it behaves physically like mixing this bulk
  // with another.
  void mix(Bulk const &other);

  // Reduce the volume of this bulk by some amount. Since a Bulk is always
  // considered to be perfectly mixed, this will never affect the composition.
  // If the deducted volume is larger than the contained volume, the volume of
  // the Bulk after this operation will be zero.
  // Returns the actual volume deducted from the bulk (always positive).
  double reduce(double volume_deducted);

  // Generate a BulkExcavation message type out of this instance's data.
  BulkExcavation generateExcavationBulkMessage() const;

  inline CompositionType const &getComposition() const {
    return m_blend.getComposition();
  }

  inline Blend const &getBlend() const {
    return m_blend;
  }

  inline double getVolume() const {
    return m_volume;
  }

  inline bool isEmpty() const {
    return m_volume == 0.0;
  }

  inline void clear() {
    m_blend.clear();
    m_volume = 0.0;
  }

private:
  Blend m_blend;
  double m_volume;

  // Enforces consistency of member variables.
  void initialize();

};

} // namespace ow_materials

#endif // MATERIAL_MIXING_H

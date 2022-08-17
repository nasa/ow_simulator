// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef AXIS_ALIGNED_GRID_H
#define AXIS_ALIGNED_GRID_H

#include <vector>
#include <memory>

#include <ignition/math/AxisAlignedBox.hh>

namespace ow_materials {

template <typename T>
class AxisAlignedGrid {
public:
  AxisAlignedGrid(double const x0, double const y0, double const z0,
                  double const x1, double const y1, double const z1,
                  double const side_length, T const initial_value);
  ~AxisAlignedGrid() = default;

  AxisAlignedGrid() = delete;
  AxisAlignedGrid(const AxisAlignedGrid&) = delete;
  AxisAlignedGrid& operator=(const AxisAlignedGrid&) = delete;

  T &getCellValue(size_t const i, size_t const j, size_t const k);
  T const &getCellValue(size_t const i, size_t const j, size_t const k) const;

  inline ignition::math::Vector3d getDiagonal() const {
    return ignition::math::Vector3(
      m_domain->XLength(), m_domain->YLength(), m_domain->ZLength()
    );
  };

  inline const ignition::math::Vector3d &getMaxCorner() const {
    return m_domain->Max();
  };

  inline const ignition::math::Vector3d &getMinCorner() const {
    return m_domain->Min();
  };

  inline ignition::math::Vector3d getCenter() const {
    return m_domain->Center();
  };

private:

  inline size_t index(size_t const i, size_t const j, size_t const k) const {
    assert(i < m_dimensions.X());
    assert(j < m_dimensions.Y());
    assert(k < m_dimensions.Z());
    return i + j * m_dimensions.X() + k * m_dimensions.X() * m_dimensions.Y();
  };

  std::unique_ptr<ignition::math::AxisAlignedBox> m_domain;

  ignition::math::Vector3<size_t> m_dimensions;

  double const m_side_length;

  std::vector<T> m_cells;
};

}

#include <AxisAlignedGrid.tpp>

#endif // AXIS_ALIGNED_GRID_H

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

  const ignition::math::Vector3d &getDiagonal() const;
  const ignition::math::Vector3d &getMaxCorner() const;
  const ignition::math::Vector3d &getMinCorner() const;
  const ignition::math::Vector3d &getCenter() const;

  inline T &getCellValue(size_t const i, size_t const j, size_t const k) {
    return m_cells[index(i, j, k)];
  };

  inline T const &getCellValue(size_t const i, size_t const j,
                               size_t const k) const {
    return m_cells[index(i, j, k)];
  };

private:

  inline size_t index(size_t const i, size_t const j, size_t const k) const {
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

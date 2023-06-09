// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef AXIS_ALIGNED_GRID_H
#define AXIS_ALIGNED_GRID_H

#include <vector>
#include <memory>

#include <ignition/math/AxisAlignedBox.hh>

namespace ow_materials {

// thrown when configuration values for the grid are not correctly formatted
class GridConfigError : public std::runtime_error
{
public:
  GridConfigError(const std::string &what_arg)
    : std::runtime_error(what_arg) { };
};

template <typename T>
class AxisAlignedGrid {
public:
  AxisAlignedGrid(double x0, double y0, double z0,
                  double x1, double y1, double z1,
                  double side_length, T initial_value);
  ~AxisAlignedGrid() = default;

  AxisAlignedGrid() = delete;
  AxisAlignedGrid(const AxisAlignedGrid&) = delete;
  AxisAlignedGrid& operator=(const AxisAlignedGrid&) = delete;

  const ignition::math::Vector3d &getDiagonal() const;
  const ignition::math::Vector3d &getMaxCorner() const;
  const ignition::math::Vector3d &getMinCorner() const;
  const ignition::math::Vector3d &getCenter() const;

  inline T &getCellValue(size_t i, size_t j, size_t k) {
    return m_cells[index(i, j, k)];
  };

  inline T const &getCellValue(size_t i, size_t j, size_t k) const {
    return m_cells[index(i, j, k)];
  };

private:

  inline size_t index(size_t i, size_t j, size_t k) const {
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

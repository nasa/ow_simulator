// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef AXIS_ALIGNED_GRID_H
#define AXIS_ALIGNED_GRID_H

#include <vector>
#include <memory>
#include <functional>

#include <gazebo/common/Assert.hh>

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
                  double cell_side_length, T initial_value);
  ~AxisAlignedGrid() = default;

  AxisAlignedGrid() = delete;
  AxisAlignedGrid(const AxisAlignedGrid&) = delete;
  AxisAlignedGrid& operator=(const AxisAlignedGrid&) = delete;

  using PositionType = ignition::math::Vector3d;
  using IndexType = ignition::math::Vector3<size_t>;

  const PositionType &getDiagonal() const;
  const PositionType &getMaxCorner() const;
  const PositionType &getMinCorner() const;
  const PositionType &getCenter() const;

  inline double getCellLength() const { return m_cell_length; }

  // const T &getCellValueAtPoint(double x, double y, double z) const;

  inline bool containsPoint(double x, double y, double z) const {
    return m_domain->Contains(ignition::math::Vector3d(x, y, z));
  }

  void runForEach(std::function<void(T, PositionType)> f) const;

  void runForEachInRectangle(PositionType v0, PositionType v1,
    // void(*f)(const &T, PositionType));
    std::function<void(T, PositionType)> f) const;

private:

  inline const T &getCellValue(IndexType i) const {
    return m_cells[index(i)];
  };

  inline T &getCellValue(IndexType i) {
    return m_cells[index(i)];
  };

  auto convertPositionToIndex(PositionType grid_coords) const;

  auto getCellCenter(IndexType i) const;

  size_t index(IndexType i) const;

  double m_cell_length;

  std::unique_ptr<ignition::math::AxisAlignedBox> m_domain;

  IndexType m_dimensions;

  std::vector<T> m_cells;
};

}

#include <AxisAlignedGrid.tpp>

#endif // AXIS_ALIGNED_GRID_H

// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef AXIS_ALIGNED_GRID_H
#define AXIS_ALIGNED_GRID_H

#include <vector>
#include <memory>
#include <cmath>
#include <functional>

#include "ignition/math/AxisAlignedBox.hh"
#include "ignition/math/Pose3.hh"

namespace ow_materials {

// thrown when configuration values for the grid are not correctly formatted
class GridConfigError : public std::runtime_error
{
public:
  GridConfigError(const std::string &what_arg)
    : std::runtime_error(what_arg) { };
};

using GridTransformType = ignition::math::Pose3d;
using GridPositionType = ignition::math::Vector3d;
using GridPositionType2D = ignition::math::Vector2d;

// TODO: rename class to something not axis aligned specific
template <typename T>
class AxisAlignedGrid {

  // An axis-aligned box which is partitioned into cubes of a given side length.
  // Each cube, or cell, or voxel, is linked to a value of type T.

  using GridIndexType = ignition::math::Vector3<std::size_t>;

public:
  AxisAlignedGrid(GridPositionType corner_1, GridPositionType corner_2,
                  double cell_side_length, GridTransformType frame_transform);
  ~AxisAlignedGrid() = default;

  AxisAlignedGrid() = delete;
  AxisAlignedGrid(const AxisAlignedGrid&) = delete;
  AxisAlignedGrid& operator=(const AxisAlignedGrid&) = delete;

  const GridPositionType &getDiagonal() const;
  const GridPositionType &getMaxCorner() const;
  const GridPositionType &getMinCorner() const;
  const GridPositionType &getCenter() const;

  const double &getCellVolume() const;
  inline double getCellLength() const { return m_cell_length; }

  // WARNING: execution will may take a long time depending on grid resolution
  // void runForEach(std::function<void(const T&, GridPositionType)> f) const;
  void runForEach(
    std::function<void(T&, GridPositionType, GridPositionType)> f);
  
  void runForEachInColumn(GridPositionType bottom, double height,
    std::function<void(const T&, GridPositionType)> f) const;

private:

  GridPositionType getCellCenter(GridIndexType i) const;

  inline const T &getCellValue(GridIndexType i) const {
    return m_cells[index(i)];
  };

  inline T &getCellValue(GridIndexType i) {
    return m_cells[index(i)];
  };

  GridPositionType transformIntoFrame(const GridPositionType &p) const {
    auto m = p - m_frame_offset;
    return GridPositionType{
       m.X() * std::cos(m_frame_yaw) + m.Y() * std::sin(m_frame_yaw),
      -m.X() * std::sin(m_frame_yaw) + m.Y() * std::cos(m_frame_yaw),
       m.Z()
    };
  };

  GridPositionType transformIntoWorld(const GridPositionType &p) const{
    auto m = GridPositionType{
      p.X() * std::cos(m_frame_yaw) - p.Y() * std::sin(m_frame_yaw),
      p.X() * std::sin(m_frame_yaw) + p.Y() * std::cos(m_frame_yaw),
      p.Z()
    };
    return m + m_frame_offset;
  };

  std::size_t index(GridIndexType i) const;

  double m_cell_length;

  GridPositionType m_frame_offset;
  double m_frame_yaw;

  std::unique_ptr<ignition::math::AxisAlignedBox> m_domain;

  GridIndexType m_dimensions;

  std::vector<T> m_cells;
};

}

#include <AxisAlignedGrid.tpp>

#endif // AXIS_ALIGNED_GRID_H

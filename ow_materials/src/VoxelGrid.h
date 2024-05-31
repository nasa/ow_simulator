// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef VOXEL_GRID_H
#define VOXEL_GRID_H

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

template <typename T>
class VoxelGrid {

  // A box which is partitioned into cubes of a constant side length.
  // Each cube/cell/voxel is given a value of type T.

  using GridIndexType = ignition::math::Vector3<std::size_t>;

public:
  VoxelGrid(GridPositionType corner_1, GridPositionType corner_2,
                  double cell_side_length, GridTransformType frame_transform);
  ~VoxelGrid() = default;

  VoxelGrid() = delete;
  VoxelGrid(const VoxelGrid&) = delete;
  VoxelGrid& operator=(const VoxelGrid&) = delete;

  const GridPositionType &getDimensions() const;
  const GridPositionType &getWorldCenter() const;

  const double &getCellVolume() const;
  inline double getCellLength() const { return m_cell_length; }

  // WARNING: Execution will may take a long time depending on grid resolution.
  void runForEach(
    std::function<void(T&, GridPositionType, GridPositionType)> f);
  
  void runForEachInColumn(GridPositionType bottom, double height,
    std::function<void(const T&, GridPositionType)> f) const;

private:

  inline const GridPositionType &getMaxCorner() const {
    return m_domain->Max();
  };
  inline const GridPositionType &getMinCorner() const {
    return m_domain->Min();
  };

  GridPositionType getCellCenter(GridIndexType i) const;

  inline const T &getCellValue(GridIndexType i) const {
    return m_cells[index(i)];
  };

  inline T &getCellValue(GridIndexType i) {
    return m_cells[index(i)];
  };

  GridPositionType transformIntoFrame(const GridPositionType &p) const;
  GridPositionType transformIntoWorld(const GridPositionType &p) const;

  std::size_t index(GridIndexType i) const;

  double m_cell_length;

  GridPositionType m_frame_offset;
  double m_frame_yaw;

  std::unique_ptr<ignition::math::AxisAlignedBox> m_domain;

  GridIndexType m_dimensions;

  std::vector<T> m_cells;
};

}

#include <VoxelGrid.tpp>

#endif // VOXEL_GRID_H

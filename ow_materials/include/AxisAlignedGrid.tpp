// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <cmath>
#include <limits>

// DEBUG
#include <iostream>
#include <gazebo/gazebo.hh>
#include <Materials.h>

using std::min, std::max, std::round, std::floor, std::ceil, std::size_t;

namespace ow_materials {

static void increaseToNearestMultiple(double &out_x, double const divisor) {
  out_x += std::remainder(out_x, divisor);
}

template <typename T>
AxisAlignedGrid<T>::AxisAlignedGrid(
  double const x0, double const y0, double const z0,
  double const x1, double const y1, double const z1,
  double const cell_side_length, T const initial_value)
  : m_cell_length(cell_side_length)
{

  // cannot have a negative/zero length
  if (m_cell_length <= 0) {
    throw GridConfigError("Side length of a cell must be positive!");
  }

  auto const min_corner = ignition::math::Vector3d(
    min(x0, x1), min(y0, y1), min(z0, z1)
  );
  auto max_corner = ignition::math::Vector3d(
    max(x0, x1), max(y0, y1), max(z0, z1)
  );
  auto diagonal = max_corner - min_corner;

  // adjust diagonal so each dimension is a perfect multiple of the side length
  increaseToNearestMultiple(diagonal.X(), m_cell_length);
  increaseToNearestMultiple(diagonal.Y(), m_cell_length);
  increaseToNearestMultiple(diagonal.Z(), m_cell_length);

  // reassign for adjusted diagonal
  max_corner = min_corner + diagonal;

  // compute number of cells in each dimension
  m_dimensions = ignition::math::Vector3<size_t>(
    static_cast<size_t>(round(diagonal.X() / m_cell_length)),
    static_cast<size_t>(round(diagonal.Y() / m_cell_length)),
    static_cast<size_t>(round(diagonal.Z() / m_cell_length))
  );

  // restrict all dimensions to cube root of the max size_t to prevent overflow
  constexpr auto max_dimension = std::cbrt(
    static_cast<float>(std::numeric_limits<size_t>::max())
  );
  if (   m_dimensions.X() >= max_dimension
      || m_dimensions.Y() >= max_dimension
      || m_dimensions.Z() >= max_dimension) {
    throw GridConfigError("Grid dimensions are too large to be encoded. Try a "
                          "smaller domain or larger cell side length.");
  }

  m_domain = std::make_unique<ignition::math::AxisAlignedBox>(
    min_corner, max_corner
  );

  // fill cells with initial value
  auto total_cells = m_dimensions.X() * m_dimensions.Y() * m_dimensions.Z();
  // m_cells.resize(total_cells, initial_value);

  // DEMONSTRATION
  m_cells.resize(total_cells);
  for (size_t i = 0; i != m_dimensions.X(); ++i) {
    for (size_t j = 0; j != m_dimensions.Y(); ++j) {
      for (size_t k = 0; k != m_dimensions.Z(); ++k) {
        getCellValue(IndexType(i, j, k)).m_blend = {
          {
            static_cast<uint8_t>(
              (i > m_dimensions.X() / 2) + (j > m_dimensions.Y() / 2)),
            1.0f
          }
        };
      }
    }
  }

};

template <typename T>
const ignition::math::Vector3d &AxisAlignedGrid<T>::getDiagonal() const {
  static const ignition::math::Vector3d diagonal(
    m_domain->XLength(), m_domain->YLength(), m_domain->ZLength()
  );
  return diagonal;
};

template <typename T>
const ignition::math::Vector3d &AxisAlignedGrid<T>::getMaxCorner() const {
  static const ignition::math::Vector3d max = m_domain->Max();
  return max;
};

template <typename T>
const ignition::math::Vector3d &AxisAlignedGrid<T>::getMinCorner() const {
  static const ignition::math::Vector3d min = m_domain->Min();
  return min;
};

template <typename T>
const ignition::math::Vector3d &AxisAlignedGrid<T>::getCenter() const {
  static const ignition::math::Vector3d center = m_domain->Center();
  return center;
};

// template <typename T>
// const T &AxisAlignedGrid<T>::getCellValueAtPoint(PositionType v) const
// {
//   auto grid_coord = v - getMinCorner();
//   // gzlog << "grid_coord = " << grid_coord.X() << " x "
//   //                   << grid_coord.Y() << " x "
//   //                   << grid_coord.Z() << " meters." << std::endl;
//   return getCellValue(convertPositionToIndex(grid_coord));
// };

template <typename T>
auto AxisAlignedGrid<T>::convertPositionToIndex(
  PositionType grid_coords) const
{
  return IndexType(
    static_cast<size_t>(floor(grid_coords.X() / m_cell_length)),
    static_cast<size_t>(floor(grid_coords.Y() / m_cell_length)),
    static_cast<size_t>(floor(grid_coords.Z() / m_cell_length))
  );
};

template <typename T>
auto AxisAlignedGrid<T>::getCellCenter(IndexType i) const
{
  // return getMinCorner() + static_cast<PositionType>(i;
  PositionType grid_coord = PositionType(
    (static_cast<double>(i.X()) + 0.5) * m_cell_length,
    (static_cast<double>(i.Y()) + 0.5) * m_cell_length,
    (static_cast<double>(i.Z()) + 0.5) * m_cell_length);
  return grid_coord + getMinCorner();
};

template <typename T>
void AxisAlignedGrid<T>::runForEachInRectangle(
  PositionType v0, PositionType v1,
  // void(*f)(T, PositionType))
  std::function<void(T, PositionType)> f) const
{
  auto grid_min = ignition::math::Vector3d(min(v0.X(), v1.X()),
                                           min(v0.Y(), v1.Y()),
                                           min(v0.Z(), v1.Z())) - getMinCorner();
  auto grid_max = ignition::math::Vector3d(max(v0.X(), v1.X()),
                                           max(v0.Y(), v1.Y()),
                                           max(v0.Z(), v1.Z())) - getMinCorner();

  // auto idx_min = convertPositionToIndex(grid_min);
  auto idx_min = IndexType(
    static_cast<size_t>(max(grid_min.X() / m_cell_length, 0.0)),
    static_cast<size_t>(max(grid_min.Y() / m_cell_length, 0.0)),
    static_cast<size_t>(max(grid_min.Z() / m_cell_length, 0.0))
  );
  auto idx_max = IndexType(
    min(static_cast<size_t>(
      ceil(grid_max.X() / m_cell_length)), m_dimensions.X()),
    min(static_cast<size_t>(
      ceil(grid_max.Y() / m_cell_length)), m_dimensions.Y()),
    min(static_cast<size_t>(
      ceil(grid_max.Z() / m_cell_length)), m_dimensions.Z())
  );

  // gzlog << "min = (" << idx_min.X() << "," << idx_min.Y() << "," << idx_min.Z() << ")\n";
  // gzlog << "max = (" << idx_max.X() << "," << idx_max.Y() << "," << idx_max.Z() << std::endl;

  for (auto x = idx_min.X(); x != idx_max.X(); ++x) {
    for (auto y = idx_min.Y(); y != idx_max.Y(); ++y) {
      for (auto z = idx_min.Z(); z != idx_max.Z(); ++z) {
        auto i = IndexType(x, y, z);
        f(getCellValue(i), getCellCenter(i));
      }
    }
  }
};

}

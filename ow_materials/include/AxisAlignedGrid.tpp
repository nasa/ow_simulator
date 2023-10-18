// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <cmath>
#include <limits>

#include <gazebo/common/Assert.hh>

using std::min, std::max, std::round, std::floor, std::ceil, std::size_t;

namespace ow_materials {

static void increaseToNearestMultiple(double &out_x, double const divisor)
{
  out_x += std::remainder(out_x, divisor);
}

static GridPositionType minPosition(GridPositionType const &a,
                                    GridPositionType const &b)
{
  return GridPositionType(min(a.X(), b.X()),
                          min(a.Y(), b.Y()),
                          min(a.Z(), b.Z()));
}

static GridPositionType maxPosition(GridPositionType const &a,
                                    GridPositionType const &b)
{
  return GridPositionType(max(a.X(), b.X()),
                          max(a.Y(), b.Y()),
                          max(a.Z(), b.Z()));
}

template <typename T>
AxisAlignedGrid<T>::AxisAlignedGrid(GridPositionType const corner_1,
                                    GridPositionType const corner_2,
                                    double const cell_side_length)
  : m_cell_length(cell_side_length)
{

  // cannot have a negative/zero length
  if (m_cell_length <= 0) {
    throw GridConfigError("Side length of a cell must be positive!");
  }

  auto const min_corner = minPosition(corner_1, corner_2);
  auto max_corner = maxPosition(corner_1, corner_2);
  auto diagonal = max_corner - min_corner;

  // adjust diagonal so each dimension is a perfect multiple of the side length
  increaseToNearestMultiple(diagonal.X(), m_cell_length);
  increaseToNearestMultiple(diagonal.Y(), m_cell_length);
  increaseToNearestMultiple(diagonal.Z(), m_cell_length);

  // reassign for adjusted diagonal
  max_corner = min_corner + diagonal;

  // compute number of cells in each dimension
  m_dimensions = GridIndexType(
    static_cast<size_t>(round(diagonal.X() / m_cell_length)),
    static_cast<size_t>(round(diagonal.Y() / m_cell_length)),
    static_cast<size_t>(round(diagonal.Z() / m_cell_length))
  );

  // restrict all dimensions to cube root of the max size_t to prevent overflow
  // CLARIFICATION: This serves the purpose of an arbitrary upper bound on all
  //    grid dimension. Dependent on the value of the other dimensions, one or
  //    two dimensions could be permitted to be larger than the cube root, but
  //    checking for that condition would be difficult due to that dependency.
  //    Were larger dimensions permitted, they would likely lead to poor
  //    performance anyways. This restriction should remain at least until a
  //    user demonstrates a need for larger grid sizes.
  constexpr size_t max_dimension = static_cast<size_t>(
    std::cbrt(static_cast<float>(std::numeric_limits<size_t>::max()))
  );
  if (   m_dimensions.X() >= max_dimension
      || m_dimensions.Y() >= max_dimension
      || m_dimensions.Z() >= max_dimension) {
    throw GridConfigError("Grid dimensions are too large to be encoded. Try a "
                          "smaller domain or larger cell side length.");
  }

  m_domain = std::make_unique<ignition::math::AxisAlignedBox>(min_corner,
                                                              max_corner);

  // fill cells with initial value
  size_t total_cells = m_dimensions.X() * m_dimensions.Y() * m_dimensions.Z();
  m_cells.resize(total_cells);

  // DEBUG: This is a fast and easy method to populate the grid with non-uniform
  //        material data. It depends on there being at least 3 unique materials
  //        in the database, and it ignores the initial_value. This is just a
  //        workaround until a proper grid generation method is implemented.
  // for (size_t i = 0; i != m_dimensions.X(); ++i) {
  //   for (size_t j = 0; j != m_dimensions.Y(); ++j) {
  //     for (size_t k = 0; k != m_dimensions.Z(); ++k) {
  //       getCellValue(GridIndexType(i, j, k)).getBlendMap() = {
  //         {
  //           static_cast<std::uint8_t>(
  //             (i > m_dimensions.X() / 2) + (j > m_dimensions.Y() / 2)),
  //           1.0f
  //         }
  //       };
  //     }
  //   }
  // }

};

template <typename T>
const GridPositionType &AxisAlignedGrid<T>::getDiagonal() const {
  static const GridPositionType diagonal(m_domain->XLength(),
                                         m_domain->YLength(),
                                         m_domain->ZLength());
  return diagonal;
};

template <typename T>
const GridPositionType &AxisAlignedGrid<T>::getMaxCorner() const {
  static const GridPositionType max = m_domain->Max();
  return max;
};

template <typename T>
const GridPositionType &AxisAlignedGrid<T>::getMinCorner() const {
  static const GridPositionType min = m_domain->Min();
  return min;
};

template <typename T>
const GridPositionType &AxisAlignedGrid<T>::getCenter() const {
  static const GridPositionType center = m_domain->Center();
  return center;
};

template <typename T>
GridPositionType AxisAlignedGrid<T>::getCellCenter(GridIndexType idx) const
{
  GridPositionType grid_coord = GridPositionType(
    (static_cast<double>(idx.X()) + 0.5) * m_cell_length,
    (static_cast<double>(idx.Y()) + 0.5) * m_cell_length,
    (static_cast<double>(idx.Z()) + 0.5) * m_cell_length);
  return grid_coord + getMinCorner();
};

template <typename T>
void AxisAlignedGrid<T>::runForEach(
  std::function<void(const T&, GridPositionType)> f) const
{
  for (size_t x = 0; x != m_dimensions.X(); ++x) {
    for (size_t y = 0; y != m_dimensions.Y(); ++y) {
      for (size_t z = 0; z != m_dimensions.Z(); ++z) {
        auto idx = GridIndexType(x, y, z);
        f(getCellValue(idx), getCellCenter(idx));
      }
    }
  }
};

template <typename T>
void AxisAlignedGrid<T>::runForEach(std::function<void(T&, GridPositionType)> f)
{
  for (size_t x = 0; x != m_dimensions.X(); ++x) {
    for (size_t y = 0; y != m_dimensions.Y(); ++y) {
      for (size_t z = 0; z != m_dimensions.Z(); ++z) {
        auto idx = GridIndexType(x, y, z);
        f(getCellValue(idx), getCellCenter(idx));
      }
    }
  }
};

// template <typename T>
// void AxisAlignedGrid<T>::modifyByColumn(
//   std::function<void(T&, GridPositionType)> f)
// {
//   for (size_t x = 0; x != m_dimensions.X(); ++x) {
//     for (size_t y = 0; y != m_dimensions.Y(); ++y) {
//       f(getCellValue(idx), getCellCenter(idx));
//       for (size_t z = 0; z != m_dimensions.Z(); ++z) {
//         auto idx = GridIndexType(x, y, z);

//       }
//     }
//   }
// }

template <typename T>
void AxisAlignedGrid<T>::runForEachInAxisAlignedBox(
  GridPositionType v0, GridPositionType v1,
  std::function<void(const T&, GridPositionType)> f) const
{
  auto grid_min = minPosition(v0, v1) - getMinCorner();
  auto grid_max = maxPosition(v0, v1) - getMinCorner();

  auto idx_min = GridIndexType(
    static_cast<size_t>(max(grid_min.X() / m_cell_length, 0.0)),
    static_cast<size_t>(max(grid_min.Y() / m_cell_length, 0.0)),
    static_cast<size_t>(max(grid_min.Z() / m_cell_length, 0.0))
  );
  auto idx_max = GridIndexType(
    min(static_cast<size_t>(grid_max.X() / m_cell_length)+1, m_dimensions.X()),
    min(static_cast<size_t>(grid_max.Y() / m_cell_length)+1, m_dimensions.Y()),
    min(static_cast<size_t>(grid_max.Z() / m_cell_length)+1, m_dimensions.Z())
  );

  for (auto x = idx_min.X(); x != idx_max.X(); ++x) {
    for (auto y = idx_min.Y(); y != idx_max.Y(); ++y) {
      for (auto z = idx_min.Z(); z != idx_max.Z(); ++z) {
        auto idx = GridIndexType(x, y, z);
        f(getCellValue(idx), getCellCenter(idx));
      }
    }
  }
};

template <typename T>
size_t AxisAlignedGrid<T>::index(GridIndexType idx) const
{
  GZ_ASSERT(idx.X() < m_dimensions.X(), "x is out of bounds");
  GZ_ASSERT(idx.Y() < m_dimensions.Y(), "y is out of bounds");
  GZ_ASSERT(idx.Z() < m_dimensions.Z(), "z is out of bounds");
  return   idx.X()
         + idx.Y() * m_dimensions.X()
         + idx.Z() * m_dimensions.X() * m_dimensions.Y();
};

}


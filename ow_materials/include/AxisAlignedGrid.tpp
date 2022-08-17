// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <cmath>
#include <algorithm>

namespace ow_materials {

static void increaseToNearestMultiple(double &out_x, double const divisor) {
  out_x += std::remainder(out_x, divisor);
}

template <typename T>
AxisAlignedGrid<T>::AxisAlignedGrid(
  double const x0, double const y0, double const z0,
  double const x1, double const y1, double const z1,
  double const side_length, T const initial_value
) : m_side_length(side_length)
{
  auto const min_corner = ignition::math::Vector3d(
    std::min(x0, x1), std::min(y0, y1), std::min(z0, z1)
  );
  auto max_corner = ignition::math::Vector3d(
    std::max(x0, x1), std::max(y0, y1), std::max(z0, z1)
  );
  auto diagonal = max_corner - min_corner;

  // adjust diagonal so each dimension is a perfect multiple of the side length
  increaseToNearestMultiple(diagonal.X(), m_side_length);
  increaseToNearestMultiple(diagonal.Y(), m_side_length);
  increaseToNearestMultiple(diagonal.Z(), m_side_length);

  // reassign for adjusted diagonal
  max_corner = min_corner + diagonal;

  m_domain = std::make_unique<ignition::math::AxisAlignedBox>(
    min_corner, max_corner
  );

  // compute number of cells in each dimension
  m_dimensions = ignition::math::Vector3<size_t>(
    static_cast<size_t>(std::round(diagonal.X() / m_side_length)),
    static_cast<size_t>(std::round(diagonal.Y() / m_side_length)),
    static_cast<size_t>(std::round(diagonal.Z() / m_side_length))
  );

  // fill cells with initial value
  // TODO: chance of int overflow?
  auto total_cells = m_dimensions.X() * m_dimensions.Y() * m_dimensions.Z();

  m_cells.resize(total_cells, initial_value);

}

template <typename T>
T &AxisAlignedGrid<T>::getCellValue(size_t const i,
                                 size_t const j,
                                 size_t const k) {
  return m_cells[index(i, j, k)];
};

template <typename T>
T const &AxisAlignedGrid<T>::getCellValue(size_t const i,
                                       size_t const j,
                                       size_t const k) const {
  return m_cells[index(i, j, k)];
};

}

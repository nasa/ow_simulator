// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <cmath>
#include <limits>

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

  // cannot have a negative/zero length
  if (m_side_length <= 0) {
    throw GridConfigError("Side length of a cell must be positive!");
  }

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

  // compute number of cells in each dimension
  m_dimensions = ignition::math::Vector3<size_t>(
    static_cast<size_t>(std::round(diagonal.X() / m_side_length)),
    static_cast<size_t>(std::round(diagonal.Y() / m_side_length)),
    static_cast<size_t>(std::round(diagonal.Z() / m_side_length))
  );

  // restrict all dimensions to cube root of the max size_t to prevent overflow
  const auto max_dimension = std::cbrt(
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
  m_cells.resize(total_cells, initial_value);

}

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

}

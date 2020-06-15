#ifndef MERGE_OPERATIONS_H
#define MERGE_OPERATIONS_H

#include <boost/optional/optional.hpp>  // TODO: replace with std::optional for c++17
#include <functional>
#include <string>
#include <unordered_map>

namespace ow_dynamic_terrain
{
// Predefined set of merge opertions used by TerrainModifier
// First parameter is the current_value of the heightmap
// Second parameter is the new_value
class MergeMethods
{
public:
  using MergeMethod = std::function<float(float, float)>;

  static const MergeMethod keep;
  static const MergeMethod replace;
  static const MergeMethod add;
  static const MergeMethod sub;
  static const MergeMethod min;
  static const MergeMethod max;

public:
  static boost::optional<const MergeMethod&> mergeOperationFromString(const std::string& operation);

private:
  static const std::unordered_map<std::string, const MergeMethod&> m_mapOperationNameToOperationFunction;
};
}  // namespace ow_dynamic_terrain

#endif  // MERGE_OPERATIONS_H
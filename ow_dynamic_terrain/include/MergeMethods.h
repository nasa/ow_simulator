// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MERGE_METHODS_H
#define MERGE_METHODS_H

#include <boost/optional/optional.hpp>  // TODO: replace with std::optional for c++17
#include <functional>
#include <string>
#include <unordered_map>

namespace ow_dynamic_terrain
{
// Predefined set of merge methods used by TerrainModifier
// First parameter is the current_value of the heightmap, second parameter is the new_value.
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
  static const MergeMethod avg;

  static boost::optional<const MergeMethod&> mergeMethodFromString(const std::string& method_name);

private:
  static const std::unordered_map<std::string, const MergeMethod&> m_merge_method_map;
};
}  // namespace ow_dynamic_terrain

#endif  // MERGE_METHODS_H

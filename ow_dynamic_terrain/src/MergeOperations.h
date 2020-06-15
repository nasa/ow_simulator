#ifndef MERGE_OPERATIONS_H
#define MERGE_OPERATIONS_H

#include <functional>
#include <string>
#include <unordered_map>
#include <boost/optional/optional.hpp>  // TODO: replace with std::optional for c++17

// Predefined set of merge opertions used by TerrainModifier
// First parameter is the current_value of the heightmap
// Second parameter is the new_value
class MergeOperations
{
public:
  using MergeOperation = std::function<float(float, float)>;

  static const MergeOperation keep;
  static const MergeOperation replace;
  static const MergeOperation add;
  static const MergeOperation sub;
  static const MergeOperation min;
  static const MergeOperation max;

public:
  static boost::optional<const MergeOperation&> mergeOperationFromString(const std::string& operation);

private:
  static const std::unordered_map<std::string, const MergeOperation&>
      m_mapOperationNameToOperationFunction;
};

#endif  // MERGE_OPERATIONS_H
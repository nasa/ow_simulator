#ifndef MERGE_OPERATIONS_H
#define MERGE_OPERATIONS_H

#include <functional>
#include <string>
#include <unordered_map>

// Predefined set of merge opertions used by TerrainModifier
// First parameter is the current_value of the heightmap
// Second parameter is the new_value
class MergeOperations
{
public:
  static const std::function<float(float, float)> keep;
  static const std::function<float(float, float)> replace;
  static const std::function<float(float, float)> add;
  static const std::function<float(float, float)> sub;
  static const std::function<float(float, float)> min;
  static const std::function<float(float, float)> max;

public:
  static std::function<float(float, float)> mergeOperationFromString(const std::string &operation);

private:
  static const std::unordered_map<std::string, std::function<float(float, float)>> mapOperationNameToOperationFunction;
};

#endif  // MERGE_OPERATIONS_H
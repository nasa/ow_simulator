#include "MergeOperations.h"

using std::unordered_map;
using std::function;
using std::string;

const unordered_map<string, function<float(float, float)>> MergeOperations::mapOperationNameToOperationFunction = {
  { "keep", MergeOperations::keep }, { "replace", MergeOperations::replace }, { "add", MergeOperations::add },
  { "sub", MergeOperations::sub },   { "min", MergeOperations::min },         { "max", MergeOperations::max },
};

const function<float(float, float)> MergeOperations::keep = [](float current_value, float new_value) {
  return current_value;
};

const function<float(float, float)> MergeOperations::replace = [](float current_value, float new_value) {
  return new_value;
};

const function<float(float, float)> MergeOperations::add = [](float current_value, float new_value) {
  return current_value + new_value;
};

const function<float(float, float)> MergeOperations::sub = [](float current_value, float new_value) {
  return current_value - new_value;
};

const function<float(float, float)> MergeOperations::min = [](float current_value, float new_value) {
  return std::min(current_value, new_value);
};

const function<float(float, float)> MergeOperations::max = [](float current_value, float new_value) {
  return std::max(current_value, new_value);
};

function<float(float, float)> MergeOperations::mergeOperationFromString(const std::string &operation)
{
  auto it = mapOperationNameToOperationFunction.find(operation);
  if (it == mapOperationNameToOperationFunction.end())
    return keep;

  return it->second;
}
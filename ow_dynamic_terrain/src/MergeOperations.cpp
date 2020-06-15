#include "MergeOperations.h"

using std::unordered_map;
using std::function;
using std::string;
using boost::optional;

const unordered_map<string, const MergeOperations::MergeOperation&> MergeOperations::m_mapOperationNameToOperationFunction = {
  { "keep", MergeOperations::keep }, { "replace", MergeOperations::replace }, { "add", MergeOperations::add },
  { "sub", MergeOperations::sub },   { "min", MergeOperations::min },         { "max", MergeOperations::max },
};

const MergeOperations::MergeOperation MergeOperations::keep = [](float current_value, float new_value) {
  return current_value;
};

const MergeOperations::MergeOperation MergeOperations::replace = [](float current_value, float new_value) {
  return new_value;
};

const MergeOperations::MergeOperation MergeOperations::add = [](float current_value, float new_value) {
  return current_value + new_value;
};

const MergeOperations::MergeOperation MergeOperations::sub = [](float current_value, float new_value) {
  return current_value - new_value;
};

const MergeOperations::MergeOperation MergeOperations::min = [](float current_value, float new_value) {
  return std::min(current_value, new_value);
};

const MergeOperations::MergeOperation MergeOperations::max = [](float current_value, float new_value) {
  return std::max(current_value, new_value);
};

optional<const MergeOperations::MergeOperation&> MergeOperations::mergeOperationFromString(const string &operation)
{
  auto it = m_mapOperationNameToOperationFunction.find(operation);
  if (it == m_mapOperationNameToOperationFunction.end())
   return optional<const MergeOperations::MergeOperation&>();
  return it->second;
}
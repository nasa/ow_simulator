// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "MergeMethods.h"

using boost::optional;
using std::function;
using std::string;
using std::unordered_map;
using namespace ow_dynamic_terrain;

const unordered_map<string, const MergeMethods::MergeMethod &> MergeMethods::m_merge_method_map = {
  { "keep", MergeMethods::keep }, { "replace", MergeMethods::replace }, { "add", MergeMethods::add },
  { "sub", MergeMethods::sub },   { "min", MergeMethods::min },         { "max", MergeMethods::max },
  { "avg", MergeMethods::avg }
};

const MergeMethods::MergeMethod MergeMethods::keep = [](float current_value, float new_value) { return current_value; };

const MergeMethods::MergeMethod MergeMethods::replace = [](float current_value, float new_value) { return new_value; };

const MergeMethods::MergeMethod MergeMethods::add = [](float current_value, float new_value) {
  return current_value + new_value;
};

const MergeMethods::MergeMethod MergeMethods::sub = [](float current_value, float new_value) {
  return current_value - new_value;
};

const MergeMethods::MergeMethod MergeMethods::min = [](float current_value, float new_value) {
  return std::min(current_value, new_value);
};

const MergeMethods::MergeMethod MergeMethods::max = [](float current_value, float new_value) {
  return std::max(current_value, new_value);
};

const MergeMethods::MergeMethod MergeMethods::avg = [](float current_value, float new_value) {
  return 0.5f * (current_value + new_value);
};

optional<const MergeMethods::MergeMethod &> MergeMethods::mergeMethodFromString(const string &method_name)
{
  auto it = m_merge_method_map.find(method_name);
  if (it == m_merge_method_map.end())
    return optional<const MergeMethods::MergeMethod &>();
  return it->second;
}

// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MEMORY_EXT_H
#define MEMORY_EXT_H

/**
 * This module adds the missing 'make_unique' when compiling against C++11 standard. For more info on this refer to:
 * Scott Meyers' Effective Modern C++ Item 21: Prefer std::make_unique and std::make_shared to direct use of new.
 */

// TODO: consider moving the file to a common place

#include <memory>

#if __cplusplus < 201402L  // if less than C++14 is detected add the extension

namespace std
{
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
}  // namespace std

#endif

#endif  // MEMORY_EXT_H
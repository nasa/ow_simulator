#ifndef MEMORY_EXT_H
#define MEMORY_EXT_H

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
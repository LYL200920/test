/*

a ref counted base object to be used with ref_counting_ptr.

*/

#ifndef includeguard_utils_ref_counted_obj_hpp_includeguard
#define includeguard_utils_ref_counted_obj_hpp_includeguard

#include <atomic>
#include <type_traits>
#include "ref_counting_ptr.hpp"

namespace utils
{

template <typename T>
class ref_counted_obj
{
public:
  constexpr ref_counted_obj (void) noexcept : m_count (0) { }
  ref_counted_obj (const ref_counted_obj&) = delete;
  ref_counted_obj (ref_counted_obj&&) = delete;

  ref_counted_obj& operator = (const ref_counted_obj&) = delete;
  ref_counted_obj& operator = (ref_counted_obj&&) = delete;

  friend void retain (T* o) noexcept
  {
    if (o != nullptr)
      ++(o->m_count);
  }

  friend void release (T* o) noexcept (noexcept (delete o))
  {
    if (o != nullptr)
      if (--(o->m_count) == 0)
        delete o;
  }

  friend int retain_count (const T* o) noexcept
  {
    return o == nullptr ? int (o->m_count) : 0;
  }

  ref_counting_ptr<T> ref_counting_from_this (void)
  {
    return ref_counting_ptr<T> ((T*)this);
  }

  ref_counting_ptr<const T> ref_counting_from_this (void) const
  {
    return ref_counting_ptr<const T> ((const T*)this);
  }

private:
  mutable std::atomic<int> m_count;
};

} // namespace utils

#endif  // includeguard_utils_ref_counted_obj_hpp_includeguard

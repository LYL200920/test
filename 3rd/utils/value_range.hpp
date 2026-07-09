#ifndef includeguard_value_range_hpp_includeguard
#define includeguard_value_range_hpp_includeguard
#ifdef __cplusplus

#include <cstdint>
#include <limits>
#include <cassert>
#include <type_traits>
#include <utility>

namespace utils
{

// --------------------------------------------------------------------------
// if possible, check the value at compile time and clamp it to the specified
// value range.
// if a compile time check is not possible, the check will be performed at
// run time.
template <typename T,
	  typename std::conditional<std::is_signed<T>::value, intmax_t, uintmax_t>::type Min,
          typename std::conditional<std::is_signed<T>::value, intmax_t, uintmax_t>::type Max>
class clamped_value
{
public:
  static_assert (std::numeric_limits<T>::lowest () <= Min
		 && std::numeric_limits<T>::max () >= Min, "bad min value");
  static_assert (std::numeric_limits<T>::lowest () <= Max
		 && std::numeric_limits<T>::max () >= Max, "bad max value");

  static constexpr T min (void) { return T (Min); }
  static constexpr T max (void) { return T (Max); }

  constexpr clamped_value (const T& v)
  : m_value (min () != std::numeric_limits<T>::lowest () && v < min ()
	     ? min ()
	     : max () != std::numeric_limits<T>::max () && v > max ()
	       ? max ()
	       : v)
  { }

  constexpr clamped_value (const clamped_value& rhs) : m_value (rhs.m_value) { }
  constexpr clamped_value (clamped_value&& rhs) : m_value (std::move (rhs.m_value)) { }

  constexpr clamped_value& operator = (const clamped_value& rhs) { m_value = rhs.m_value; return *this; }
  constexpr clamped_value& operator = (clamped_value&& rhs) { m_value = std::move (rhs.m_value); return *this; }

  constexpr clamped_value operator = (const T& val) { *this = clamped_value (val); return *this; }

  constexpr operator T (void) const { return m_value; }

private:
  T m_value;
};

// --------------------------------------------------------------------------
// check (at run time) that the value is in the specified range.
template <typename T,
	  typename std::conditional<std::is_signed<T>::value, intmax_t, uintmax_t>::type Min,
          typename std::conditional<std::is_signed<T>::value, intmax_t, uintmax_t>::type Max>
class assert_value_range
{
public:
  static_assert (std::numeric_limits<T>::lowest () <= Min
		 && std::numeric_limits<T>::max () >= Min, "bad min value");
  static_assert (std::numeric_limits<T>::lowest () <= Max
		 && std::numeric_limits<T>::max () >= Max, "bad max value");

  static constexpr T min (void) { return T (Min); }
  static constexpr T max (void) { return T (Max); }

  assert_value_range (const T& v) : m_value (v)
  {
    assert (v >= min ());
    assert (v <= max ());
  }

  constexpr assert_value_range (const assert_value_range& rhs) : m_value (rhs.m_value) { }
  constexpr assert_value_range (assert_value_range&& rhs) : m_value (std::move (rhs.m_value)) { }

  constexpr assert_value_range& operator = (const assert_value_range& rhs) { m_value = rhs.m_value; return *this; }
  constexpr assert_value_range& operator = (assert_value_range&& rhs) { m_value = std::move (rhs.m_value); return *this; }

  constexpr assert_value_range operator = (const T& val) { *this = assert_value_range (val); return *this; }

  constexpr operator T (void) const { return m_value; }

private:
  T m_value;
};

} // namespace utils

#endif // __cplusplus
#endif // includeguard_value_range_hpp_includeguard

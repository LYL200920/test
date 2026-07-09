#ifndef includeguard_utils_text_hpp_includeguard
#define includeguard_utils_text_hpp_includeguard

#include <string>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <utility>
#include <algorithm>
#include <string_view>
#include <charconv>
#include <optional>
#include <limits>
#include <utils/langcomp.hpp>
#include <cstring>
#include <iterator>

#if __cpp_exceptions
  #include <stdexcept>
#endif

namespace utils
{

char* itoa (int value, char* result, int base = 10);

template <typename... Args>
std::string printf_format (const std::string_view& format, Args... args)
{
  const size_t size = snprintf( nullptr, 0, format.data (), std::forward<Args> (args)...);
  std::unique_ptr<char[]> buf (new char[size + 1]);
  std::snprintf (buf.get (), size + 1, format.data (), std::forward<Args> (args)...);
  return std::string (buf.get (), buf.get () + size);
}

// locale invariant (i.e. default C locale) version of std::isblank
inline constexpr bool isblank (char c)
{
  return c == 0x20 || c == 0x09;
}

template <typename Iterator, typename IsBlankFunc>
inline std::string_view
trim_string (Iterator i0, Iterator i1, IsBlankFunc&& isblank_func)
{
  for (; i0 < i1; ++i0)
    if (!isblank_func (*i0))
      break;

  for (; i0 < i1; --i1)
    if (!isblank_func (*(i1 - 1)))
      break;

  assume_always_true (i0 <= i1);

  // some std library debug builds have iterator checking enabled.
  // if 'i0 == i1' and it's an end-iterator (into an empty string or such)
  // the below might derefefence an end-iterator which is invalid and might
  // trigger debug traps.  so don't do it and catch that case separately.
  if (i0 == i1)
    return { };
  else
    return std::string_view (&*i0, i1 - i0);
}

template <typename Iterator>
inline std::string_view
trim_string (Iterator i0, Iterator i1)
{
  return trim_string (i0, i1, &utils::isblank);
}

inline std::string_view
trim_string (const std::string_view& s)
{
  return trim_string (s.begin (), s.end ());
}

template <typename IsBlankFunc>
inline std::string_view
trim_string (const std::string_view& s, IsBlankFunc&& isblank_func)
{
  return trim_string (s.begin (), s.end (), isblank_func);
}


inline constexpr bool isdigit (char c) noexcept
{
  return c >= '0' && c <= '9';
}

// convert a digit character to an integer in the range 0...9
inline constexpr int  digit_toi (char c) noexcept
{
  return (int)c - '0';
}

inline constexpr int ito_digit (char c) noexcept
{
  return (int)c + '0';
}

// this is the same function as std::isxdigit, but it doesn't use the c library.
// with newlib, we have to link the whole c library because it's not inlined.
// instead its implementation uses a look-up table, which is also bigger than
// this this function here.
#if __cpp_constexpr >= 201304
inline constexpr bool isxdigit (char c) noexcept
{
  switch (c)
  {
    default:
      return false;

    case '0': case '1': case '2': case '3': case '4': case '5': case '6':
    case '7': case '8': case '9':
    case 'A': case 'B': case 'C': case 'D': case 'E': case 'F':
    case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
      return true;
  }
}
#else
inline constexpr bool isxdigit(char c) noexcept
{
  return c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5'
         || c == '6' || c == '7' || c == '8' || c == '9'
         || c == 'A' || c == 'B' || c == 'C' || c == 'D' || c == 'E' || c == 'F'
         || c == 'a' || c == 'b' || c == 'c' || c == 'd' || c == 'e' || c == 'f';
}
#endif

#if __cpp_constexpr >= 201304
inline constexpr bool isxdigit_upper (char c) noexcept
{
  switch (c)
  {
    default:
      return false;

    case '0': case '1': case '2': case '3': case '4': case '5': case '6':
    case '7': case '8': case '9':
    case 'A': case 'B': case 'C': case 'D': case 'E': case 'F':
      return true;
  }
}
#else
inline constexpr bool isxdigit_upper(char c) noexcept
{
  return c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5'
         || c == '6' || c == '7' || c == '8' || c == '9'
         || c == 'A' || c == 'B' || c == 'C' || c == 'D' || c == 'E' || c == 'F';
}
#endif

#if __cpp_constexpr >= 201304
inline constexpr bool isxdigit_lower (char c) noexcept
{
  switch (c)
  {
    default:
      return false;

    case '0': case '1': case '2': case '3': case '4': case '5': case '6':
    case '7': case '8': case '9':
    case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
      return true;
  }
}
#else
inline constexpr bool isxdigit_lower (char c) noexcept
{
  return c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5'
         || c == '6' || c == '7' || c == '8' || c == '9'
         || c == 'a' || c == 'b' || c == 'c' || c == 'd' || c == 'e' || c == 'f';
}
#endif

// convert an xdigit (upper case) to an integer in the range 0x0...0xF
inline constexpr char xdigit_upper_toi (char c) noexcept
{
  return isdigit (c) ? digit_toi (c) : ((int)c - 'A' + 0x0A);
}

// convert an xdigit (lower case) to an integer in the range 0x0...0xF
inline constexpr char xdigit_lower_toi (char c) noexcept
{
  return isdigit (c) ? digit_toi (c) : (int)c - 'a' + 0x0A;
}

// convert an xdigit (lower or upper case) to an integer in the range 0x0...0xF
inline constexpr char xdigit_toi (char c) noexcept
{
  static_assert ('A' < 'a', "");

  return isdigit (c) ? digit_toi (c) : (int)c - (c >= 'a' ? 'a' : 'A') + 0x0A;
}

inline constexpr char ito_xdigit_upper (char c) noexcept
{
  return c + ((int)c < 0xA ? '0' : ('A' - 0xA));
}

inline constexpr char ito_xdigit_lower (char c) noexcept
{
  return c + ((int)c < 0xA ? '0' : ('a' - 0xA));
}

// copy the specified string from input to output and replace each wildcard
// in the form "%<digits>" with the according string from the arguments.
// the max. number of digits supported is 10.  they are assumed to represent
// a decimal number.
template <typename OutputIter, typename... Args>
inline OutputIter fill_wildcards (const char* str, OutputIter out, Args&&... aargs)
{
  std::array<const char*, sizeof...(aargs)> args = {{ std::forward<Args> (aargs)... }};

  constexpr unsigned int max_digits = 10;
  char digits[max_digits];
  unsigned int cur_digit_count = 0;
  bool in_wildcard = false;

  while (true)
  {
    auto c = *str++;

    if (in_wildcard)
    {
      if (isdigit (c))
      {
	if (cur_digit_count == max_digits)
	{
	  // ignore it
	  *out++ = '%';
	  for (unsigned int i = 0; i < cur_digit_count; ++i)
	    *out++ = digits[i];

	  in_wildcard = false;
	}
	else
	  digits[cur_digit_count++] = c;
      }
      else
      {
	// hit the first non-digit char -> end of number

	if (cur_digit_count == 0)
	{
	  // treat %% as escaped single %
	  if (c == '%')
	  {
	    // c will be written to out below
	  }
	  else
	    *out++ = '%';
	}
	else
	{
	  uint64_t number = 0;
	  uint64_t scale = 1;
	  for (unsigned int i = 0; i < cur_digit_count; ++i)
	  {
	    number += scale * digit_toi (digits[cur_digit_count - i - 1]);
	    scale *= 10;
	  }

	  if (number >= args.size ())
	  {
	    // ignore it
	    *out++ = '%';
	    for (unsigned int i = 0; i < cur_digit_count; ++i)
	      *out++ = digits[i];
	  }
	  else
	  {
	    for (const char* s = args[number]; *s != '\0'; ++s)
	      *out++ = *s;
	  }
	}

	in_wildcard = false;
      }
    }
    else if (c == '%')
    {
      in_wildcard = true;
      cur_digit_count = 0;
    }

    if (c == '\0')
      break;

    if (!in_wildcard)
      *out++ = c;
  }

  return out;
}

// check if 'str' starts with a substring 'x'.
// also works with std::string
inline bool starts_with (const std::string_view& str,
			 const std::string_view& x)
{
  if (str.length () < x.length () || x.empty ())
    return false;

  assume_always_true (str.length ()  > 0);
  assume_always_true (x.length ()  > 0);

  // string_view::substr will always check for out_of_range
  // even if that's never the case.  avoid it by open coding the substr
  // special case here.
  // return str.substr (0, x.length ()) == x;
  return std::string_view (
	str.data (), std::min (str.length (), x.length ())) == x;
}

// check if 'str' ends with a substring 'x'.
// also works with std::string
inline bool ends_with (const std::string_view& str,
		       const std::string_view& x)
{
  if (str.length () < x.length () || x.empty ())
    return false;

  assume_always_true (str.length ()  > 0);
  assume_always_true (x.length ()  > 0);

  // string_view::substr will always check for out_of_range
  // even if that's never the case.  avoid it by open coding the substr
  // special case here.
//   return str.substr (str.length () - x.length ()) == x;

  auto pos = str.length () - x.length ();
  auto rlen = str.length () - pos;

  return std::string_view (str.data () + pos, rlen) == x;
}

template <typename InputIter, typename OutputIter> inline
OutputIter to_hex (InputIter begin, InputIter end, OutputIter out)
{
  static const char* hex_str_map = "0123456789ABCDEF";

  for (; begin != end; ++begin)
  {
    uint8_t x = *begin;
    *out++ = hex_str_map[ x >> 4 ];
    *out++ = hex_str_map[ x & 0x0F ];
  }

  return out;
}

template <typename T, size_t N, typename OutputIter> inline
OutputIter to_hex (const std::array<T, N>& d, OutputIter out)
{
  return to_hex (d.begin (), d.end (), out);
}

template <typename T, size_t N> inline
std::string to_hex (const std::array<T, N>& d)
{
  std::string res;

  const size_t byte_count = N * sizeof (T);
  res.reserve (byte_count * 2);

  to_hex (d, std::back_inserter (res));

  return res;
}

template <typename Container>
inline typename std::enable_if<sizeof (typename Container::value_type) == 1, std::string>::type
to_hex (const Container& d)
{
  static const char* hex_str_map = "0123456789ABCDEF";

  std::string res;
  res.reserve (d.size () * 2);
  auto out = std::back_inserter (res);
  for (auto c : d)
  {
    *out++ = hex_str_map[ (c >> 4) & 0x0F ];
    *out++ = hex_str_map[ (c >> 0) & 0x0F ];
  }

  return res;
}

template <typename T, size_t N>
inline typename std::enable_if<sizeof (T) == 1, std::string>::type
to_hex (const T (&d)[N])
{
  std::string res;
  res.reserve (N * 2);
  to_hex (std::begin (d), std::end (d), std::back_inserter (res));
  return res;
}

template <typename InputIter, typename EndIter, typename OutputIter> inline
OutputIter from_hex (InputIter i, const EndIter& i_end, OutputIter out)
{
  while (i != i_end)
  {
    auto c0 = *i;
    ++i;

    if (!isxdigit (c0))
    {
      #if __cpp_exceptions
	throw std::invalid_argument (printf_format ("invalid xdigit character '%c'", c0));
      #else
	break;
      #endif
    }

    if (i == i_end)
    {
      // write out partial byte
      *out++ = xdigit_toi (c0) << 4;
      break;
    }

    auto c1 = *i;
    ++i;

    if (!isxdigit (c1))
    {
      #if __cpp_exceptions
	throw std::invalid_argument (printf_format ("invalid xdigit character '%c'", c1));
      #else
	break;
      #endif
    }

    *out = (xdigit_toi (c0) << 4) | xdigit_toi (c1);
    ++out;
  }

  return out;
}

// std::atoi re-implementation which also tells whether the result is valid or zero.
// does not use the actual std::atoi for more compact size.
// the c library implementations tend to drag in a whole bunch of other stuff.
// for example, it might use std::strol to implement std::atoi and std::strol
// requires setting of errno, which requires re-entrancy handling and so on.

inline std::pair<int, bool>
atoi (std::string_view str)
{
  auto i0 = str.begin ();
  auto i1 = str.end ();

  // skip leading white space
  for (; i0 != i1; ++i0)
    if (!utils::isblank (*i0))
      break;

  if (i0 == i1)
    return { 0, false };

  // check for first sign character
  bool negative = false;
  if (*i0 == '-')
  {
    negative = true;
    ++i0;
  }
  else if (*i0 == '+')
    ++i0;

  unsigned int value = 0;
  unsigned int ok_digit_count = 0;

  constexpr unsigned int max_val_pos = std::numeric_limits<int>::max ();
  constexpr unsigned int max_val_neg = (unsigned int)0 - (unsigned int)std::numeric_limits<int>::lowest ();

  constexpr unsigned int max_val_prev_base_pos = max_val_pos / 10;
  constexpr unsigned int max_val_prev_base_neg = max_val_neg / 10;

  constexpr unsigned int max_val_digit_pos = max_val_pos % 10;
  constexpr unsigned int max_val_digit_neg = max_val_neg % 10;

  const auto max_val_prev_base = negative ? max_val_prev_base_neg : max_val_prev_base_pos;
  const auto max_val_digit = negative ? max_val_digit_neg : max_val_digit_pos;

  for (; i0 != i1; ++i0)
  {
    if (!utils::isdigit (*i0))
      break;

    unsigned int d = utils::digit_toi (*i0);
    ++ok_digit_count;

    if (value > max_val_prev_base
	|| (value == max_val_prev_base && d > max_val_digit))
      return { 0, false };

    value = value * 10 + d;
  }

  if (ok_digit_count == 0)
    return { 0, false };

  return { negative ? -(int)value : (int)value, true };
}


template <typename T, typename Str> inline std::optional<T>
from_chars_to (Str&& s)
{
  T val;
  return std::from_chars (s.data (), s.data () + s.size (), val).ec == std::errc ()
	 ? std::optional<T> (val) : std::nullopt;
}


// ---------------------------------------
// convert string into fixed-sized array.
// the result is always a zero-terminated string.
// if the input string is shorter than the result array the array is
// padded with zeros.

template <typename ArrayValueType, unsigned int ArraySize>
inline std::array<ArrayValueType, ArraySize>
str_to_array (const std::string_view& str)
{
  static_assert (sizeof (ArrayValueType) == sizeof (std::remove_reference_t<decltype (str)>::value_type));
  std::array<ArrayValueType, ArraySize> r;
  auto copy_sz = std::min (str.size (), r.size ()-1);
  std::memcpy (r.data (), str.begin (), copy_sz);
  std::memset (r.data () + copy_sz, 0, r.size () - copy_sz);
  return r;
}


} // namespace utils
#endif // includeguard_utils_text_hpp_includeguard

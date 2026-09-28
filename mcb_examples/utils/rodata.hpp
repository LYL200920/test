#ifndef includeguard_rodata_hpp_includeguard
#define includeguard_rodata_hpp_includeguard

#include <array>
#include <utility>
#include <stdexcept>
#include <type_traits>
#include <functional>
#include <string_view>

namespace utils
{

// ---------------------------------------------------------------------------
// assuming that the argument is a compile-time string, which is an array
// of characters, return its length, excluding the zero-terminating char.
template <size_t N> constexpr unsigned int conststr_length (char const (&)[N])
{
  return N - 1;
}

// ---------------------------------------------------------------------------
// construct a std::string_view from something that is known to be a compile
// time constant string.
template <size_t N> constexpr std::string_view inline
conststr_string_view (char const (&str)[N])
{
  return { str, N - 1 };
}

template <size_t N> constexpr std::string_view inline
conststr_string_view (const std::array<char, N>& str)
{
  return { str.data (), N };
}

// ---------------------------------------------------------------------------
// some ways how to construct an std::array

// from function argument list
template <typename... TT> constexpr inline
std::array<typename std::common_type<TT...>::type, sizeof... (TT)>
make_array (TT... tt)
{
  return {{ tt... }};
}

// from plain array
// this can also be used to convert a string literal (which really
// is just a zero-terminated char array) to a std::array<char, N> at
// compile time.
template <typename T, size_t N, size_t... I> constexpr inline std::array<T, N>
make_array_1 (const T (&chr)[N], std::index_sequence<I...>)
{
  return {{ (chr[I])... }};
}

template <typename T, size_t N> constexpr inline std::array<T, N>
make_array (const T (&chr)[N])
{
  return make_array_1 (chr, std::make_index_sequence<N> ());
}

// repeat a single value N times.
template <typename T, size_t N, T (*F)(size_t), size_t... I>
constexpr inline std::array<T, N>
make_array_2 (std::index_sequence<I...>)
{
  return {{ F (I)... }};
}

template <typename T, size_t N, T (*F)(size_t)> constexpr inline std::array<T, N>
make_array (void)
{
  return make_array_2<T, N, F> (std::make_index_sequence<N> ());
}


// concatenate std::array and argument list into a single std::array

template <typename T, size_t N, size_t... I,
	  typename... TT>
constexpr inline
std::array<typename std::common_type<T, TT...>::type, N + sizeof... (TT)>
make_array_3 (const std::array<T, N>& a, std::index_sequence<I...>, TT... tt)
{
  return {{ a[I]..., tt... }};
}

template <typename T, size_t N, typename... TT> constexpr inline auto
make_array (const std::array<T, N>& a, TT... tt)
{
  return make_array_3 (a, std::make_index_sequence<N> (), tt...);
}

template <typename T, std::size_t N, std::size_t M> constexpr inline auto
concat_array (std::array<T, N> a, std::array<T, M> b)
{
  std::array < T, N + M > r = { };
  auto out = r.begin ();

  for (auto& c : a)
    *out++ = c;

  for (auto& c : b)
    *out++ = c;

  return r;
}

// ---------------------------------------------------------------------------
// string comparison of std::array<char, N> strings.

template <size_t M, size_t N>
constexpr int strcmp_1 (const std::array<char, M>& a,
                        const std::array<char, N>& b, size_t i)
{
  // have to check i against M and N or else it won't be evaluated at
  // compile time.
  return i < M && i < N && a[i] == b[i]
         ? strcmp_1 (a, b, i + 1)
         : (a[i] - b[i]) > 0
           ? 1
           : (a[i] - b[i]) < 0
             ? -1
             : 0;
}

template <size_t M, size_t N>
constexpr int strcmp (const std::array<char, M>& a,
                      const std::array<char, N>& b)
{
  return strcmp_1 (a, b, 0);
}

// ---------------------------------------------------------------------------
// sort array.  the array element type must be comparable using operator <

template <typename T, size_t N> constexpr inline std::array<T, N>
sort_array (const std::array<T, N>& elements)
{
  T a[N] = { };

  for (size_t i = 0; i < N; ++i)
    a[i] = elements[i];

  for (size_t i = 0; i < N; ++i)
    for (size_t j = 0; j < N - 1; ++j)
      if (!(a[j] < a[j+1]))
      {
	auto a0 = a[j];
	auto a1 = a[j+1];
	a[j] = a1;
	a[j+1] = a0;
      }

  return make_array (a);
}

// ---------------------------------------------------------------------------
// partiion array using a predicate function.
// all elements for which F returns true will come first, followed by
// all elements for which F returns false.

template <typename T, bool (*F)(const T&), size_t N> constexpr inline std::array<T, N>
partition_array (const std::array<T, N>& elements)
{
  T a[N] = { };

  for (size_t i = 0; i < N; ++i)
    a[i] = elements[i];

  size_t first = 0;
  for (; first < N && F (a[first]); ++first);

  if (first < N)
    for (size_t i = first + 1; i < N; ++i)
      if (F (a[i]))
      {
	auto tmp = a[first];
	a[first] = a[i];
	a[i] = tmp;
	++first;
      }

  return make_array (a);
}


// ---------------------------------------------------------------------------
// copy-convert

template <typename From, typename To, size_t N, size_t... I>
constexpr inline std::array<To, N>
convert_array_1 (const std::array<From, N>& src, std::index_sequence<I...>)
{
  return {{ (src[I])... }};
}

template <typename From, typename To, size_t N>
constexpr inline std::array<To, N>
convert_array (const std::array<From, N>& src)
{
  return convert_array_1<From, To, N> (src, std::make_index_sequence<N> ());
}

// ---------------------------------------------------------------------------
// replace elements in the first array with elements in the second array.

template <typename T, bool (*F)(const T&, const T&), size_t N, size_t M>
constexpr inline std::array<T, N>
replace_array_elements (const std::array<T, N>& first,
			const std::array<T, M>& second)
{
  T a[N] = { };
  for (size_t i = 0; i < N; ++i)
    a[i] = first[i];

  for (size_t i = 0; i < M; ++i)
    for (size_t j = 0; j < N; ++j)
      if (F (a[j], second[i]))
	a[j] = second[i];

  return make_array (a);
}

template <typename T, size_t N, size_t M>
constexpr inline std::array<T, N>
replace_array_elements (const std::array<T, N>& first,
			const std::array<T, M>& second)
{
  T a[N] = { };
  for (size_t i = 0; i < N; ++i)
    a[i] = first[i];

  for (size_t i = 0; i < M; ++i)
    for (size_t j = 0; j < N; ++j)
      if ((a[j] == second[i]))
	a[j] = second[i];

  return make_array (a);
}

// ---------------------------------------------------------------------------
// check that there are no duplicate elements in the sorted array.
// uses operator == for the element comparison.

extern bool array_verify_unique_1_fail (void);

template <typename T, size_t N>
inline constexpr bool
array_verify_unique_1 (const std::array<T, N>& a, size_t i = 0)
{
  return i < N-1 ? a[i] == a[i+1] ? array_verify_unique_1_fail ()
				: array_verify_unique_1 (a, i + 1)
	       : true;
}

template <typename T, size_t N>
constexpr inline std::array<T, N>
array_verify_unique (const std::array<T, N>& elements)
{
  return array_verify_unique_1 (elements), elements;
}

// ---------------------------------------------------------------------------
// a struct that is often used to construct string tables during compile time.

template <typename IndexType>
struct str_table_entry
{
  IndexType num;
  std::string_view str;

  constexpr str_table_entry (void) noexcept : num (), str () { }
  constexpr str_table_entry (IndexType n, std::string_view s) noexcept : num (n), str (s) { }
  constexpr str_table_entry (IndexType n) noexcept : num (n), str () { }

  constexpr bool operator < (const str_table_entry& rhs) const noexcept
  {
    return std::less<IndexType> () (num, rhs.num);
  }

  constexpr bool operator == (const str_table_entry& rhs) const noexcept
  {
    return std::equal_to<IndexType> () (num, rhs.num);
  }

  constexpr operator std::string_view (void) const { return str; }

  // the assumption here is that the string_view is constructed from a
  // compile-time string.  thus we know it's zero-terminated and we can treat
  // it as a plain c string.
  constexpr operator const char* (void) const { return str.data (); }
};

// ---------------------------------------------------------------------------
// format an integer to a string at compile time

// original idea taken from here
// https://stackoverflow.com/questions/6713420/c-convert-integer-to-string-at-compile-time

// alternative implementation just for reference

#if 0
// calculate number of digits needed, including minus sign, excluding zero-terminator
constexpr unsigned int calc_str_length (int x)
{
  return x < 0 ? 1 + calc_str_length (-x) : x < 10 ? 1 : 1 + calc_str_length (x / 10);
}

template<char... Args> struct const_chars
{
  static constexpr const char data[sizeof... (Args)] = { Args ... };
  static constexpr unsigned int size = sizeof... (Args);
};

// recursive number-printing template, general case (for three or more digits)
template<unsigned int Size, int Value, char... Args> struct const_itoa_impl
{
  using type = typename const_itoa_impl<Size - 1, Value / 10, '0' + std::abs (Value) % 10, Args...>::type;
};

// special case for two digits; minus sign is handled here
template<int Value, char... Args> struct const_itoa_impl<2, Value, Args...>
{
  using type = const_chars <Value < 0 ? '-' : '0' + Value / 10, '0' + std::abs (Value) % 10, Args...>;
};

// special case for one digit (positive numbers only)
template<int Value, char... Args> struct const_itoa_impl<1, Value, Args...>
{
   using type = const_chars <'0' + Value, Args...>;
};


template <int N> constexpr std::string_view const_itoa (void)
{
  constexpr typename const_itoa_impl< calc_str_length (N), N, '\0'>::type x = { };
  return { x.data, x.size };
}

#endif


template <int N, bool ZeroTerminate = false> struct const_itoa
{
  static constexpr bool zero_terminated = ZeroTerminate;

  // calculate number of 10-base digits needed to represent the number
  // incl. the minus sign, excl. zero terminator
  static constexpr unsigned int digit10_count (int x)
  {
    return x < 0 ? 1 + digit10_count (-x) : x < 10 ? 1 : 1 + digit10_count (x / 10);
  }

  static constexpr auto to_chars (void)
  {
    std::array<char, digit10_count (N) + (zero_terminated ? 1 : 0)> chars = { };

    auto out = chars.rbegin ();

    if (zero_terminated)
      *out++ = '\0';

    if (N == 0)
      *out++ = '0';

    for (auto n = std::abs (N); n != 0; n /= 10)
      *out++ = '0' + (n % 10);

    if (N < 0)
      *out++ = '-';

    return chars;
  }

  // this is the main trick, to assign the template variant data to a
  // static variable, which then later can be referred to by std::string_view.
  // to allow compile time concatination of the string with another string,
  // return the raw array type.
  static constexpr auto value = to_chars ();
/*
  static constexpr auto char_data = to_chars ();
  static constexpr std::string_view value = { char_data.data (),
					      char_data.size () - (zero_terminated ? 1 : 0) };
*/
};

template <int N> inline constexpr auto const_itoa_v = const_itoa<N>::value;


} // namespace utils
#endif // includeguard_rodata_hpp_includeguard

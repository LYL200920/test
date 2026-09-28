/*

iterate the input container (or range) while counting the elements
(assigning indices on the fly) and copy those elements into the output
iterator, which are within the specified index range.

this can be useful for situations where a known number of parameters
at known positions is expected by the user code.

*/

#ifndef includeguard_copy_index_range_hpp_includeguard
#define includeguard_copy_index_range_hpp_includeguard

#include <cstddef>
#include <utils/iterator.hpp>

namespace utils
{

struct copy_from_index
{
  constexpr copy_from_index (void) : value (0) { }
  constexpr copy_from_index (std::size_t v) : value (v) { }
  constexpr copy_from_index (const copy_from_index& rhs) : value (rhs.value) { }

  std::size_t value;
};

struct copy_to_index
{
  constexpr copy_to_index (void) : value (0) { }
  constexpr copy_to_index (std::size_t v) : value (v) { }

  std::size_t value;
};

template <typename InputIter, typename OutputIter>
inline OutputIter
copy_index_range (InputIter first,
		  typename std::enable_if< is_iterator<InputIter>::value, InputIter>::type last,
		  OutputIter out,
		  copy_from_index from, copy_to_index to)
{
  std::size_t i = 0;
  for (; first != last; ++first)
  {
    if (i >= from.value && i <= to.value)
      *out++ = *first;

    ++i;
  }

  return out;
}

template <typename Container, typename OutputIter>
inline OutputIter copy_index_range (const Container& c, OutputIter out,
		  copy_from_index from, copy_to_index to)

{
  return copy_index_range (c.begin (), c.end (), out, from, to);
}


} // namespace utils
#endif // includeguard_copy_index_range_hpp_includeguard

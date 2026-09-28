/*

a utiliy to remap bits.

the bit remap is given a set of bit position pairs, which define a mapping
   src bitpos -> dst bitpos

then it can be used to forward and reverse map up to 32 bits.

FIXME: use constexpr functions and helpers to construct the mapping arrays.
       ideally we'd get compile-time generated mapping arrays in .rodata.
       for example (needs c++14 for constexpr make_pair) ...

         static const bit_remap mymap (std::make_pair (4, 5),
                                       std::make_pair (5, 4));

       ... would result in the whole bit_remap object to be placed in .rodata.

FIXME: maybe handle special cases such as full word bit reversals.
*/

#ifndef includeguard_bit_remap_hpp_includeguard
#define includeguard_bit_remap_hpp_includeguard

#include <array>
#include <type_traits>
#include <utility>
#include <algorithm>
#include <limits>

namespace utils
{

class bit_remap
{
public:
  typedef unsigned int bit_pos;
  typedef std::pair<bit_pos, bit_pos> bit_pos_pair;

  typedef unsigned int use_uint_type;

  bit_remap (void) { }

  template <typename... Pairs>
  bit_remap (Pairs&&... pairs)
  {
    // capture all values in an array
    std::array<bit_pos_pair, sizeof... (Pairs)> arr {{ std::forward<Pairs> (pairs)... }};

    static_assert (arr.size () <= std::numeric_limits<use_uint_type>::digits,
		   "too many bit mappings");

    // sort array by the source bit position.
    std::sort (arr.begin (), arr.end (),
	[] (const bit_pos_pair& a, const bit_pos_pair& b) { return a.first < b.first; });

    // fill the src -> dst mapping array
    for (bit_pos idst = 0, isrc = 0; idst < m_src_dst.size (); ++idst)
    {
      if (arr[isrc].first == idst)
      {
	m_src_dst[idst] = use_uint_type (1) << arr[isrc].second;
	++isrc;
      }
      else
	m_src_dst[idst] = 0;
    }

    // sort array by the destination bit position.
    std::sort (arr.begin (), arr.end (),
	[] (const bit_pos_pair& a, const bit_pos_pair& b) { return a.second < b.second; });

    // fill the dst -> src mapping array
    for (bit_pos idst = 0, isrc = 0; idst < m_dst_src.size (); ++idst)
    {
      if (arr[isrc].second == idst)
      {
	m_dst_src[idst] = use_uint_type (1) << arr[isrc].first;
	++isrc;
      }
      else
	m_dst_src[idst] = 0;
    }
  }

  // forward mapping (src -> dst)
  use_uint_type fwd (use_uint_type bits_in) const
  {
    return remap (bits_in, m_src_dst.data (), m_src_dst.size ());
  }

  // reverse mapping (dst -> src)
  use_uint_type rev (use_uint_type bits_in) const
  {
    return remap (bits_in, m_dst_src.data (), m_dst_src.size ());
  }

private:
  static unsigned long remap (use_uint_type bits_in, const use_uint_type* table,
			      unsigned int count)
  {
    unsigned int res = 0;

    for (unsigned int i = 0; i < count && bits_in != 0; ++i)
    {
      auto t = *table++;
      auto mask = 0 - (bits_in & 1);

      res |= t & mask;
      bits_in >>= 1;
    }

    return res;
  }

  // the array which holds the destination bit position OR masks.
  // for every source bit there is one entry.
  std::array<use_uint_type, std::numeric_limits<use_uint_type>::digits> m_src_dst;

  // same as above but for mapping dst -> src
  std::array<use_uint_type, std::numeric_limits<use_uint_type>::digits> m_dst_src;
};

} // namespace utils
#endif // includeguard_bit_remap_hpp_includeguard

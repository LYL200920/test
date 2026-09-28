/*

a bitset that is used for storing enum values.  the assumption is that each
enum value is unique and all enums form a number sequence like 0,1,2,3,4,5 ...
the number sequence minimum and maximum values can be arbitrary but the
conversion from some integral number to the enum value must be valid.

*/

#ifndef includeguard_utils_enum_bitset_hpp_includeguard
#define includeguard_utils_enum_bitset_hpp_includeguard

#include <bitset>
#include <iterator>
#include <algorithm>
#include <limits>

namespace utils
{

template < typename EnumType, EnumType MinVal, EnumType MaxVal,
	   typename BitContainer = std::bitset<(std::size_t)MaxVal - (std::size_t)MinVal + 1> >
class enum_bitset
{
  static constexpr std::size_t from_enum (EnumType e) { return (std::size_t)e - (std::size_t)MinVal; }
  static constexpr EnumType to_enum (std::size_t i) { return (EnumType)(i + (std::size_t)MinVal); }

public:
  constexpr enum_bitset (void) { }

  constexpr explicit enum_bitset (const BitContainer& b) : m_bits (b) { }

  const BitContainer& raw_bits (void) const { return m_bits; }

  bool operator == (const enum_bitset& rhs) const { return m_bits == rhs.m_bits; }
  bool operator != (const enum_bitset& rhs) const { return m_bits != rhs.m_bits; }

  typename BitContainer::reference operator[] (EnumType e) { return m_bits[from_enum (e)]; }
  constexpr bool operator[] (EnumType e) const { return m_bits[from_enum (e)]; }

  bool test (EnumType e) const { return m_bits.test (from_enum (e)); }

  bool all (void) const { return m_bits.all (); }
  bool any (void) const { return m_bits.any (); }
  bool none (void) const { return m_bits.none (); }

  std::size_t count (void) const { return m_bits.count (); }

  constexpr std::size_t size (void) const { return m_bits.size (); }

  enum_bitset& operator &= (const enum_bitset& rhs) { m_bits &= rhs.m_bits; return *this; }
  enum_bitset& operator |= (const enum_bitset& rhs) { m_bits |= rhs.m_bits; return *this; }
  enum_bitset& operator ^= (const enum_bitset& rhs) { m_bits ^= rhs.m_bits; return *this; }
  enum_bitset operator ~ (void) { return { ~m_bits }; }

  // bit shifts not included on purpose

  enum_bitset& set (void) { m_bits.set (); return *this; }
  enum_bitset& set (EnumType e) { m_bits.set (from_enum (e)); return *this; }

  enum_bitset& reset (void) { m_bits.reset (); return *this; }
  enum_bitset& reset (EnumType e) { m_bits.reset (from_enum (e)); return *this; }

  enum_bitset& flip (void) { m_bits.flip (); return *this; }
  enum_bitset& flip (EnumType e) { m_bits.flip (from_enum (e)); return *this; }


  class iterator : public std::iterator<std::bidirectional_iterator_tag,
					EnumType,
					std::ptrdiff_t>
  {
  public:
    constexpr iterator (void) : m_i (i_end ()), m_bits (nullptr) { }

    iterator (const BitContainer& bits) : m_i (from_enum (MinVal)), m_bits (&bits)
    {
      // find the first bit that is set.
      auto i = m_i;
      for (; i != i_end () && !(*m_bits)[i]; ++i);
      m_i = i;
    }

    void swap (iterator& other) noexcept
    {
      std::swap (m_i, other.m_i);
    }

    iterator& operator ++ (void)
    {
      // find next bit that is set.

      auto i = m_i;
      ++i;
      for (; i != i_end () && !(*m_bits)[i]; ++i);

      m_i = i;
      return *this;
    }

    iterator operator ++ (int)
    {
      iterator r = *this;
      operator++ ();
      return r;
    }

    iterator& operator -- (void)
    {
      // find previous bit that is set.

      auto i = m_i;
      --i;
      for (; !(*m_bits)[i]; --i);

      m_i = i;
      return *this;
    }

    iterator operator -- (int)
    {
      iterator r = *this;
      operator-- ();
      return r;
    }

    bool operator == (const iterator& rhs) const { return m_i == rhs.m_i; }
    bool operator != (const iterator& rhs) const { return m_i != rhs.m_i; }

    EnumType operator * (void) const { return to_enum (m_i); }

  private:
    static constexpr std::size_t i_end () { return from_enum (MaxVal) + 1; }

    std::size_t m_i;
    const BitContainer* m_bits;
  };

  // when ptr is 16 bit, std::ptrdiff_t might not be enough to represent
  // all difference values for e.g. 32 bit enum values.
  static_assert ((from_enum (MaxVal) + 1) - from_enum (MinVal)
		   <= std::numeric_limits<typename iterator::difference_type>::max (), "");

  using const_iterator = iterator;

  iterator begin (void) const { return iterator (m_bits); }
  iterator end (void) const { return iterator (); }

  const_iterator cbegin (void) const { return const_iterator (m_bits); }
  const_iterator cend (void) const { return const_iterator (); }

private:
  BitContainer m_bits;
};

} // namespace utils

template < typename EnumType, EnumType MinVal, EnumType MaxVal>
utils::enum_bitset < EnumType, MinVal, MaxVal >
operator & (const utils::enum_bitset < EnumType, MinVal, MaxVal >& lhs,
	    const utils::enum_bitset < EnumType, MinVal, MaxVal >& rhs)
{
  return { lhs.raw_bits () & rhs.raw_bits () };
}

template < typename EnumType, EnumType MinVal, EnumType MaxVal>
utils::enum_bitset < EnumType, MinVal, MaxVal >
operator | (const utils::enum_bitset < EnumType, MinVal, MaxVal >& lhs,
	    const utils::enum_bitset < EnumType, MinVal, MaxVal >& rhs)
{
  return { lhs.raw_bits () | rhs.raw_bits () };
}

template < typename EnumType, EnumType MinVal, EnumType MaxVal>
utils::enum_bitset < EnumType, MinVal, MaxVal >
operator ^ (const utils::enum_bitset < EnumType, MinVal, MaxVal >& lhs,
	    const utils::enum_bitset < EnumType, MinVal, MaxVal >& rhs)
{
  return { lhs.raw_bits () ^ rhs.raw_bits () };
}

namespace std
{

/*
// FIXME: this doesn't match.  can't deduce template parameter EnumType...

template<typename EnumType, EnumType MinVal, EnumType MaxVal>
inline void
swap (typename utils::enum_bitset<EnumType, MinVal, MaxVal>::iterator& a,
      typename utils::enum_bitset<EnumType, MinVal, MaxVal>::iterator& b, int) noexcept;
{
  a.swap (b);
}

*/

} // namespace utils

#endif // includeguard_utils_enum_bitset_hpp_includeguard

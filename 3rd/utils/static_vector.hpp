#ifndef includeguard_utils_static_vector_hpp_includeguard
#define includeguard_utils_static_vector_hpp_includeguard

#include <type_traits>
#include <stdexcept>
#include <cstdlib>
#include <initializer_list>

// for is_iterator
#include <utils/iterator.hpp>

// for assert_unreachable
#include <cassert>
#include <utils/langcomp.hpp>

namespace utils
{

// FIXME: add specialization for static_vector<bool, N> to use std::bitset

template <typename T, std::size_t N> class static_vector
{
public:
  using value_type = T;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;
  using reference = T&;
  using const_reference = const T&;
  using pointer = T*;
  using const_pointer = const T*;
  using iterator = pointer;
  using const_iterator = const_pointer;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  static_vector (void) noexcept { m_size = 0; }

  static_vector (size_type count, const T& value = T ())
  {
    // every ctor of T might throw.  keep the m_size valid at all times
    // so that the static_vector destructor can delete fully constructed
    // elements.
    m_size = 0;
    check_capacity (count);

    for (size_type i = 0; i < count; ++i)
    {
      new (&m_data[m_size]) T (value);
      ++m_size;
    }
  }

  static_vector (size_type count)
  {
    m_size = 0;
    check_capacity (count);

    for (size_type i = 0; i < count; ++i)
    {
      new (data () + m_size) T ();
      ++m_size;
    }
  }

  template <class InputIt>
  static_vector (InputIt first,
		 typename std::enable_if< is_iterator<InputIt>::value, InputIt>::type last)
  {
    m_size = 0;

    for (; first != last; ++first)
    {
      check_capacity (size () + 1);
      new (data () + m_size) T (*first);
      ++m_size;
    }
  }

  static_vector (const static_vector& other)
  {
    m_size = 0;
    for (auto&& t : other)
    {
      new (data () + m_size) T (t);
      ++m_size;
    }
  }

  static_vector (static_vector&& other)
  {
    m_size = 0;
    for (auto&& t : other)
    {
      new (data () + m_size) T (std::move (t));
      ++m_size;
    }
    other.clear ();
  }

  static_vector (std::initializer_list<T> init)
  {
    m_size = 0;
    check_capacity (init.size ());

    for (auto&& t : init)
    {
      new (data () + m_size) T (t);
      ++m_size;
    }
  }

  ~static_vector (void)
  {
    clear ();
  }


  static_vector& operator = (const static_vector& other)
  {
    if (&other != this)
    {
      clear ();
      for (auto&& t : other)
      {
	new (data () + m_size) T (t);
	++m_size;
      }
    }
    other.clear ();
    return *this;
  }

  static_vector& operator = (static_vector&& other)
  {
    if (&other != this)
    {
      clear ();
      for (auto&& t : other)
      {
	new (data () + m_size) T (std::move (t));
	++m_size;
      }
    }
    other.clear ();
    return *this;
  }

  static_vector& operator = (std::initializer_list<T> init)
  {
    check_capacity (init.size ());

    clear ();
    for (auto&& t : init)
    {
      new (data () + m_size) T (t);
      ++m_size;
    }
    return *this;
  }

  void assign (size_type count, const T& value)
  {
    check_capacity (count);
    clear ();
    for (size_type i = 0; i < count; ++i)
    {
      new (data () + m_size) T (value);
      ++m_size;
    }
  }

  template <class InputIt> void
  assign (InputIt first,
	  typename std::enable_if<
		is_iterator<InputIt>::value
		&& !std::is_same<InputIt, iterator>::value
		&& !std::is_same<InputIt, const_iterator>::value
		&& !std::is_same<InputIt, reverse_iterator>::value
		&& !std::is_same<InputIt, const_reverse_iterator>::value, InputIt>::type last)
  {
    clear ();

    for (; first != last; ++first)
    {
      check_capacity (size () + 1);
      new (data () + m_size) T (*first);
      ++m_size;
    }
  }

  template <class InputIt> void
  assign (InputIt first,
	  typename std::enable_if<
		is_iterator<InputIt>::value
		&& (std::is_same<InputIt, iterator>::value
		    || std::is_same<InputIt, const_iterator>::value), InputIt>::type last)
  {
    if (first == last)
    {
      clear ();
      return;
    }

    if (&*first >= &m_data[0] && &*first <= &m_data[N-1])
    {
      // if the first iterator points into the data area of this vector
      // it's a self assignment of some sort.
      // e.g.  x.assign (x.begin () + 1, x.end () - 1);
      // there are two cases
      // 1) the source range starts at begin (), which is effectively a
      //    an erase starting from the end () iterator onwards.
      // 2) the source range starts after begin (), which means the elements
      //    can be moved by forward iteration.
      if (&*first == &m_data[0])
      {
	while (last () != end ())
	  pop_back ();
      }
      else
      {
	size_type new_size = last - first;
	for (iterator i = begin (); first != last; ++i, ++first)
	  *i = std::move (*first);

	while (size () > new_size)
	  pop_back ();
      }
    }
    else
    {
      // iterators point into some other container, not this.
      clear ();

      for (; first != last; ++first)
      {
        check_capacity (size () + 1);
        new (data () + m_size) T (*first);
        ++m_size;
      }
    }
  }


  template <class InputIt> void
  assign (InputIt first,
	  typename std::enable_if<
		is_iterator<InputIt>::value
		&& (std::is_same<InputIt, reverse_iterator>::value
		    || std::is_same<InputIt, const_reverse_iterator>::value), InputIt>::type last)
  {
    if (first == last)
    {
      clear ();
      return;
    }

    if (&*first >= &m_data[0] && &*first <= &m_data[N-1])
    {
      // if the first iterator points into the data area of this vector
      // it's a self assignment of some sort.
      // e.g.  x.assign (x.rbegin () + 1, x.rend () - 1);
      // because of reverse iteration, it's more complicated to handle
      // overlapping source and destination ranges...
      // leave it unimplemented for now.
      assert_unreachable ();
    }
    else
    {
      // iterators point into some other container, not this.
      clear ();

      for (; first != last; ++first)
      {
        check_capacity (size () + 1);
        new (data () + m_size) T (*first);
        ++m_size;
      }
    }
  }

  void assign (std::initializer_list<T> init)
  {
    check_capacity (init.size ());
    clear ();
    for (auto&& i : init)
    {
      new (data () + m_size) T (i);
      ++m_size;
    }
  }

  reference at (size_type pos)
  {
    if (!pos < size ())
    {
      #ifdef __cpp_exceptions
	throw std::out_of_range ("static_vector out of range");
      #else
	std::abort ();
      #endif
    }
    return *(data () + pos);
  }

  const_reference at (size_type pos) const
  {
    if (!pos < size ())
    {
      #ifdef __cpp_exceptions
	throw std::out_of_range ("static_vector out of range");
      #else
	std::abort ();
      #endif
    }
    return *(data () + pos);
  }

  reference operator [] (size_type pos) { return *(data () + pos); }
  const_reference operator [] (size_type pos) const { return *(data () + pos); }

  const_reference front (void) const noexcept { return *data (); }
  reference front (void) noexcept { return *data (); }

  const_reference back (void) const noexcept { return *(data () + m_size - 1); }
  reference back (void) noexcept { return *(data () + m_size - 1); }

  pointer data (void) noexcept { return (pointer)m_data; }
  const_pointer data (void) const noexcept { return (const_pointer)m_data; }


  iterator begin (void) noexcept { return data (); }
  const_iterator begin (void) const noexcept { return data (); }
  const_iterator cbegin (void) const noexcept { return data (); }

  iterator end (void) noexcept { return data () + m_size; }
  const_iterator end (void) const noexcept { return data () + m_size; }
  const_iterator cend (void) const noexcept { return data () + m_size; }

  reverse_iterator rbegin (void) noexcept { return reverse_iterator (end ()); }
  const_reverse_iterator rbegin (void) const noexcept { return const_reverse_iterator (end ()); }
  const_reverse_iterator crbegin (void) const noexcept { return const_reverse_iterator (end ()); }

  reverse_iterator rend (void) noexcept { return reverse_iterator (begin ()); }
  const_reverse_iterator rend (void) const noexcept { return const_reverse_iterator (begin ()); }
  const_reverse_iterator crend (void) const noexcept { return const_reverse_iterator (begin ()); }


  bool empty (void) const noexcept { return size () == 0; }
  size_type size (void) const noexcept { return m_size; }
  constexpr size_type max_size (void) const noexcept { return N; }
  void reserve (size_type) { }
  constexpr size_type capacity (void) const noexcept { return N; }
  void shrink_to_fit (void) { }


  void clear ()
  {
    while (!empty ())
      pop_back ();
  }

// FIXME: implement insert
// FIXME: implement emplace
// FIXME: implement erase

  void push_back (const T& value)
  {
    check_capacity (size () + 1);
    new (data () + m_size) T (value);
    ++m_size;
  }

  void push_back_unchecked (const T& value)
  {
    new (data () + m_size) T (value);
    ++m_size;
  }

  void push_back (T&& value)
  {
    check_capacity (size () + 1);
    new (data () + m_size) T (std::move (value));
    ++m_size;
  }

  void push_back_unchecked (T&& value)
  {
    new (data () + m_size) T (std::move (value));
    ++m_size;
  }

  template <typename... Args> reference emplace_back (Args&&... args)
  {
    check_capacity (size () + 1);
    new (data () + m_size) T (std::forward<Args> (args)...);
    ++m_size;
    return back ();
  }

  template <typename... Args> reference emplace_back_unchecked (Args&&... args)
  {
    new (data () + m_size) T (std::forward<Args> (args)...);
    ++m_size;
    return back ();
  }


  void pop_back (void)
  {
    back ().~T ();
    --m_size;
  }

  void resize (size_type count)
  {
    if (count < size ())
    {
      while (size () > count)
        pop_back ();
    }
    else if (count > size ())
    {
      check_capacity (count);
      const auto append_count = count - size ();
      for (unsigned int i = 0; i < append_count; ++i)
	emplace_back ();
    }
  }

  void resize (size_type count, const value_type& value)
  {
    if (count < size ())
    {
      while (size () > count)
        pop_back ();
    }
    else if (count > size ())
    {
      check_capacity (count);
      const auto append_count = size () - count;
      for (unsigned int i = 0; i < append_count; ++i)
	emplace_back (value);
    }
  }

// FIXME: implement swap

private:
  constexpr void check_capacity (size_type desired_count)
  {
    if (desired_count > capacity ())
    {
      #ifdef __cpp_exceptions
	throw std::length_error ("static_vector capacity exceeded");
      #else
	std::abort ();
      #endif
    }
  }

  std::size_t m_size;
  std::aligned_storage_t<sizeof (T)> m_data[N];
};

} // namespace utils
#endif // includeguard_utils_static_vector_hpp_includeguard

#ifndef includeguard_utils_ring_buffer_hpp_includeguard
#define includeguard_utils_ring_buffer_hpp_includeguard

#include <type_traits>

namespace utils
{

template <typename T, std::size_t N> class ring_buffer
{
public:
  typedef T value_type;

  // FIXME: allow specifying the size type or choose it dynamically
  // based on N
  typedef std::size_t size_type;

  typedef value_type& reference;
  typedef const value_type& const_reference;
  typedef value_type* pointer;
  typedef const value_type* const_pointer;


  bool empty (void) const { return m_size == 0; }

  // access the first element (that has been pushed in first)
  T& front (void) { return *buffer (m_rd_pos); }
  const T& front (void) const { return *buffer (m_rd_pos); }

  // access the last element (that has been pushed in last)
  T& back (void) { return *buffer(m_wr_pos == 0 ? (N - 1) : (m_wr_pos - 1)); }
  const T& back (void) const { return *buffer(m_wr_pos == 0 ? (N - 1) : (m_wr_pos - 1)); }

  // returns the number of elements in the buffer
  size_type size (void) const { return m_size; }

  // returns the maximum number of elements that can be in the buffer
  constexpr size_type capacity (void) const { return N; }
  constexpr size_type max_size (void) const { return N; }

  // clear the buffer by destroying all objects.
  void clear (void)
  {
    const auto size_a = std::min (m_size, N - m_rd_pos);
    const auto size_b = m_size - size_a;

    for (unsigned int i = 0; i < size_a; ++i)
      buffer (m_rd_pos + i)->~T ();

    for (unsigned int i = 0; i < size_b; ++i)
      buffer (i)->~T ();

    m_wr_pos = m_rd_pos = m_size = 0;
  }

  template <typename... Args> void push_back (Args&&... args)
  {
    if (m_size == N)
    {
      // buffer overrun, drop new element.
    }
    else
    {
      // this might throw.  so we modify the wr_pos counter only after
      // successful element insertion.
      new (buffer (m_wr_pos)) T (std::forward<Args> (args)...);

      m_wr_pos += 1;
      if (m_wr_pos == N)
        m_wr_pos = 0;

      m_size += 1;
    }
  }

  template <typename... Args> void emplace_back (Args&&... args)
  {
    push_back (std::forward<Args> (args)...);
  }

  void pop_front (void)
  {
    if (!empty ())
    {
      buffer (m_rd_pos)->~T ();

      m_rd_pos += 1;
      if (m_rd_pos == N)
	m_rd_pos = 0;

      m_size -= 1;
    }
  }


private:
  // there is an ambiguity when rd and wr are pointing at the same position.
  // either the buffer is empty, or the buffer is full.
  // if the counters are not wrapped around at the size, it can be resolved
  // with additional comparisons.  instead we just use a separate size field
  // to count the number of elements.
  std::size_t m_wr_pos = 0;
  std::size_t m_rd_pos = 0;
  std::size_t m_size = 0;

  std::aligned_storage_t<sizeof (T) * N, alignof (T)> m_buffer;

  T* buffer (std::size_t i = 0) { return ((T*)&m_buffer) + i; }
  const T* buffer (std::size_t i) const { return ((const T*)&m_buffer) + i; }
};

} // namespace utils
#endif // includeguard_utils_ring_buffer_hpp_includeguard

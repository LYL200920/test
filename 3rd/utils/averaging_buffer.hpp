#ifndef includeguard_utils_averaging_buffer_hpp_includeguard
#define includeguard_utils_averaging_buffer_hpp_includeguard

#include <array>
#include <type_traits>

namespace utils
{

template <typename T, std::size_t N, typename SumT = T> class averaging_buffer
{
public:
  static_assert (std::is_trivial<T>::value, "");

  typedef T value_type;

  // FIXME: allow specifying the size type or choose it dynamically
  // based on N
  typedef std::size_t size_type;

  typedef value_type& reference;
  typedef const value_type& const_reference;
  typedef value_type* pointer;
  typedef const value_type* const_pointer;

  bool empty (void) const { return m_size == 0; }

  // returns the number of elements in the buffer
  size_type size (void) const { return m_size; }

  // returns the maximum number of elements that can be in the buffer
  constexpr size_type capacity (void) const { return N; }
  constexpr size_type max_size (void) const { return N; }

  // clear the buffer
  // FIXME: this will not properly destroy the elements, just reset the
  // index variables.
  // should actually destroy the elements that are in the buffer.
  void clear (void)
  {
    m_wr_pos = m_size = 0;
    m_sum = m_avg = 0;
  }

  // fill the buffer with the specified value.
  template <typename TT> void fill (TT&& val)
  {
    m_sum = 0;

    for (auto& i : m_buffer)
    {
      i = std::forward<T> (val);
      m_sum += val;
    }

    m_wr_pos = 0;
    m_size = N;
    m_avg = m_sum / N;
  }

  template <typename... Args> void push_back (Args&&... args)
  {
    // if the buffer is full, replace the oldest element.  else add a new one.
    // update the sum and averaged result.

    auto prev_val = m_buffer[m_wr_pos];

    m_buffer[m_wr_pos] = { std::forward<Args> (args)... };
    m_sum += m_buffer[m_wr_pos];

    ++m_wr_pos;
    if (m_wr_pos == N)
      m_wr_pos = 0;

    if (m_size == N)
    {
      m_sum -= prev_val;
      m_avg = m_sum / N;
    }
    else
    {
      ++m_size;
      m_avg = m_sum / m_size;
    }
  }

  const T& value (void) const { return m_avg; }

private:
  std::size_t m_wr_pos = 0;
  std::size_t m_size = 0;
  std::array<T, N> m_buffer;
  SumT m_sum = 0;
  T m_avg = 0;
};

} // namespace utils
#endif // includeguard_utils_averaging_buffer_hpp_includeguard

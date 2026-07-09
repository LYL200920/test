
#ifndef includeguard_utils_iterator_hpp_includeguard
#define includeguard_utils_iterator_hpp_includeguard

#include <iterator>
#include <type_traits>

namespace utils
{

template<typename T, typename = void>
struct is_iterator
{
 static constexpr bool value = false;
};

template<typename T>
struct is_iterator<T, typename std::enable_if<!std::is_same<typename std::iterator_traits<T>::value_type, void>::value>::type>
{
 static constexpr bool value = true;
};

// -------------------------------------------------------------------
// like back_inserter but uses a begin-end range.  once end is reached,
// elements will be silently dropped.

template <class Iter>
class overwrite_range_iterator
{
public:
  using iterator_category = std::output_iterator_tag;
  using value_type = typename std::iterator_traits<Iter>::value_type;
  using difference_type = void;
  using pointer = typename std::iterator_traits<Iter>::pointer;
  using reference = typename std::iterator_traits<Iter>::pointer;

  overwrite_range_iterator (Iter begin_iter, Iter end_iter)
  : m_begin_iter (begin_iter), m_end_iter (end_iter) { }

  overwrite_range_iterator& operator = (const value_type& val)
  {
    if (m_begin_iter != m_end_iter)
      *m_begin_iter = val;

    return *this;
  }

  overwrite_range_iterator& operator = (value_type&& val)
  {
    if (m_begin_iter != m_end_iter)
      *m_begin_iter = std::move (val);

    return *this;
  }

  overwrite_range_iterator& operator* (void)
  {
    return *this;
  }

  overwrite_range_iterator& operator++ (void)
  {
    if (m_begin_iter != m_end_iter)
      ++m_begin_iter;
    return *this;
  }

  overwrite_range_iterator& operator++ (int)
  {
    if (m_begin_iter != m_end_iter)
      ++m_begin_iter;
    return *this;
  }

private:
  Iter m_begin_iter;
  Iter m_end_iter;
};

template <class Iter> inline
overwrite_range_iterator<Iter> overwrite_range (Iter ibegin, Iter iend)
{
  return { ibegin, iend };
}

}
#endif // includeguard_utils_iterator_hpp_includeguard

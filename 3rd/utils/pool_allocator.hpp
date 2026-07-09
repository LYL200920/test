#ifndef includeguard_utils_pool_allocator_hpp_includeguard
#define includeguard_utils_pool_allocator_hpp_includeguard

#include <memory>
#include <type_traits>
#include <utils/static_vector.hpp>
#include <new>
#include <cstdlib>

namespace utils
{

template <typename T, std::size_t N>
class static_pool_allocator
{
  static_assert (N > 0, "");

public:
  using value_type = T;
  using size_type = std::size_t;

  template< class U > struct rebind { typedef static_pool_allocator<U, N> other; };

  static_pool_allocator (const static_pool_allocator&) = delete;
  static_pool_allocator (static_pool_allocator&&) = delete;

  static_pool_allocator (void)
  {
    for (std::size_t i = 0; i < N; ++i)
      m_freelist.push_back (buffer (i));
  }
  
  // limit n to 1
  // pool allocator can't guarantee allocation of contiguous memory block.
  T* allocate (size_type n)
  {
    if (n == 0)
      return nullptr;

    if (n > 1 || m_freelist.empty ())
    {
      #ifdef __cpp_exceptions
	throw std::bad_alloc ();
      #else
	std::abort ();
      #endif
    }

    T* r = m_freelist.back ();
    m_freelist.pop_back ();
    return r;
  }

  void deallocate (T* ptr, std::size_t n)
  {
    if (n == 0)
      return;

    if (n > 1 || m_freelist.size () == N)
    {
      #ifdef __cpp_exceptions
	throw std::bad_alloc ();
      #else
	std::abort ();
      #endif
    }

    m_freelist.push_back (ptr);
  }

  bool operator == (const static_pool_allocator& other) const
  {
    return this == &other;
  }

  bool operator != (const static_pool_allocator& other) const
  {
    return this != &other;
  }

  std::size_t allocated_count (void) const { return N - m_freelist.size (); }
  std::size_t free_count (void) const { return m_freelist.size (); }

  constexpr std::size_t max_size (void) const { return N; }

private:
  utils::static_vector<T*, N> m_freelist;

  std::aligned_storage_t<sizeof (T) * N, alignof (T)> m_buffer;

  T* buffer (std::size_t i = 0) { return ((T*)&m_buffer) + i; }
  const T* buffer (std::size_t i) const { return ((const T*)&m_buffer) + i; }
};


} // namespace utils
#endif // includeguard_utils_pool_allocator_hpp_includeguard

#ifndef includeguard_utils_allocator_hpp_includeguard
#define includeguard_utils_allocator_hpp_includeguard

#include <memory>

namespace utils
{

// -----------------------------------------------------------------------
// for using a reference to an allocator across multiple containers
// and to satisfy CopyConstructible requirement.

template <typename Allocator> class allocator_reference
{
public:
  using value_type = typename Allocator::value_type;

  template <typename T> constexpr allocator_reference (T& ref) : m_ref (&ref) { }
  template <typename T> constexpr allocator_reference (T* ref) : m_ref (ref) { }
  constexpr allocator_reference (const allocator_reference& other) : m_ref (other.m_ref) { }
  constexpr allocator_reference (allocator_reference&& other) : m_ref (std::move (other.m_ref)) { }

  value_type* allocate (std::size_t n) { return m_ref->allocate (n); }
  void deallocate (value_type* p, std::size_t n) { return m_ref->deallocate (p, n); }
  
private:
  Allocator* m_ref;
};

// -----------------------------------------------------------------------
// for use with std::unique_ptr

template <typename Allocator> class allocator_delete
{
public:
  allocator_delete (void) = delete;
  constexpr allocator_delete (Allocator* allocator) : m_allocator (allocator) { }
  constexpr allocator_delete (Allocator& allocator) : m_allocator (&allocator) { }

  template <typename T> void operator () (T* ptr)
  {
    // Allocator must be CopyConstructible as per standards requirements.
    // libstdc++ allocator_traits implementation breaks if that is not
    // the case.
    // to play along, we use a level of indirection here, which will be
    // eliminated by the compiler.
    allocator_reference<Allocator> a (m_allocator);
    std::allocator_traits<decltype (a)>::destroy (a, ptr);
    std::allocator_traits<decltype (a)>::deallocate (a, ptr, 1);
  }

private:
  Allocator* m_allocator;
};


} // namespace utils
#endif // includeguard_utils_allocator_hpp_includeguard

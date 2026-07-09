/*

smart pointer for intrusive reference counting.

*/

#ifndef includeguard_utils_ref_counting_ptr_hpp_includeguard
#define includeguard_utils_ref_counting_ptr_hpp_includeguard

#include <cstddef>
#include <functional>
#include <memory>
#include <algorithm>

namespace utils
{

namespace impl
{
template <typename T> struct ref_counting_ptr_helper;
}

template <class T> class ref_counting_ptr
{
public:
  typedef T element_type;

  constexpr ref_counting_ptr (void) noexcept : m_ptr (nullptr) { }
  constexpr ref_counting_ptr (std::nullptr_t) noexcept : ref_counting_ptr () { }

  template <class Y> explicit ref_counting_ptr (Y* p) : m_ptr (p)
  {
    retain (p);
  }

//  template<class Y, class D> shared_ptr(Y* p, D d);
//  template<class Y, class D, class A> shared_ptr(Y* p, D d, A a);
//  template <class D> shared_ptr(std::nullptr_t p, D d);
//  template <class D, class A> shared_ptr(std::nullptr_t p, D d, A a);
//  template<class Y> shared_ptr(const shared_ptr<Y>& r, T* p) noexcept;

  ref_counting_ptr (const ref_counting_ptr& r) noexcept : m_ptr (r.m_ptr)
  {
    retain (m_ptr);
  }

  template<class Y> ref_counting_ptr (const ref_counting_ptr<Y>& r) noexcept
  : m_ptr (r.get ())
  {
    retain (m_ptr);
  }

  ref_counting_ptr (ref_counting_ptr&& r) noexcept
  : m_ptr (r.m_ptr)
  {
    r.m_ptr = nullptr;
  }

  template<class Y> ref_counting_ptr (ref_counting_ptr<Y>&& r) noexcept
  : m_ptr (r.get ())
  {
    impl::ref_counting_ptr_helper<Y>::set_null (r);
  }

//  template<class Y> explicit ref_counting_ptr(const weak_ptr<Y>& r);
//  template <class Y, class D> shared_ptr(unique_ptr<Y, D>&& r);


  ~ref_counting_ptr (void) noexcept (noexcept (release ( (T*)nullptr)))
  {
    release (m_ptr);
  }


  ref_counting_ptr& operator = (const ref_counting_ptr& r) noexcept (noexcept (release ( (T*)nullptr)))
  {
    retain (r.m_ptr);
    release (m_ptr);
    m_ptr = r.m_ptr;
    return *this;
  }

  template<class Y> ref_counting_ptr& operator = (const ref_counting_ptr<Y>& r) noexcept (noexcept (release ( (T*)nullptr)))
  {
    retain (r.get ());
    release (m_ptr);
    m_ptr = r.get ();
    return *this;
  }

  ref_counting_ptr& operator = (ref_counting_ptr&& r) noexcept (noexcept (release ( (T*)nullptr)))
  {
    release (m_ptr);
    m_ptr = r.m_ptr;
    r.m_ptr = nullptr;
    return *this;
  }

  template<class Y> ref_counting_ptr& operator = (ref_counting_ptr<Y>&& r) noexcept (noexcept (release ( (T*)nullptr)))
  {
    release (m_ptr);
    m_ptr = r.get ();
    impl::ref_counting_ptr_helper<Y>::set_null (r);
    return *this;
  }

  //template <class Y, class D> shared_ptr& operator=(unique_ptr<Y, D>&& r);

  void swap (ref_counting_ptr& r) noexcept
  {
    std::swap (m_ptr, r.m_ptr);
  }

  void reset (void) noexcept (noexcept (release ( (T*)nullptr)))
  {
    release (m_ptr);
    m_ptr = nullptr;
  }

  template<class Y> void reset (Y* p)
  {
    retain (p);
    release (m_ptr);
    m_ptr = p;
  }

  // template<class Y, class D> void reset(Y* p, D d);
  // template<class Y, class D, class A> void reset(Y* p, D d, A a);

  T* get () const noexcept { return m_ptr; }
  T& operator*() const noexcept { return *m_ptr; }
  T* operator->() const noexcept { return m_ptr; }

  long use_count() const noexcept { return retain_count (m_ptr); }
  bool unique() const noexcept { return retain_count (m_ptr) == 1; }

  explicit operator bool() const noexcept { return m_ptr != nullptr; }

  // template<class U> bool owner_before(shared_ptr<U> const& b) const;
  // template<class U> bool owner_before(weak_ptr<U> const& b) const;

private:
/*
  template <typename Y> friend inline void set_null (ref_counting_ptr<Y>& y)
  {
    y.m_ptr = nullptr;
  }
*/

  T* m_ptr;

  template <typename Y> friend struct impl::ref_counting_ptr_helper;
};

namespace impl
{

template <typename T> struct ref_counting_ptr_helper
{
  static void set_null (ref_counting_ptr<T>& p)
  {
    p.m_ptr = nullptr;
  }
};
}



template<class T, class... Args> inline ref_counting_ptr<T>
make_ref_counted (Args&&... args)
{
  return ref_counting_ptr<T> (new T (std::forward<Args> (args)...));
}

/*
template<class T, class Alloc, class... Args> inline ref_counting_ptr<T>
allocate_ref_counted (const Alloc& alloc, Args&&... args)
{
}
*/


template<class T, class U> inline bool
operator == (ref_counting_ptr<T> const& a, ref_counting_ptr<U> const& b) noexcept
{
  return a.get () == b.get ();
}

template<class T, class U> inline bool
operator != (ref_counting_ptr<T> const& a, ref_counting_ptr<U> const& b) noexcept
{
  return !(a.get () == b.get ());
}

template<class T, class U> inline bool
operator < (ref_counting_ptr<T> const& a, ref_counting_ptr<U> const& b) noexcept
{
  return std::less<const void*> () ((const void*)a.get (), (const void*)b.get ());
}

template<class T, class U> inline bool
operator > (ref_counting_ptr<T> const& a, ref_counting_ptr<U> const& b) noexcept
{
  return b.get () < a.get ();
}

template<class T, class U> inline bool
operator <= (ref_counting_ptr<T> const& a, ref_counting_ptr<U> const& b) noexcept
{
  return !(b.get () < a.get ());
}

template<class T, class U> inline bool
operator >= (ref_counting_ptr<T> const& a, ref_counting_ptr<U> const& b) noexcept
{
  return !(a.get () < b.get ());
}

template<class T> inline bool
operator == (const ref_counting_ptr<T>& x, std::nullptr_t) noexcept
{
  return !x;
}

template <class T> inline bool
operator == (std::nullptr_t, const ref_counting_ptr<T>& y) noexcept
{
  return !y;
}

template <class T> inline bool
operator != (const ref_counting_ptr<T>& x, std::nullptr_t) noexcept
{
  return (bool)x;
}

template <class T> inline bool
operator != (std::nullptr_t, const ref_counting_ptr<T>& y) noexcept
{
  return (bool)y;
}

template <class T> inline bool
operator < (const ref_counting_ptr<T>& x, std::nullptr_t) noexcept
{
  return std::less<T*> () (x.get (), nullptr);
}

template <class T> inline bool 
operator < (std::nullptr_t, const ref_counting_ptr<T>& y) noexcept
{
  return std::less<T*> () (nullptr, y.get ());
}

template <class T> inline bool
operator <= (const ref_counting_ptr<T>& x, std::nullptr_t) noexcept
{
  return nullptr < x;
}

template <class T> inline bool
operator <= (std::nullptr_t, const ref_counting_ptr<T>& y) noexcept
{
  return y < nullptr;
}

template <class T> inline bool
operator > (const ref_counting_ptr<T>& x, std::nullptr_t) noexcept
{
  return !(nullptr < x);
}

template <class T> inline bool
operator > (std::nullptr_t, const ref_counting_ptr<T>& y) noexcept
{
  return !(y < nullptr);
}

template <class T> inline bool
operator >= (const ref_counting_ptr<T>& x, std::nullptr_t) noexcept
{
  return !(x < nullptr);
}

template <class T> inline bool
operator >= (std::nullptr_t, const ref_counting_ptr<T>& y) noexcept
{
  return !(nullptr < y);
}

namespace is_ref_counting_ptr_impl
{
template <typename T, typename = void> struct check
{
  static constexpr bool value = false;
};

template <typename T> struct check <ref_counting_ptr<T>>
{
  static constexpr bool value = true;
};

}

template <typename T> struct is_ref_counting_ptr
{
  static constexpr bool value = is_ref_counting_ptr_impl::check <
	typename std::remove_cv <
	  typename std::remove_reference<T>::type>::type >::value;
};

} // namespace utils


namespace std
{

template<class T> inline void
swap (utils::ref_counting_ptr<T>& lhs, utils::ref_counting_ptr<T>& rhs)
{
  lhs.swap (rhs);
}

template<class T, class U> inline utils::ref_counting_ptr<T>
static_pointer_cast (utils::ref_counting_ptr<U> const & p) noexcept
{
  return utils::ref_counting_ptr<T> (static_cast<T*> (p.get ()));
}

template<class T, class U> inline utils::ref_counting_ptr<T>
const_pointer_cast (utils::ref_counting_ptr<U> const & p) noexcept
{
  return utils::ref_counting_ptr<T> (const_cast<T*> (p.get ()));
}

template<class T, class U> inline utils::ref_counting_ptr<T>
dynamic_pointer_cast (utils::ref_counting_ptr<U> const & p) noexcept
{
  return utils::ref_counting_ptr<T> (dynamic_cast<T*> (p.get()));
}

} // namespace std


#endif  // includeguard_utils_ref_counting_ptr_hpp_includeguard

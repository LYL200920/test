/*

some utilities for dealing with pointer-to-member functions and data fields.

*/


#ifndef includeguard_utils_ptm_hpp_includeguard
#define includeguard_utils_ptm_hpp_includeguard

#include <type_traits>
#include <tuple>

namespace utils
{

// ----------------------------------------------------------------------------
// get the class type of a pointer to member

template <typename T> struct ptm_class { };

template <typename T, typename U> struct ptm_class<T U::*>
{
  typedef U type;
};

template <typename T, typename U> struct ptm_class<T U::* const>
{
  typedef U type;
};

// ----------------------------------------------------------------------------
// get the return value or value type of a pointer to member.
// if the pointer to member is a function, it will be the function return value
// type.  if the pointer to member is a data field, it will be the type of
// the data field.

template <typename T> struct ptm_result { };

template <typename R, typename U, typename... Args> struct ptm_result<R (U::*)(Args...)>
{
  typedef R type;
};

template <typename R, typename U, typename... Args> struct ptm_result<R (U::*)(Args...) const>
{
  typedef R type;
};

// ----------------------------------------------------------------------------
// get the arguments of a pointer to member function as a std::tuple.

template <typename T> struct ptm_args { };

template <typename R, typename U, typename... Args> struct ptm_args<R (U::*)(Args...)>
{
  typedef std::tuple<Args...> args_tuple;
  constexpr static std::size_t size = sizeof... (Args);
};

template <typename R, typename U, typename... Args> struct ptm_args<R (U::*)(Args...) const>
{
  typedef std::tuple<Args...> args_tuple;
  constexpr static std::size_t size = sizeof... (Args);
};

// ----------------------------------------------------------------------------
// an adapter to use a pointer to member function with an object pointer
// as a callback function, where normally the callback function would consist
// of a regular function + void* user parameter.
//
// the key point of this thing here is that the pointer to member function
// pointer object is actually not stored as data anywhere, but used as a
// template parameter.  this allows the compiler to directly invoke the member
// function and also perform inlining etc.
//
// unfortunately it's not possible to do that with a template function, as
// it would capture the function pointer itself as an function argument, which
// then can't be used as a template argument.  so the way to use this thing
// here is:
//   utils::ptm_clb<decltype (&my_class::my_func), &my_class::my_func>
//

template <typename Func, Func func> struct ptm_clb
{
  typedef typename ptm_class<Func>::type class_type;

  template <typename... Args>

#if defined (__GNUC__)
  [[gnu::flatten]]
#elif defined (_MSC_VER)
  [[msvc::flatten]]
#endif
  static void invoke (void* p, Args... args)
  {
    (((class_type*)p)->*func) (std::forward<Args> (args)...);
  }
};


} // namespace utils
#endif // includeguard_utils_ptm_hpp_includeguard

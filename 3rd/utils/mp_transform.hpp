#ifndef includeguard_utils_mp_transform_hpp_includeguard
#define includeguard_utils_mp_transform_hpp_includeguard

namespace utils
{

// http://pdimov.com/cpp2/simple_cxx11_metaprogramming.html

template<template<class...> class F, class L> struct mp_transform_impl;

template<template<class...> class F, class L>
using mp_transform = typename mp_transform_impl<F, L>::type;

template<template<class...> class F, template<class...> class L, class... T>
struct mp_transform_impl<F, L<T...>>
{
  using type = L< typename F<T>::type...>;
};

} // namespace utils
#endif // includeguard_utils_mp_transform_hpp_includeguard

#ifndef includeguard_utils_tuple_hpp_includeguard
#define includeguard_utils_tuple_hpp_includeguard

#include <tuple>
#include <utility>
#include <type_traits>
#include <functional>


// -----------------------------------------------------------------------
// get the size of a tuple variable.

namespace utils
{

template <typename Tuple>
inline constexpr std::size_t tuple_size (Tuple&& t)
{
  return std::tuple_size_v < std::remove_reference_t<decltype (t)> >;
}

} // namespace utils

// -----------------------------------------------------------------------
// tuple_for_each
// tuple_for_each_n

// iterate over all elements of a tuple and invoke the specified function.
// the function is passed the index as well as the tuple element.
//
//  auto xx = std::make_tuple (0, 1, 2);
//
//  tuple_for_each (xx, [] (std::size_t ii, auto&& x)
//  {
//    std::cout << ii << ": " << x.name () << std::endl;
//  });
//
// in addition, if the specified function returns a loop control
// value, the iteration loop can be terminated early.
//
// tuple_for_each (xx, [] (std::size_t ii, auto& x)
// {
//   std::cout << ii << ": " << x.name () << std::endl;
//   if (x == 1)
//     return tuple_for_each_break;
//   else
//     return tuple_for_each_continue;
// });

namespace utils
{

enum tuple_for_each_loop_ctrl
{
  tuple_for_each_continue,
  tuple_for_each_break
};

template <typename Tuple, typename Func, std::size_t I = 0>
inline constexpr void
tuple_for_each (Tuple&& t, Func&& f)
{
  // std::invoke is constexpr in c++20, not c++17
  //std::invoke (f, I, std::get<I> (t));
  if constexpr (std::is_same_v<decltype (f (I, std::get<I> (t))), tuple_for_each_loop_ctrl>)
  {
    auto ctrl = f (I, std::get<I> (t));
    if (ctrl == tuple_for_each_break)
      return;
  }
  else
  {
    f (I, std::get<I> (t));
  }

  if constexpr (I+1 < std::tuple_size_v<std::remove_reference_t<Tuple>>)
    tuple_for_each<Tuple, Func, I+1> (std::forward<Tuple> (t), std::forward<Func> (f));
}

// iterate over a sub-range of the tuple.

template <typename Tuple, typename Func, std::size_t I = 0>
inline constexpr void
tuple_for_each_n (Tuple&& t, std::size_t i, std::size_t count, Func&& f)
{
  if (I >= i)
  {
    if constexpr (std::is_same_v<decltype (f (I, std::get<I> (t))), tuple_for_each_loop_ctrl>)
    {
      auto ctrl = f (I, std::get<I> (t));
      if (ctrl == tuple_for_each_break)
        return;
    }
    else
    {
//    std::invoke (f, I, std::get<I> (t));
      f (I, std::get<I> (t));
    }

  }

  if constexpr (I+1 < std::tuple_size_v<std::remove_reference_t<Tuple>>)
  {
    if (I+1 < i+count)
	tuple_for_each_n<Tuple, Func, I+1> (std::forward<Tuple> (t), i, count, std::forward<Func> (f));
  }
}

} // namespace utils

// -----------------------------------------------------------------------
// tuple_remove

// sometimes some undesired types need to be removed from a tuple.  for instance
// 'void', which have been inserted due to something else.
//
// the solutions are basically taken from
// http://stackoverflow.com/questions/23855712/how-can-a-type-be-removed-from-a-template-parameter-pack


// it seems the following simple solution doesn't work on GCC 4.7, it says
//    error: no matching function for call to 'declval()'
//    error: wrong number of template arguments (2, should be 1)
//
// and there's a possibly related PR for GCC 4.7
//    https://gcc.gnu.org/bugzilla/show_bug.cgi?id=51973
//

namespace utils
{

/*
template<typename...Ts>
using tuple_cat_t = decltype(std::tuple_cat(std::declval<Ts>()...));

template <typename T, typename... Ts>
struct remove
{
  using type = tuple_cat_t <
	typename std::conditional< std::is_same<T, Ts>::value,
			  std::tuple<>,
			  std::tuple<Ts>
			>::type ...>;
};

*/

// this solution works OK with GCC 4.7

namespace detail
{

template <typename T, typename Tuple, typename Res = std::tuple<>> struct removeT_helper;

// terminating specialization where it arrives when the input tuple has been
// drained completely.
template <typename T, typename Res> struct removeT_helper<T, std::tuple<>, Res>
{
  using type = Res;
};

// specialization which filters out type T if the input tuple matches
// std::tuple<T, ...>.  in this case, the T is removed as the input tuple's
// head and it proceeds with the tail via inheritace.
template<typename T, typename... Ts, typename... TRes>
struct removeT_helper < T,
		        std::tuple<T, Ts...>,   // input tuple
			std::tuple<TRes...>>    // output tuple
  : removeT_helper < T,
		     std::tuple<Ts...>,
		     std::tuple<TRes...>>
{ };

// specialization which matches not the type T in the tuple head.
// effectively it removes the input tuple head and appends it to the result
// tuple.
template<typename T, typename T1, typename ...Ts, typename... TRes>
struct removeT_helper < T,
			std::tuple<T1, Ts...>,  // input tuple
			std::tuple<TRes...>>    // output tuple
  : removeT_helper < T,
		     std::tuple<Ts...>,
		     std::tuple<TRes..., T1>>
{ };

} // namespace detail

template <typename T, typename...Ts> struct tuple_remove
{
  using type = typename detail::removeT_helper<T, std::tuple<Ts...>>::type;
};


} // namespace utils
#endif // includeguard_utils_tuple_hpp_includeguard

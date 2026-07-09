
#ifndef includeguard_utils_container_hpp_includeguard
#define includeguard_utils_container_hpp_includeguard

#include <type_traits>

namespace utils
{


namespace is_container_impl
{

template <typename T, typename = void> struct has_value_type
{
  static constexpr bool value = false;
};

template <typename T>
struct has_value_type <T, typename std::enable_if<!std::is_same<typename T::value_type, void>::value>::type>
{
  static constexpr bool value = true;
};


template <typename T> struct has_size
{
  template <typename C> static bool test_size (decltype (&C::size));
  template <typename C> static int test_size (...);

  static constexpr bool value = sizeof (test_size<T> (0)) == sizeof (bool);
};

template <typename T, typename = void> struct has_data
{
  static constexpr bool value = false;
};

template <typename T>
struct has_data<T, typename std::enable_if<!std::is_same< decltype ( ((const T*)0)->data () ), void>::value>::type>
{
  static constexpr bool value = true;
};

}

template<typename T, typename = void>
struct is_contiguous_container
{
 static constexpr bool value = false;
};

template<typename T>
struct is_contiguous_container<T, typename std::enable_if<
		is_container_impl::has_size<T>::value
		&& is_container_impl::has_data<T>::value
		&& is_container_impl::has_value_type<T>::value
	>::type>
{
 static constexpr bool value = true;
};

}
#endif // includeguard_utils_container_hpp_includeguard

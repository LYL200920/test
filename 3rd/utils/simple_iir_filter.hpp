#ifndef includeguard_utils_simple_iir_filter_hpp_includeguard
#define includeguard_utils_simple_iir_filter_hpp_includeguard

#include <type_traits>
#include <cstdint>
#include <limits>
#include <algorithm>

namespace utils
{

template <typename T, typename WidenedT, typename OneValueFunc>
class simple_iir_filter
{
public:
  using value_type = T;
  using widened_value_type = WidenedT;

  void push_back (const T& new_val)
  {
    auto new_wide_val =	(widened_value_type)m_prev_value * m_inv_k
			+ (widened_value_type)new_val * m_k;

    m_prev_value = (value_type)std::clamp (new_wide_val / one_value (),
				std::numeric_limits<value_type>::lowest (),
				std::numeric_limits<value_type>::max ());
  }

  const value_type& value (void) const { return m_prev_value; }

  static value_type k_one_value (void) { return one_value (); }

  void set_k (value_type k)
  {
    m_k = k;
    m_inv_k = one_value () - k;
  }

  void reset (value_type val)
  {
    m_prev_value = val;
  }

private:
  static value_type one_value (void) { return OneValueFunc () (); }


  value_type m_prev_value = 0;
  value_type m_k = one_value () / 2;
  value_type m_inv_k = one_value () / 2;
};

struct simple_iir_filter_one_256i
{
  unsigned int operator () (void) { return 256; }
};

struct simple_iir_filter_one_float
{
  float operator () (void) { return 1; }
};

} // namespace utils
#endif // includeguard_utils_simple_iir_filter_hpp_includeguard


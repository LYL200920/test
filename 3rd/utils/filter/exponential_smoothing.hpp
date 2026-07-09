
/*
    Exponential Smoothing Filter:

      1. Single Exponential Smoothing:
            Vt= Xt * alpha + V(t-1) * (1−a)
               - Xt     : current observation at time t.
               - Vt     : smoothed value at time t.
               - V(t-1) : smoothed value at the previous time (t-1).
               - alpha  : smoothed weight parameter (0 <= alpha <= 1).

      2. Second Exponential Smoothing:
            Vt= Xt * a + ( V(t-1) + B(t-1) ) * (1−a)
            Bt= ( Vt - V(t-1) ) * b + B(t-1) * (1−b)
               - Xt     : current observation at time t.
               - Vt     : smoothed value at time t.
               - V(t-1) : smoothed value at the previous time (t-1).
               - alpha  : smoothed weight parameter (0 <= alpha <= 1).
               - Bt     : trend value at time t.
               - B(t-1) : trend value at the previous time (t-1).
               - beta   : trend weight parameter (0 <= b <= 1).

*/

#ifndef includeguard_utils_filter_exponential_smoothing_hpp_includeguard
#define includeguard_utils_filter_exponential_smoothing_hpp_includeguard

#include <algorithm>

namespace utils
{
namespace filter
{

enum class exponential_smoothing_type
{
  single,
  second,
};

template <exponential_smoothing_type Type>
struct exponential_smoothing_params;

template <exponential_smoothing_type Type>
struct default_exponential_smoothing_params;

template < exponential_smoothing_type Type, typename InitParams = default_exponential_smoothing_params<Type>>
class exponential_smoothing;


// ---------------------------------------------------------------------------------------------------------------
// Single Exponential Smoothing

template <>
struct exponential_smoothing_params <exponential_smoothing_type::single>
{
  float m_value;
  float m_alpha;
};

template <>
struct default_exponential_smoothing_params <exponential_smoothing_type::single>
{
  exponential_smoothing_params <exponential_smoothing_type::single> operator () (void) const
  {
    return { 0.0f, 0.2f};
  }
};

template < typename InitParams >
class exponential_smoothing <exponential_smoothing_type::single, InitParams>
{
public:
  exponential_smoothing (InitParams&& params = InitParams ())
  : m_init_params (std::move (params))
  {
    m_params = m_init_params ();
  }

  void clear () { is_first_val = true; m_params = m_init_params (); }

  float value (void) const { return m_params.m_value; }

  float alpha (void) const { return m_params.m_alpha; }
  void set_alpha (const float val) { m_params.m_alpha = val; }

  void push_back (const float val)
  {
    if (is_first_val)
    {
      is_first_val = false;
      m_params.m_value = val;
      return;
    }

    // Vt= Xt * alpha + V(t-1) * (1−a)
    m_params.m_value = val * m_params.m_alpha + m_params.m_value * (1.0f - m_params.m_alpha);
  }

private:
  InitParams m_init_params;
  exponential_smoothing_params <exponential_smoothing_type::single> m_params;

  bool is_first_val = true;
};

// ---------------------------------------------------------------------------------------------------------------
// Second Exponential Smoothing

template <>
struct exponential_smoothing_params <exponential_smoothing_type::second>
{
  float m_value;
  float m_trend;
  float m_alpha;
  float m_beta;
};

template <>
struct default_exponential_smoothing_params <exponential_smoothing_type::second>
{
  exponential_smoothing_params <exponential_smoothing_type::second> operator () (void) const
  {
    return { 0.0f, 0.0f, 0.2f, 0.2f};
  }
};

template < typename InitParams >
class exponential_smoothing <exponential_smoothing_type::second, InitParams>
{
public:
  exponential_smoothing (InitParams&& params = InitParams ())
  : m_init_params (std::move (params))
  {
    m_params = m_init_params ();
  }

  void clear () { is_first_val = true; m_params = m_init_params (); }

  float value (void) const { return m_params.m_value; }

  float alpha (void) const { return m_params.m_alpha; }
  void set_alpha (const float val) { m_params.m_alpha = val; }

  float beta (void) const { return m_params.m_beta; }
  void set_beta (const float val) { m_params.m_beta = val; }

  void push_back (const float val)
  {
    if (is_first_val)
    {
      is_first_val = false;
      m_params.m_value = val;
      return;
    }

    float old_value = m_params.m_value;

    // Vt= Xt * a + ( V(t-1) + B(t-1) ) * (1−a)
    m_params.m_value = val * m_params.m_alpha + (old_value + m_params.m_trend) * (1.0f - m_params.m_alpha);
    // Bt= ( Vt - V(t-1) ) * b + B(t-1) * (1−b)
    m_params.m_trend = (m_params.m_value - old_value) * m_params.m_beta + m_params.m_trend * (1.0f - m_params.m_beta);
  }

private:
  InitParams m_init_params;
  exponential_smoothing_params <exponential_smoothing_type::second> m_params;

  bool is_first_val = true;
};

} // namespace filter

} // namespace utils


#endif // includeguard_utils_filter_exponential_smoothing_hpp_includeguard

#ifndef includeguard_utils_filter_kalman_hpp_includeguard
#define includeguard_utils_filter_kalman_hpp_includeguard

namespace utils
{
namespace filter
{

template <typename T>
struct kalman_params
{
  T x; // state, x(k) = a*x(k-1) + b*u(k)
  T a; // state transfer matrix
  T b; // control matrix (no effect for this 1 dimensional kalman filter)
  T h; // observer matrix
  T p; // estimate covariance
  T q; // process covariance
  T r; // observe covariance
  T g; // kalman gain
};


template <typename T>
struct default_kalman_params
{
  kalman_params<T> operator () (void) const
  {
    return { 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.002f, 0.05f, 0.0f};
  }
};


template <typename ValueType = float, typename InitParams = default_kalman_params<ValueType>>
class kalman
{
public:

  kalman (InitParams&& params = InitParams ())
  : m_init_params (std::move (params))
  {
    m_params = m_init_params ();  // use the InitParams struct as a functor
  }

  void clear () { is_first_val = true; m_params = m_init_params (); }

  ValueType value (void) const { return m_params.x; }

  void push_back (const float val)
  {
    if (is_first_val)
    {
      is_first_val = false;
      m_params.x = val;
      return;
    }

    /* predict, time update */
    m_params.x = m_params.a * m_params.x + m_params.b * val;
    m_params.p = m_params.a * m_params.p * m_params.a + m_params.q;

    /* correct, measurement update */
    m_params.g = m_params.p * m_params.h / (m_params.h * m_params.p * m_params.h + m_params.r);
    m_params.x = m_params.x + m_params.g * (val - m_params.h * m_params.x);
    m_params.p = (1 - m_params.g * m_params.h) * m_params.p;
  }

private:
  InitParams m_init_params;
  kalman_params<ValueType> m_params;

  bool is_first_val = true;
};

} // namespace filter

} // namespace utils


#endif // includeguard_utils_filter_kalman_hpp_includeguard
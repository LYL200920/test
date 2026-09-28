/*

a reference wrapper like a std::reference_wrapper, but provides access to
the variable via operator ().  so like std::mem_fn, but for variables.

std::reference_wrapper also has a operator () but it's available only for
refs to functions.
*/

#ifndef includeguard_utils_var_fn_hpp_includeguard
#define includeguard_utils_var_fn_hpp_includeguard

#include <functional>

namespace utils
{

template <typename T> class var_fn
{
public:
  var_fn (T& x) noexcept : m_ref (x) { }
  var_fn (T&& x) noexcept = delete;
  var_fn (const var_fn& rhs) noexcept : m_ref (rhs.m_ref) { }

  var_fn& operator = (const var_fn& rhs) noexcept
  {
    m_ref = rhs.m_ref;
    return *this;
  }

  T& operator () (void) const { return m_ref.get (); }
  void operator () (T& newval) { m_ref.get () = newval; }

private:
  std::reference_wrapper<T> m_ref;
};

template <typename T> inline var_fn<T> make_var_fn (T& v) { return { v }; }
template <typename T> inline var_fn<T> make_var_fn (T&& v) { return { std::move (v) }; }

} // namespace utils
#endif // includeguard_utils_var_fn_hpp_includeguard

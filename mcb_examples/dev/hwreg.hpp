/*

Example usage with register struct/block (with hw_reg):

  struct lots_of_regs
  {
    hw_reg_r<int32_t> first_reg;	// read-only (compiler checked)
    hw_reg_w<int32_t> second_reg;	// write-only (compiler checked)
    hw_reg_rw<int32_t> super_reg;	// read-write (compiler checked)
  };

Example usage with a constant address register:

  static constexpr hw_reg_rw<int32_t, const_addr<0xFF000020>> SH4_TRA;

*/

#ifndef includeguard_dev_hwreg_includeguard
#define includeguard_dev_hwreg_includeguard

#include <cstdint>
#include <type_traits>

namespace dev
{

// compile-time constant address type
// similar to std::integral_constant
template <uintptr_t ADDR> struct const_addr
{
  static constexpr uintptr_t value = ADDR;
  typedef uintptr_t value_type;
  typedef const_addr type;
  constexpr operator value_type () const { return value; }
};

// hwreg variable with some unsupported address type
template <typename T, typename A> struct hw_reg_var
{
  static constexpr bool is_valid = false;

  // provide address_of to avoid additional errors when instantiating this template.
  // the main error source will be the static_assert.
  const void* address_of (void) const noexcept { return nullptr; }
  void* address_of (void) noexcept { return nullptr; }
};

// hwreg variable without specified address
// i.e. used as a member of a hw register block struct
template <typename T> struct hw_reg_var<T, void>
{
  static constexpr bool is_valid = true;

  volatile T m_var;
		
  T read (void) const { return m_var; }
  void write (const T& val) { m_var = val;  }
		
  const T* address_of (void) const { return const_cast<const T*> (&m_var); }
  T* address_of (void) { return const_cast<T*> (&m_var); }
};

// hwreg variable with specified constant address
template <typename T, uintptr_t A> struct hw_reg_var<T, const_addr<A>>
{
  static constexpr bool is_valid = true;

  typedef volatile T var_type;
  typedef var_type* ptr_type;
		
  const ptr_type address_of (void) const noexcept { return reinterpret_cast<const ptr_type> (A); }
  ptr_type address_of (void) noexcept { return reinterpret_cast<ptr_type> (A); }

  T read (void) const { return *(reinterpret_cast<ptr_type> (A)); }
  void write (const T& val) const { *(reinterpret_cast<ptr_type> (A)) = val; }
};


template <typename T, bool R, bool W, typename A> class hw_reg
{
private:
  static const std::integral_constant<bool, R> can_read;
  static const std::integral_constant<bool, W> can_write;

  mutable hw_reg_var<T,A> m_var;

  T read (std::true_type) const { return m_var.read (); }
  const hw_reg& write (const T& val, std::true_type) const { m_var.write (val); return *this; }

  static_assert (hw_reg_var<T,A>::is_valid, "constant hw_reg address must be of type const_addr");

public:
  typedef T base_type;

  hw_reg (void) noexcept = default;
  hw_reg (const hw_reg&) = delete;
  ~hw_reg (void) noexcept = default;
  hw_reg& operator = (const hw_reg&) = delete;

  // address of 
  auto operator & (void) const noexcept -> decltype (m_var.address_of ()) { return m_var.address_of (); }
  auto operator & (void) noexcept -> decltype (m_var.address_of ()) { return m_var.address_of (); }

  // read-only
  T read (void) const { return read (can_read); }
  operator T (void) const { return read (can_read); }
			
  // write-only
  const hw_reg& write (const T& val) const { write (val, can_write); return *this; }
  const hw_reg& operator = (const T& val) const { return write (val, can_write); }

  // read-write
  const hw_reg& operator += (const T& val) const { return write (read (can_read) + val, can_write); }
  const hw_reg& operator -= (const T& val) const { return write (read (can_read) - val, can_write); }
  const hw_reg& operator *= (const T& val) const { return write (read (can_read) * val, can_write); }
  const hw_reg& operator /= (const T& val) const { return write (read (can_read) / val, can_write); }
  const hw_reg& operator %= (const T& val) const { return write (read (can_read) % val, can_write); }
  const hw_reg& operator ^= (const T& val) const { return write (read (can_read) ^ val, can_write); }
  const hw_reg& operator &= (const T& val) const { return write (read (can_read) & val, can_write); }
  const hw_reg& operator |= (const T& val) const { return write (read (can_read) | val, can_write); }
  const hw_reg& operator >>= (const T& val) const { return write (read (can_read) >> val, can_write); }
  const hw_reg& operator <<= (const T& val) const { return write (read (can_read) << val, can_write); }
};

template<typename T, typename A = void> using hw_reg_w = hw_reg<T,false,true,A>;
template<typename T, typename A = void> using hw_reg_r = hw_reg<T,true,false,A>;
template<typename T, typename A = void> using hw_reg_rw = hw_reg<T,true,true,A>;

} // namespace dev
#endif // includeguard_dev_hwreg_includeguard


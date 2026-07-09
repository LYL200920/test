
#ifndef includeguard_utils_bits_includeguard
#define includeguard_utils_bits_includeguard

#include <cstdint>
#include <bitset>
#include <limits>
#include <atomic>

namespace utils
{

inline constexpr bool is_pow2 (unsigned int val) noexcept
{
  return (val & (val - 1)) == 0;
}

// assuming that value is a power-of-2 (single bit) value, get the log2,
// i.e. the bit position.
// https://graphics.stanford.edu/~seander/bithacks.html#IntegerLog

#if __cplusplus >= 201402L
inline constexpr unsigned int pow2_log2 (uint32_t val) noexcept
#else
inline unsigned int pow2_log2 (uint32_t val) noexcept
#endif
{
  unsigned int r = (val & 0xAAAAAAAA) != 0;
  r |= ((val & 0xFFFF0000) != 0) << 4;
  r |= ((val & 0xFF00FF00) != 0) << 3;
  r |= ((val & 0xF0F0F0F0) != 0) << 2;
  r |= ((val & 0xCCCCCCCC) != 0) << 1;

  return r;
}

// round up the specified number to the next higher power-of-two number.
#if __cplusplus >= 201402L
inline constexpr uint32_t ceil_pow2 (uint32_t val) noexcept
#else
inline uint32_t ceil_pow2 (uint32_t val) noexcept
#endif
{
  // for details see
  // http://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
  val--;
  val = (val >> 1) | val;
  val = (val >> 2) | val;
  val = (val >> 4) | val;
  val = (val >> 8) | val;
  val = (val >> 16) | val;
  val++;
  return val;
}

// round up the specified number "val" to the next higher power-of-two number "pow2".
#if __cplusplus >= 201402L
template <typename T> inline constexpr T ceil_pow2 (T val, T pow2) noexcept
#else
template <typename T> inline T ceil_pow2 (T val, T pow2) noexcept
#endif
{
  return (val + pow2-1) & ~(pow2-1);
}

// round down the specified number to the next lower power-of-two number.
#if __cplusplus >= 201402L
inline constexpr uint32_t floor_pow2 (uint32_t val) noexcept
#else
inline uint32_t floor_pow2 (uint32_t val) noexcept
#endif
{
  // from http://stackoverflow.com/questions/2679815/previous-power-of-2
  if (0 == val)
    return 0;

  val |= (val >> 1);
  val |= (val >> 2);
  val |= (val >> 4);
  val |= (val >> 8);
  val |= (val >> 16);
  return val - (val >> 1);
}

// round down the specified number to the next lower power-of-two number "pow2".
#if __cplusplus >= 201402L
template <typename T> inline constexpr T floor_pow2 (T val, T pow2) noexcept
#else
template <typename T> inline T floor_pow2 (T val, T pow2) noexcept
#endif
{
  return val & ~(pow2-1);
}

template <typename T> inline constexpr T*
ceil_ptr (T* p, uintptr_t val) noexcept
{
  return (T*)(((uintptr_t)p + (val - 1)) & ~(val - 1));
}

template <typename T> inline constexpr bool
get_bit (T x, unsigned int n) noexcept
{
  return x & (1 << n);
}

template <std::size_t N> inline constexpr bool
get_bit (const std::bitset<N>& b, unsigned int n) noexcept
{
  return b[n];
}

template <typename T> inline constexpr T
set_bit (T x, unsigned int n, bool val = true) noexcept
{
  return (x & ~(1 << n)) | (val << n);
}

template <std::size_t N> inline constexpr
typename std::enable_if< (N <= std::numeric_limits<unsigned long>::digits),
			 std::bitset<N>>::type
set_bit (std::bitset<N> x, unsigned int n, bool val = true) noexcept
{
  auto xx = x.to_ulong ();
  return { (xx & ~((decltype (xx)(1)) << n)) | (decltype (xx)(val) << n) };
}

template <std::size_t N> inline constexpr
typename std::enable_if< (N > std::numeric_limits<unsigned long>::digits)
			 && (N <= std::numeric_limits<unsigned long long>::digits),
			 std::bitset<N>>::type
set_bit (std::bitset<N> x, unsigned int n, bool val = true) noexcept
{
  auto xx = x.to_ullong ();
  return { (xx & ~((decltype (xx)(1)) << n)) | (decltype (xx)(val) << n) };
}

template <std::size_t N> inline constexpr
typename std::enable_if< (N > std::numeric_limits<unsigned long long>::digits),
			std::bitset<N>>::type
set_bit (std::bitset<N> x, unsigned int n, bool val = true) noexcept
{
  x[n] = val;
  return x;
}


template <typename T> inline constexpr T
clear_bit (T x, unsigned int n) noexcept
{
  return set_bit (x, n, false);
}

// take 8 bits from the bitset at position 'i' and return the reversed bits.
template <std::size_t N>
inline uint8_t rev_8bits (const std::bitset<N>& bits, std::size_t i) noexcept
{
  return (bits[i + 0] << 7) | (bits[i + 1] << 6) | (bits[i + 2] << 5)
         | (bits[i + 3] << 4) | (bits[i + 4] << 3) | (bits[i + 5] << 2)
	 | (bits[i + 6] << 1) | (bits[i + 7] << 0);
}

inline constexpr uint8_t rev_8bits (uint8_t b) noexcept
{
  return (((b >> 0) & 1) << 7) | (((b >> 1) & 1) << 6) | (((b >> 2) & 1) << 5)
 	 | (((b >> 3) & 1) << 4) | (((b >> 4) & 1) << 3) | (((b >> 5) & 1) << 2)
	 | (((b >> 6) & 1) << 1) | (((b >> 7) & 1) << 0);
}

// return the reversed lower n bits of val.
template < unsigned int N, typename T >
inline constexpr T reverse_bits (T val) noexcept
{
  T out = 0;

  for (unsigned int i = 0; i < N; ++i)
    if (val & (T (1) << i))
      out |= T (1) << (N - i - 1);

  return out;
}

// merge bits A with B according the mask M.
// for each bit where M=0 the bit from A is taken.
// for each bit where M=1 the bit from B is taken.
// T can be some integer type or std::bitset
// see also https://graphics.stanford.edu/~seander/bithacks.html#MaskedMerge
template < typename T > inline T
merge_bits (const T& a, const T& b, const T& m)
{
  return a ^ ((a ^ b) & m);
}

#ifdef _MSC_VER
inline void __builtin_prefetch (const void*) { }
#endif

template <typename T> static inline void prefetch (const T* p)
{
  for (unsigned int i = 0; i < std::max ((size_t)1, sizeof (T) / 32); ++i)
    __builtin_prefetch ((char*)p + i*32);
}

// count odd bits only.
// result is in the most significant byte, lower bytes contain garbage
// see also https://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
inline constexpr uint32_t popcount_odd_shl24 (uint32_t v)
{
  v = ((v >> 0) & 0x11111111) + ((v >> 2) & 0x11111111);
  return (((v + (v >> 4)) & 0xF0F0F0F) * 0x01010101);
}

// count even bits only
// result is in the most significant byte, lower bytes contain garbage
// see also https://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
inline constexpr uint32_t popcount_even_shl24 (uint32_t v)
{
  v = ((v >> 1) & 0x11111111) + ((v >> 3) & 0x11111111);
  return (((v + (v >> 4)) & 0xF0F0F0F) * 0x01010101);
}


//-----------------------------------------------------------------------------
// bit permutation functions
// http://programming.sirrida.de/calcperm.php
// http://programming.sirrida.de/perm_fn.html#bit_permute_step

inline constexpr uint32_t bit_permute_step (uint32_t x, uint32_t m, unsigned int shift)
{
  uint32_t t = ((x >> shift) ^ x) & m;
  return (x ^ t) ^ (t << shift);
}

inline constexpr uint32_t bit_permute_step_simple (uint32_t x, uint32_t m, unsigned int shift)
{
  return ((x & m) << shift) | ((x >> shift) & m);
}

//-----------------------------------------------------------------------------

#ifdef __RX__
[[gnu::always_inline]]
inline void atomic_clear_bit (unsigned int bitnum, volatile void* byte_addr)
{
  if (__builtin_constant_p (bitnum))
    bitnum &= 7;
  asm volatile ("bclr	%1,%0.B"
		: "+Q" (*(volatile unsigned char*)byte_addr) : "ir" (bitnum) : "memory");
}

[[gnu::always_inline]]
inline void atomic_invert_bit (unsigned int bitnum, volatile void* byte_addr)
{
  if (__builtin_constant_p (bitnum))
    bitnum &= 7;
  asm volatile ("bnot	%1,%0.B"
		: "+Q" (*(volatile unsigned char*)byte_addr) : "ir" (bitnum) : "memory");
}

[[gnu::always_inline]]
inline void atomic_set_bit (unsigned int bitnum, volatile void* byte_addr)
{
  if (__builtin_constant_p (bitnum))
    bitnum &= 7;
  asm volatile ("bset	%1,%0.B"
		: "+Q" (*(volatile unsigned char*)byte_addr) : "ir" (bitnum) : "memory");
}

[[gnu::always_inline]]
inline void atomic_set_bit (bool val, unsigned int bitnum, volatile void* byte_addr)
{
  if (__builtin_constant_p (val))
  {
    if (val)
      atomic_set_bit (bitnum, byte_addr);
    else
      atomic_clear_bit (bitnum, byte_addr);
  }
  else if (!__builtin_constant_p (bitnum))
  {
    if (val)
      atomic_set_bit (bitnum, byte_addr);
    else
      atomic_clear_bit (bitnum, byte_addr);
  }
  else
    asm volatile (
	"tst	%1,%1\n\t"
	"bmnz	%2,%0.B"
	: "+Q" (*(volatile unsigned char*)byte_addr) : "r" (val), "i" (bitnum & 7) : "memory", "cc");
}

// copy the bit value from a register or byte memory to a bit in byte memory
template <typename SrcReg> [[gnu::always_inline]] inline
std::enable_if_t < std::is_integral_v<SrcReg> >
atomic_copy_bit (unsigned int src_bitnum, SrcReg src_reg,
		 unsigned int dst_bitnum, volatile void* dst_mem)
{
  if (!__builtin_constant_p (dst_bitnum))
  {
    if ((1u << src_bitnum) & src_reg)
      atomic_set_bit (dst_bitnum, dst_mem);
    else
      atomic_clear_bit (dst_bitnum, dst_mem);
  }
  else
  {
    if constexpr (sizeof (SrcReg) == 1)
    {
      if (__builtin_constant_p (src_bitnum))
	src_bitnum &= 7;

      asm volatile (
	"btst	%1,%Q2\n\t"
	"bmnz	%3,%0.B"
	: "+Q" (*(volatile unsigned char*)dst_mem)
	: "ir" (src_bitnum), "Qr" (src_reg), "i" (dst_bitnum & 7)
	: "memory", "cc");
    }
    else
    {
      if (__builtin_constant_p (src_bitnum))
	src_bitnum &= 31;

      asm volatile (
	"btst	%1,%2\n\t"
	"bmnz	%3,%0.B"
	: "+Q" (*(volatile unsigned char*)dst_mem)
	: "ir" (src_bitnum), "r" (src_reg), "i" (dst_bitnum & 7)
	: "memory", "cc");
    }
  }
}

#else // __RX__

// FIXME: in this file, the generic atomic byte variants should be implemented
// the RX specific things are just optimizations for atomic byte op 1 bit
// constants and should go into the compiler.

#if 1
inline void atomic_clear_bit (unsigned int bitnum, volatile void* byte_addr)
{
  std::atomic_fetch_and ((volatile std::atomic<uint8_t>*)byte_addr, (uint8_t)(~(1u << bitnum)));
}

inline void atomic_invert_bit (unsigned int bitnum, volatile void* byte_addr)
{
  std::atomic_fetch_xor ((volatile std::atomic<uint8_t>*)byte_addr, (uint8_t)(1u << bitnum));
}

inline void atomic_set_bit (unsigned int bitnum, volatile void* byte_addr)
{
  std::atomic_fetch_or ((volatile std::atomic<uint8_t>*)byte_addr, (uint8_t)(1u << bitnum));
}
#endif


#if 0
inline void atomic_clear_bit (unsigned int bitnum, volatile void* byte_addr)
{
  auto&& p = (volatile uint8_t*)byte_addr;
  *p = *p & (uint8_t)(~(1u << bitnum));
}

inline void atomic_invert_bit (unsigned int bitnum, volatile void* byte_addr)
{
  auto&& p = (volatile uint8_t*)byte_addr;
  *p = *p ^ (uint8_t)(1u << bitnum);
}

inline void atomic_set_bit (unsigned int bitnum, volatile void* byte_addr)
{
  auto&& p = (volatile uint8_t*)byte_addr;
  *p = *p | (uint8_t)(1u << bitnum);
}


inline void atomic_set_bit (bool val, unsigned int bitnum, volatile void* byte_addr)
{
  if (val)
    atomic_set_bit (bitnum, byte_addr);
  else
    atomic_clear_bit (bitnum, byte_addr);
}

inline void atomic_copy_bit (unsigned int src_bitnum, unsigned int src_reg,
			     unsigned int dst_bitnum, volatile void* dst_mem)
{
  atomic_set_bit ((src_reg & (1u << src_bitnum)) != 0, dst_bitnum, dst_mem);
}
#endif


#endif


//-----------------------------------------------------------------------------


} // namespace utils
#endif // includeguard_utils_bits_includeguard

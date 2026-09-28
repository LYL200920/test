/*

MCB v2 board specific driver for the digital inputs and digital outputs.

the inputs and outputs are latches attached to the RX data bus
and memory mapped in the CS7 memory area (0x0100'0000 - 0x01FF'FFFF).
only address bits A1,A2,A3 are connected, so only 16 bit read/writes should
be done in that area.

to avoid conflicts with dipswitch readings, always set the unconnected address
bits PA6, PA6, PA7 during the bus access, i.e. use 0x0100'00E0 as a base
address for CS7.

RX CS7 area mapping:
0x00 - trigger outputs (lower 8 bit)
0x02 - digital outputs [15..0]
0x04 - digital outputs [31..16]
0x06 - mcx axis outputs (XOUT1..8, YOUT1..8)
0x08 - mcx axis outputs (ZOUT1..8, UOUT1..8)
0x0A - digital inputs [15..0]
0x0C - digital inputs [31..16]
0x0E - NAND flash RDY/BSY (read only)

the latches for the inputs can only be read from.
the latches for the outputs can only be written to.  because of that we
keep a copy of the output bit values in RAM.

*/

#ifndef includeguard_mcbv2_board_dev_digital_io_includeguard
#define includeguard_mcbv2_board_dev_digital_io_includeguard

#include <cstdint>
#include <bitset>
#include <array>
#include <chrono>

#include <utils/bits.hpp>
#include <utils/langcomp.hpp>
#include <utils/byte_order.hpp>
#include <dev/digital_io_port.hpp>

namespace dev
{

class digital_inputs final : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 32;

  digital_inputs (void)
  {
    // inputs are inverted by default.
    m_and_mask.set ();
    m_or_mask.reset ();
    m_xor_mask.set ();
  }

  std::bitset<port_count> and_mask (void) const { return m_and_mask; }
  std::bitset<port_count> or_mask (void) const { return m_or_mask; }
  std::bitset<port_count> xor_mask (void) const { return m_xor_mask; }

  void set_and_mask (std::bitset<port_count> val) { m_and_mask = val; }
  void set_or_mask (std::bitset<port_count> val) { m_or_mask = val; }
  void set_xor_mask (std::bitset<port_count> val) { m_xor_mask = val; }

  std::bitset<port_count> read (void) const
  {
    // read both 16 bit words as a contiguous 32 bit bitset.

    auto&& p = *(volatile uint32_t*)__builtin_assume_aligned ((void*)0x0100'00EA, 2);
    std::bitset<port_count> inputs (p);

    return ((inputs & m_and_mask) | m_or_mask) ^ m_xor_mask;
  }

  void write (const std::bitset<port_count>&)
  {
  }

  void exec (std::chrono::high_resolution_clock::time_point /*cur_time*/)
  {
  }

  void sync (void)
  {
  }

  virtual bool read_port (unsigned int n) const override
  {
    // FIXME: this will always read all 32 IOs, i.e. do 2 bus accesses.
    return n <= port_count ? read ()[n] : false;
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  std::bitset<port_count> m_and_mask;
  std::bitset<port_count> m_or_mask;
  std::bitset<port_count> m_xor_mask;
};

// ---------------------------------------------------------------------------

class digital_outputs final : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 32;

  digital_outputs (void)
  {
    m_and_mask = ~0u;
    m_or_mask = 0;
    m_xor_mask = 0;
    update_set_clear_val ();
  }

  std::bitset<port_count> and_mask (void) const { return { m_and_mask }; }
  std::bitset<port_count> or_mask (void) const { return { m_or_mask }; }
  std::bitset<port_count> xor_mask (void) const { return { m_xor_mask }; }

  void set_and_mask (std::bitset<port_count> val) { m_and_mask = val.to_ulong (); update_set_clear_val (); }
  void set_or_mask (std::bitset<port_count> val) { m_or_mask = val.to_ulong (); update_set_clear_val (); }
  void set_xor_mask (std::bitset<port_count> val) { m_xor_mask = val.to_ulong (); update_set_clear_val (); }

  // when reading the outputs, re-apply the AOX masks so that
  // write (read ()) will result in a no-operation.
  std::bitset<port_count> read (void) const
  {
    return std::bitset<port_count> (m_outputs ^ m_xor_mask);
  }

  void write (const std::bitset<port_count>& val)
  {
    m_outputs = ((val.to_ulong () & m_and_mask) | m_or_mask) ^ m_xor_mask;

    // the addresses are not 32 bit aligned, so it's not safe to use
    // a direct 32 bit access here.  normally would use memcpy and let the
    // compiler do the right thing.  however, it must not fallback to byte-wise
    // memcpy.
    // the following will split the access to 2x 16 bit in software on
    // strict-alignment targets.  on non-strict-alignment targets (like RX)
    // it will emit only one store instruction.
    auto&& p = *(volatile uint32_t*)__builtin_assume_aligned ((void*)0x0100'00E2, 2);
    p = m_outputs;
  }

  void exec (std::chrono::high_resolution_clock::time_point /*cur_time*/)
  {
  }

  void sync (void)
  {
  }

  virtual bool read_port (unsigned int i) const override
  {
    return utils::get_bit (m_outputs ^ m_xor_mask, i);
  }


  // __ZN3dev15digital_outputs10write_portEjb
  virtual void write_port (unsigned int i, bool val)
  {
    constexpr bool is_be = utils::native_byte_order () == utils::big_endian;
    const unsigned int n = (i / 8u) ^ (is_be ? 0b11 : 0b00);

    [[gnu::may_alias]] volatile uint16_t* const word_ptr = (volatile uint16_t*)&m_outputs;
    [[gnu::may_alias]] volatile int8_t* const byte_ptr = (volatile int8_t*)&m_outputs + n;

    const unsigned int word_num = n / 2;
    [[gnu::may_alias]] volatile uint16_t* const base_reg_addr = (volatile uint16_t*)0x0100'00E2;

    // pre-apply the mask values to the single bit and
    // atomically set or clear the resulting bit into the cached outputs variable.
    // then update the the corresponding 16-bit word in the outputs registers.

    if (__builtin_constant_p (i))
    {
      // this function can get optimized after devirtualization and inlining.

      // if the bit number is a constant, let the compiler figure out the best
      // way to address the thing.  in most cases it will use reg+disp addressing
      // for all the fields, which is OK.

      // also, reference the set/clear variables as bytes to get better RX code.

      utils::atomic_copy_bit (i & 7, *((uint8_t*)(&m_clear_set_val[val]) + n), i & 7, byte_ptr);
      std::atomic_signal_fence (std::memory_order_release);

      *(base_reg_addr + word_num) = *(word_ptr + word_num);
    }
    else
    {
      // unfortunately, due to lack of proper AMS optimization, the compiler
      // generates unnecessary instructions to calulate addresses in the
      // dynamic case. hence use the hand optimized version.
      asm volatile (

"	shll	#2,%0"		"\n"
"	add	%1,%0"		"\n"
"	btst	%2,16[%0].B"	"\n"
"	bz	0f"		"\n"
"	bset	%2,[%1]"	"\n"
"	bra	1f"		"\n"
"0:\n"
"	bclr	%2,[%1]"	"\n"
"1:\n"
"	mov.w	[%3,%4],%0"	"\n"
"	mov.w	%0,[%3,%5]"	"\n"
	:
	: "r" (val), "r" (byte_ptr), "r" (i), "r" (word_num),
	  "r" (word_ptr), "r" (base_reg_addr)
	: "memory", "cc");
    }
  }

  digital_io_port operator [] (unsigned int n)
  {
    if (unlikely (n >= port_count))
      return { &digital_io_port::g_null_dev, n };

    return { this, n };
  }

private:
  static_assert (port_count == 32);

  uint32_t m_outputs;		// 4
  uint32_t m_and_mask;		// 8
  uint32_t m_or_mask;		// 12
  uint32_t m_xor_mask;		// 16
  uint32_t m_clear_set_val[2];	// 20, 24

  void update_set_clear_val (void)
  {
    m_clear_set_val[0] = ((0x00000000 & m_and_mask) | m_or_mask) ^ m_xor_mask;
    m_clear_set_val[1] = ((0xFFFFFFFF & m_and_mask) | m_or_mask) ^ m_xor_mask;
  }

};

} // namespace dev
#endif // includeguard_mcbv2_board_dev_digital_io_includeguard

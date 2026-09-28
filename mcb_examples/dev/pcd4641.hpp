#ifndef includeguard_dev_pcd4641_hpp_includeguard
#define includeguard_dev_pcd4641_hpp_includeguard

#include <cstdint>
#include <cstddef>
#include <array>
#include <bitset>
#include <mutex>

#include <utils/value_range.hpp>
#include <utils/bits.hpp>
#include <utils/byte_order.hpp>

#include <dev/interrupt.hpp>
#include <dev/pulse_type.hpp>

namespace dev
{
namespace pcd4641
{

// -----------------------------------------------------------------------
// bus interfaces

enum reg_number
{
  regno_rmv = 0,
  regno_rfl = 1,                 // bit[15:0], read/write
  regno_start_mode_cmd = 1,      // bit[23:16] read-only
  regno_rfh = 2,                 // bit[15:0] read/write
  regno_control_mode_cmd = 2,    // bit[23:16] read-only
  regno_rud = 3,                 // bit[15:0] read/write
  regno_register_select_cmd = 3, // bit[23:16] read-only
  regno_rmg = 4,                 // bit[15:0] read/write
  regno_output_mode_cmd = 4,     // bit[23:16] read-only
  regno_rdp = 5,
  regno_ridl = 6,  // bit[7:0] read/write
  regno_rspd = 6,  // bit[23:8] read-only
  regno_renv = 7,  // bit[15:0] read/write
  regno_ridc = 7,  // bit[23:16] read-only
  regno_rcun = 8,
  regno_rsts = 9,  // bit[15:0] read-only
  regno_riop = 10,
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// 8 bit parallel
// this relies on the bus controller to split the accesses to 8 bits.
template <uint32_t RegBaseAddr,
	  uint32_t CmdRegBaseAddr,
	  typename CmdWriteDelayFunc,
	  utils::byte_order_t HostByteOrder>
class if_parallel8_le
{
public:
  void write_start_mode_cmd (unsigned int axis, uint8_t param)
  {
    cmd_regs (axis).combf_msts = 0b00'000000 | (param & 0b00'111111);
    CmdWriteDelayFunc () ();
  }

  void write_ctrl_mode_cmd (unsigned int axis, uint8_t param)
  {
    regs (axis).combf_msts = 0b01'000000 | (param & 0b00'111111);
  }

  void write_output_mode_cmd (unsigned int axis, uint8_t param)
  {
    // always set OCM5 bit for PCD46x1 mode.
    regs (axis).combf_msts = 0b11'100000 | (param & 0b00'111111);
  }

  // irq bit 1: External start interrupt
  // irq bit 0: Ramping-down point interrupt
  void write_irq_sel (unsigned int axis, int8_t irqbits)
  {
    m_reg_sel_irq_bits[axis] = irqbits << 4;

    // setting the IRQ bits is done by selecting any register.
    // in this case, we select register 0xF (15), which does not actually exist.
    regs (axis).combf_msts = 0b10'00'1111 | m_reg_sel_irq_bits[axis];
  }

  void write_reg24 (unsigned int axis, unsigned int regno, uint32_t val)
  {
    auto& r = regs (axis);
    r.combf_msts = 0b10'00'0000 | m_reg_sel_irq_bits[axis] | (regno & 0b00'00'1111);
    r.rb2 = (uint8_t)(val >> 16);
    r.rb1 = (uint8_t)(val >>  8);
    r.rb0 = (uint8_t)(val >>  0); // this triggers the actual register write.
  }

  void write_reg16 (unsigned int axis, unsigned int regno, uint16_t val)
  {
    auto& r = regs (axis);
    r.combf_msts = 0b10'00'0000 | m_reg_sel_irq_bits[axis] | (regno & 0b00'00'1111);
    r.rb1 = (uint8_t)(val >>  8);
    r.rb0 = (uint8_t)(val >>  0); // this triggers the actual register write.
  }

  void write_reg8 (unsigned int axis, unsigned int regno, uint8_t val)
  {
    auto& r = regs (axis);
    r.combf_msts = 0b10'00'0000 | m_reg_sel_irq_bits[axis] | (regno & 0b00'00'1111);
    r.rb0 = val; // this triggers the actual register write inside.
  }

  uint32_t read_reg24 (unsigned int axis, unsigned int regno)
  {
    auto& r = regs (axis);
    r.combf_msts = 0b10'00'0000 | m_reg_sel_irq_bits[axis] | (regno & 0b00'00'1111);

    // read order is arbitrary
    return (r.rb0 << 0) | (r.rb1 << 8) | (r.rb2 << 16);
  }

  // read a 24 bit register and shift it right by 8.
  uint16_t read_reg24_shr8 (unsigned int axis, unsigned int regno)
  {
    auto& r = regs (axis);
    r.combf_msts = 0b10'00'0000 | m_reg_sel_irq_bits[axis] | (regno & 0b00'00'1111);

    // read order is arbitrary
    return (r.rb1 << 0) | (r.rb2 << 8);
  }

  // read a 24 bit register and shift it right by 16.
  uint8_t read_reg24_shr16 (unsigned int axis, unsigned int regno)
  {
    auto& r = regs (axis);
    r.combf_msts = 0b10'00'0000 | m_reg_sel_irq_bits[axis] | (regno & 0b00'00'1111);
    return r.rb2;
  }

  uint16_t read_reg16 (unsigned int axis, unsigned int regno)
  {
    auto& r = regs (axis);
    r.combf_msts = 0b10'00'0000 | m_reg_sel_irq_bits[axis] | (regno & 0b00'00'1111);

    // read order is arbitrary
    return (r.rb0 << 0) | (r.rb1 << 8);
  }

  uint8_t read_reg8 (unsigned int axis, unsigned int regno)
  {
    auto& r = regs (axis);
    r.combf_msts = 0b10'00'0000 | m_reg_sel_irq_bits[axis] | (regno & 0b00'00'1111);
    return r.rb0;
  }

  uint8_t read_msts (unsigned int axis)
  {
    return regs (axis).combf_msts;
  }

private:
  struct axis_regs_t
  {
    volatile uint8_t combf_msts; // write: combf, read: msts
    volatile uint8_t rb0;	 // register read/write buffer bits 7:0
    volatile uint8_t rb1;	 // register read/write buffer bits 15:8
    volatile uint8_t rb2;	 // register read/write buffer bits 23:16
  };
  static_assert (sizeof (axis_regs_t) == 4, "");


  static constexpr axis_regs_t& regs (unsigned int n)
  {
    return *(axis_regs_t*)(RegBaseAddr + n*sizeof (axis_regs_t));
  }

  static constexpr axis_regs_t& cmd_regs (unsigned int n)
  {
    return *(axis_regs_t*)(CmdRegBaseAddr + n*sizeof (axis_regs_t));
  }

  // there are 2 interrupt control bits in the register select
  // command byte.  each time we write a register select, we must preserve
  // those bits.  to do that, we keep a shadow copy of those bits in RAM.
  uint8_t m_reg_sel_irq_bits[4];
};


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// 8 bit parallel, twidded A0 and A1 lines
// this relies on the bus controller to split 32 bit accesses to 8 bit accesses
// and access the bytes in the address order n, n+1, n+2, n+3.

// if the host byte order can be big endian, register writes can be carried
// out efficiently.  otherwise a byte-swap has to be inserted.

// big endian 32 bit write order:
// reg bits	write addr	A1,A0	A1^A0,A0	pcd reg
// 31:24	4n		00	00		combf
// 23:16	4n+1		01	11		rb2
// 15:8		4n+2		10	10		rb1
// 7:0		4n+3		11	01		rb0


// little endian 32 bit write order:
// reg bits	write addr	A1,A0	A1^A0,A0	pcd reg
// 7:0		4n		00	00		combf
// 15:8		4n+1		01	11		rb2
// 23:16	4n+2		10	10		rb1
// 31:24	4n+3		11	01		rb0

struct dummy_mutex
{
  void lock (void) { }
  void unlock (void) { }
};

template <uint32_t RegBaseAddr,
	  uint32_t CmdRegBaseAddr,
	  typename CmdWriteDelayFunc,
	  utils::byte_order_t HostByteOrder,
	  typename RegReadMutex = dummy_mutex>
class if_a0a1_twiddled_parallel8
{
public:
  if_a0a1_twiddled_parallel8 (void)
  {
    m_combf_reg_sel_roip_all = 0
      | (combf_reg_sel_roip_init_val () << 0)
      | (combf_reg_sel_roip_init_val () << 8)
      | (combf_reg_sel_roip_init_val () << 16)
      | (combf_reg_sel_roip_init_val () << 24);
  }

  void write_start_mode_cmd (unsigned int axis, uint8_t param)
  {
    // do two write access cycles to the command area
    // the 2nd write will go to the rb2 register, which does no harm.
    // it streches the bus access timing by inserting another cycle.
    // because the PCD is normally clocked much slower, it is difficult to
    // append the necessary wait delay with a single write cycle.
    *(volatile uint16_t*)(CmdRegBaseAddr + axis*sizeof (axis_regs_t)) =
	(0b00'000000 | (param & 0b00'111111)) << (HostByteOrder == utils::big_endian ? 8 : 0);

    CmdWriteDelayFunc () ();
  }

  void write_ctrl_mode_cmd (unsigned int axis, uint8_t param)
  {
    regs (axis).combf_msts = 0b01'000000 | (param & 0b00'111111);
  }

  void write_output_mode_cmd (unsigned int axis, uint8_t param)
  {
    // always set OCM5 bit for PCD46x1 mode.
    regs (axis).combf_msts = 0b11'100000 | (param & 0b00'111111);
  }

  // irq bit 1: External start interrupt
  // irq bit 0: Ramping-down point interrupt
  void write_irq_sel (unsigned int axis, std::bitset<2> irqbits)
  {
    m_reg_sel_irq_bits[axis] = irqbits.to_ulong () << 4;
    m_combf_reg_sel_roip[axis] = (irqbits.to_ulong () << 4) | combf_reg_sel_roip_init_val ();

    // setting the IRQ bits is done by selecting any register.
    // in this case, we select register 0xF (15), which does not actually exist.
    regs (axis).combf_msts = 0b10'00'1111 | m_reg_sel_irq_bits[axis];
  }

  void write_reg24 (unsigned int axis, unsigned int regno, uint32_t val)
  {
    m_last_combf_reg_sel[axis] = regno;

    val = ((0b10'00'0000 | (regno & 0b00'00'1111) | m_reg_sel_irq_bits[axis]) << 24)
	  | (val & 0x00FFFFFF);

    if (HostByteOrder != utils::big_endian)
      val = utils::bswap (val);

    *(volatile uint32_t*)&regs (axis) = val;
  }

  void write_reg16 (unsigned int axis, unsigned int regno, uint16_t val)
  {
    // it's more efficient to write 4 bytes than 3 individual ones.
    write_reg24 (axis, regno, val);
  }

  void write_reg8 (unsigned int axis, unsigned int regno, uint8_t val)
  {
    // even if writing 2 bytes could be faster than writing 4 bytes at once,
    // a single write will makes register write operations atomic.
    write_reg24 (axis, regno, val);
  }

  uint32_t read_reg24 (unsigned int axis, unsigned int regno)
  {
    auto& r = regs (axis);
    uint32_t val;

    do
    {
      std::lock_guard<RegReadMutex> rd_lock (m_reg_read_mutex);

      m_last_combf_reg_sel[axis] = regno;
      r.combf_msts = 0b10'00'0000 | m_reg_sel_irq_bits[axis] | (regno & 0b00'00'1111);

      // read order of register bytes is arbitrary.
      val = (r.rb0 << 0) | (r.rb1 << 8) | (r.rb2 << 16);
    } while (m_last_combf_reg_sel[axis] != regno);

    return val;
  }

  // read a 24 bit register and shift it right by 8.
  uint16_t read_reg24_shr8 (unsigned int axis, unsigned int regno)
  {
    auto& r = regs (axis);
    uint16_t val;

    do
    {
      std::lock_guard<RegReadMutex> rd_lock (m_reg_read_mutex);

      m_last_combf_reg_sel[axis] = regno;
      r.combf_msts = 0b10'00'0000 | m_reg_sel_irq_bits[axis] | (regno & 0b00'00'1111);

      // read order is arbitrary
      val = (r.rb1 << 0) | (r.rb2 << 8);
    } while (m_last_combf_reg_sel[axis] != regno);

    return val;
  }

  // read a 24 bit register and shift it right by 16.
  uint8_t read_reg24_shr16 (unsigned int axis, unsigned int regno)
  {
    auto& r = regs (axis);
    uint8_t val;

    do
    {
      std::lock_guard<RegReadMutex> rd_lock (m_reg_read_mutex);

      m_last_combf_reg_sel[axis] = regno;
      r.combf_msts = 0b10'00'0000 | m_reg_sel_irq_bits[axis] | (regno & 0b00'00'1111);
      val = r.rb2;
    } while (m_last_combf_reg_sel[axis] != regno);

    return val;
  }

  uint16_t read_reg16 (unsigned int axis, unsigned int regno)
  {
    auto& r = regs (axis);
    uint16_t val;

    do
    {
      std::lock_guard<RegReadMutex> rd_lock (m_reg_read_mutex);

      m_last_combf_reg_sel[axis] = regno;
      r.combf_msts = 0b10'00'0000 | m_reg_sel_irq_bits[axis] | (regno & 0b00'00'1111);

      // read order is arbitrary
      val = (r.rb0 << 0) | (r.rb1 << 8);
    } while (m_last_combf_reg_sel[axis] != regno);

    return val;
  }

  uint8_t read_reg8 (unsigned int axis, unsigned int regno)
  {
    auto& r = regs (axis);
    uint8_t val;

    do
    {
      std::lock_guard<RegReadMutex> rd_lock (m_reg_read_mutex);

      m_last_combf_reg_sel[axis] = regno;
      r.combf_msts = 0b10'00'0000 | m_reg_sel_irq_bits[axis] | (regno & 0b00'00'1111);
      val = r.rb0;
    } while (m_last_combf_reg_sel[axis] != regno);

    return val;
  }

  uint8_t read_msts (unsigned int axis)
  {
    return regs (axis).combf_msts;
  }

  // for direct RIOP register access by DMA-like devices, e.g. RX DTC
  auto& combf_reg_sel_riop_value (unsigned int axis) { return m_combf_reg_sel_roip[axis]; }
  auto& combf_reg_sel_riop_values (void) { return m_combf_reg_sel_roip_all; }

  auto& last_combf_reg_sel_value (unsigned int axis) { return m_last_combf_reg_sel[axis]; }
  auto& last_combf_reg_sel_values (void) { return m_last_combf_reg_sel_all; }

  auto* combf_reg_addr (unsigned int axis) { return &(regs (axis).combf_msts); }
  auto* rb0_reg_addr (unsigned int axis) { return &(regs (axis).rb0); }

private:
  struct axis_regs_t
  {
    volatile uint8_t combf_msts; // write: combf, read: msts
    volatile uint8_t rb2;	 // register read/write buffer bits 23:16
    volatile uint8_t rb1;	 // register read/write buffer bits 15:8
    volatile uint8_t rb0;	 // register read/write buffer bits 7:0
  };
  static_assert (sizeof (axis_regs_t) == 4, "");


  static constexpr axis_regs_t& regs (unsigned int n)
  {
    return *(axis_regs_t*)(RegBaseAddr + n*sizeof (axis_regs_t));
  }

  // there are 2 interrupt control bits in the register select
  // command byte.  each time we write a register select, we must preserve
  // those bits.  to do that, we keep a shadow copy of those bits in RAM.
  uint8_t m_reg_sel_irq_bits[4];

  // keep track of the selected register in combf for interrupt save register
  // access.
  union
  {
    volatile uint8_t m_last_combf_reg_sel[4];
    volatile uint32_t m_last_combf_reg_sel_all;
  };

  // pre-computed combf register value for selecting the RIOP register.
  // this value is set by the CPU whenever the interrupt bits are changed
  // this value can be read by DMA-like devices for direct RIOP register access.
  static constexpr uint8_t combf_reg_sel_roip_init_val (void)
  {
    return 0b10'00'0000 | regno_riop;
  }

  union
  {
    volatile uint8_t m_combf_reg_sel_roip[4];
    volatile uint32_t m_combf_reg_sel_roip_all;
  };

  RegReadMutex m_reg_read_mutex;
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// PCD4641A also supports SPI ...


// -----------------------------------------------------------------------

enum axis_num
{
  axis_x = 0,
  axis_y = 1,
  axis_z = 2,
  axis_u = 3
};


// -----------------------------------------------------------------------
// speed register value calculations

// convert the speed [pps] value into FH [pps] and magnification value RMG.
// basically this is a special floating point format.
// for a given speed value (1..2457300), find the minimum RMG value (2..1023)
// and the corresponding FH/FL value (1..8191)
// the RMG value depends on the clock speed of the PCD.
struct fp_speed
{
  uint16_t exp;
  uint16_t fract;
};

// given the calculated exponent (RMG value) and the desired speed value,
// calculate the fractional part (FH/FL value).
// if the clock speed is a multiple of 16 we can use 32 bit calculations.
// otherwise for maximum values we need 36 bits.
// speed value = max. 19 bit
// exp value = max. 10 bit
// fract value = max. 13 bit
template <unsigned int ClockHz_1> inline constexpr uint16_t
calc_speed_fract_clk (uint32_t speed, uint16_t exp)
{
  return (ClockHz_1 & 15) == 0
	 ? (uint16_t)( ((speed * exp) * (8192/16) + (ClockHz_1/16 - 1)) / (ClockHz_1/16) )
	 : (uint16_t)( ((uint64_t)(speed * exp) * 8192 + (ClockHz_1-1)) / ClockHz_1 );
}

template <unsigned int ClockHz_1> inline constexpr fp_speed
calc_speed_fract_clk (uint32_t speed, fp_speed fp)
{
  return { fp.exp, calc_speed_fract_clk<ClockHz_1> (speed, fp.exp) };
}

// given the desired speed value, calculate the exponent (RMG value).
// if the clock speed is a multiple of 16 we can use 32 bit calculations.
// otherwise for maximum values we need 36 bits.
template <unsigned int ClockHz_1> inline constexpr fp_speed
calc_speed_exp_clk (uint32_t speed)
{
  constexpr unsigned int max_fract = 8191;

  return { speed < 8192
	   ? (uint16_t)(ClockHz_1 / 8192)
	   : (uint16_t)std::max (uint16_t (2),
		(ClockHz_1 & 15) == 0
		? (uint16_t)( ((ClockHz_1/16)*max_fract ) / (speed*(8192/16)) )
		: (uint16_t)( (((uint64_t)ClockHz_1*max_fract)) / (speed*8192) )),
	   0 };
}

template <unsigned int ClockHz_1> inline constexpr fp_speed
calc_speed_clk (uint32_t speed)
{
  return calc_speed_fract_clk<ClockHz_1> (speed, calc_speed_exp_clk<ClockHz_1> (speed));
}

// given an exponent (RMG value) and a fractional part (FH/FL value),
// calculate the actual speed in [pps].
// if the clock speed fits into 23 bits and is a multiple of 16 we can use
// 32 bit calculations.  otherwise for maximum values we need 36 bits.
template <unsigned int ClockHz_1> inline constexpr uint32_t
calc_speed_pps_clk (uint16_t exp, uint16_t fract)
{
  // (4915200 / (600 * 8192)) * 8000
  // (ClockHz / (exp * 8192)) * fract = (ClockHz * fract) / (exp * 8192)
  //                                        23   +  13    /    10 + 13    bits
  //                                              36      /    23         bits
  return ClockHz_1 < (1 << 23) && (ClockHz_1 & 15) == 0
	 ? ( ((ClockHz_1/16) * fract) / (exp * (8192/16)) )
	 : (uint32_t)( ((uint64_t)ClockHz_1 * fract) / (exp * 8192) );
}


// -----------------------------------------------------------------------
// status word as returned by a MSTS register read
class status_t
{
public:
  constexpr status_t (void) : m_value (0) { }
  explicit constexpr status_t (uint8_t val) : m_value (val) { }

  constexpr uint8_t value (void) const { return m_value; }

  // pending interrupt requests
  constexpr bool irq_stop (void) const { return !utils::get_bit (m_value, 0); }
  constexpr bool irq_ramping_down_point (void) const { return !utils::get_bit (m_value, 1); }
  constexpr bool irq_external_start (void) const { return !utils::get_bit (m_value, 2); }

  // operation status monitor
  constexpr bool busy (void) const { return utils::get_bit (m_value, 3); }
  constexpr bool is_driving (void) const { return busy (); }

  // remaining pulse 0 monitor
  // if RMV == 0: 1
  // if RMV != 0: 0
  constexpr bool plsz (void) const { return utils::get_bit (m_value, 4); }

  // ramp-down point passed monitor
  // if RMV <= RDP: 1
  // if RMV > RDP: 0
  constexpr bool sdp (void) const { return utils::get_bit (m_value, 5); }

  // acceleration/deceleration monitor
  constexpr bool accelerating (void) const { return utils::get_bit (m_value, 6); }
  constexpr bool decelerating (void) const { return utils::get_bit (m_value, 7); }

private:
  uint8_t m_value;
};

// -----------------------------------------------------------------------
// extended status word as returned by RSTS register read
class ext_status_t
{
public:
  constexpr ext_status_t (void) : m_value (0) { }
  explicit constexpr ext_status_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  // negative (minus) limit
  constexpr bool mel (void) const { return utils::get_bit (m_value, 0); }

  // positive limit
  constexpr bool pel (void) const { return utils::get_bit (m_value, 1); }

  // origin/home
  constexpr bool org (void) const { return utils::get_bit (m_value, 2); }

  // external stop
  constexpr bool stp (void) const { return utils::get_bit (m_value, 3); }

  // external start
  constexpr bool sta (void) const { return utils::get_bit (m_value, 4); }

  // negative slow-down
  constexpr bool msd (void) const { return utils::get_bit (m_value, 5); }

  // positive slow-down
  constexpr bool psd (void) const { return utils::get_bit (m_value, 6); }

  // excitation origin
  constexpr bool phz (void) const { return utils::get_bit (m_value, 7); }

  // motor phase (sph1, sph2, sph3, sph4 in the manual)
  constexpr unsigned int ph (void) const { return (m_value >> 8) & 0x0F; }

  // -po/dir output signal monitor
  constexpr bool mpo (void) const { return utils::get_bit (m_value, 12); }

  // +po/pls output signal monitor
  constexpr bool spo (void) const { return utils::get_bit (m_value, 13); }

  // ots output signal monitor
  constexpr bool ots (void) const { return utils::get_bit (m_value, 14); }

  // interrupt request (sint in the manual, can't use 'int' as function name)
  constexpr bool irq (void) const { return utils::get_bit (m_value, 15); }

private:
  uint16_t m_value;
};

// -----------------------------------------------------------------------
// output mode command word
enum output_logic_t
{
  negative_logic = 0,
  positive_logic = 1
};

enum sensor_sensitivity_t
{
  // no digital signal filter (1 clock pulse response)
  high_sensitivity = 0,

  // digital signal filter (4 clock pulse response)
  low_sensitivity = 1
};

class output_mode_t
{
public:
  constexpr output_mode_t (void) : m_value (0) { }
  explicit constexpr output_mode_t (uint8_t val) : m_value (val) { }

  constexpr uint8_t value (void) const { return m_value; }

  constexpr output_logic_t output_logic (void) const { return (output_logic_t)bit (0); }
  output_mode_t& set_output_logic (output_logic_t val) { return set_bit (0, val); }

  // pulse output mask.  if pulse output is disabled, counting operations
  // continue internally, but the outputs will not be output.
  // some problems have been observed when enabling the pulse output after
  // power-on/reset without actually starting axis driving.
  constexpr bool pulse_output (void) const { return !bit (1); }
  output_mode_t& set_pulse_output (bool val = true) { return set_bit (1, !val); }

  // step motor excitation sequence output
  constexpr bool sequence_signal_output (void) const { return !bit (2); }
  output_mode_t& set_sequence_signal_output (bool val = true) { return set_bit (2, !val); }

  // if set, acceleration or deceleration will stopped and the current speed
  // will be fixed.  if unset, acceleration/deceleration will continue.
  constexpr bool stop_during_accel_decel (void) const { return bit (3); }
  output_mode_t& set_stop_during_accel_decel (bool val = true) { return set_bit (3, val); }

  constexpr sensor_sensitivity_t sensor_sensitivity (void) const { return (sensor_sensitivity_t)bit (4); }
  output_mode_t& set_sensor_sensitivity (sensor_sensitivity_t val) { return set_bit (4, val); }

private:
  uint8_t m_value;

  constexpr bool bit (unsigned int n) const { return utils::get_bit (m_value, n); }
  output_mode_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }
};

// -----------------------------------------------------------------------
// control mode command word
enum direction_t
{
  positive_dir = 0,
  negative_dir = 1
};

enum accel_decel_mode_t
{
  linear = 0,
  scurve = 1
};

class control_mode_t
{
public:
  constexpr control_mode_t (void) : m_value (0) { }
  explicit constexpr control_mode_t (uint8_t val) : m_value (val) { }

  constexpr uint8_t value (void) const { return m_value; }

  constexpr bool org_input_signal (void) const { return bit (0); }
  control_mode_t& set_org_input_signal (bool val = true) { return set_bit (0, val); }

  constexpr bool sd_input_signal (void) const { return bit (1); }
  control_mode_t& set_sd_input_signal (bool val = true) { return set_bit (1, val); }

  constexpr bool positioning_mode (void) const { return bit (2); }
  control_mode_t& set_positioning_mode (bool val = true) { return set_bit (2, val); }

  constexpr direction_t direction (void) const { return (direction_t)bit (3); }
  control_mode_t& set_direction (direction_t val) { return set_bit (3, val); }

  constexpr bool ots_output (void) const { return bit (4); }
  control_mode_t& set_ots_output (bool val) { return set_bit (4, val); }

  constexpr accel_decel_mode_t accel_decel_mode (void) const { return (accel_decel_mode_t)bit (5); }
  control_mode_t& set_accel_decel_mode (accel_decel_mode_t val) { return set_bit (5, val); }

private:
  uint8_t m_value;

  constexpr bool bit (unsigned int n) const { return utils::get_bit (m_value, n); }
  control_mode_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }
};

// -----------------------------------------------------------------------
// start mode command word
enum start_stop_mode_bits_t
{
  stop_immediately = 1,
  start = 2,
  decel_stop = 3
};

enum speed_sel_t
{
  fl_speed = 0,
  fh_speed = 1
};

enum speed_mode_t
{
  constant_speed = 0,
  high_speed_accel_decel = 1,
};

class drive_mode_t
{
public:
  constexpr drive_mode_t (void) : m_value (0) { }
  explicit constexpr drive_mode_t (uint8_t val) : m_value (val) { }

  constexpr uint8_t value (void) const { return m_value; }

  constexpr speed_sel_t speed_sel (void) const { return (speed_sel_t)bit (0); }
  drive_mode_t& set_speed_sel (speed_sel_t val) { return set_bit (0, val); }

  // when starting wait for the !STA input.
  // default is not to wait for the !STA input.
  constexpr bool hold_start (void) const { return bit (1); }
  drive_mode_t& set_hold_start (bool val = true) { return set_bit (1, val); }

  constexpr speed_mode_t speed_mode (void) const { return (speed_mode_t)bit (2); }
  drive_mode_t& set_speed_mode (speed_mode_t val) { return set_bit (2, val); }

  constexpr start_stop_mode_bits_t cmd (void) const { return (start_stop_mode_bits_t)((m_value >> 3) & 3); }
  drive_mode_t& set_cmd (start_stop_mode_bits_t c){ m_value = (uint8_t)((m_value & ~(3 << 3)) | (c << 3)); return *this; }

  constexpr bool irq_when_stopped (void) const { return bit (5); }
  drive_mode_t& set_irq_when_stopped (bool val = true) { return set_bit (5, val); }

private:
  uint8_t m_value;

  constexpr bool bit (unsigned int n) const { return utils::get_bit (m_value, n); }
  drive_mode_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }
};

// -----------------------------------------------------------------------
// RENV environment/operation mode settings
enum pulse_output_mode_t
{
  // outputs (+) direction pulse from the +PO/PLS terminal and (-) direction
  // pulse from the -PO/DIR terminal.
  independent_double_pulse = 0,

  // outputs pulses from the +PO/PLS terminal and direction signal
  // from the -PO/DIR terminal.
  single_pulse_single_direction = 1
};

constexpr inline bool pulse_output_type_supported (pulse_type pt)
{
  return pt == pulse_type::single_pulse_direction
	 || pt == pulse_type::double_pulse;
}

enum ramp_down_mode_t
{
  manual = 0,
  automatic = 1
};

enum sensor_stop_mode_t
{
  sensor_stop_immediately = 0,
  sensor_stop_decel = 1
};

enum position_counter_dir_t
{
  count_inc = 0,
  count_dec = 1
};

enum p1p2p3p4_mode_t
{
  phase_signals = 0,
  p1p2p3p4_io = 1
};

enum p1p2p3p4_io_mode_t
{
  output = 0,
  input = 1
};

class env_t
{
public:
  constexpr env_t (void) : m_value (0) { }
  explicit constexpr env_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  constexpr pulse_output_mode_t pulse_output_mode (void) const { return (pulse_output_mode_t)bit (0); }
  env_t& set_pulse_output_mode (pulse_output_mode_t val) { return set_bit (0, val); }

  // set generic pulse output type.
  // if not supported do nothing.
  env_t& set_pulse_output_mode (dev::pulse_type pt)
  {
    if (pt == pulse_type::single_pulse_direction)
      set_pulse_output_mode (single_pulse_single_direction);
    else if (pt == pulse_type::double_pulse)
      set_pulse_output_mode (independent_double_pulse);

    return *this;
  }

  constexpr bool positioning_down_counter (void) const { return !bit (2); }
  env_t& set_positioning_down_counter (bool val = true) { return set_bit (2, !val); }

  constexpr ramp_down_mode_t ramp_down_mode (void) const { return (ramp_down_mode_t)bit (3); }
  env_t& set_ramp_down_mode (ramp_down_mode_t val) { return set_bit (3, val); }

  constexpr sensor_stop_mode_t stp_stop_mode (void) const { return (sensor_stop_mode_t)bit (4); }
  env_t& set_stp_stop_mode (sensor_stop_mode_t val) { return set_bit (4, val); }

  constexpr sensor_stop_mode_t el_stop_mode (void) const { return (sensor_stop_mode_t)bit (5); }
  env_t& set_el_stop_mode (sensor_stop_mode_t val) { return set_bit (5, val); }

  constexpr sensor_stop_mode_t org_stop_mode (void) const { return (sensor_stop_mode_t)bit (6); }
  env_t& set_org_stop_mode (sensor_stop_mode_t val) { return set_bit (6, val); }

  // Reset automatically at the falling edge of !ORG input (OFF to ON) in
  // origin return operation
  constexpr bool position_counter_auto_reset (void) const { return bit (7); }
  env_t& set_position_counter_auto_reset (bool val = true) { return set_bit (7, val); }

  constexpr bool position_counter_enable (void) const { return !bit (8); }
  env_t& set_position_counter_enable (bool val = true) { return set_bit (8, !val); }

  constexpr position_counter_dir_t position_counter_dir (void) const { return (position_counter_dir_t)bit (9); }
  env_t& set_position_counter_dir (position_counter_dir_t val) { return set_bit (9, val); }

  constexpr p1p2p3p4_mode_t p1p2p3p4_mode (void) const { return (p1p2p3p4_mode_t)bit (11); }
  env_t& set_p1p2p3p4_mode (p1p2p3p4_mode_t val) { return set_bit (11, val); }

  constexpr p1p2p3p4_io_mode_t p1_io_mode (void) const { return (p1p2p3p4_io_mode_t)bit (12); }
  env_t& set_p1_io_mode (p1p2p3p4_io_mode_t val) { return set_bit (12, val); }

  constexpr p1p2p3p4_io_mode_t p2_io_mode (void) const { return (p1p2p3p4_io_mode_t)bit (13); }
  env_t& set_p2_io_mode (p1p2p3p4_io_mode_t val) { return set_bit (13, val); }

  constexpr p1p2p3p4_io_mode_t p3_io_mode (void) const { return (p1p2p3p4_io_mode_t)bit (14); }
  env_t& set_p3_io_mode (p1p2p3p4_io_mode_t val) { return set_bit (14, val); }

  constexpr p1p2p3p4_io_mode_t p4_io_mode (void) const { return (p1p2p3p4_io_mode_t)bit (15); }
  env_t& set_p4_io_mode (p1p2p3p4_io_mode_t val) { return set_bit (15, val); }

private:
  uint16_t m_value;

  constexpr bool bit (unsigned int n) const { return utils::get_bit (m_value, n); }
  env_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }
};



static constexpr unsigned int axis_count = 4;

// -----------------------------------------------------------------------
// hardware instance

template <typename BusInterface, unsigned int ClockHz, typename InterruptLine,
	  typename AxisInputs, typename AxisOutputs, typename AxisNumFunc>
class hw_inst
{
public:
  static constexpr unsigned int clock_hz = ClockHz;
  typedef BusInterface bus_interface;

  static constexpr unsigned int axis_count = dev::pcd4641::axis_count;


  static constexpr uint16_t calc_speed_fract (utils::clamped_value<uint32_t, 0, clock_hz/2> speed, uint16_t exp) { return calc_speed_fract_clk<clock_hz> (speed, exp); }
  static constexpr fp_speed calc_speed_fract (utils::clamped_value<uint32_t, 0, clock_hz/2> speed, fp_speed fp) { return calc_speed_fract_clk<clock_hz> (speed, fp); }
  static constexpr fp_speed calc_speed_exp (utils::clamped_value<uint32_t, 0, clock_hz/2> speed) { return calc_speed_exp_clk<clock_hz> (speed); }
  static constexpr fp_speed calc_speed (utils::clamped_value<uint32_t, 0, clock_hz/2> speed) { return calc_speed_clk<clock_hz> (speed); }

  static constexpr uint32_t calc_speed_pps (fp_speed fp) { return calc_speed_pps_clk<clock_hz> (fp.exp, fp.fract); };
  static constexpr uint32_t calc_speed_pps (uint16_t exp, uint16_t fract) { return calc_speed_pps_clk<clock_hz> (exp, fract); };

  // -----------------------------------------------------------------------

  class axis_t
  {
  public:
    static constexpr unsigned int this_size (void)
    {
      #pragma GCC diagnostic push
      #pragma GCC diagnostic ignored "-Winvalid-offsetof"

      // the size of this object as stored in an array container
      struct test { axis_t tmp[2]; };

      return offsetof (test, tmp[1]) - offsetof (test, tmp[0]);

      #pragma GCC diagnostic pop
    }

    axis_t (const axis_t&) = delete;
    axis_t& operator = (const axis_t&) = delete;

    axis_t (void)
    {
      m_cached_control_mode = 0;
      m_cached_env = 0;
      m_cached_drive_mode = 0;
      m_cached_output_mode = 0;
      m_cached_ridl = 0;
      m_irq_en = 0;
      m_cached_rdp = 0;
      m_cached_rmg = 0;
      m_cached_rud = 0;
      m_cached_rfl_rfh[0] = 0;
      m_cached_rfl_rfh[1] = 0;
    }

    unsigned int num (void) const
    {
      return AxisNumFunc () (this);
    }

    hw_inst& container (void) const
    {
      #pragma GCC diagnostic push
      #pragma GCC diagnostic ignored "-Winvalid-offsetof"

      return *(hw_inst*)((uintptr_t)this - this_size ()*num () - offsetof (hw_inst, m_axes));

      #pragma GCC diagnostic pop
    }

    auto&& inputs (void) { return m_axis_inputs; }
    auto&& outputs (void) { return m_axis_outputs; }

    status_t status (void) const { return status_t (container ().m_bif.read_msts (num ())); }
    ext_status_t ext_status (void) const { return ext_status_t (read_reg16 (regno_rsts)); }

    // -------------------------------------------------
    // command register access

    void set_output_mode (output_mode_t m)
    {
      m_cached_output_mode = m.value ();
      container ().m_bif.write_output_mode_cmd (num (), m.value ());
    }

    output_mode_t output_mode (void) const { return output_mode_t (m_cached_output_mode); }

    void drive (drive_mode_t m)
    {
/*
      // write a dummy command before writing the actual start command.
      // the dummy command value has bit 4 cleared to 0.
      // a start command has bits 4 and 3 set to "10".

      // also, even if the !WRQ wait signal is being used, the PCD will insert
      // wait cycles for the CPU, but for the start/dummy command we have to
      // wait for one PCD cyle ... or is it the number of currently set
      // "idling pulse count" ???

      // see also "6-5-4. Procedure to write a start command" in the PCD4641
      // manual.

      // MCB-177 FIXME: the newer PCD4641A does not have this problem
      // and it needs 3 wait cycles instead of 2, but its !WRQ handling has
      // been fixed.  if a PCD4641A is used, it should be OK to omit the
      // additional waits.

      // for now we always omit this and assume we're always using the PCD4641A.
      // there are only a few early prototype boards which have the PCD4641...

      uint8_t val = m.value ();

      if ((val & 0x18) == 0x10)
      {
	regs ()->combf_msts = (start_mode_cmd | val) & ~(1 << 4);

	// wait for (at least) 1 PCD clock cycle.
	std::chrono::duration<unsigned int, std::ratio<1, clock_hz>> wait_time (1);
	std::this_thread::sleep_for (wait_time);
      }
      regs ()->combf_msts = start_mode_cmd | val;
*/

      m_cached_drive_mode = m.value ();
      container ().m_bif.write_start_mode_cmd (num (), m.value ());
    }

    drive_mode_t start_mode (void) const { return drive_mode_t (m_cached_drive_mode); }

    control_mode_t control_mode (void) const { return control_mode_t (m_cached_control_mode); }

    // direct access to cache variable is needed for axis outputs to modify
    // OTS output atomically
    volatile uint8_t* control_mode_cache_var (void) { return &m_cached_control_mode; }
    void control_mode_write_cache_var (void)
    {
      container ().m_bif.write_ctrl_mode_cmd (num (), m_cached_control_mode);
    }

    void set_control_mode (control_mode_t m)
    {
      m_cached_control_mode = m.value ();
      control_mode_write_cache_var ();
    }

    void enable_ramp_down_point_irq (bool val = true)
    {
      m_irq_en = utils::set_bit (m_irq_en, 0, val);
      container ().m_bif.write_irq_sel (num (), m_irq_en);
    }

    bool ramp_down_point_irq_enabled (void) const
    {
      return utils::get_bit (m_irq_en, 0);
    }

    void enable_exernal_start_irq (bool val = true)
    {
      m_irq_en = utils::set_bit (m_irq_en, 1, val);
      container ().m_bif.write_irq_sel (num (), m_irq_en);
    }

    bool external_start_irq_enabled (void) const
    {
      return utils::get_bit (m_irq_en, 1);
    }

    // -------------------------------------------------
    // register access

    // 24 bit output pulse/step counter in positioning mode.
    // defines the distance (in pulses) to drive.

    static constexpr uint32_t feed_min_value = 0;
    static constexpr uint32_t feed_max_value = 16777215;

    void set_feed (utils::clamped_value<uint32_t, feed_min_value, feed_max_value> val) { write_reg24 (regno_rmv, val); }
    uint32_t feed (void) const { return read_reg24 (regno_rmv); }

    // FL and FH speed settings (13 bit)
    // notice, if FL/FH speed is 0 the motor might not stop.
    // thus the minimum value for FL/FH should be 1.
    static constexpr uint32_t speed_min_value = 1;
    static constexpr uint32_t speed_max_value = 8191;

    void set_speed (speed_sel_t s, utils::clamped_value<uint32_t, speed_min_value, speed_max_value> val)
    {
      m_cached_rfl_rfh[s] = val;
      write_reg16 (regno_rfl + s, val);
    }
    uint32_t speed (speed_sel_t s) const { return m_cached_rfl_rfh[s]; }

    static constexpr uint32_t speed_min_value_pps = 1;
    static constexpr uint32_t speed_max_value_pps = clock_hz / 2;

    void set_speed_pps (speed_sel_t s, utils::clamped_value<uint32_t, speed_min_value_pps, speed_max_value_pps> val)
    {
      auto speed_mag = calc_speed ((uint32_t)val);
      set_speed (s, speed_mag.fract);
      set_speed_magnification (speed_mag.exp);
    }

    uint32_t speed_pps (void) const
    {
      return calc_speed_pps (speed_magnification (), speed ());
    }

    // acceleration and deceleration rate control (16 bit)
    // Linear acceleration / deceleration
    //   Time of acceleration / deceleration [s]
    //     = (FH speed - FL speed) × accel_decel_rate / (ref clock [Hz])
    //
    // S-curve acceleration / deceleration
    //   Time of acceleration / deceleration [s]
    //     = (FH speed - FL speed) × accel_decel_rate × 2 / (ref clock [Hz])
    void set_accel_decel_rate (utils::clamped_value<uint16_t, 1, 65535> val) { m_cached_rud = val; write_reg16 (regno_rud, val); }
    uint16_t accel_decel_rate (void) const { return m_cached_rud; }

    // speed magnification (10 bit)
    void set_speed_magnification (utils::clamped_value<uint16_t, 2, 1023> val) { m_cached_rmg = val; write_reg16 (regno_rmg, val); }
    uint16_t speed_magnification (void) const { return m_cached_rmg & 1023; }

    // ramping-down point (24 bit)
    // defines after which distance (in pulses) the ramp phase should start.
    // in manual mode (ASDP = 0) it's a 24 bit unsigned value.
    // in automatic mode (ASDP = 1) it's a 24 bit signed value.
    void set_ramp_down_point_u24 (utils::clamped_value<uint32_t, 0, 16777215> val) { m_cached_rdp = val; write_reg24 (regno_rdp, val); }
    uint32_t ramp_down_point_u24 (void) const { return m_cached_rdp; }

    void set_ramp_down_point_s24 (utils::clamped_value<int32_t, -8388608, 8388607> val) { m_cached_rdp = val; write_reg24 (regno_rdp, val); }
    int32_t ramp_down_point_s24 (void) const { return m_cached_rdp; }
    
    // idling pulse count (3 bit)
    void set_idling_pulses (utils::clamped_value<uint8_t, 0, 7> val) { m_cached_ridl = val; write_reg8 (regno_ridl, val); }
    uint8_t idling_pulses (void) const { return m_cached_ridl; }

    // motor speed monitor in pulses/steps (13 bit)
    //   operation speed [pps] = (speed monitor value) × (speed magnification)
    uint16_t current_speed (void) const { return read_reg24_shr8 (regno_rspd); }

    uint32_t current_speed_pps (void) const { return calc_speed_pps (speed_magnification (), current_speed ()); }

    // set environment / operation mode
    void set_env (env_t val)
    {
      m_cached_env = val.value ();
      write_reg16 (regno_renv, val.value () | (1 << 1)); // always enable PCD46x1 mode
    }

    env_t env (void) const { return env_t (m_cached_env); }

    // current position counter (signed or unsigned 24 bit)
    void set_current_position_u24 (utils::clamped_value<uint32_t, 0, 16777215> val) { write_reg24 (regno_rcun, val); }
    uint32_t current_position_u24 (void) const { return read_reg24 (regno_rcun); }

    void set_current_position_s24 (utils::clamped_value<int32_t, -8388608, 8388607> val) { write_reg24 (regno_rcun, val); }
    int32_t current_position_s24 (void) const { return s24_to_s32 (read_reg24 (regno_rcun)); }

    // GPIO control
    // bit 0: P0
    // bit 1: P1
    // bit 2: P2
    // bit 3: P3
    // bit 4: U/B
    // bit 5: F/H
    void set_riop (uint8_t val) { write_reg8 (regno_riop, val); }
    uint8_t riop (void) const { return (uint8_t)read_reg8 (regno_riop); }

  private:
    volatile uint8_t m_cached_control_mode;
    uint8_t m_cached_drive_mode;
    uint8_t m_cached_output_mode;
    uint8_t m_cached_ridl;
    int8_t m_irq_en;
    uint16_t m_cached_env;
    uint16_t m_cached_rmg;
    uint16_t m_cached_rud;
    uint16_t m_cached_rfl_rfh[2];
    uint32_t m_cached_rdp;

    mutable AxisOutputs m_axis_outputs;
    mutable AxisInputs m_axis_inputs;

    void write_reg24 (unsigned int regno, uint32_t val) { container ().m_bif.write_reg24 (num (), regno, val); }
    void write_reg16 (unsigned int regno, uint16_t val) { container ().m_bif.write_reg16 (num (), regno, val); }
    void write_reg8 (unsigned int regno, uint8_t val) { container ().m_bif.write_reg8 (num (), regno, val); }

    auto read_reg24 (unsigned int regno) const { return container ().m_bif.read_reg24 (num (), regno); }
    auto read_reg24_shr8 (unsigned int regno) const { return container ().m_bif.read_reg24_shr8 (num (), regno); }
    auto read_reg24_shr16 (unsigned int regno) const { return container ().m_bif.read_reg24_shr16 (num (), regno); }
    auto read_reg16 (unsigned int regno) const { return container ().m_bif.read_reg16 (num (), regno); }
    auto read_reg8 (unsigned int regno) const { return container ().m_bif.read_reg8 (num (), regno); }


    static int32_t s24_to_s32 (uint32_t val)
    {
      if (val & (1 << 23))
	val |= 0xFF000000;

      return (int32_t)val;
    }

    friend class hw_inst;
  };

  [[gnu::cold]] hw_inst (BusInterface&& bif = { })
  {
    // enable PCD46x1 mode for each axis.  make sure that interrupts are
    // disabled by default (register select cmd).
    for (auto& a : axes ())
    {
      m_bif.write_irq_sel (a.num (), 0);
      a.set_output_mode (output_mode_t ());
      a.set_env (env_t ());
    }

    isr0_t (this).enable (interrupt::falling_edge, interrupt::priority_6);
  }

  constexpr std::array<axis_t, axis_count>& axes (void)
  {
    return m_axes;
  }

  constexpr axis_t& axis (unsigned int n)
  {
    return axes ()[n];
  }

  bus_interface& bus_if (void) { return m_bif; }

private:
  void isr_func (void)
  {
    // FIXME: register access is not interrupt safe.  have to add a lock
    // to the reg access functions or disable the interrupt during reg access.
    // (but only this interrupt, not all the other interrupts ...
    //  .. well not quite.. other ISRs might want to access the PCD, e.g.
    //     trigger input ISR starts motor driving or something
/*
  for (auto& a : axes ())
    if (a.ext_status ().irq ())
    {
      auto s = a.status ();
      if (s.irq_stop ())
        a.irq_stop_fuction ();
      if (s.irq_ramping_down_point ())
        a.irq_ramping_down_point_function ();
      if (s.irq_external_start ())
        a.irq_external_start_function ();
    }
*/
  }

public:
  typedef interrupt::connected_isr<InterruptLine,
	interrupt::func<decltype (&hw_inst::isr_func), &hw_inst::isr_func>> isr0_t;

private:
  std::array<axis_t, axis_count> m_axes;

  mutable BusInterface m_bif;
};

} // namespace pcd4641
} // namespace dev
#endif // includeguard_dev_pcd4641_hpp_includeguard

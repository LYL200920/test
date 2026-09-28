#ifndef includeguard_dev_mcx51x_hpp_includeguard
#define includeguard_dev_mcx51x_hpp_includeguard

#include <cstdint>
#include <cstddef>
#include <array>
#include <bitset>
#include <thread>
#include <type_traits>
#include <functional>
#include <atomic>
#include <ratio>
#include <limits>
#include <chrono>

#include <utils/value_range.hpp>
#include <utils/bits.hpp>
#include <utils/byte_order.hpp>
#include <utils/langcomp.hpp>

#include <dev/interrupt.hpp>
#include <dev/i2c.hpp>
#include <dev/pulse_type.hpp>

// enable or disable MCX register access logging.
// generally not interrupt safe.
// #define MCX51x_I2C_BUS_IF_DEBUG
// #define MCX51x_PARALLEL_BUS_IF_DEBUG


#if defined (MCX51x_I2C_BUS_IF_DEBUG) || defined (MCX51x_PARALLEL_BUS_IF_DEBUG)
#include <cstdio>
#endif

namespace dev
{


namespace mcx51x
{

// ----------------------------------------------------------------------------
// bus interfaces

// the MCX expects register data in little endian format.
// if the host system's bus controller supports endian swapping, it should
// be set to little endian.
// on a big endian system, if the bus controller does not support byte swapping,
// set the host byte order parameter to big endian.


template <utils::byte_order_t HostByteOrder> struct to_mcx_byte_order;
template <utils::byte_order_t HostByteOrder> struct to_cpu_byte_order;

template<> struct to_mcx_byte_order<utils::little_endian>
{
  static constexpr uint16_t convert (uint16_t val) { return val; }
  static constexpr uint32_t convert (uint32_t val) { return val; }
};

template<> struct to_cpu_byte_order<utils::little_endian>
{
  static constexpr uint16_t convert (uint16_t val) { return val; }
  static constexpr uint32_t convert (uint32_t val) { return val; }
};


template<> struct to_mcx_byte_order<utils::big_endian>
{
  static constexpr uint16_t convert (uint16_t val) { return utils::bswap (val); }
  static constexpr uint32_t convert (uint32_t val) { return utils::bswap (val); }
};

template<> struct to_cpu_byte_order<utils::big_endian>
{
  static constexpr uint16_t convert (uint16_t val) { return utils::bswap (val); }
  static constexpr uint32_t convert (uint32_t val) { return utils::bswap (val); }
};


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// 16 bit parallel interface
// this relies on the bus controller to split the accesses to 16 bits.

template < uintptr_t RegBaseAddr,		// base address for normal register access
	   uintptr_t CmdRegBaseAddr,		// base address for command register write
						// (additional 2 cycle delay added by bus controller)
	   typename CmdWriteDelayFunc,		// functor type which implements a 2 cycle delay
						// in software, which is necessary after command writes.
	   utils::byte_order_t HostByteOrder>
class if_parallel16
{
private:
  union regs_t
  {
    union
    {
      volatile uint16_t wr16[8];
      volatile uint16_t rr16[8];
    };

    union
    {
      volatile uint32_t wr32[4];
      volatile uint32_t rr32[4];
    };
  };

  constexpr regs_t* regs (void) { return (regs_t*)RegBaseAddr; }
  constexpr regs_t* cmd_regs (void) { return (regs_t*)CmdRegBaseAddr; }

  void write_cmd_wait (void) { CmdWriteDelayFunc ()();  }

  // in order to be able to safely access registers and issue data read/write
  // commands from within interrupt handlers, we need to implement some
  // extra measures to keep the stateful register access in a consistent state
  // when it is interrupted.
  // there are different types of mcx accesses:
  // - write command type with/without data
  //      write data in wr6,wr7, then command to wr0
  //
  // - read command type with data
  //      write command to wr0, read data from rr6,rr7
  //
  // - axis register read
  //      write axis/RR3 page selection to wr0, read data from r[1..5]
  //
  // - axis register write
  //      write axis selection to wr0, write data to wr[1..5]
  //
  // rr1 and wr1 are special cases and used only in the interrupt handler.
  //
  // before carrying out the access sequence, we atomically store the wr0
  // value and some additional bits that encode the access sequence type
  // in a variable, along with the data.  in case the sequence gets interrupted
  // that data will be used to restore the previous state of the sequence.
  // because the top most 4 bits of MCX wr0 are always 0 we use those bit to
  // encode some additional information.


  std::atomic<unsigned int> m_in_cmd = ATOMIC_VAR_INIT (0);
  std::atomic<uint32_t> m_in_write_data = ATOMIC_VAR_INIT (0);

public:

  enum access_sequence_type
  {
    // a normal write command with/without data
    in_cmd_write = 0b0001'0000'0000'0000,

    // a normal read command with data or axis mode register read/write
    in_cmd_read_mode_read_write = 0b0010'0000'0000'0000,

    // when processing interrupt handlers (in the driver), we know that the
    // access sequences can't be interrupted and the context save/restore needs
    // to be done only once.  set the sequence code to 0 which will disable
    // context save/restore for each subsequent individual register access.
    in_isr_context = 0b0000'0000'0000'0000,
  };

  struct reg_access_ctx
  {
    unsigned int in_cmd;
    uint32_t write_data;
  };

  struct reg_access_ctx_no_data
  {
    unsigned int in_cmd;
  };

  reg_access_ctx save_reg_context (access_sequence_type access_type,
				   unsigned int new_wr0, uint32_t new_wr6_wr7)
  {
    reg_access_ctx r { m_in_cmd.exchange (new_wr0 | access_type),
		       m_in_write_data.exchange (new_wr6_wr7) };

    // if we have interrupted a command write, insert a wait.
    if (unlikely (r.in_cmd != 0))
      write_cmd_wait ();

    return r;
  }

  reg_access_ctx_no_data save_reg_context (access_sequence_type access_type,
					   unsigned int new_wr0)
  {
    reg_access_ctx_no_data r { m_in_cmd.exchange (new_wr0 | access_type) };

    // if we have interrupted a command write, insert a wait.
    if (unlikely (r.in_cmd != 0))
      write_cmd_wait ();

    return r;
  }

  void restore_reg_context (reg_access_ctx ctx)
  {
    if (unlikely (ctx.in_cmd != 0))
    {
      if ((ctx.in_cmd & in_cmd_write) != 0)
      {
	// a command write has been interrupted.
	// if it was interrupted before the WR0 write this will restore the
	// WR6,WR7 data registers for the command write.
	// if it was interrupted after the WR0 write, the WR6,WR7 data registers
	// will be written again without any effect.

	// could differentiate whether to restore 16 bit or 32 bit regs.
	// but this case is unlikely, save some code and always restore 32 bit.

	write_reg32 (6, ctx.write_data);
      }
      else if ((ctx.in_cmd & in_cmd_read_mode_read_write) != 0)
      {
	// either a read command or a mode register read/write has been
	// interrupted.

	// for read commands, this will write WR0 multiple times and normally
	// has no side effect, except for RR1.

	// for mode read/writes, if interrupted before the register read/write,
	// this will restore the axis and/or RR3 page selection before the
	// actual register access.

	// for mode read/writes, if interrupted after the register read/write,
	// this will restore the axis an/or RR3 page selection without any further
	// effect.

	write_cmd_reg16 (0, ctx.in_cmd & 0x0FFF);
	write_cmd_wait ();
      }
    }

    // it's possible that a register access sequence is interrupted multiple
    // times.  in this case, make sure that the context data is always restored
    // to its previous state.  notice that when restoring the data of an
    // interrupted context, we assume to be in an (uninterruptable) ISR.
    m_in_cmd = ctx.in_cmd;
    m_in_write_data = ctx.write_data;
  }

  void restore_reg_context (reg_access_ctx_no_data ctx)
  {
    if (unlikely (ctx.in_cmd != 0))
    {
      if ((ctx.in_cmd & in_cmd_write) != 0)
      {
	// a command write has been interrupted, but WR6,WR7 has not been
	// modified.  hence there is nothing more to do.
      }
      else if ((ctx.in_cmd & in_cmd_read_mode_read_write) != 0)
      {
	// either a read command or a mode register read/write has been
	// interrupted.

	// for read commands, this will write WR0 multiple times and normally
	// has no side effect, except for RR1.

	// for mode read/writes, if interrupted before the register read/write,
	// this will restore the axis and/or RR3 page selection before the
	// actual register access.

	// for mode read/writes, if interrupted after the register read/write,
	// this will restore the axis an/or RR3 page selection without any further
	// effect.

	write_cmd_reg16 (0, ctx.in_cmd & 0x0FFF);
	write_cmd_wait ();
      }
    }

    // it's possible that a register access sequence is interrupted multiple
    // times.  in this case, make sure that the context data is always restored
    // to its previous state.  notice that when restoring the data of an
    // interrupted context, we assume to be in an (uninterruptable) ISR.
    m_in_cmd = ctx.in_cmd;
  }

  // copy construction is not really possible because of the atomic vars.
  if_parallel16 (void) { }
  if_parallel16 (const if_parallel16&) : if_parallel16 () { }
  if_parallel16 (if_parallel16&&) : if_parallel16 () { }

  // axis_bits is assumed to have the axis bits in the proper place
  // i.e. (1 << axis_num) << 8.

  // for faster multiple register access in ISR handlers, provide _isr
  // variants which do not save/restore the context.

  void write_cmd (unsigned int axis_bits, unsigned int cmdno)
  {
    const unsigned int wr0 = axis_bits | (cmdno & 0xFF);
    const auto reg_ctx = save_reg_context (in_cmd_write, wr0);

    write_cmd_reg16 (0, wr0);
    write_cmd_wait ();

    restore_reg_context (reg_ctx);
  }

  void write_cmd_isr (unsigned int axis_bits, unsigned int cmdno)
  {
    write_cmd_reg16 (0, axis_bits | (cmdno & 0xFF));
    write_cmd_wait ();
  }

  void write_cmd16 (unsigned int axis_bits, unsigned int cmdno, uint16_t data)
  {
    const unsigned int wr0 = axis_bits | (cmdno & 0xFF);
    const auto reg_ctx = save_reg_context (in_cmd_write, wr0, data);

    write_reg16 (6, data);
    write_cmd_reg16 (0, wr0);
    write_cmd_wait ();

    restore_reg_context (reg_ctx);
  }

  void write_cmd16_isr (unsigned int axis_bits, unsigned int cmdno, uint16_t data)
  {
    write_reg16 (6, data);
    write_cmd_reg16 (0, axis_bits | (cmdno & 0xFF));
    write_cmd_wait ();
  }

  void write_cmd32 (unsigned int axis_bits, unsigned int cmdno, uint32_t data)
  {
    const unsigned int wr0 = axis_bits | (cmdno & 0xFF);
    const auto reg_ctx = save_reg_context (in_cmd_write, wr0, data);

    write_reg32 (6, data);
    write_cmd_reg16 (0, wr0);
    write_cmd_wait ();

    restore_reg_context (reg_ctx);
  }

  void write_cmd32_isr (unsigned int axis_bits, unsigned int cmdno, uint32_t data)
  {
    write_reg32 (6, data);
    write_cmd_reg16 (0, axis_bits | (cmdno & 0xFF));
    write_cmd_wait ();
  }

  void write_axis_reg16 (unsigned int axis_bits, unsigned int cmdno, unsigned int regno, uint16_t val)
  {
    // assume that regno never refers to WR6 or WR7 (there is no use for that).
    const unsigned int wr0 = axis_bits | (cmdno & 0xFF);
    const auto reg_ctx = save_reg_context (in_cmd_read_mode_read_write, wr0);

    write_cmd_reg16 (0, wr0);
    write_cmd_wait ();
    write_reg16 (regno, val);

    restore_reg_context (reg_ctx);
  }

  void write_axis_reg16_isr (unsigned int axis_bits, unsigned int cmdno, unsigned int regno, uint16_t val)
  {
    write_cmd_reg16 (0, axis_bits | (cmdno & 0xFF));
    write_cmd_wait ();
    write_reg16 (regno, val);
  }


  // read commands first write a command and then read the data from the chip.
  // the data can be in one of the read registers, depending on the command
  // used.  normal data-read commands return data in rr6/rr7.  status reads
  // return data in rr1/rr2/rr3.
  uint16_t read_cmd16 (unsigned int axis_bits, unsigned int cmdno, unsigned int regno)
  {
    const unsigned int wr0 = axis_bits | (cmdno & 0xFF);
    const auto reg_ctx = save_reg_context (in_cmd_read_mode_read_write, wr0);

    write_cmd_reg16 (0, wr0);
    write_cmd_wait ();
    auto result = read_reg16 (regno);

    restore_reg_context (reg_ctx);
    return result;
  }

  uint16_t read_cmd16_isr (unsigned int axis_bits, unsigned int cmdno, unsigned int regno)
  {
    write_cmd_reg16 (0, axis_bits | (cmdno & 0xFF));
    write_cmd_wait ();
    return read_reg16 (regno);
  }

  uint32_t read_cmd32 (unsigned int axis_bits, unsigned int cmdno, unsigned int regno)
  {
    const unsigned int wr0 = axis_bits | (cmdno & 0xFF);
    const auto reg_ctx = save_reg_context (in_cmd_read_mode_read_write, wr0);

    write_cmd_reg16 (0, axis_bits | (cmdno & 0xFF));
    write_cmd_wait ();
    auto result = read_reg32 (regno);

    restore_reg_context (reg_ctx);
    return result;
  }

  uint32_t read_cmd32_isr (unsigned int axis_bits, unsigned int cmdno, unsigned int regno)
  {
    write_cmd_reg16 (0, axis_bits | (cmdno & 0xFF));
    write_cmd_wait ();
    return read_reg32 (regno);
  }

  // assume that single register reads/writes are atomic
  void write_reg16 (unsigned int regno, uint16_t val)
  {
    #ifdef MCX51x_PARALLEL_BUS_IF_DEBUG
      printf ("MCX WR%u <- 0x%04x\n", regno, val);
    #endif

    regs ()->wr16[regno] = to_mcx_byte_order<HostByteOrder>::convert (val);
  }

  void write_cmd_reg16 (unsigned int regno, uint16_t val)
  {
    #ifdef MCX51x_PARALLEL_BUS_IF_DEBUG
      printf ("MCX WR%u <- 0x%04x\n", regno, val);
    #endif

    cmd_regs ()->wr16[regno] = to_mcx_byte_order<HostByteOrder>::convert (val);
  }

  void write_reg32 (unsigned int regno, uint32_t val)
  {
    #ifdef MCX51x_PARALLEL_BUS_IF_DEBUG
      printf ("MCX WR%u <- 0x%04x\nMCX WR%u <- 0x%04x\n",
	      regno, regno + 1,
	      (unsigned int)(val & 0xFFFF), (unsigned int)((val >> 16) & 0xFFFF));
    #endif

    // let the bus controller split the 32 bit access into 2x16 bit.
    regs ()->wr32[regno/2] = to_mcx_byte_order<HostByteOrder>::convert (val);
  }

  uint16_t read_reg16 (unsigned int regno)
  {
    uint16_t val = to_cpu_byte_order<HostByteOrder>::convert (regs ()->rr16[regno]);

    #ifdef MCX51x_PARALLEL_BUS_IF_DEBUG
      printf ("MCX RR%u -> 0x%04x\n", regno, val);
    #endif

    return val;
  }

  uint32_t read_reg32 (unsigned int regno)
  {
    // let the bus controller split the 32 bit access into 2x16 bit.
    uint32_t val = to_cpu_byte_order<HostByteOrder>::convert (regs ()->rr32[regno/2]);

    #ifdef MCX51x_PARALLEL_BUS_IF_DEBUG
      printf ("MCX RR%u -> 0x%04x\nMCX RR%u -> 0x%04x\n",
	      regno, regno + 1,
	      (unsigned int)(val & 0xFFFF), (unsigned int)((val >> 16) & 0xFFFF));
    #endif

    return val;
  }

};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// 8 bit parallel interface for little endian host
// notice, the WR0 command register must be written with the high byte first
// and the low byte second.  this means, for WR0 write a different byte order
// has to be used by the bus controller and the register value has to be
// pre-swapped before the write.
template <uintptr_t RegBaseAddr, utils::byte_order_t HostByteOrder>
struct if_parallel8
{
  // FIXME: implement
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template <uint8_t SlaveAddr, unsigned int MaxSpeedBps> class if_i2c
{
private:
  dev::i2c_master* m_i2c_master;

public:
  if_i2c (void) = delete;
  if_i2c (dev::i2c_master& d) : m_i2c_master (&d) { }

  void write_cmd (unsigned int axis_bits, unsigned int cmdno)
  {
    #ifdef MCX51x_I2C_BUS_IF_DEBUG
    // printf ("i2c write cmd\n");
    #endif

    write_reg16 (0, ((axis_bits & 0x0F) << 8) | (cmdno & 0xFF));
  }

  void write_cmd16 (unsigned int axis_bits, unsigned int cmdno, uint16_t data)
  {
    write_reg16 (6, data);
    write_reg16 (0, ((axis_bits & 0x0F) << 8) | (cmdno & 0xFF));
  }

  void write_cmd32 (unsigned int axis_bits, unsigned int cmdno, uint32_t data)
  {
    write_reg32 (6, data);
    write_reg16 (0, ((axis_bits & 0x0F) << 8) | (cmdno & 0xFF));
  }

  uint16_t read_cmd16 (unsigned int axis_bits, unsigned int cmdno, unsigned int regno)
  {
    write_reg16 (0, ((axis_bits & 0x0F) << 8) | (cmdno & 0xFF));
    return read_reg16 (regno);
  }

  uint32_t read_cmd32 (unsigned int axis_bits, unsigned int cmdno, unsigned int regno)
  {
    write_reg16 (0, ((axis_bits & 0x0F) << 8) | (cmdno & 0xFF));
    return read_reg32 (regno);
  }

  void write_reg16 (unsigned int regno, uint16_t val)
  {
    #ifdef MCX51x_I2C_BUS_IF_DEBUG
    //printf ("i2c write reg %u  0x%04x.. ", regno, val);
    try {
    #endif

    if (regno == 0)
    {
      // when writing the wr0 register, the high byte must be written first
      // to select the axis, followed by the low byte to initiate the command.
      const uint8_t wr_data0[] = { (uint8_t)(val & 0xFF) };
      const uint8_t wr_data1[] = { (uint8_t)((val >> 8) & 0xFF) };

      m_i2c_master->send ((SlaveAddr << 5) | ((1 << 1) | 0), wr_data1);
      m_i2c_master->send ((SlaveAddr << 5) | ((0 << 1) | 0), wr_data0);
    }
    else
    {
      const uint8_t wr_data[] { (uint8_t)((val >> 0) & 0xFF), (uint8_t)((val >> 8) & 0xFF) };
      m_i2c_master->send ((SlaveAddr << 5) | ((regno*2 & 0x0F) << 1) | 0, wr_data);
    }

    #ifdef MCX51x_I2C_BUS_IF_DEBUG
    //printf (" OK\n");
    }
    // catch (...) { printf ("NG\n"); }
    catch (...) { printf ("i2c write reg %u  0x%04x.. NG\n", regno, val); }
    #endif
  }

  void write_reg32 (unsigned int regno, uint32_t val)
  {
    #ifdef MCX51x_I2C_BUS_IF_DEBUG
    //printf ("i2c write data 32 0x%08x.. ", (unsigned int)val);
    try {
    #endif

    uint8_t wr_data[] = { (uint8_t)((val >> 0) & 0xFF), (uint8_t)((val >> 8) & 0xFF),
			  (uint8_t)((val >> 16) & 0xFF), (uint8_t)((val >> 24) & 0xFF) };

    m_i2c_master->send ((SlaveAddr << 5) | ((regno*2 & 0x0F) << 1) | 0, wr_data);

    #ifdef MCX51x_I2C_BUS_IF_DEBUG
    //printf (" OK\n");
    }
    //catch (...) { printf ("NG\n"); }
    catch (...) { printf ("i2c write reg32 0x%08x.. NG\n", (unsigned int)val); }
    #endif
  }


  uint16_t read_reg16 (unsigned int regno)
  {
    #ifdef MCX51x_I2C_BUS_IF_DEBUG
    //printf ("i2c read reg %u.. ", regno);
    try {
    #endif

    //return regs ()->rr[regno];
    uint8_t rd_data[] = { 0, 0 };
    m_i2c_master->recv ((SlaveAddr << 5) | ((regno*2 & 0x0F) << 1) | 1, rd_data);

    uint16_t result = rd_data[0] | (rd_data[1] << 8);

    #ifdef MCX51x_I2C_BUS_IF_DEBUG
    //printf (" = 0x%04x  OK\n", result);
    #endif

    return result;

    #ifdef MCX51x_I2C_BUS_IF_DEBUG
    }
    //catch (...) { printf ("NG\n"); return 0; }
    catch (...) { printf ("i2c read reg16 %u..  NG\n", regno); return 0; }
    #endif
  }

  uint32_t read_reg32 (unsigned int regno)
  {
    // FIXME: could try to use little-endian 32 bit access
    // and let the bus controller split the access

    #ifdef MCX51x_I2C_BUS_IF_DEBUG
    //printf ("i2c read reg %u.. ", regno);
    try {
    #endif

    //return regs ()->rr[regno];
    uint8_t rd_data[] = { 0, 0, 0, 0 };
    m_i2c_master->recv ((SlaveAddr << 5) | ((regno*2 & 0x0F) << 1) | 1, rd_data);

    uint32_t result = (rd_data[0] << 0) | (rd_data[1] << 8)
		      | (rd_data[2] << 16) | (rd_data[3] << 24);

    #ifdef MCX51x_I2C_BUS_IF_DEBUG
    //printf (" = 0x%04x  OK\n", result);
    #endif

    return result;

    #ifdef MCX51x_I2C_BUS_IF_DEBUG
    }
    //catch (...) { printf ("NG\n"); return 0; }
    catch (...) { printf ("i2c read reg32 %u..  NG\n", regno); return 0; }
    #endif
  }
};

// ----------------------------------------------------------------------------
// speed calculations
//
// the standard MCX51x reference clock is 16 MHz.  with that clock, the speed
// values are normalized to pulse-per-second (pps) units.
// if the clock differs, additional calculations have to be made to convert
// between the native values and pps values.
// sometimes values can be pre-calculated and the resulting native values
// can be used more efficiently.  so allow the user to do both instead of
// always doing the calculations internally.


namespace speed_calc_impl
{
template <typename Ratio, uint32_t MaxInputValue = std::numeric_limits<uint32_t>::max ()>
inline constexpr uint32_t ratio_mult (uint32_t value)
{
  if ((uint64_t)MaxInputValue * Ratio::num > std::numeric_limits<uint32_t>::max ())
    return (uint32_t)((value * (uint64_t)Ratio::num + Ratio::den/2) / Ratio::den);
  else
    return (value * (uint32_t)Ratio::num + Ratio::den/2) / Ratio::den;
}

template <typename McxClockHzRatio>
struct clock_scale
{
  using RefClockHz = std::ratio <16'000'000, 1>;

  using pow1 = std::ratio_divide <McxClockHzRatio, RefClockHz>;
  using pow2 = std::ratio_multiply <pow1, pow1>;
  using pow3 = std::ratio_multiply <pow2, pow1>;

  using inv_pow1 = std::ratio_divide <std::ratio<1,1>, pow1>;
  using inv_pow2 = std::ratio_divide <std::ratio<1,1>, pow2>;
  using inv_pow3 = std::ratio_divide <std::ratio<1,1>, pow3>;
};
} // namespace speed_calc_impl


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// - initial speed      max val = max clock / 2 = 10'000'000
// - drive speed        max val = max clock / 2 = 10'000'000
// - home search speed  max val = 8000000
// - speed inc/dec      max val = 1000000
// - timer value        max val = 2147483647 (31 bits)

// value [pps] = value [native] x (fclk/16'000'000)

template <typename McxClockHzRatio, uint32_t MaxInputValue = std::numeric_limits<uint32_t>::max ()>
inline constexpr uint32_t native_to_pps (uint32_t value)
{
  using namespace speed_calc_impl;
  return ratio_mult < typename clock_scale<McxClockHzRatio>::pow1, MaxInputValue > (value);
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// value [native] = value [pps] / (fclk/16'000'000)
//                = value [pps] x (16'000'000 / fclk)

template <typename McxClockHzRatio, uint32_t MaxInputValue = std::numeric_limits<uint32_t>::max ()>
inline constexpr uint32_t pps_to_native (uint32_t value)
{
  using namespace speed_calc_impl;
  return ratio_mult < typename  clock_scale<McxClockHzRatio>::inv_pow1, MaxInputValue > (value);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// - acceleration      max val = 536870911 (29 bits)
// - deceleration      max val = 536870911 (29 bits)

// value [pps/sec] = value [native] x (fclk/16'000'000)²

template <typename McxClockHzRatio, uint32_t MaxValue = std::numeric_limits<uint32_t>::max ()>
inline constexpr uint32_t native_to_pps2 (uint32_t value)
{
  using namespace speed_calc_impl;
  return ratio_mult < typename clock_scale<McxClockHzRatio>::pow2, MaxValue > (value);
}

// value [native] = value [pps/sec] / (fclk/16'000'000)²
//                = value [pps/sec] * (1 / (fclk/16'000'000)²)

template <typename McxClockHzRatio, uint32_t MaxInputValue = std::numeric_limits<uint32_t>::max ()>
inline constexpr uint32_t
pps2_to_native (uint32_t value)
{
  using namespace speed_calc_impl;
  return ratio_mult < typename clock_scale<McxClockHzRatio>::inv_pow2, MaxInputValue > (value);
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// - jerk              max val = 1073741823 (30 bits)
// - decel inc rate    max val = 1073741823 (30 bits)
//

// value [pps/sec²] = value [native] x (fclk/16'000'000)³

template <typename McxClockHzRatio, uint32_t MaxInputValue = std::numeric_limits<uint32_t>::max ()>
inline constexpr uint32_t native_to_pps3 (uint32_t value)
{
  using namespace speed_calc_impl;
  return ratio_mult < typename clock_scale<McxClockHzRatio>::pow3, MaxInputValue> (value);
}

template <typename McxClockHzRatio, uint32_t MaxInputValue = std::numeric_limits<uint32_t>::max ()>
inline constexpr uint32_t
pps3_to_native (uint32_t value)
{
  using namespace speed_calc_impl;
  return ratio_mult < typename clock_scale<McxClockHzRatio>::inv_pow3, MaxInputValue> (value);
}



// ----------------------------------------------------------------------------
// parameter invariant register classes


// - - - - - - - - - - - - - - - - - - - - - - - -
// main status register: RR0
class status_t
{
public:
  constexpr status_t (void) : m_value (0) { }
  explicit constexpr status_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  // if a bit is set, the corresponding axis is driving.
  std::bitset<4> axis_driving (void) const { return (m_value >> 0) & 0x0F; }

  // if a bit is set, the corresponding axis is in some error condition.
  std::bitset<4> axis_error (void) const { return (m_value >> 4) & 0x0F; }

  constexpr uint8_t circular_interpolation_quadrant (void) const { return (m_value >> 8) & 7; }

  // for continuous interpolation, tells whether the next position coordinate
  // can be written.
  constexpr bool can_write_next_pos (void) const { return get_bit (11); }

  // for continuous interpolation, tells the current pre-buffer stack count.
  constexpr uint8_t pre_buffer_stack_count (void) const { return (m_value >> 12) & 0x0F; }

private:
  constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }

  uint16_t m_value;
};

// - - - - - - - - - - - - - - - - - - - - - - - -
// status register 1: RR1
// reading the irq_status will act as an interrupt acknowledge and
// will clear the irq bits in RR1.
// this is handled by the driver internally and should not be used by user code.



// - - - - - - - - - - - - - - - - - - - - - - - -
// status register 2: RR2
// use 'clear_finish_status' to clear the bits.
class drive_status_t
{
public:
  constexpr drive_status_t (void) : m_value (0) { }
  explicit constexpr drive_status_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  // SLMT+, SLMT-     driving stopped due to software limit.
  constexpr bool sw_limit_plus (void) const { return get_bit (0); }
  constexpr bool sw_limit_minus (void) const { return get_bit (1); }

  // HLMT+. HLMT-     driving stopped due to hardware limit signal.
  //
  constexpr bool hw_limit_plus (void) const { return get_bit (2); }
  constexpr bool hw_limit_minus (void) const { return get_bit (3); }

  // ALARM            alarm signal is on.
  constexpr bool alarm (void) const { return get_bit (4); }

  // EMG              emergency signal is on.
  constexpr bool emergency (void) const { return get_bit (5); }

  // HOME             error occured during automatic homing sequence.
  constexpr bool home_error (void) const { return get_bit (6); }

  // CERR             continuous interpolation error.
  constexpr bool interpolation_error (void) const { return get_bit (7); }

  // SYNC             driving stopped by one of the sync actions.
  constexpr bool sync_stop (void) const { return get_bit (8); }

  // STOP2..0         driving stopped by one of the stop signals.
  constexpr bool stop0_stop (void) const { return get_bit (9); }
  constexpr bool stop1_stop (void) const { return get_bit (10); }
  constexpr bool stop2_stop (void) const { return get_bit (11); }

  // LMT+, LMT-       driving stopped by limit signal (nLMTP)
  constexpr bool limit_plus_stop (void) const { return get_bit (12); }
  constexpr bool limit_minux_stop (void) const { return get_bit (13); }

  // ALARM            driving stopped by alarm signal
  constexpr bool alarm_stop (void) const { return get_bit (14); }

  // EMG              driving stopped by emergency signal
  constexpr bool emergency_stop (void) const { return get_bit (15); }

 private:
  constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }

  uint16_t m_value;
};


// - - - - - - - - - - - - - - - - - - - - - - - -
// status register 3: RR3, page 0
class signal_status_t
{
public:
  constexpr signal_status_t (void) : m_value (0) { }
  explicit constexpr signal_status_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  // STOP2..0          stop signal line values.
  constexpr bool stop0 (void) const { return get_bit (0); }
  constexpr bool stop1 (void) const { return get_bit (1); }
  constexpr bool stop2 (void) const { return get_bit (2); }

  // ECA               status of encoder input signal nECA/PPIN
  // the signal state is inverted and swapped with ECB by the driver according
  // to the mode3 setting.
  constexpr bool encoder_input_a (void) const { return get_bit (3); }

  // ECB               status of encoder input signal nECB/PMIN
  // the signal state is inverted and swapped with ECA by the driver according
  // to the mode3 setting.
  constexpr bool encoder_input_b (void) const { return get_bit (4); }

  // INPOS             status of in-poisition signal nINPOS
  // the signal state is inverted by the driver according to the mode2 setting.
  constexpr bool in_position (void) const { return get_bit (5); }

  // ALARM             status of alarm input signal nALARM
  // the signal state is inverted by the driver according to the mode2 setting.
  constexpr bool alarm (void) const { return get_bit (6); }

  // LMTP              status of hardware limit + input signal nLMTP
  // the signal state is inverted and swapped with LMTM by the driver according
  // to the mode2 and mode3 settings.
  constexpr bool hw_limit_pos (void) const { return get_bit (7); }

  // LMTM              status of hardware limit - input signal nLMTM
  // the signal state is inverted and swapped with LMTP by the driver according
  // to the mode2 and mode3 settings.
  constexpr bool hw_limit_neg (void) const { return get_bit (8); }

  // HSST5..0          automatic home sequence state number
  // 0: idle, waits for automatic home search execution command
  //
  // 3: step 1, waits for activation of a detection signal in the specified
  //    search direction
  //
  // 6: step 1, the timer is running between step 1 and step 2
  //
  // 11: step 2, waits for activation of a detection signal in the direction
  //     opposite to the specified search direction (irregular operation)
  //
  // 15: step 2, waits for deactivation of a detection signal in the direction
  //     opposite to the specified search direction (irregular operation)
  //
  // 18: step 2, the timer is running after irregular operation
  //
  // 20: step 2, waits for activation of a detection signal in the specified
  //     search direction
  //
  // 23: the timer is running between step 2 and step 3, or deviation counter
  //     clear is outputting
  //
  // 28: step 3, waits for activation of nSTOP2 signal in the specified
  //     search direction
  //
  // 32: the timer is running between step 3 and step 4, or deviation counter
  //     clear is outputting
  //
  // 36: offset driving in the specified search direction
  constexpr uint8_t home_search_state (void) const { return (m_value >> 9) & 63; }

 private:
  constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }

  uint16_t m_value;
};


// - - - - - - - - - - - - - - - - - - - - - - - -
// status register 3: RR3, page 1
class misc_status_t
{
public:
  constexpr misc_status_t (void) : m_value (0) { }
  explicit constexpr misc_status_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  // SYNC3..0          sync action is active.
  constexpr uint8_t sync (void) const { return m_value & 7; }

  // ASND              drive is currently accelerating
  constexpr bool accelerating (void) const { return get_bit (4); }

  // CNST              drive is at constant speed
  constexpr bool constant_speed (void) const { return get_bit (5); }

  // DSND              drive is currently decelerating
  constexpr bool decelerating (void) const { return get_bit (6); }

  // AASND             in s-curve acceleration/deceleration is increasing
  constexpr bool s_curve_accel_decel_inc (void) const { return get_bit (7); }

  // ACNST             in s-curve, acceleration/deceleration is constant
  constexpr bool s_curve_accel_decel_const (void) const { return get_bit (8); }

  // ADSND             in s-curve, acceleration/deceleration is decreasing
  constexpr bool s_curve_accel_decel_dec (void) const { return get_bit (9); }

  // TIMER             timer is enabled
  constexpr bool timer_enabled (void) const { return get_bit (10); }

  // SPLIT             split pulse is enabled
  constexpr bool split_pulse_enabled (void) const { return get_bit (11); }

  // MCERR             finishing point transfer error during multichip operation
  constexpr bool multi_chip_error (void) const { return get_bit (12); }

private:
  constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }

  uint16_t m_value;
};


// - - - - - - - - - - - - - - - - - - - - - - - -
// mode register 1 (WR1)
// per-axis interrupt mask is managed by the driver and should not be
// modified by user code.

// - - - - - - - - - - - - - - - - - - - - - - - -
// the position counter is referenced in two places
// sw limit counter and sync actions.
// to be able to use the same symbolic name, use a share enum.
// luckily the actual bit values are the same in both cases.
enum position_counter_t
{
  logical_position_counter = 0,
  real_position_counter = 1
};


// - - - - - - - - - - - - - - - - - - - - - - - -
// mode register 2 (WR2) setting

enum logic_t
{
  active_low = 0,
  negative = 0,

  active_high = 1,
  positive = 1
};

enum stop_mode_t
{
  stop_immediately = 0,
  decel_stop = 1
};

class mode2_t
{
public:
  constexpr mode2_t (void) : m_value (0) { }
  constexpr explicit mode2_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  constexpr logic_t stop0_logic (void) const { return (logic_t)get_bit (0); }
  mode2_t& set_stop0_logic (logic_t val) { return set_bit (0, val); }

  constexpr bool stop0_enable (void) const { return get_bit (1); }
  mode2_t& set_stop0_enable (bool val = true) { return set_bit (1, val); }

  constexpr logic_t stop1_logic (void) const { return (logic_t)get_bit (2); }
  mode2_t& set_stop1_logic (logic_t val) { return set_bit (2, val); }

  constexpr bool stop1_enable (void) const { return get_bit (3); }
  mode2_t& set_stop1_enable (bool val = true) { return set_bit (3, val); }

  constexpr logic_t stop2_logic (void) const { return (logic_t)get_bit (4); }
  mode2_t& set_stop2_logic (logic_t val) { return set_bit (4, val); }

  constexpr bool stop2_enable (void) const { return get_bit (5); }
  mode2_t& set_stop2_enable (bool val = true) { return set_bit (5, val); }

  constexpr logic_t in_position_logic (void) const { return (logic_t)get_bit (6); }
  mode2_t& set_in_position_logic (logic_t val) { return set_bit (6, val); }

  constexpr bool in_position_enable (void) const { return get_bit (7); }
  mode2_t& set_in_position_enable (bool val = true) { return set_bit (7, val); }

  constexpr logic_t alarm_logic (void) const { return (logic_t)get_bit (8); }
  mode2_t& set_alarm_logic (logic_t val) { return set_bit (8, val); }

  constexpr bool alarm_enable (void) const { return get_bit (9); }
  mode2_t& set_alarm_enable (bool val = true) { return set_bit (9, val); }

  constexpr logic_t hw_limit_logic (void) const { return (logic_t)get_bit (10); }
  mode2_t& set_hw_limit_logic (logic_t val) { return set_bit (10, val); }

  constexpr bool hw_limit_enable (void) const { return get_bit (11); }
  mode2_t& set_hw_limit_enable (bool val = true) { return set_bit (11, val); }

  constexpr stop_mode_t hw_limit_stop_mode (void) const { return (stop_mode_t)get_bit (12); }
  mode2_t& set_hw_limit_stop_mode (stop_mode_t val) { return set_bit (12, val); }

  constexpr bool sw_limit_enable (void) const { return get_bit (13); }
  mode2_t& set_sw_limit_enable (bool val = true) { return set_bit (13, val); }

  constexpr position_counter_t sw_limit_type (void) const { return (position_counter_t)get_bit (14); }
  mode2_t& set_sw_limit_type (position_counter_t val) { return set_bit (14, val); }

  constexpr stop_mode_t sw_limit_stop_mode (void) const { return (stop_mode_t) (!get_bit (15)); }
  mode2_t& set_sw_limit_stop_mode (stop_mode_t val) { return set_bit (15, !val); }

private:
  constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
  mode2_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

  uint16_t m_value;
};


// - - - - - - - - - - - - - - - - - - - - - - - -
// mode register 3 (WR3) setting
enum decel_mode_t
{
  automatic = 0,
  manual = 1
};

enum accel_decel_symmetry_t
{
  symmetric = 0,
  asymmetric = 1
};

enum accel_decel_mode_t
{
  linear = 0,
  scurve = 1
};

enum pulse_output_mode_t
{
  //                                DPMD1,     DPMD0
  independent_double_pulse      = (0 << 1) | (0 << 0),
  single_pulse_single_direction = (0 << 1) | (1 << 0),
  quad_pulse_quad_edge_eval     = (1 << 1) | (0 << 0),
  quad_pulse_double_edge_eval   = (1 << 1) | (1 << 0),
};

constexpr inline bool pulse_output_type_supported (pulse_type pt)
{
  return pt == pulse_type::single_pulse_direction
	 || pt == pulse_type::double_pulse
	 || pt == pulse_type::ab_phase_double_edge
	 || pt == pulse_type::ab_phase_quad_edge;
}

enum encoder_pulse_input_t
{
  //                                   PIMD1,     PIMD0
  quad_pulse_input_quad_edge_eval   = (0 << 1) | (0 << 0),
  quad_pulse_input_double_edge_eval = (0 << 1) | (1 << 0),
  quad_pulse_input_single_edge_eval = (1 << 1) | (0 << 0),
  independent_double_pulse_input    = (1 << 1) | (1 << 0)   // "up down pulse" in the manual.
};

constexpr inline bool pulse_input_type_supported (pulse_type pt)
{
  return pt == pulse_type::double_pulse
	 || pt == pulse_type::ab_phase_single_edge
	 || pt == pulse_type::ab_phase_double_edge
	 || pt == pulse_type::ab_phase_quad_edge;
}

enum timer_mode_t
{
  one_shot = 0,
  repeat = 1,
};

class mode3_t
{
public:
  constexpr mode3_t (void) : m_value (0) { }
  constexpr explicit mode3_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  // MANLD
  constexpr decel_mode_t decel_mode (void) const { return (decel_mode_t)get_bit (0); }
  mode3_t& set_decel_mode (decel_mode_t val) { return set_bit (0, val); }

  // DSNDE
  constexpr accel_decel_symmetry_t accel_decel_symmetry (void) const { return (accel_decel_symmetry_t)get_bit (1); }
  mode3_t& set_accel_decel_symmetry (accel_decel_symmetry_t val) { return set_bit (1, val); }

  // SACC
  constexpr accel_decel_mode_t accel_decel_mode (void) const { return (accel_decel_mode_t)get_bit (2); }
  mode3_t& set_accel_decel_mode (accel_decel_mode_t val) { return set_bit (2, val); }

  // DPMD1,0
  pulse_output_mode_t pulse_output_mode (void) const { return (pulse_output_mode_t)((m_value >> 3) & 3); }
  mode3_t& set_pulse_output_mode (pulse_output_mode_t val) { m_value = (uint16_t)((m_value & ~(3 << 3)) | (val << 3)); return *this; }

  // set generic pulse output type.
  // if not supported do nothing.
  mode3_t& set_pulse_output_mode (dev::pulse_type pt)
  {
    if (pt == pulse_type::single_pulse_direction)
      set_pulse_output_mode (single_pulse_single_direction);
    else if (pt == pulse_type::double_pulse)
      set_pulse_output_mode (independent_double_pulse);
    else if (pt == pulse_type::ab_phase_double_edge)
      set_pulse_output_mode (quad_pulse_double_edge_eval);
    else if (pt == pulse_type::ab_phase_quad_edge)
      set_pulse_output_mode (quad_pulse_quad_edge_eval);

    return *this;
  }

  // DP-L
  logic_t pulse_output_logic (void) const { return get_bit (5) ? negative : positive; }
  mode3_t& set_pulse_output_logic (logic_t val) { return set_bit (5, val == negative ? true : false); }

  // DIR-L
  // logic level of the direction signal (when using 1-pulse-1-direction output mode)
  // active_high: positive direction is high, negative direction is low
  // active_low: positive direction is low, negative direction is high
  logic_t direction_output_logic (void) const { return get_bit (6) ? positive : negative; }
  mode3_t& set_direction_output_logic (logic_t val) { return set_bit (6, val == positive ? true : false); }

  // DPINV
  // swaps the two drive pulse output signals
  bool swap_pulse_output_signals (void) const { return get_bit (7); }
  mode3_t& set_swap_pulse_output_signals (bool val = true) { return set_bit (7, val); }

  // PIMD1,0
  encoder_pulse_input_t encoder_pulse_input (void) const { return (encoder_pulse_input_t)((m_value >> 8) & 3); }
  mode3_t& set_encoder_pulse_input (encoder_pulse_input_t val) { m_value = (uint16_t)((m_value & ~(3 << 8)) | (val << 8)); return *this; }

  // set generic pulse input type.
  // if not supported do nothing.
  mode3_t& set_encoder_pulse_input (dev::pulse_type pt)
  {
    if (pt == pulse_type::double_pulse)
      set_encoder_pulse_input (independent_double_pulse_input);
    else if (pt == pulse_type::ab_phase_single_edge)
      set_encoder_pulse_input (quad_pulse_input_single_edge_eval);
    else if (pt == pulse_type::ab_phase_double_edge)
      set_encoder_pulse_input (quad_pulse_input_double_edge_eval);
    else if (pt == pulse_type::ab_phase_quad_edge)
      set_encoder_pulse_input (quad_pulse_input_quad_edge_eval);

    return *this;
  }

  // PI-L
  logic_t encoder_input_logic (void) const { return get_bit (10) ? negative : positive; }
  mode3_t& set_encoder_input_logic (logic_t val) { return set_bit (10, val == negative ? true : false); }

  // PIINV
  // swaps the two encoder AB pulse input signals
  bool swap_encoder_input_signals (void) const { return get_bit (11); }
  mode3_t& set_swap_encoder_input_signals (bool val = true) { return set_bit (11, val); }

  // LMINV
  // swaps positive limit signal with negative limit signal.
  bool swap_hw_limit_input_signals (void) const { return get_bit (12); }
  mode3_t& set_swap_hw_limit_input_signals (bool val = true) { return set_bit (12, val); }

  // AVTR1
  bool enable_triangle_form_prevention (void) const { return !get_bit (13); }
  mode3_t& set_enable_triangle_form_prevention (bool val = true) { return set_bit (13, !val); }

  // TMMD
  timer_mode_t timer_mode (void) const { return (timer_mode_t)get_bit (14); }
  mode3_t& set_timer_mode (timer_mode_t val) { return set_bit (14, val); }

private:
  constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
  mode3_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

  uint16_t m_value;
};


// - - - - - - - - - - - - - - - - - - - - - - - -
// H1M - automatic home search mode setting 1
enum s1_limit_t
{
  s1_use_stop0 = 0,
  s1_use_stop1 = 1,
  s1_use_limit = 2
};

enum s2_limit_t
{
  s2_use_stop1 = 0,
  s2_use_limit = 1
};

class auto_home_search_mode1_t
{
public:
  constexpr auto_home_search_mode1_t (void) : m_value (0) { }
  constexpr explicit auto_home_search_mode1_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  // step 1 - high speed search
  constexpr bool high_speed_search (void) const { return get_bit (0); }
  auto_home_search_mode1_t& set_high_speed_search (bool val = true) { return set_bit (0, val); }

  // positive: + direction
  // negative: - direction
  constexpr logic_t high_speed_search_dir (void) const { return get_bit (1) ? negative : positive; }
  auto_home_search_mode1_t& set_high_speed_search_dir (logic_t val) { return set_bit (1, val == negative ? true : false); }

  constexpr s1_limit_t high_speed_search_use_limit (void) const { return (s1_limit_t)((m_value >> 2) & 3); }
  auto_home_search_mode1_t& set_high_speed_search_use_limit (s1_limit_t val) { m_value = (uint16_t)((m_value & ~(3 << 2)) | (val << 2)); return *this; }


  // step 2 - low speed search
  constexpr bool low_speed_search (void) const { return get_bit (4); }
  auto_home_search_mode1_t& set_low_speed_search (bool val = true) { return set_bit (4, val); }

  constexpr logic_t low_speed_search_dir (void) const { return get_bit (5) ? negative : positive; }
  auto_home_search_mode1_t& set_low_speed_search_dir (logic_t val) { return set_bit (5, val == negative ? true : false); }

  constexpr s2_limit_t low_speed_search_use_limit (void) const { return (s2_limit_t)get_bit (6); }
  auto_home_search_mode1_t& set_low_speed_search_use_limit (s2_limit_t val) { return set_bit (6, val); }

  constexpr bool low_speed_search_clear_deviation_counter (void) const { return get_bit (7); }
  auto_home_search_mode1_t& set_low_speed_search_clear_deviation_counter (bool val = true) { return set_bit (7, val); }

  constexpr bool low_speed_search_clear_real_position_counter (void) const { return get_bit (8); }
  auto_home_search_mode1_t& set_low_speed_search_clear_real_position_counter (bool val = true) { return set_bit (8, val); }

  constexpr bool low_speed_search_clear_logical_position_counter (void) const { return get_bit (9); }
  auto_home_search_mode1_t& set_low_speed_search_clear_logical_position_counter (bool val = true) { return set_bit (9, val); }


  // step 3 -- low speed z phase search
  constexpr bool z_phase_search (void) const { return get_bit (10); }
  auto_home_search_mode1_t& set_z_phase_search (bool val = true) { return set_bit (10, val); }

  constexpr logic_t z_phase_search_dir (void) const { return get_bit (11) ? negative : positive; }
  auto_home_search_mode1_t& set_z_phase_search_dir (logic_t val) { return set_bit (11, val == negative ? true : false); }

  constexpr bool z_phase_search_clear_deviation_counter (void) const { return get_bit (12); }
  auto_home_search_mode1_t& set_z_phase_search_clear_deviation_counter (bool val = true) { return set_bit (12, val); }

  constexpr bool z_phase_search_clear_real_position_counter (void) const { return get_bit (13); }
  auto_home_search_mode1_t& set_z_phase_clear_real_position_counter (bool val = true) { return set_bit (13, val); }

  constexpr bool z_phase_search_clear_logical_position_counter (void) const { return get_bit (14); }
  auto_home_search_mode1_t& set_z_phase_clear_logical_position_counter (bool val = true) { return set_bit (14, val); }


  // step 4 -- high speed offset drive
  constexpr bool high_speed_offset_drive (void) const { return get_bit (15); }
  auto_home_search_mode1_t& set_high_speed_offset_drive (bool val = true) { return set_bit (15, val); }

private:
  constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
  auto_home_search_mode1_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

  uint16_t m_value;
};

// - - - - - - - - - - - - - - - - - - - - - - - -
//
// FIXME: these values are valid only for the 16 MHz clock
enum dcc_pulse_width_t
{
  dcc_10usec = 0,
  dcc_20usec = 1,
  dcc_100usec = 2,
  dcc_200usec = 3,
  dcc_1msec = 4,
  dcc_2msec = 5,
  dcc_10msec = 6,
  dcc_20msec = 7
};

// FIXME: these values are valid only for the 16 MHz clock
enum inter_step_timer_t
{
  tm_1msec = 0,
  tm_2msec = 1,
  tm_10msec = 2,
  tm_20msec = 3,
  tm_100msec = 4,
  tm_200msec = 5,
  tm_500msec = 6,
  tm_1000msec = 7
};

class auto_home_search_mode2_t
{
public:
  constexpr auto_home_search_mode2_t (void) : m_value (0) { }
  constexpr explicit auto_home_search_mode2_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  // when this is enabled, and when nSTOP1 signal is active and nSTOP2 signal
  // changes to active, the operation of step 3 will stop.
  constexpr bool sand (void) const { return get_bit (0); }
  auto_home_search_mode2_t& set_sand (bool val = true) { return set_bit (0, val); }

  // clear real position counter at the end of automatic home search.
  constexpr bool clear_real_position_counter (void) const { return get_bit (1); }
  auto_home_search_mode2_t& set_clear_real_position_counter (bool val = true) { return set_bit (1, val); }

  // clear logical position counter at the end of automatic home search.
  constexpr bool clear_logical_position_counter (void) const { return get_bit (2); }
  auto_home_search_mode2_t& set_clear_logical_position_counter (bool val = true) { return set_bit (2, val); }

  // assert deviation counter clear output signal after automatic home search.
  constexpr bool clear_deviation_counter (void) const { return get_bit (3); }
  auto_home_search_mode2_t& set_clear_deviation_counter (bool val = true) { return set_bit (3, val); }

  constexpr dcc_pulse_width_t deviation_counter_clear_pulse_width (void) const { return (dcc_pulse_width_t)((m_value >> 4) & 7); }
  auto_home_search_mode2_t& set_deviation_counter_clear_pulse_width (dcc_pulse_width_t val) { m_value = (uint16_t)((m_value & ~(7 << 4)) | (val << 4)); return *this; }

  constexpr bool use_inter_step_timer (void) const { return get_bit (7); }
  auto_home_search_mode2_t& set_use_inter_step_timer (bool val = true) { return set_bit (7, val); }

  constexpr inter_step_timer_t inter_step_timer (void) const { return (inter_step_timer_t)((m_value >> 8) & 7); }
  auto_home_search_mode2_t& set_inter_step_timer (inter_step_timer_t val) { m_value = (uint16_t)((m_value & ~(7 << 8)) | (val << 8)); return *this; }

private:
  constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
  auto_home_search_mode2_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

  uint16_t m_value;
};

enum pio_mode_t
{
  // the pin is used as an input
  general_input = 0,

  // the pin is used as an output
  general_output = 1,

  // the pin is used as a drive status output
  drive_status_output = 2,

  // nPIO7..0 signals become an output state.
  // nPIO3..0 output synchronous pulses and nPIO7..4 output MRm comparison
  // value.  the comparative object and comparison condition can be set
  // by multi-purpose register mode setting (axis_t::set_mr_mode)
  sync_pulse_mr_comp_output = 3
};

class pio_mode1_t
{
public:
  constexpr pio_mode1_t (void) : m_value (0) { }
  constexpr explicit pio_mode1_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  constexpr pio_mode_t pin (unsigned int i) const { return (pio_mode_t)((m_value >> (i*2)) & 3); }
  pio_mode1_t& set_pin (unsigned int i, pio_mode_t val)
  {
    m_value = (m_value & ~(3 << (i*2))) | (val << (i*2));
    return *this;
  }

private:
  uint16_t m_value;
};

// MCX512 has more per-axis IOs.  those additional IOs are configured by
// the pio_mode3 setting.
// pin numbers are 8-13 and the only supported values are
// general_input and general_output.
class pio_mode3_t
{
public:
  constexpr pio_mode3_t (void) : m_value (0) { }
  constexpr explicit pio_mode3_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  constexpr pio_mode_t pin (unsigned int i) const { return (pio_mode_t)((m_value >> (i-8)) & 1); }
  pio_mode3_t& set_pin (unsigned int i, pio_mode_t val)
  {
    m_value = (m_value & ~(1 << (i-8))) | ((val & 1) << (i-8));
    return *this;
  }

private:
  uint16_t m_value;
};

enum const_vector_speed_t
{
  invalid = 0,
  simple_2_axis = 1,
  simple_3_axis = 2,
  high_accuracy_2_axis = 3
};

enum multi_chip_mode_t
{
  single_chip = 0,
  multi_chip_master = 1,
  multi_chip_slave = 2
};

class interpolation_mode_t
{
public:
  constexpr interpolation_mode_t (void) : m_value (0) { }
  constexpr explicit interpolation_mode_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  // specifies which axes should participate in multi-axis interpolation
  // mode driving.  the lowest axis becomes the primary/main axis for the
  // driving parameters.
  constexpr std::bitset<4> axis_use (void) const { return { (unsigned int)m_value & 0x000F }; }
  interpolation_mode_t& set_axis_use (const std::bitset<4>& val)
  {
    m_value = (m_value & 0xFFF0) | (uint16_t)val.to_ulong ();
    return *this;
  }

  // set bit of axis number
  interpolation_mode_t& set_axis_use (unsigned int axis_bit_num)
  {
    m_value = m_value | (1 << (axis_bit_num & 3));
    return *this;
  }

  bool circular_axis_exchange (void) const { return get_bit (4); }
  interpolation_mode_t& set_circular_axis_exchange (bool val = true) { return set_bit (4, val); }

  const_vector_speed_t const_vector_speed_mode (void) const { return (const_vector_speed_t)((m_value >> 6) & 3); }
  interpolation_mode_t& set_const_vector_speed_mode (const_vector_speed_t val)
  {
    m_value = (m_value & ~(3 << 6)) | (val << 6);
    return *this;
  }

  // short axis pulse equalization is an 8x oversampling mode.
  // because of that some limitations apply:
  // - max drive speed = 1/8
  // - max inital speed = 1/8
  // - max accel/decel = 1/8
  // - max finish point = 1/8
  // - max arc center point = 1/8
  // - no s-curve accel/decel
  // - no multichip interpolation
  // - no single step interpolation
  // - no bit pattern interpolation
  // - no continuous interpolation driving
  // - no comparing operation of current drive speed using multi-purpose
  //   register
  // - no synchronous action to set current speed of driving and accel/decel
  //   to a mult-purpose register
  bool short_axis_pulse_equalization (void) const { return get_bit (8); }
  interpolation_mode_t& set_short_axis_pulse_equalization (bool val = true) { return set_bit (8, val); }

  bool single_step_mode (void) const { return get_bit (9); }
  interpolation_mode_t& set_single_step_mode (bool val = true) { return set_bit (9, val); }

  multi_chip_mode_t multi_chip_mode (void) const { return (multi_chip_mode_t)((m_value >> 10) & 3); }
  interpolation_mode_t& set_multi_chip_mode (multi_chip_mode_t val)
  {
    m_value = (m_value & ~(3 << 10)) | (val << 10);
    return *this;
  }

  // when enabled, determine the maximum finishing point value automatically
  // during interpolation.
  // when disabled, the value set by axis::set_interp_finish_point_max_value
  // is used.
  bool auto_finish_point_max_value (void) const { return get_bit (12); }
  interpolation_mode_t& set_auto_finish_point_max_value (bool val = true) { return set_bit (12, !val); }

  // when enabled, a continuous interpolation interrupt is issued when the
  // number of entries in the position buffer drops from 4 to 3.
  bool cont_interp_interrupt_4_3 (void) const { return get_bit (14); }
  interpolation_mode_t& set_cont_interp_interrupt_4_3 (bool val = true) { return set_bit (14, val); }

  // when enabled, a continuous interpolation interrupt is issued when the
  // number of entries in the position buffer drops from 8 to 7.
  bool cont_interp_interrupt_8_7 (void) const { return get_bit (15); }
  interpolation_mode_t& set_cont_interp_interrupt_8_7 (bool val = true) { return set_bit (15, val); }

private:
  constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
  interpolation_mode_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

  uint16_t m_value;
};


// - - - - - - - - - - - - - - - - - - - - - - - -

struct input_signal_filter_t
{
public:
  constexpr input_signal_filter_t (void) : m_value (0) { }
  constexpr explicit input_signal_filter_t (uint16_t val) : m_value (val) { }

  constexpr uint16_t value (void) const { return m_value; }

  // the filter time constants in the hardware manual are given in
  // nano / microseconds for a 16 MHz reference clock.
  // 16 MHz = 62.5 ns.  the smallest possible value is 500 ns,
  // 500 / 62.5 = 8 clocks.

  // FIXME (MCB-115):
  // add functions to convert from and to std::chrono::duration values.

  // sensor input signals and encoder input signals have two different
  // filter time settings.

  unsigned int signals_filter_time (void) const { return (m_value >> 8) & 15; }
  input_signal_filter_t& set_signals_filter_time (utils::clamped_value<unsigned int, 0, 15> val)
  {
    m_value = (m_value & 0xF0FF) | (val << 8);
    return *this;
  }

  // EMG bit is present on all axes, but is only valid for x axis since there
  // is only one EMG input pin.
  bool emg_filter_enable (void) const { return get_bit (0); }
  input_signal_filter_t& set_emg_filter_enable (bool val = true) { return set_bit (0, val); }

  bool limitp_limitn_filter_enable (void) const { return get_bit (1); }
  input_signal_filter_t& set_limitp_limitn_filter_enable (bool val = true) { return set_bit (1, val); }

  bool stop0_stop1_filter_enable (void) const { return get_bit (2); }
  input_signal_filter_t& set_stop0_stop1_filter_enable (bool val = true) { return set_bit (2, val); }

  bool inpos_alarm_filter_enable (void) const { return get_bit (3); }
  input_signal_filter_t& set_inpos_alarm_filter_enable (bool val = true) { return set_bit (3, val); }

  bool pio0123_filter_enable (void) const { return get_bit (4); }
  input_signal_filter_t& set_pio0123_filter_enable (bool val = true) { return set_bit (4, val); }

  bool pio4567_filter_enable (void) const { return get_bit (5); }
  input_signal_filter_t& set_pio4567_filter_enable (bool val = true) { return set_bit (5, val); }

  // ------------

  unsigned int encoder_filter_time (void) const { return (m_value >> 12) & 15; }
  input_signal_filter_t& set_encoder_filter_time (utils::clamped_value<unsigned int, 0, 15> val)
  {
    m_value = (m_value & 0x0FFF) | (val << 12);
    return *this;
  }

  bool stop2_filter_enable (void) const { return get_bit (6); }
  input_signal_filter_t& set_stop2_filter_enable (bool val = true) { return set_bit (6, val); }

  bool eca_ecb_filter_enable (void) const { return get_bit (7); }
  input_signal_filter_t& set_eca_ecb_filter_enable (bool val = true) { return set_bit (7, val); }

private:
  constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
  input_signal_filter_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

  uint16_t m_value;
};

// - - - - - - - - - - - - - - - - - - - - - - - -

struct split_pulse_shape_t
{
  constexpr split_pulse_shape_t (void) : m_length (2), m_width (1) { }
  constexpr split_pulse_shape_t (const split_pulse_shape_t& rhs) = default;

  constexpr split_pulse_shape_t (utils::clamped_value<uint16_t, 2, 65535> l,
			   utils::clamped_value<uint16_t, 1, 65534> w)
  : m_length (l), m_width (w) { }

  constexpr split_pulse_shape_t (uint32_t raw_val)
  : m_length ((uint16_t)raw_val), m_width ((uint16_t)(raw_val >> 16)) { }

  constexpr uint32_t value (void) const { return (m_width << 16) | m_length; }

  // length of a split pulse cycle in drive pulses
  constexpr uint16_t length (void) const { return m_length; }

  // width of the pulse in the pulse cycle in drive pulses
  constexpr uint16_t width (void) const { return m_width; }


private:
  uint16_t m_length;
  uint16_t m_width;
};

// - - - - - - - - - - - - - - - - - - - - - - - -
// an MCX synchronous action set consists of an trigger ("activation factor" in
// the hardware manual) and an action.
//
// each axis has 4 sync sets, but not all of them have the same
// capabilities.  the 4 multi-purpose registers (MR) are hardwired to the
// sync sets as well as some of the external trigger signals/pins numbers.
// hence use different types for each sync set.

enum sync_common_trigger_t
{
  // no operation
  // nop = 0x00,  don't use nop to avoid enum name conflicts
  no_trigger = 0x00,

  // timer expired
  timer_expired = 0x02,

  // axis driving started
  drive_started = 0x03,

  // axis driving entered constant speed area
  drive_started_constant_speed = 0x04,

  // axis driving left constant speed area
  drive_ended_constant_speed = 0x05,

  // axis driving stopped
  drive_stopped = 0x06,

  // split pulse output started
  split_pulse_started = 0x07,

  // split pulse output stopped
  split_pulse_ended = 0x08,

  // split pulse output (logical off -> on edge)
  split_pulse_output = 0x09,
};

// MR compare-match trigger is normally set along with comparison
// code and operand.
enum sync_common_mr_comp_match_trigger_t
{
  // MR compare match condition changed from false to true
  mr_comp_match = 0x01,
};


enum sync_common_action_t
{
  // no operation
//  nop = 0x00,  avoid enum name conflicts
  no_action = 0x00,

  // MRm .. m is hardwired to the sync action set (0,1,2,3)

  // load MRm -> drive speed (DV)
  load_mr_to_drive_speed = 0x01,

  // load MRm -> drive pulse number (TP)
  load_mr_to_drive_pulse_number = 0x02,

  // load MRm -> split pulse data 1 (split pulse shape)
  load_mr_to_split_pulse_shape = 0x03,


  // save logical position counter (LP) to MRm
  save_logical_position_counter_to_mr = 0x05,

  // save real position counter (RP) to MRm
  save_real_position_counter_to_mr = 0x06,

  // save current timer counter to MRm
  save_timer_counter_to_mr = 0x07,

  drive_relative = 0x0A,
  drive_relative_rev = 0x0B,

  drive_absolute = 0x0C,
  drive_pos_dir_pulse_continuous = 0x0D,
  drive_neg_dir_pulse_continuous = 0x0E,

  // relative position driving by drive pulse number of MRm value
  drive_relative_mr_pulse_count = 0x0F,

  // absolute drive to the finish point of MRm value
  drive_abolute_to_finish_point = 0x10,

  drive_decel_stop = 0x11,
  drive_stop = 0x12,
  speed_inc = 0x13,
  speed_dec = 0x14,
  start_timer = 0x15,
  stop_timer = 0x16,
  start_split_pulse = 0x17,
  stop_split_pulse = 0x18
};


enum sync0_trigger_t
{
  pio0_rising_edge = 0x0A,
  pio0_falling_edge = 0x0B,

  // AB phase type of trigger input
  pio0_rising_edge_pio4_low = 0x0C,
  pio0_rising_edge_pio4_high = 0x0D,
  pio0_falling_edge_pio4_low = 0x0E,
  pio0_falling_edge_pio4_high = 0x0F
};

enum sync0_action_t
{
  // load MRm -> logical position counter (LP)
  load_mr_to_logical_position_counter = 0x04,

  // save current drive speed to MRm
  save_current_drive_speed_to_mr = 0x08,

  // synchronous pulse output on PIO0
  // for this, PIO mode needs to be set to sync pulse output mode.
  pio0_pulse_output = 0x09,
};

enum sync1_trigger_t
{
  pio1_rising_edge = 0x0A,
  pio1_falling_edge = 0x0B,

  // AB phase type of trigger input
  pio1_rising_edge_pio5_low = 0x0C,
  pio1_rising_edge_pio5_high = 0x0D,
  pio1_falling_edge_pio5_low = 0x0E,
  pio1_falling_edge_pio5_high = 0x0F
};

enum sync1_action_t
{
  // load MRm -> real position counter (RP)
  load_mr_to_real_position_counter = 0x04,

  // save current acceleration/deceleration value to MRm
  save_current_accel_decel_to_mr = 0x08,

  // synchronous pulse output on PIO1
  // for this, PIO mode needs to be set to sync pulse output mode.
  pio1_pulse_output = 0x09,
};

enum sync2_trigger_t
{
  pio2_rising_edge = 0x0A,
  pio2_falling_edge = 0x0B,

  // AB phase type of trigger input
  pio2_rising_edge_pio6_low = 0x0C,
  pio2_rising_edge_pio6_high = 0x0D,
  pio2_falling_edge_pio6_low = 0x0E,
  pio2_falling_edge_pio6_high = 0x0F
};

enum sync2_action_t
{
  // load MRm -> initial speed (SV)
  load_mr_to_initial_speed = 0x04,

  // synchronous pulse output on PIO2
  // for this, PIO mode needs to be set to sync pulse output mode.
  pio2_pulse_output = 0x09,
};

enum sync3_trigger_t
{
  pio3_rising_edge = 0x0A,
  pio3_falling_edge = 0x0B,

  // AB phase type of trigger input
  pio3_rising_edge_pio7_low = 0x0C,
  pio3_rising_edge_pio5_high = 0x0D,
  pio3_falling_edge_pio5_low = 0x0E,
  pio3_falling_edge_pio5_high = 0x0F
};

enum sync3_action_t
{
  // load MRm -> acceleration (AC)
  load_mr_to_acceleration = 0x04,

  // synchronous pulse output on PIO3
  // for this, PIO mode needs to be set to sync pulse output mode.
  pio3_pulse_output = 0x09,
};

template <unsigned int SyncNum> struct sync_set_specific;

template<> struct sync_set_specific<0>
{
  using trigger_t = sync0_trigger_t;
  using action_t = sync0_action_t;
};

template<> struct sync_set_specific<1>
{
  using trigger_t = sync1_trigger_t;
  using action_t = sync1_action_t;
};

template<> struct sync_set_specific<2>
{
  using trigger_t = sync2_trigger_t;
  using action_t = sync2_action_t;
};

template<> struct sync_set_specific<3>
{
  using trigger_t = sync3_trigger_t;
  using action_t = sync3_action_t;
};



enum mr_comp_code_t
{
  // operand >= MR
  greater_equal = 0b00,

  // operand > MR
  greater = 0b01,

  // operand == MR
  equal = 0b10,

  // operand < MR
  less = 0b11
};

enum mr_comp_op_t
{
  //logical_position_counter = 0b00,   // -> use position_counter_t enum overload
  //real_position_counter = 0b01,      // -> use position_counter_t enum overload
  current_drive_speed_value = 0b10,
  current_timer_value = 0b11
};


// - - - - - - - - - - - - - - - - - - - - - - - -

enum axis_num
{
  axis_x = 0,
  axis_y = 1,
  axis_z = 2,
  axis_u = 3
};


template <typename BusInterface, typename ClockHz,
	  typename INT0N_InterruptLine,
	  typename INT1N_InterruptLine,
	  typename AxisInputs, typename AxisOutputs,
	  typename GetAxesFunc>
class mcx51x_hw_inst;

// ----------------------------------------------------------------------------
// MCX512 and MCX514 common per-axis functions

template <typename HwInst, typename ClockHzRatio, unsigned int MaxAxisNum>
class mcx51x_axis
{
public:
  using clock_hz = ClockHzRatio;

  constexpr unsigned int num (void) const { return m_num; }
  auto& container (void) const { return m_container; }

  auto&& inputs (void) { auto&& c = container (); return c.m_axis_inputs (c, *this); }
  auto&& outputs (void) { auto&& c = container (); return c.m_axis_outputs (c, *this); }

  // axis status
  drive_status_t drive_status (void) const { return drive_status_t (const_cast<mcx51x_axis*> (this)->rd_cmd16 (0x1F, 2)); }

  signal_status_t signal_status (void) const
  {
    auto raw_val = const_cast<mcx51x_axis*> (this)->rd_cmd16 (0x7A, 3);

    // invert signals based on mode2 and mode3 settings.
    auto tmp = raw_val ^ m_rr3_inv_mask;

    // swap bits for encoder and limit signals based on mode3 settings.
    tmp = ((tmp & m_rr3_swap_r_mask) >> 1) | ((tmp & m_rr3_swap_l_mask) << 1)
	  | (tmp & ~(m_rr3_swap_r_mask | m_rr3_swap_l_mask));

    return signal_status_t (tmp);
  }

  misc_status_t misc_status (void) const { return misc_status_t (const_cast<mcx51x_axis*> (this)->rd_cmd16 (0x7B, 3)); }

  // mode register 1 (WR1)
  // per-axis interrupt mask/enable control.
  // handled automatically by the driver when the respective function is
  // set or cleared (set to nullptr).


  // mode register 2 (WR2) setting
  // FIXME: use cached register
  void set_mode2 (mode2_t val)
  {
    wr_axis_reg16 (0x1F, 2, val.value ());

    m_rr3_inv_mask =   (m_rr3_inv_mask & 0b11000) // preserve ECA,ECB bits which are set by mode3
			| ((unsigned int)(val.stop0_logic () == 0) << 0)
			| ((unsigned int)(val.stop1_logic () == 0) << 1)
			| ((unsigned int)(val.stop2_logic () == 0) << 2)
			| ((unsigned int)(val.in_position_logic () == 0) << 5)
			| ((unsigned int)(val.alarm_logic () == 0) << 6)
			| ((unsigned int)(val.hw_limit_logic () == 0) << 7)
			| ((unsigned int)(val.hw_limit_logic () == 0)  << 8);
  }

  mode2_t mode2 (void) const			{ return mode2_t (rd_cmd16 (0x3E)); }

  // mode register 3 (WR3) setting
  // FIXME: use cached register
  void set_mode3 (mode3_t val)
  {
    wr_axis_reg16 (0x1F, 3, val.value ());

    m_rr3_inv_mask =    (m_rr3_inv_mask & 0b111100111) // preserve bits which are set by mode2
			| ((unsigned int)(val.encoder_input_logic () == 0) << 3)
			| ((unsigned int)(val.encoder_input_logic () == 0) << 4);

    m_rr3_swap_r_mask =   (val.swap_encoder_input_signals ()  ? 0b000010000 : 0b000000000)
			  | (val.swap_hw_limit_input_signals () ? 0b100000000 : 0b000000000);

    m_rr3_swap_l_mask = m_rr3_swap_r_mask >> 1;
  }


  mode3_t mode3 (void) const			{ return mode3_t (rd_cmd16 (0x3F)); }


  // driving
  bool is_driving (void) const { return container ().status ().axis_driving ()[num ()]; }

  // start relative driving
  void drive_relative (void)										{ wr_cmd (0x50); }

  // start counter relative position driving
  void drive_counter_relative (void)									{ wr_cmd (0x51); }

  // + direction continuous pulse driving
  void drive_pos_dir_pulse_continuous (void)								{ wr_cmd (0x52); }

  // - direction continuous pulse driving
  void drive_neg_dir_pulse_continuous (void)								{ wr_cmd (0x53); }

  // absolute position driving
  void drive_absolute (void)										{ wr_cmd (0x54); }

  // stop with deceleration
  void drive_decel_stop (void)										{ wr_cmd (0x56); }

  // stop immediately
  void drive_stop (void)										{ wr_cmd (0x57); }

  // direction signal + setting
  // (sets the direction signal output in 1-pulse 1-direction pulse output mode)
  void set_pos_dir (void)										{ wr_cmd (0x58); }

  // direction signal - setting
  // (sets the direction signal output in 1-pulse 1-direction pulse output mode)
  void set_neg_dir (void)										{ wr_cmd (0x59); }

  // automatic home search
  void drive_auto_home_search (void)									{ wr_cmd (0x5A); }

  // increase speed by the current value of the speed incdec setting.
  void speed_inc (void)											{ wr_cmd (0x70); }

  // decrease speed by the current value of the speed incdec setting.
  void speed_dec (void)											{ wr_cmd (0x71); }

  // output deviation counter clear pulses from the nDCC output pin.
  void clear_deviation_counter (void)									{ wr_cmd (0x72); }

  // timer control
  void start_timer (void)										{ wr_cmd (0x73); }
  void stop_timer (void)										{ wr_cmd (0x74); }

  // split pulse control
  void start_split_pulse (void)										{ wr_cmd (0x75); }
  void stop_split_pulse (void)										{ wr_cmd (0x76); }

  // hold driving start until it is released
  void hold_drive (void)										{ wr_cmd (0x77); }

  void release_hold_drive (void)                                                                        { wr_cmd (0x78); }

  // maximum finish point clear
  void clear_maximum_finish_point (void)								{ wr_cmd (0x7C); }

  // error / finishing status clear
  void clear_finish_status (void)									{ wr_cmd (0x79); }


  // axis parameters

  // jerk setting [pps/sec²] (acceleration increasing rate).
  static constexpr int32_t jerk_min_value = 1;
  static constexpr int32_t jerk_max_value = 1073741823;
  void set_jerk (utils::clamped_value<int32_t, jerk_min_value, jerk_max_value> val)			{ wr_cmd32 (0x00, val); }

  static constexpr int32_t jerk_min_value_pps = native_to_pps3<clock_hz> (jerk_min_value);
  static constexpr int32_t jerk_max_value_pps = native_to_pps3<clock_hz> (jerk_max_value);
  void set_jerk_pps (utils::clamped_value<int32_t, jerk_min_value_pps, jerk_max_value_pps> val)
  {
    set_jerk (pps3_to_native<clock_hz, jerk_max_value_pps> (val));
  }

  // deceleration increasing rate setting [pps/sec²]
  static constexpr int32_t decel_inc_rate_min_value = 1;
  static constexpr int32_t decel_inc_rate_max_value = 1073741823;
  void set_decel_inc_rate (utils::clamped_value<int32_t, decel_inc_rate_min_value, decel_inc_rate_max_value> val)	{ wr_cmd32 (0x01, val); }

  static constexpr int32_t decel_inc_rate_min_value_pps = native_to_pps3<clock_hz> (decel_inc_rate_min_value);
  static constexpr int32_t decel_inc_rate_max_value_pps = native_to_pps3<clock_hz> (decel_inc_rate_max_value);
  void set_decel_inc_rate_pps (utils::clamped_value<int32_t, decel_inc_rate_min_value_pps, decel_inc_rate_max_value_pps> val)
  {
    set_decel_inc_rate (pps3_to_native<clock_hz, decel_inc_rate_max_value_pps> (val));
  }

  // acceleration setting [pps/sec]
  // - when using short axis equalization, the maximum is 1/8.
  static constexpr int32_t accel_min_value = 1;
  static constexpr int32_t accel_max_value = 536870911;
  void set_accel (utils::clamped_value<int32_t, accel_min_value, accel_max_value> val)			{ wr_cmd32 (0x02, val); }
  int32_t accel (void) const										{ return rd_cmd32 (0x43); }

  static constexpr int32_t accel_min_value_pps = native_to_pps2<clock_hz> (accel_min_value);
  static constexpr int32_t accel_max_value_pps = native_to_pps2<clock_hz> (accel_max_value);
  void set_accel_pps (utils::clamped_value<int32_t, accel_min_value_pps, accel_max_value_pps> val)
  {
    set_accel (pps2_to_native<clock_hz, accel_max_value_pps> (val));
  }

  int32_t accel_pps (void) const
  {
    return native_to_pps2<clock_hz, accel_max_value> (accel ());
  }

  // deceleration setting [pps/sec]
  // - when using short axis equalization, the maximum is 1/8.
  static constexpr int32_t decel_min_value = 1;
  static constexpr int32_t decel_max_value = 536870911;
  void set_decel (utils::clamped_value<int32_t, decel_min_value, decel_max_value> val)			{ wr_cmd32 (0x03, val); }

  static constexpr int32_t decel_min_value_pps = native_to_pps2<clock_hz> (decel_min_value);
  static constexpr int32_t decel_max_value_pps = native_to_pps2<clock_hz> (decel_max_value);
  void set_decel_pps (utils::clamped_value<int32_t, decel_min_value_pps, decel_max_value_pps> val)
  {
    set_decel (pps2_to_native<clock_hz, decel_max_value_pps> (val));
  }

  // initial speed setting [pps]
  // - when using short axis equalization, the maximum is 1/8.
  static constexpr int32_t initial_speed_min_value = 1;
  static constexpr int32_t initial_speed_max_value = 8'000'000;
  void set_initial_speed (utils::clamped_value<int32_t, initial_speed_min_value, initial_speed_max_value> val)	{ wr_cmd32 (0x04, val); }

  static constexpr int32_t initial_speed_min_value_pps = native_to_pps<clock_hz> (initial_speed_min_value);
  static constexpr int32_t initial_speed_max_value_pps = native_to_pps<clock_hz> (initial_speed_max_value);
  void set_initial_speed_pps (utils::clamped_value<int32_t, initial_speed_min_value_pps, initial_speed_max_value_pps> val)
  {
    set_initial_speed (pps_to_native<clock_hz, initial_speed_max_value_pps> (val));
  }

  int32_t initial_speed (void) const									{ return rd_cmd32 (0x44); }
  int32_t initial_speed_pps (void) const
  {
    return native_to_pps<clock_hz, initial_speed_max_value> (initial_speed ());
  }

  // drive speed setting [pps]
  // - when using short axis equalization, the maximum is 1/8.
  // - when using continuous interpolation mode, the maximum is 1/2 and
  //   the speed is constant (see datasheet 3.7.4)
  static constexpr int32_t drive_speed_min_value = 1;
  static constexpr int32_t drive_speed_max_value = 8'000'000;
  void set_drive_speed (utils::clamped_value<int32_t, drive_speed_min_value, drive_speed_max_value> val)	{ wr_cmd32 (0x05, val); }

  static constexpr int32_t drive_speed_min_value_pps = native_to_pps<clock_hz> (drive_speed_min_value);
  static constexpr int32_t drive_speed_max_value_pps = native_to_pps<clock_hz> (drive_speed_max_value);
  void set_drive_speed_pps (utils::clamped_value<int32_t, drive_speed_min_value_pps, drive_speed_max_value_pps> val)
  {
    set_drive_speed (pps_to_native<clock_hz, drive_speed_max_value_pps> (val));
  }

  int32_t drive_speed (void) const									{ return rd_cmd32 (0x45); }
  int32_t drive_speed_pps (void) const
  {
    return native_to_pps<clock_hz, drive_speed_max_value> (drive_speed ());
  }

  int32_t current_drive_speed (void) const								{ return rd_cmd32 (0x32); }
  int32_t current_drive_speed_pps (void) const
  {
    return native_to_pps<clock_hz, drive_speed_max_value> (current_drive_speed ());
  }

  // drive pulse number / finish point setting
  // - when using short axis equalization, the maximum is 1/8.
  static constexpr int32_t drive_pulse_number_min_value = -2147483646;
  static constexpr int32_t drive_pulse_number_max_value = +2147483646;
  void set_drive_pulse_number (utils::clamped_value<int32_t, drive_pulse_number_min_value, drive_pulse_number_max_value> val)	{ wr_cmd32 (0x06, val); }
  int32_t drive_pulse_number (void) const								{ return rd_cmd32 (0x46); }

  // in bitpattern interpolation mode, each axis gets its bitstream through
  // command 0x06.  the bits for positive and negative directions are mutually
  // exclusive.  if both bits are set, it means end-of-interpolation data.
  // the order of the bits used for driving is LSB to MSB.
  void set_bitpattern_interpolation_data (uint16_t pos_pulse_bits, uint16_t neg_pulse_bits)
  {
    wr_cmd32 (0x06, pos_pulse_bits | (neg_pulse_bits << 16));
  }

  // manual deceleration point setting
  static constexpr uint32_t manual_deceleration_point_min_value = 0;
  static constexpr uint32_t manual_deceleration_point_max_value = 4294967292;
  void set_manual_deceleration_point (utils::clamped_value<uint32_t, manual_deceleration_point_min_value, manual_deceleration_point_max_value> val)		{ wr_cmd32 (0x07, val); }

  // circular center point setting
  // when using short axis equalization, the maximum is 1/8.
  static constexpr int32_t circular_center_point_min_value = -1073741823;
  static constexpr int32_t circular_center_point_max_value = +1073741823;
  void set_circular_center_point (utils::clamped_value<int32_t, circular_center_point_min_value, circular_center_point_max_value> val)		{ wr_cmd32 (0x08, val); }

  // logical position counter setting
  static constexpr int32_t logical_position_counter_min_value = -2147483648;
  static constexpr int32_t logical_position_counter_max_value = +2147483647;
  void set_logical_position_counter (utils::clamped_value<int32_t, logical_position_counter_min_value, logical_position_counter_max_value> val)	{ wr_cmd32 (0x09, val); }
  int32_t logical_position_counter (void) const								{ return rd_cmd32 (0x30); }

  // real position counter setting
  static constexpr int32_t real_position_counter_min_value = -2147483648;
  static constexpr int32_t real_position_counter_max_value = +2147483647;
  void set_real_position_counter (utils::clamped_value<int32_t, real_position_counter_min_value, real_position_counter_max_value> val)		{ wr_cmd32 (0x0A, val); }
  int32_t real_position_counter (void) const								{ return rd_cmd32 (0x31); }

  static constexpr int32_t software_limit_min_value = -2147483648;
  static constexpr int32_t software_limit_max_value = +2147483647;

  // software limit + setting
  void set_software_limit_max (utils::clamped_value<int32_t, software_limit_min_value, software_limit_max_value> val)		{ wr_cmd32 (0x0B, val); }

  // software limit - setting
  void set_software_limit_min (utils::clamped_value<int32_t, software_limit_min_value, software_limit_max_value> val)		{ wr_cmd32 (0x0C, val); }

  // acceleration counter offsetting
  static constexpr int16_t accel_count_offset_min_value = -32768;
  static constexpr int16_t accel_count_offset_max_value = +32767;
  void set_accel_count_offset (utils::clamped_value<int16_t, accel_count_offset_min_value, accel_count_offset_max_value> val)			{ wr_cmd16 (0x0D, val); }

  // logical position counter maximum value setting (wrap-around point)
  void set_max_logical_position_counter (utils::clamped_value<int32_t, 1, 2147483647> val)		{ wr_cmd32 (0x0E, val); }
  void disable_max_logical_position_counter (void)							{ wr_cmd32 (0x0E, 0xFFFFFFFF); }

  // real position counter maximum value setting (wrap-around point)
  void set_max_position_counter (utils::clamped_value<int32_t, 1, 2147483647>	val)			{ wr_cmd32 (0x0F, val); }
  void disable_max_position_counter (void)								{ wr_cmd32 (0x0F, 0xFFFFFFFF); }


  // home search speed setting [pps]
  // same limitations as for regular drive speed
  void set_home_search_speed (utils::clamped_value<int32_t, drive_speed_min_value, drive_speed_max_value> val)		{ wr_cmd32 (0x14, val); }
  void set_home_search_speed_pps (utils::clamped_value<int32_t, drive_speed_min_value_pps, drive_speed_max_value_pps> val)
  {
    set_home_search_speed (pps_to_native<clock_hz, drive_speed_max_value_pps> (val));
  }

  // speed increasing/decreasing value setting [pps]
  static constexpr int32_t speed_incdec_min_value = 1;
  static constexpr int32_t speed_incdec_max_value = 1000000;
  void set_speed_incdec (utils::clamped_value<int32_t, speed_incdec_min_value, speed_incdec_max_value> val)	{ wr_cmd32 (0x15, val); }

  static constexpr int32_t speed_incdec_min_value_pps = native_to_pps<clock_hz> (speed_incdec_min_value);
  static constexpr int32_t speed_incdec_max_value_pps = native_to_pps<clock_hz> (speed_incdec_max_value);
  void set_speed_incdec_pps (utils::clamped_value<int32_t, speed_incdec_min_value_pps, speed_incdec_max_value_pps> val)
  {
    set_speed_incdec (pps_to_native<clock_hz, speed_incdec_max_value_pps> (val));
  }

  // timer value setting
  // time value is in 16 MCX clocks units (1 usec if 16 MHz clk)
  void set_timer_value (utils::clamped_value<int32_t, 1, 2147483647> val)				{ wr_cmd32 (0x16, val); }
  int32_t timer_value (void) const									{ return rd_cmd32 (0x38); }

  using timer_duration = std::chrono::duration<int32_t, std::ratio_divide <std::ratio<16,1>, clock_hz>>;
  using timer_rep = typename timer_duration::rep;
  using timer_period = typename timer_duration::period;
  using timer_time_point = std::chrono::time_point<mcx51x_axis, timer_duration>;

  void set_timer (timer_duration val)
  {
    set_timer_value (val.count ());
  }

  timer_duration timer (void) const
  {
    return timer_duration (timer_value ());
  }

  // split pulse setting 1
  // it is OK to change the pulse shape while the split pulse signal is being output.
  // FIXME MCB-136: use cached register
  void set_split_pulse_shape (split_pulse_shape_t val) { wr_cmd32 (0x17, val.value ()); }
  split_pulse_shape_t split_pulse_shape (void) const { return split_pulse_shape_t (rd_cmd32 (0x47)); }

  // split pulse setting 2
  // a counter of 0 means to output infinite number of split pulses, until
  // the split pulse output is stopped manually.
  // FIXME MCB-136: this value can't be read-back from the chip.  it has to be
  //                cached in RAM.

  static constexpr uint16_t split_pulse_count_min_value = 0;
  static constexpr uint16_t split_pulse_count_max_value = 65535;
  void set_split_pulse_count (utils::clamped_value<uint16_t, split_pulse_count_min_value, split_pulse_count_max_value> val)				{ wr_cmd16 (0x18, val); }

  // interpolation / finish point maximum value setting
  void set_interp_finish_point_max_value (utils::clamped_value<int32_t, 1, 2147483646> val)		{ wr_cmd32 (0x19, val); }
  int32_t interp_finish_point_max_value (void) const							{ return rd_cmd32 (0x39); }

  // PIO signal setting 1
  // FIXME MCB-136: use cached register
  void set_pio1 (pio_mode1_t val)									{ wr_cmd16 (0x21, val.value ()); }
  pio_mode1_t pio1 (void) const										{ return pio_mode1_t (rd_cmd16 (0x41)); }

  // PIO signal setting 2 / other settings
  void set_pio2 (uint16_t val)										{ wr_cmd16 (0x22, val); }
  uint16_t pio2 (void) const										{ return rd_cmd16 (0x42); }

  // PIO inputs/outputs are handled differently on MCX512 and MCX514.
  // see subclasses

  // automatic home search mode setting 1
  // FIXME MCB-136: use cached register
  void set_auto_home_search_mode1 (auto_home_search_mode1_t val)					{ wr_cmd16 (0x23, val.value ()); }

  // automatic home search mode setting 2
  // FIXME MCB-136: use cached register
  void set_auto_home_search_mode2 (auto_home_search_mode2_t val)					{ wr_cmd16 (0x24, val.value ()); }

  // input signal filter mode setting
  // FIXME MCB-136: use cached register
  void set_input_signal_filter_mode (input_signal_filter_t val)						{ wr_cmd16 (0x25, val.value ()); }

  template <unsigned int SyncNum>
  struct sync_action_set
  {
    sync_action_set (void) = delete;
    sync_action_set (const sync_action_set&) = delete;
    sync_action_set& operator = (const sync_action_set&) = delete;

    sync_action_set& set_trigger (sync_common_trigger_t val) { return set_trigger_1 (val); }
    sync_action_set& set_trigger (typename sync_set_specific<SyncNum>::trigger_t val) { return set_trigger_1 (val); }

    sync_action_set& set_trigger (sync_common_mr_comp_match_trigger_t val,
				  mr_comp_code_t comp_code,
				  mr_comp_op_t comp_operand)
    {
      return set_trigger_1 (val, comp_code, comp_operand);
    }

    sync_action_set& set_trigger (sync_common_mr_comp_match_trigger_t val,
				  mr_comp_code_t comp_code,
				  position_counter_t comp_operand)
    {
      return set_trigger_1 (val, comp_code, comp_operand);
    }

    // set a function that should be executed when this set gets triggered.
    // it will be inside an interrupt context.
    template <typename F>
    sync_action_set& set_trigger_func (F&& f)
    {
      m_axis.set_irq_func (irq_sync0 + this_sync_num, std::move (f));
      return *this;
    }

    sync_action_set& set_action (sync_common_action_t val) { return set_action_1 (val); }
    sync_action_set& set_action (typename sync_set_specific<SyncNum>::action_t val) { return set_action_1 (val); }

    // link another sync set on the same axis, which will be triggered along
    // with this sync set.
    sync_action_set& clear_action_link_sync_set (void)
    {
      m_sync_reg &= 0b1111'0001'1111'1111;
      return *this;
    }

    sync_action_set& set_action_link_sync_set (unsigned int sync_num, bool en = true)
    {
      if (this_sync_num != sync_num)
      {
	m_sync_reg = utils::set_bit (m_sync_reg, ((sync_num - this_sync_num) & 3) - 1 + 9, en);
	write_sync_reg ();
      }
      return *this;
    }

    // link sync set 0 on another axis, which will be triggered along with this
    // sync set.
    sync_action_set& clear_action_link_other_axis_sync0_set (void)
    {
      m_sync_reg &= 0b1000'1111'1111'1111;
      write_sync_reg ();
      return *this;
    }

    sync_action_set& set_action_link_other_axis_sync0_set (unsigned int axis_num, bool en = true)
    {
      axis_num &= MaxAxisNum;
      const unsigned int this_axis_num = m_axis.num ();
      if (this_axis_num != axis_num)
      {
	m_sync_reg = utils::set_bit (m_sync_reg, ((axis_num - this_axis_num) & MaxAxisNum) - 1 + 12, en);
	write_sync_reg ();
      }
      return *this;
    }

    // if set, the action will be repeated.  otherwise it's just a one-shot action.
    sync_action_set& set_repeat (bool val = true)
    {
      m_sync_reg = utils::set_bit (m_sync_reg, 15, val);
      write_sync_reg ();
      return *this;
    }

    sync_action_set& set_one_shot (bool val = true) { return set_repeat (!val); }


    sync_action_set& set_mr (int32_t val)
    {
      m_axis.set_mr (this_sync_num, val);
      return *this;
    }

    int32_t mr (void)
    {
      return m_axis.mr (this_sync_num);
    }

    sync_action_set& enable (void)
    {
      m_axis.wr_cmd (0x80 | (1 << this_sync_num));
      return *this;
    }

    sync_action_set& disable (void)
    {
      m_axis.wr_cmd (0x90 | (1 << this_sync_num));
      return *this;
    }

    sync_action_set& trigger (void)
    {
      m_axis.wr_cmd (0xA0 | (1 << this_sync_num));
      return *this;
    }

  private:
    static constexpr unsigned int this_sync_num = SyncNum;

    mcx51x_axis& m_axis;
    uint16_t m_sync_reg = 0;

    // not useful at the moment.  could be useful when using the sync_action_set
    // as an rvalue to batch setter calls and minimize register access.
    //uint16_t m_mr_mode = 0;  // only the bits relevant for this sync set.
			       // other bits are zero.

    constexpr sync_action_set (mcx51x_axis& axis) : m_axis (axis) { }

    auto& set_trigger_1 (unsigned int val)
    {
      m_sync_reg = (m_sync_reg & ~(0b1111 << 0)) | val;
      write_sync_reg ();
      return *this;
    }

    auto& set_trigger_1 (sync_common_mr_comp_match_trigger_t val,
			 unsigned int comp_code,
			 unsigned int comp_operand)
    {
      m_axis.set_mr_compare_match_1 (this_sync_num, comp_code, comp_operand);
      return set_trigger_1 (val);
    }

    auto& set_action_1 (unsigned int val)
    {
      m_sync_reg = (m_sync_reg & ~(0b11111 << 4)) | (val << 4);
      write_sync_reg ();
      return *this;
    }

    void write_sync_reg (void)
    {
      m_axis.wr_cmd16 (0x26 + this_sync_num, m_sync_reg);
    }

    friend class mcx51x_axis;
  };

  using sync0_action_set = sync_action_set<0>;
  using sync1_action_set = sync_action_set<1>;
  using sync2_action_set = sync_action_set<2>;
  using sync3_action_set = sync_action_set<3>;

  auto& sync_action_0 (void) { return m_sync0; }
  auto& sync_action_1 (void) { return m_sync1; }
  auto& sync_action_2 (void) { return m_sync2; }
  auto& sync_action_3 (void) { return m_sync3; }


  // in addition to the synchronous actions it's also possible to use
  // the MR compare-match function together with CPU trigger functions (interrupts)
  // without synchronous action.  because the MR registers and comparison settings
  // are shared, they will be overwritten by whatever is selected last.
  // however, if the synchronous action doesn't use the MR register, it's possible
  // to use both in parallel.
  // for example, synchronous action 0 triggers on "drive_stopped"
  // and MR0 compare match triggers the CPU at something else.
  void set_mr_compare_match (unsigned int mr_num,
			     mr_comp_code_t comp_code, mr_comp_op_t comp_operand)
  {
    set_mr_compare_match_1 (mr_num & 3, comp_code, comp_operand);
  }

  void set_mr_compare_match (unsigned int mr_num,
			     mr_comp_code_t comp_code, position_counter_t comp_operand)
  {
    set_mr_compare_match_1 (mr_num & 3, comp_code, comp_operand);
  }

  void set_mr (unsigned int mr_num, int32_t val)
  {
    wr_cmd32 (0x10 + (mr_num & 3), val);
  }

  int32_t mr (unsigned int mr_num)
  {
    return rd_cmd32 (0x34 + (mr_num & 3));
  }

  // axis event interrupt functions.
  // the functions will be invoked from within an interrupt context.
  // to clear the function, set it to nullptr.
  // setting/clearing the functions is not interrupt safe and should not be
  // done from within any interrupt context.
  // setting a non-nullptr function will automatically enable
  // the corresponding interrupt.
  template <typename F>
  void set_mr_compare_match_func (unsigned int mr_num, F&& f)
  {
    set_irq_func (irq_cmr0 + (mr_num & 3), std::move (f));
  }

  template <typename F>
  void set_drive_start_func (F&& f)
  {
    set_irq_func (irq_drive_start, std::move (f));
  }

  template <typename F>
  void set_constant_speed_start_func (F&& f)
  {
    set_irq_func (irq_constant_speed_start, std::move (f));
  }

  template <typename F>
  void set_constant_speed_end_func (F&& f)
  {
    set_irq_func (irq_constant_speed_end, std::move (f));
  }

  template <typename F>
  void set_drive_stop_func (F&& f)
  {
    set_irq_func (irq_drive_stop, std::move (f));
  }

  template <typename F>
  void set_auto_homing_end_func (F&& f)
  {
    set_irq_func (irq_auto_homing_end, std::move (f));
  }

  template <typename F>
  void set_timer_func (F&& f)
  {
    set_irq_func (irq_timer, std::move (f));
  }

  template <typename F>
  void set_split_pulse_func (F&& f)
  {
    set_irq_func (irq_split_pulse_rising_edge, std::move (f));
  }

  template <typename F>
  void set_split_pulse_end_func (F&& f)
  {
    set_irq_func (irq_split_pulse_end, std::move (f));
  }

  // enable/disable axis event interrupt functions selectively.
  // this allows to quickly turn interrupts on/off without changing
  // the actual callback function that has been set before.
  // this can be useful when there is a need to change the triggering
  // axis between x <-> y in e.g. circular motion.
  void enable_mr_compare_match_func (unsigned int mr_num, bool en_val)
  {
    set_irq_mask (irq_cmr0 + (mr_num & 3), en_val);
  }

  void enable_drive_start_func (bool en_val)
  {
    set_irq_mask (irq_drive_start, en_val);
  }

  void enable_constant_speed_start_func (bool en_val)
  {
    set_irq_mask (irq_constant_speed_start, en_val);
  }

  void enable_constant_speed_end_func (bool en_val)
  {
    set_irq_mask (irq_constant_speed_end, en_val);
  }

  void enable_drive_stop_func (bool en_val)
  {
    set_irq_mask (irq_drive_stop, en_val);
  }

  void enable_auto_homing_end_func (bool en_val)
  {
    set_irq_mask (irq_auto_homing_end, en_val);
  }

  void enable_timer_func (bool en_val)
  {
    set_irq_mask (irq_timer, en_val);
  }

  void enable_split_pulse_func (bool en_val)
  {
    set_irq_mask (irq_split_pulse_rising_edge, en_val);
  }

  void enable_split_pulse_end_func (bool en_val)
  {
    set_irq_mask (irq_split_pulse_end, en_val);
  }


  mcx51x_axis (void) = delete;

protected:
  enum interrupt_num
  {
    irq_cmr0 = 0,
    irq_cmr1 = 1,
    irq_cmr2 = 2,
    irq_cmr3 = 3,

    // D-STA
    irq_drive_start = 4,

    // C-STA
    irq_constant_speed_start = 5,

    // C-END
    irq_constant_speed_end = 6,

    // D-END
    irq_drive_stop = 7,

    // H-END
    irq_auto_homing_end = 8,

    // TIMER
    irq_timer = 9,

    // SPLTP (seems not available on MCX512 ??)
    irq_split_pulse_rising_edge = 10,

    // SPLTE
    irq_split_pulse_end = 11,

    // SYNC3..0
    irq_sync0 = 12,
    irq_sync1 = 13,
    irq_sync2 = 14,
    irq_sync3 = 15,
  };


  constexpr mcx51x_axis (unsigned int num, HwInst& c)
  : m_num (num), m_num_shift ((1 << num) << 8), m_container (c),
    m_sync0 (*this), m_sync1 (*this), m_sync2 (*this), m_sync3 (*this) { }

  // axis number
  const unsigned int m_num;

  // (1 << axis number) << 8, directly written to WR0 as axis selection bits.
  const unsigned int m_num_shift;

  HwInst& m_container;

  // although it is possible to invert the logic of the signals and even swap
  // some signal pairs, reading the signal state will always return the raw
  // pin state.  to simplify software apply the according bit inversion and
  // bit swapping in software when reading the signals.
  unsigned int m_rr3_inv_mask = 0;
  unsigned int m_rr3_swap_r_mask = 0;
  unsigned int m_rr3_swap_l_mask = 0;

  sync0_action_set m_sync0;
  sync1_action_set m_sync1;
  sync2_action_set m_sync2;
  sync3_action_set m_sync3;

  // when an irq function is being modified, set a lock bit so that it will
  // be ignored by the axis-interrupt handler.  otherwise it might try to
  // invoke a partially written std::function and blow up.  while the lock bit
  // for a particular function is set, the interrupt handler will ignore it.
  // an alternative to the lock bits is to disable interrupts or only the
  // MCX axis interrupt temporarily.  but do not want to disable interrupts.
  std::atomic<unsigned int> m_irq_funcs_lock = ATOMIC_VAR_INIT (0);

  // cached interrupt mask bits (WR1).
  unsigned int m_cached_irq_mask = 0;

  std::array<std::function<void (void)>, 16> m_irq_funcs;

  void wr_reg16 (unsigned int regno, uint16_t val) { container ().write_reg16 (regno, val); }
  uint16_t rd_reg16 (unsigned int regno) const { return container ().read_reg16 (regno); }
  void wr_cmd32 (unsigned int cmdno, uint32_t data) { container ().write_cmd32 (m_num_shift, cmdno, data); }
  void wr_cmd16 (unsigned int cmdno, uint16_t data) { container ().write_cmd16 (m_num_shift, cmdno, data); }
  void wr_cmd (unsigned int cmdno) { container ().write_cmd (m_num_shift, cmdno); }
  void wr_axis_reg16 (unsigned int cmdno, unsigned int regno, uint16_t val) { container ().write_axis_reg16 (m_num_shift, cmdno, regno, val); }

  uint16_t rd_cmd16 (unsigned int cmdno, unsigned int regno = 6) const { return container ().read_cmd16 (m_num_shift, cmdno, regno); }
  uint16_t rd_cmd16_isr (unsigned int cmdno, unsigned int regno = 6) const { return container ().read_cmd16_isr (m_num_shift, cmdno, regno); }

  uint32_t rd_cmd32 (unsigned int cmdno, unsigned int regno = 6) const { return container ().read_cmd32 (m_num_shift, cmdno, regno); }
  uint32_t rd_cmd32_isr (unsigned int cmdno, unsigned int regno = 6) const { return container ().read_cmd32_isr (m_num_shift, cmdno, regno); }

  template <typename F>
  void set_irq_func (unsigned int i, F&& f)
  {
    i &= 15;
    m_irq_funcs_lock |= (1 << i);
    m_irq_funcs[i] = std::move (f);
    const bool func_set = m_irq_funcs[i] != nullptr;

    std::atomic_signal_fence (std::memory_order_release);
    m_irq_funcs_lock &= ~(1 << i);

    set_irq_mask (i, func_set);
  }

  void set_irq_mask (unsigned int i, bool val)
  {
    m_cached_irq_mask = (m_cached_irq_mask & ~(1 << i)) | (val << i);
    wr_axis_reg16 (0x1F, 1, m_cached_irq_mask);
  }

  uint16_t get_clear_irq_status (void)
  {
    // reading the per-axis interrupt status from RR1 automatically
    // clears the bits.
    return rd_cmd16_isr (0x1F, 1);
  }

  uint16_t irq_mask (void) const
  {
    return rd_cmd16 (0x3D);
  }

  void set_mr_compare_match_1 (unsigned int mr_num,
			       unsigned int comp_code, unsigned int comp_operand)
  {
    uint16_t r = rd_cmd16 (0x40) & ~(0b1111 << (mr_num * 4));
    wr_cmd16 (0x20, r | (((comp_code << 2) | (comp_operand << 0)) << (mr_num * 4)));
  }


  friend sync0_action_set;
  friend sync1_action_set;
  friend sync2_action_set;
  friend sync3_action_set;

  template <typename, typename, typename, typename, typename, typename,
	    typename> friend class mcx51x_hw_inst;
};

// ----------------------------------------------------------------------------
// MCX51x common hardware instance

// the base class handles interrupts for MCX514 and MCX512.  because
// the axes array is stored in the sub-classes, need to give the base class
// access to it.  the per-axis interrupt handler needs to iterate over all
// axes

template <typename HwInst> struct get_axes_func
{
  template <typename HwInstBase>
  auto& operator () (HwInstBase* thiss) const
  {
    return static_cast<HwInst*> (thiss)->axes ();
  }
};

template <typename BusInterface, typename ClockHzRatio,
	  typename INT0N_InterruptLine,
	  typename INT1N_InterruptLine,
	  typename AxisInputs, typename AxisOutputs,
	  typename GetAxesFunc>
class mcx51x_hw_inst
{
public:
  using clock_hz = ClockHzRatio;
  using bus_interface = BusInterface;

  static constexpr unsigned int clock_hz_i = clock_hz::num / clock_hz::den;

  static_assert (clock_hz_i <= 20'000'000, "");

  // reset the IC
  [[gnu::cold]] void reset (void)
  {
    write_cmd (0, 0xFF);
    std::this_thread::sleep_for (std::chrono::nanoseconds (500));
  }

  // main status (chip status)
  status_t status (void) const						{ return status_t (read_reg16 (0)); }

  // start driving axes simultaneously, for which the drive has been held before.
  void release_drive_hold (void)					{ write_cmd (0x0F << 8, 0x78); }


  // interpolation commands (combined axis driving)

  void set_interpolation_mode (const interpolation_mode_t& val)
  {
    write_cmd16 (0, 0x2A, val.value () & 0b1101'1111'1101'1111);
  }

  void drive_1axis_linear_multichip (void)				{ write_cmd (0, 0x60); }

  void drive_2axis_linear (void)					{ write_cmd (0, 0x61); }

  void drive_2axis_cw_circular (void)					{ write_cmd (0, 0x64); }
  void drive_2axis_ccw_circular (void)					{ write_cmd (0, 0x65); }

  void drive_2axis_bitpattern (void)					{ write_cmd (0, 0x66); }

  // enable / disable deceleration during interpolation for the next segment
  // for point-to-point motion with a single segment (not continuous multi-segment motion)
  // this should be turned on before driving to enable deceleration.
  void enable_interp_decel (bool val)					{ write_cmd (0, 0x6E - val); }

  void clear_interp_interrupt (void)					{ write_cmd (0, 0x6F); }

  template <typename F>
  void set_interp_coord_func (F&& f)
  {
    m_interp_irq_func = std::move (f);
    std::atomic_signal_fence (std::memory_order_release);
  }

  // convenience functions for speed value conversions
  static constexpr uint32_t native_to_pps (uint32_t val)
  {
    return mcx51x::native_to_pps<clock_hz> (val);
  }

  static constexpr uint32_t native_to_pps2 (uint32_t val)
  {
    return mcx51x::native_to_pps2<clock_hz> (val);
  }

  static constexpr uint32_t native_to_pps3 (uint32_t val)
  {
    return mcx51x::native_to_pps3<clock_hz> (val);
  }

  static constexpr uint32_t pps_to_native (uint32_t val)
  {
    return mcx51x::pps_to_native<clock_hz> (val);
  }

  static constexpr uint32_t pps2_to_native (uint32_t val)
  {
    return mcx51x::pps2_to_native<clock_hz> (val);
  }

  static constexpr uint32_t pps3_to_native (uint32_t val)
  {
    return mcx51x::pps3_to_native<clock_hz> (val);
  }

private:
  void int0n_axis_isr (void)
  {
    auto ctx = m_bif.save_reg_context (BusInterface::in_isr_context, 0, 0);

    for (auto& a : GetAxesFunc () (this))
    {
      unsigned int irq_bits = a.get_clear_irq_status () & a.m_cached_irq_mask
			      & ~(a.m_irq_funcs_lock.load ());

      auto* irq_func = a.m_irq_funcs.data ();

      while (irq_bits != 0)
      {
	if ((irq_bits & 0b1111) != 0)
	{
	  if ((irq_bits & 0b0001) != 0 && irq_func[0] != nullptr)
	    irq_func[0] ();

	  if ((irq_bits & 0b0010) != 0 && irq_func[1] != nullptr)
	    irq_func[1] ();

	  if ((irq_bits & 0b0100) != 0 && irq_func[2] != nullptr)
	    irq_func[2] ();

	  if ((irq_bits & 0b1000) != 0 && irq_func[3] != nullptr)
	    irq_func[3] ();
	}

	irq_bits >>= 4;
	irq_func += 4;
      }
    }

    m_bif.restore_reg_context (ctx);
  }


  void int1n_interpolation_isr (void)
  {
    // continuous interpolation driving interrupt

    // save and clear out the current reg access context.
    // if this has interrupted a register access, clear it out so that
    // subsequent accesses will not restore it unnecessarily
    auto ctx = m_bif.save_reg_context (BusInterface::in_isr_context, 0, 0);

    // do the callback to write more interpolation data or complete the
    // interpolation drive.
    if (m_interp_irq_func)
      m_interp_irq_func ();

    m_bif.restore_reg_context (ctx);
  }

public:
  typedef interrupt::connected_isr<INT0N_InterruptLine,
	interrupt::func<decltype (&mcx51x_hw_inst::int0n_axis_isr), &mcx51x_hw_inst::int0n_axis_isr>> isr0_t;
  typedef interrupt::connected_isr<INT1N_InterruptLine,
	interrupt::func<decltype (&mcx51x_hw_inst::int1n_interpolation_isr), &mcx51x_hw_inst::int1n_interpolation_isr>> isr1_t;


  template <typename AI, typename AO>
  [[gnu::cold]] mcx51x_hw_inst (AI&& ai, AO&& ao, BusInterface&& bif = { })
  : m_int0n (this), m_int1n (this), m_bif (bif),
    m_axis_inputs ({std::forward<AI> (ai)}), m_axis_outputs ({std::forward<AO> (ao)})
  {
    m_int0n.enable (interrupt::low_level, interrupt::priority_6);
    m_int1n.enable (interrupt::low_level, interrupt::priority_6);
  }

protected:
  isr0_t m_int0n;
  isr1_t m_int1n;

  mutable BusInterface m_bif;
  mutable AxisInputs m_axis_inputs;
  mutable AxisOutputs m_axis_outputs;

  std::function<void(void)> m_interp_irq_func;

  void write_cmd (unsigned int axis, unsigned int cmdno) { m_bif.write_cmd (axis, cmdno); }
  void write_cmd16 (unsigned int axis, unsigned int cmdno, uint16_t data) { m_bif.write_cmd16 (axis, cmdno, data); }
  void write_cmd32 (unsigned int axis, unsigned int cmdno, uint32_t data) { m_bif.write_cmd32 (axis, cmdno, data); }
  void write_axis_reg16 (unsigned int axis, unsigned int cmdno, unsigned int regno, uint16_t val) { m_bif.write_axis_reg16 (axis, cmdno, regno, val); }

  uint16_t read_cmd16 (unsigned int axis, unsigned int cmdno, unsigned int regno) const { return m_bif.read_cmd16 (axis, cmdno, regno); }
  uint16_t read_cmd16_isr (unsigned int axis, unsigned int cmdno, unsigned int regno) const { return m_bif.read_cmd16_isr (axis, cmdno, regno); }
  uint32_t read_cmd32 (unsigned int axis, unsigned int cmdno, unsigned int regno) const { return m_bif.read_cmd32 (axis, cmdno, regno); }
  uint32_t read_cmd32_isr (unsigned int axis, unsigned int cmdno, unsigned int regno) const { return m_bif.read_cmd32_isr (axis, cmdno, regno); }

  void write_reg16 (unsigned int regno, uint16_t val) { m_bif.write_reg16 (regno, val); }
  void write_reg32 (unsigned int regno, uint32_t val) { m_bif.write_reg32 (regno, val); }

  uint16_t read_reg16 (unsigned int regno) const { return m_bif.read_reg16 (regno); }
  uint32_t read_reg32 (unsigned int regno) const { return m_bif.read_reg32 (regno); }


  friend class axis_t;
  template <typename, typename, unsigned int> friend class mcx51x_axis;
};

// ----------------------------------------------------------------------------
// MCX514 hardware instance

template <typename BusInterface, typename ClockHz,
	  typename INT0N_InterruptLine,
	  typename INT1N_InterruptLine,
	  typename AxisInputs, typename AxisOutputs>

class mcx514_hw_inst
 : public mcx51x_hw_inst<
	BusInterface, ClockHz, INT0N_InterruptLine, INT1N_InterruptLine,
	AxisInputs, AxisOutputs,
	get_axes_func< mcx514_hw_inst < BusInterface, ClockHz,
					INT0N_InterruptLine, INT1N_InterruptLine,
					AxisInputs, AxisOutputs> > >
{
  typedef mcx51x_hw_inst<BusInterface, ClockHz, INT0N_InterruptLine,
			 INT1N_InterruptLine, AxisInputs, AxisOutputs,
			 get_axes_func< mcx514_hw_inst < BusInterface, ClockHz,
					INT0N_InterruptLine, INT1N_InterruptLine,
					AxisInputs, AxisOutputs> >> base_class;
public:
  static constexpr unsigned int axis_count = 4;
  using base_class::clock_hz;
  using base_class::clock_hz_i;

  class axis_t : public mcx51x_axis<mcx514_hw_inst, ClockHz, axis_count - 1>
  {
    typedef mcx51x_axis<mcx514_hw_inst, ClockHz, axis_count - 1> base_class;

  public:
    // helical rotation number setting
    void set_helical_rotation_number (utils::clamped_value<uint16_t, 0, 65535> val)			{ this->wr_cmd16 (0x1A, val); }
    uint16_t helical_rotation_number (void) const							{ return this->rd_cmd16 (0x3A); }

    // helical calculation value setting
    void set_helical_calculation_value (utils::clamped_value<int32_t, 1, 2147483646> val)		{ this->wr_cmd32 (0x1B, val); }
    int32_t helical_calculation_value (void) const							{ return this->rd_cmd32 (0x3B); }

    // PIO inputs/outputs
    // via (set_)pio_xy_values and (set_)pio_zu_values
    //
    // CAUTION: these are normally setup in the board hardware init function.
    //          the pin input/output configuration should be set according to
    //          the hardware circuit of the pin.
    uint8_t pios (void) const
    {
      // RR4: yyyyyyyy'xxxxxxxx
      // RR5: uuuuuuuu'zzzzzzzz
      const unsigned int n = this->num ();

      return this->rd_reg16 (4 + ((n >> 1) & 1)) >> ((n & 1) * 8);
    }

    void set_pios (uint8_t val)
    {
      // WR4: yyyyyyyy'xxxxxxxx
      // WR5: uuuuuuuu'zzzzzzzz
      const unsigned int n = this->num ();
      const uint16_t mask = 0b00000000'11111111 << ((n & 1) * 8);

      // pio input values are the sampled pin input states, they are not
      // the output values that are used when the PIO is configured as output.
      // thus we have to keep the other axis' PIO values in a cache variable
      // outside the MCX.
      uint16_t& r_cache = this->container ().m_wr4_wr5_cache[(n >> 1) & 1];
      uint16_t r_new = (r_cache & ~mask) | (val << ((n & 1) * 8));
      r_cache = r_new;
      this->wr_reg16 (4 + ((n >> 1) & 1), r_new);
    }

    axis_t (void) = delete;
    constexpr axis_t (unsigned int num, mcx514_hw_inst& c) : base_class (num, c) { }

  private:
  };

  template <typename AI, typename AO>
  [[gnu::cold]] mcx514_hw_inst (AI&& ai, AO&& ao, BusInterface&& bif = { })
  : base_class (std::forward<AI> (ai), std::forward<AO> (ao), std::forward<BusInterface> (bif)),
    m_axes {{ {0, *this}, {1, *this}, {2, *this}, {3, *this} }}
  {
  }


  constexpr std::array<axis_t, axis_count>& axes (void) { return m_axes; }
  axis_t& axis (unsigned int n) { return axes ()[n]; }

  // general purpose input value reading
  // pin 132..139, not the same as the per-axis PIOs
  // notice that these signals have different functions depending on
  // the wiring of the board.
  uint16_t gpi_values (void) const					{ return read_cmd16 (0, 0x48, 7); }

  // when wired in i2c mode, the D[15:0] pins can be used as general inputs.
  uint16_t databus_values (void) const					{ return read_cmd16 (0, 0x48, 6); }


  // interpolation commands (combined axis driving)

  void drive_1axis_linear_multichip (void)				{ write_cmd (0, 0x60); }

  void drive_3axis_linear (void)					{ write_cmd (0, 0x62); }
  void drive_4axis_linear (void)					{ write_cmd (0, 0x63); }

  void drive_3axis_bitpattern (void)					{ write_cmd (0, 0x67); }
  void drive_4axis_bitpattern (void)					{ write_cmd (0, 0x68); }

  void drive_cw_helical (void)						{ write_cmd (0, 0x69); }
  void drive_ccw_helical (void)						{ write_cmd (0, 0x6A); }
  void calc_cw_helical (void)						{ write_cmd (0, 0x6B); }
  void calc_ccw_helical (void)						{ write_cmd (0, 0x6C); }

private:
  std::array<axis_t, axis_count> m_axes;
  std::array<uint16_t, 2> m_wr4_wr5_cache = { 0, 0 };

  using base_class::write_cmd;
  using base_class::write_cmd16;
  using base_class::write_cmd32;
  using base_class::write_axis_reg16;

  using base_class::read_cmd16;
  using base_class::read_cmd32;

  using base_class::write_reg16;
  using base_class::write_reg32;

  using base_class::read_reg16;
  using base_class::read_reg32;

  friend class axis_t;
  friend class mcx51x_axis<mcx514_hw_inst, ClockHz, axis_count - 1>;
};

// ----------------------------------------------------------------------------
// MCX512 hardware instance

template <typename BusInterface, typename ClockHz,
	  typename INT0N_InterruptLine,
	  typename INT1N_InterruptLine,
	  typename AxisInputs, typename AxisOutputs>


class mcx512_hw_inst
 : public mcx51x_hw_inst<
	BusInterface, ClockHz, INT0N_InterruptLine, INT1N_InterruptLine,
	AxisInputs, AxisOutputs,
	get_axes_func< mcx512_hw_inst < BusInterface, ClockHz,
					INT0N_InterruptLine, INT1N_InterruptLine,
					AxisInputs, AxisOutputs> > >
{
  typedef mcx51x_hw_inst<BusInterface, ClockHz, INT0N_InterruptLine,
			 INT1N_InterruptLine, AxisInputs, AxisOutputs,
			 get_axes_func< mcx512_hw_inst < BusInterface, ClockHz,
					INT0N_InterruptLine, INT1N_InterruptLine,
					AxisInputs, AxisOutputs> >> base_class;
public:
  static constexpr unsigned int axis_count = 2;
  using base_class::clock_hz;
  using base_class::clock_hz_i;

  class axis_t : public mcx51x_axis<mcx512_hw_inst, ClockHz, axis_count - 1>
  {
    typedef mcx51x_axis<mcx512_hw_inst, ClockHz, axis_count - 1> base_class;

  public:
    // PIO signal setting 3
    void set_pio3 (pio_mode3_t val)									{ this->wr_cmd16 (0x2B, val.value ()); }
    pio_mode3_t pio3 (void) const									{ return pio_mode3_t (this->rd_cmd16 (0x49)); }

    // PIO inputs/outputs
    // via (set_)pio_xy_values and (set_)pio_zu_values
    //
    // CAUTION: these are normally setup in the board hardware init function.
    //          the pin input/output configuration should be set according to
    //          the hardware circuit of the pin.
    uint16_t pios (void) const
    {
      // RR4: x axis IOs, RR5: y axis IOs
      return this->container ().read_reg16 (4 + this->num ());
    }

    void set_pios (uint16_t val)
    {
      // WR4: x axis IOs, WR5: y axis IOs
      this->container ().write_reg16 (4 + this->num (), val);
    }

    axis_t (void) = delete;
    constexpr axis_t (unsigned int num, mcx512_hw_inst& c) : base_class (num, c) { }

  private:
  };

  template <typename AI, typename AO>
  [[gnu::cold]] mcx512_hw_inst (AI&& ai, AO&& ao, BusInterface&& bif = { })
  : base_class (std::forward<AI> (ai), std::forward<AO> (ao), std::forward<BusInterface> (bif)),
    m_axes {{ {0, *this}, {1, *this} }}
  {
  }


  constexpr std::array<axis_t, axis_count>& axes (void) { return m_axes; }
  axis_t& axis (unsigned int n) { return axes ()[n]; }


  // when wired in i2c mode, the D[15:0] pins can be used as general inputs.
  // unlike MCX514, there are no dedicated general input pins otherwise.
  uint16_t databus_values (void) const					{ return read_cmd16 (0, 0x48, 6); }



private:
  std::array<axis_t, axis_count> m_axes;

  using base_class::write_cmd;
  using base_class::write_cmd16;
  using base_class::write_cmd32;
  using base_class::write_axis_reg16;

  using base_class::read_cmd16;
  using base_class::read_cmd32;

  using base_class::write_reg16;
  using base_class::write_reg32;

  using base_class::read_reg16;
  using base_class::read_reg32;

  friend class axis_t;
  friend class mcx51x_axis<mcx512_hw_inst, ClockHz, axis_count - 1>;
};

} // namespace mcx51x
} // namespace dev
#endif // includeguard_dev_mcx51x_hpp_includeguard

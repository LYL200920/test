/*

MDIO master station implementation for RX EtherC devices.

the rx etherc devices do not have any sophisticated support for implementing
the MDIO serial communication interface, which is normally used to communicate
with PHYs.  it is a bitbang GPIO interface to output the clock signal and
read or write the data signal.  bit timing and everything else has to be
done in software.

the following functions take care of that.

  output the MDC clock as follows

       ____
    __|    |__
     a   b   c

  a: 1/2 cycle time clock low  (100 ns)
  b: 1 cycle time clock high   (200 ns)
  c: 1/2 cycle time clock low  (100 ns)

  we always start at 1/2 clock-low cycle and write the new data to the output.
  this assures that at the next clock rising edge the data is valid.

  the timing is controlled by doing multiple PIR register reads or writes.
  for every LSI we know how long one register read/write takes.  using this
  time and the desired cycle time we can calculate the number of reads or
  writes to do.

  -------------------
  example RX63N:

  PCLKA = 96 MHz = 10.41 ns
  1 write PIR access = 5 PCLKA cycles = 52 ns

  // a (104 ns)
  PIR = 0b0010;
  PIR = 0b0010;

  // b (208 ns)
  PIR = 0b0011;
  PIR = 0b0011;
  PIR = 0b0011;
  PIR = 0b0011;

  // c (104 ns)
  PIR = 0b0010;
  PIR = 0b0010;

  -------------------
  example SH7216:

  Iclk = 200  Bclk = 50 n = 3, I = 1

  etherc register write = 8 * Icy + 9 * Bcy
                        = 11 * Bcy
                        = 11 * 20 ns = 220 ns

  // a (220 ns)
  PIR = 0b0010;

  // b (220 ns)
  PIR = 0b0011;

  // c (220 ns)
  PIR = 0b0010;

  it is impossible to create a symmetric duty cycle without slowing down 
  register access too much.

  -------------------
  example SH7786

  Ick = 1/2  Pck = 1/24   Bck = 1/12 = 89 MHz = 11.2 ns

  etherc register write = clks3 = 11.2 x 3 = 34 ns

  // a (102 ns)
  PIR = 0b0010;
  PIR = 0b0010;
  PIR = 0b0010;

  // b (204 ns)
  PIR = 0b0011;
  PIR = 0b0011;
  PIR = 0b0011;
  PIR = 0b0011;
  PIR = 0b0011;
  PIR = 0b0011;

  // c (102 ns)
  PIR = 0b0010;
  PIR = 0b0010;
  PIR = 0b0010;


 ----------------------------------------------

FIXME MCB-133: use timer + DTC for asynchronous communication without CPU.

*/

#ifndef includeguard_dev_rx_etherc_mdio_sta_hpp_includeguard
#define includeguard_dev_rx_etherc_mdio_sta_hpp_includeguard

#include <chrono>

#include <dev/mdio.hpp>
#include <dev/hwreg.hpp>

namespace dev
{

struct rx_etherc_mdio_sta_helper
{

  // register access with loop unrolling to control the access timing.
  template <unsigned int AccessCount> static uint32_t
  read_reg (volatile uint32_t& reg)
  {
    [[gnu::unused]] uint32_t r = reg;
    return read_reg<AccessCount - 1> (reg);
  }

  template <unsigned int AccessCount> static void
  write_reg (volatile uint32_t& reg, uint32_t val)
  {
    reg = val;
    write_reg<AccessCount - 1> (reg, val);
  }
};

template<> inline void rx_etherc_mdio_sta_helper::write_reg<1> (volatile uint32_t& reg, uint32_t val) { reg = val; }
template<> inline uint32_t rx_etherc_mdio_sta_helper::read_reg<1> (volatile uint32_t& reg) { return reg; }



// ------------------------------------------------------------------------

template < uintptr_t PIR_addr, unsigned int ModuleClockHz,
	   unsigned int PIRRegAccessModuleClockCycles >
class rx_etherc_mdio_sta final : public mdio_sta
{
public:
  // MDIO port access

  virtual uint16_t mdio_read_reg (unsigned int mmd_addr, unsigned int cycle_speed_ns,
				  unsigned int reg) override
  {
    mdio_write_preamble32 ();
    mdio_write_cmd14 (mmd_addr, reg, mdio_op_read);
    mdio_highz ();

    uint16_t data = mdio_read16 ();

    mdio_highz ();

    return data;
  }

  virtual void mdio_write_reg (unsigned int mmd_addr, unsigned int cycle_speed_ns,
			       unsigned int reg, uint16_t data) override
  {
    mdio_write_preamble32 ();
    mdio_write_cmd14 (mmd_addr, reg, mdio_op_write);
    mdio_write (1);
    mdio_write (0);
    mdio_write16 (data);
    mdio_highz ();
  }

private:
  static constexpr hw_reg_rw<uint32_t, const_addr< PIR_addr >> PIR = { };

  // number of nanoseconds for one PIR register access.
  static constexpr unsigned int pir_access_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds> (
	std::chrono::duration<uint64_t, std::ratio<1, ModuleClockHz>> (
				PIRRegAccessModuleClockCycles)).count ();

  // standard MDIO cycle time in nanoseconds
  static constexpr unsigned int mdio_cycle_time_ns = 400;

  // number of accesses for a quarter-cycle (rounded up)
  static constexpr unsigned int pir_quarter_cycle_count =
    (mdio_cycle_time_ns/4 + pir_access_ns-1) / pir_access_ns;


  // do a PIR register read or write for a quarter of the total MDIO cycle time.
  void write_pir (uint32_t val)
  {
    rx_etherc_mdio_sta_helper::write_reg<pir_quarter_cycle_count> (*&PIR, val);
  }

  uint32_t read_pir (void)
  {
    return rx_etherc_mdio_sta_helper::read_reg<pir_quarter_cycle_count> (*&PIR);
  }

  // write 1 bit
  void mdio_write (bool val)
  {
    uint32_t regvals = val ? 0x06070706 : 0x02030302;

    for (int j = 0; j < 4; ++j)
    {
      write_pir (regvals & 0xF);
      regvals >>= 8;
    }
  }

  // write 1 bit of high-z state
  void mdio_highz (void)
  {
    uint32_t regvals = 0x00010100;

    for (int j = 0; j < 4; ++j)
    {
      write_pir (regvals & 0xF);
      regvals >>= 8;
    }
  }

  // a simple MDIO frame
  //  unsigned int data = 0b01'00'00000'00000'00
  //                        ^^ ^^ ^^^^^ ^^^^^ ^^
  //                        st op  phy   reg  ta

  enum mdio_op_type
  {
    mdio_op_write = 0b01'01'00000'00000'00,
    mdio_op_read  = 0b01'10'00000'00000'00
  };

  // write the 14 bits frame/command header
  void mdio_write_cmd14 (unsigned int phy_addr, unsigned short reg_addr,
			 mdio_op_type op)
  {
    uint16_t data = op | ((phy_addr & 0b11111) << 7)
		       | ((reg_addr & 0b11111) << 2);

    for (int i = 0; i < 14; ++i)
    {
      mdio_write ((data & 0x8000) != 0);
      data <<= 1;
    }
  }

  // read 16 bits of data
  uint16_t mdio_read16 (void)
  {
    uint16_t data = 0;

    for (int i = 0; i < 16; ++i)
    {
      write_pir (0);

      // set the clock output high for 1/4 cycle time
      // then read input, keeping the clock output at high  for another
      // 1/4 cycle time.
      write_pir (1);
      data = (data << 1) | ((read_pir () >> 3) & 1);

      write_pir (0);
    }

    return data;
  }

  // write 16 bits of data
  void mdio_write16 (uint16_t data)
  {
    // data is written one bit at a time
    for (int i = 0; i < 16; ++i)
    {
      mdio_write ((data & 0x8000) != 0);
      data <<= 1;
    }
  }

  // write 32 bit preamble
  void mdio_write_preamble32 (void)
  {
    for (int i = 0; i < 32; ++i)
      mdio_write (1);
  }
};

} // namespace dev
#endif // includeguard_dev_rx_etherc_mdio_sta_hpp_includeguard

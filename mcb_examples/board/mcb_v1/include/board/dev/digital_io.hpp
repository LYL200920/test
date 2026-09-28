/*

MCB v1 board specific driver for the digital inputs and digital outputs.

there are 2x PCA9698, connected via I2C to the MCU.
although the IOs of the PCAs can be freely configured, the circuity
around the IOs is designed either for inputs or outputs.  thus we can
basically use only one fixed configuration for the two PCAs.

the PCA that is used for inputs can also trigger an interrupt to the MCU.

PCA addr 0x40 (all inputs)
  IO bank 0: DI 0..7 (EXP1_00..EXP1_07)
  IO bank 1: DI 8..15 (EXP1_10..EXP1_17)
  IO bank 2: DI 16..23 (EXP1_20..EXP1_27)
  IO bank 3: XIN3..XIN6 (CN14A, p21..p24) -> (EXP1_30..EXP1_33)
             YIN3..YIN6 (CN14B, p21..p24) -> (EXP1_34..EXP1_37)
  IO bank 4: ZIN3..ZIN6 (CN15A, p21..p24) -> (EXP1_40..EXP1_43)
             UIN3..UIN6 (CN15B, p21..p24) -> (EXP1_44..EXP1_47)

  IO bank 3 and bank 4 are used as additional servo motor control, which is
  part of the MCX block.  thus we don't provide access to these inputs here.

PCA addr 0x42 (input / output)
  IO bank 0: DO 0..7 (EXP2_00..EXP2_07)
  IO bank 1: DO 8..15 (EXP2_10..EXP2_17)
  IO bank 2: DO 16..23 (EXP2_20..EXP27)
  IO bank 3: XOUT10 (EXP2_30)
             YOUT10 (EXP2_31)
             ZOUT10 (EXP2_32)
             UOUT10 (EXP2_33)
             TP2_1..TP2_3 (EXP2_34..EXP2_37)
  IO bank 4: DIPSW3..DIPSW6 (EXP2_40..EXP2_43)
             TP1_1..TP1_4 (EXP2_44..EXP2_47)

  IO bank 3 is used as additional servo motor inputs, which is part of the
  MCX block.  thus we don't provide access to these inputs here.

  IO bank4 is used for the dipswitch (and some test points) and thus
  we don't provide access to these inputs here.


the digital_io class configures all the IOs of the PCAs correctly, but the
non-general-purpose IOs are not exposed here.  they are accessed by the
corresponding classes directly instead.

in addition to the 24 PCA inputs and 24 PCA outputs, there are 8 inputs and
8 outputs directly connected to the MCU:

EXP_IN10 - DI24 - P02 (IRQ10)
EXP_IN11 - DI25 - P07 (IRQ15)
EXP_IN12 - DI26 - P32 (IRQ2-DS)
EXP_IN13 - DI27 - P25
EXP_IN20 - DI28 - P24
EXP_IN21 - DI29 - P22
EXP_IN22 - DI30 - P87
EXP_IN23 - DI31 - P86

EXP_OUT10 - DO24 - P23
EXP_OUT11 - DO25 - P54
EXP_OUT12 - DO26 - P55
EXP_OUT13 - DO27 - P56
EXP_OUT20 - DO28 - P73
EXP_OUT21 - DO29 - PB7
EXP_OUT22 - DO30 - PB2
EXP_OUT23 - DO31 - P70

because of the slow PCA access over I2C, the input/output values are cached
in RAM and periodically synchronized.  this makes the software that uses
the digital IOs simpler, as it can just manipulate the bits in RAM.

*/

#ifndef includeguard_mcbv1_board_dev_digital_io_includeguard
#define includeguard_mcbv1_board_dev_digital_io_includeguard

#include <cstdint>
#include <bitset>
#include <array>
#include <chrono>

#include <dev/pca9698.hpp>
#include <dev/renesas/rx_gpio.hpp>
#include <utils/bits.hpp>
#include <dev/digital_io_port.hpp>

namespace dev
{

/*
I2C access time
  1.5 MBit/sec (1 clock = 0.6666 usec)
  24 IOs = 3 bytes + 1 address byte = 4 bytes = 32 bits
  -> assume about 40 clocks -> 26 usec

  because we need to read and write to two different PCAs,
  the access time is double, i.e. about 53 usec

the propagation delay of the digital output circuit is
    60 usec TLP290-4 optocoupler ( max turn-off time @ 5V)
  + 16 nsec = 0.016 usec QS6K21 MOSFET
  = 61 usec

the propagation delay of the digital input circuit is
    60 usec TLP290-4 optocoupler ( max turn-off time @ 5V)

  however the measured maximum output turn off time is 113 usec and
  the measured turn on time is 10 usec.

for digital IOs this means we have a ratio of 26:113 ~ 1:4.5, in other words,
we can write the outputs 4.5 times faster than they can actually switch.

even at an update rate of 120 usec, doing the I2C access completely on the
CPU will slow down the CPU significantly.  because of that, it is currently
limited to 1200 usec.

FIXME: should implement asynchronous communication in the I2C driver and
       use a periodic timer to initiate the transfer to get a 120 usec
       period for inputs and outputs.
       120 usec total frame time - 2x26 i2c delay = ~70 usec timer period
       after the last i2c transfer has finished.
*/
static constexpr auto digital_io_min_update_time = std::chrono::microseconds (120);
static constexpr auto digital_io_poll_time = digital_io_min_update_time * 10;

template < typename PCADev >
class digital_inputs final : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 32;

  typedef PCADev pca9698_t;

  digital_inputs (pca9698_t& p) : m_pca9698 (p)
  {
    // inputs are inverted by default.
    m_and_mask.set ();
    m_or_mask.reset ();
    m_xor_mask.set ();

    sync ();
  }

  std::bitset<port_count> and_mask (void) const { return m_and_mask; }
  std::bitset<port_count> or_mask (void) const { return m_or_mask; }
  std::bitset<port_count> xor_mask (void) const { return m_xor_mask; }

  void set_and_mask (std::bitset<port_count> val) { m_and_mask = val; }
  void set_or_mask (std::bitset<port_count> val) { m_or_mask = val; }
  void set_xor_mask (std::bitset<port_count> val) { m_xor_mask = val; }

  std::bitset<port_count> read (void) const
  {
    return ((m_inputs & m_and_mask) | m_or_mask) ^ m_xor_mask;
  }

  void write (const std::bitset<port_count>&)
  {
  }

  void exec (std::chrono::high_resolution_clock::time_point cur_time)
  {
    if (cur_time - m_update_time >= digital_io_poll_time)
    {
      m_update_time = cur_time;
      sync_1 ();
    }
  }

  virtual void sync (void) override
  {
    m_update_time = std::chrono::high_resolution_clock::now ();
    sync_1 ();
  }

  virtual bool read_port (unsigned int n) const override
  {
    return n <= port_count ? read ()[n] : false;
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  pca9698_t& m_pca9698;

  std::bitset<port_count> m_inputs;
  std::chrono::high_resolution_clock::time_point m_update_time;

  std::bitset<port_count> m_and_mask;
  std::bitset<port_count> m_or_mask;
  std::bitset<port_count> m_xor_mask;

  void sync_1 (void)
  {
    using utils::get_bit;

    static_assert (port_count == 32, "");

    uint32_t pca_bits = m_pca9698.template input_ports <0, 24> ().to_ulong ();
    uint8_t p0 = dev::rx_gpio::pidr::p0;
    uint8_t p2 = dev::rx_gpio::pidr::p2;
    uint8_t p3 = dev::rx_gpio::pidr::p3;
    uint8_t p8 = dev::rx_gpio::pidr::p8;

    m_inputs = std::bitset<port_count> (
	pca_bits | (get_bit (p0, 2) << 24)
		 | (get_bit (p0, 7) << 25)
		 | (get_bit (p3, 2) << 26)
		 | (get_bit (p2, 5) << 27)
		 | (get_bit (p2, 4) << 28)
		 | (get_bit (p2, 2) << 29)
		 | (get_bit (p8, 7) << 30)
		 | (get_bit (p8, 6) << 31));
  }
};

// ---------------------------------------------------------------------------

template < typename PCADev >
class digital_outputs final : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 32;

  typedef PCADev pca9698_t;

  digital_outputs (pca9698_t& p) : m_pca9698 (p)
  {
    m_and_mask.set ();
    m_or_mask.reset ();
    m_xor_mask.reset ();

    sync ();
  }

  std::bitset<port_count> and_mask (void) const { return m_and_mask; }
  std::bitset<port_count> or_mask (void) const { return m_or_mask; }
  std::bitset<port_count> xor_mask (void) const { return m_xor_mask; }

  void set_and_mask (std::bitset<port_count> val) { m_and_mask = val; }
  void set_or_mask (std::bitset<port_count> val) { m_or_mask = val; }
  void set_xor_mask (std::bitset<port_count> val) { m_xor_mask = val; }

  // when reading the outputs, re-apply the AOX masks so that
  // write (read ()) will result in a no-operation.
  std::bitset<port_count> read (void) const
  {
    return m_outputs ^ m_xor_mask;
  }

  void write (const std::bitset<port_count>& val)
  {
    m_outputs = ((val & m_and_mask) | m_or_mask) ^ m_xor_mask;
  }

  void exec (std::chrono::high_resolution_clock::time_point cur_time)
  {
    if (cur_time - m_update_time >= digital_io_poll_time)
    {
      m_update_time = cur_time;
      sync_1 ();
    }
  }

  virtual void sync (void) override
  {
    m_update_time = std::chrono::high_resolution_clock::now ();
    sync_1 ();
  }

  virtual bool read_port (unsigned int n) const override
  {
    return n <= port_count ? read ()[n] : false;
  }

  virtual void write_port (unsigned int n, bool val) override
  {
    if (n < port_count)
      write (read ().set (n, val));
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  pca9698_t& m_pca9698;

  std::bitset<port_count> m_outputs;
  std::chrono::high_resolution_clock::time_point m_update_time;

  std::bitset<port_count> m_and_mask;
  std::bitset<port_count> m_or_mask;
  std::bitset<port_count> m_xor_mask;

  void sync_1 (void)
  {
    const auto cur_outputs = m_outputs;

    using utils::set_bit;
    using utils::get_bit;

    m_pca9698.template set_output_ports <0, 24> (cur_outputs.to_ulong ());

    // the output ports are shared with some other stuff.  to write a bit
    // in a port, it needs to be read, modified and written back.  if there's
    // an interrupt in between, something else could modify the port and
    // it would write back wrong data.
    // port 2: no other outputs (can overwrite whole output port register)
    // port 5: no other outputs (can overwrite whole output port register)
    // port 7: no other outputs (can overwrite whole output port register)
    // port b: pb4 = DAC_CS3N, pb5 = DAC_CS2N, pb6 = DAC_CS1N

    uint8_t p2 = 0;
    uint8_t p5 = 0;
    uint8_t p7 = 0;

    p2 = set_bit (p2, 3, cur_outputs[24]);

    p5 = set_bit (p5, 4, cur_outputs[25]);
    p5 = set_bit (p5, 5, cur_outputs[26]);
    p5 = set_bit (p5, 6, cur_outputs[27]);

    p7 = set_bit (p7, 3, cur_outputs[28]);
    p7 = set_bit (p7, 0, cur_outputs[31]);

    dev::rx_gpio::podr::p2 = p2;
    dev::rx_gpio::podr::p5 = p5;
    dev::rx_gpio::podr::p7 = p7;

    // be careful when modifying bits in output ports which are shared with
    // other stuff.  can't just read-modify-write the registgers.
    dev::rx_gpio::shared_output_port< decltype (dev::rx_gpio::podr::pb), 7 > ()
	.set_from (29, cur_outputs.to_ulong ());

    dev::rx_gpio::shared_output_port< decltype (dev::rx_gpio::podr::pb), 2 > ()
	.set_from (30, cur_outputs.to_ulong ());
  }

};

} // namespace dev
#endif // includeguard_mcbv1_board_dev_digital_io_includeguard

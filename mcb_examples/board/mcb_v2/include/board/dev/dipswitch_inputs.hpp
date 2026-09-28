
/*
  the 8-pin dispswitch "SW1"

  pin1: PA5 (*)
  pin2: PA6 (*)
  pin3: PA7 (*)
  pin4: P73
  pin5: PB2
  pin6: P35 (NMI)
  pin7: PC7
  pin8: MD (MDMONR)

  the resulting bitset will contain a '1' if a switch is in the logical
  on state and '0' for the off state.  for some switches this might not
  correspond to the electrical state, since some of them are inverted.

  pins marked with (*) are shared with the RX address bus.
  due to restrictions of the RX pin configuration options, the address lines
  are always output.  although address lines A5, A6, A6 are not used for
  anything on this board, they might toggle when software or DMA does contiguous
  accesses in the LAN9250 FIFO area.
  thus, when reading the dipswitch inputs, check that there is no external
  bus access going on, by reading the states of the CS output lines.
  if any of the CS line is on, ignore the PA5, PA6, PA7 line states and do
  not update them in RAM.
  when other devices are accessed on the RX external bus, address lines
  A5, A6, A7 are always set to avoid conflicts.

  FIXME: this scheme is subject to starvation effects, i.e. when something
         constantly accesses the external bus, it will be impossible to read
         the dipswitches.  some form of locking would help here.  at least
         it could be used to periodically update the switch status by
         suspending/blocking DMA transfers.
*/

#ifndef includeguard_board_mcbv2_dev_dipswitch_inputs_includeguard
#define includeguard_board_mcbv2_dev_dipswitch_inputs_includeguard

#include <bitset>
#include <dev/hwreg.hpp>
#include <dev/digital_io_port.hpp>
#include <dev/renesas/rx_gpio.hpp>

namespace dev
{

class dipswitch_inputs final : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 8;

  dipswitch_inputs (void)
  {
    // the first read to get the initial state is issued by the board
    // setup code, when it's safe to do so (it depends on other hw setup).
  }

  std::bitset<port_count> read (void) const
  {
    // A7,A6,A5 are configured as open-drain.  normally the address outputs
    // are low, if nothing uses the bus.  thus the open-drain output will
    // pull the line low.  open it for a short time by reading from the 8 bit
    // PCD RB2 register read area.  this will have no side-effect on the PCD
    // operation (i.e. does not clear interrupt bits) and will allow reading
    // the dipswitch status right after the access.

    std::bitset<8> porta_bits;
    std::bitset<8> port6_bits;
    port6_bits.set ();

    // during the read access, the open-drain address bits will open and
    // slowly rise to high-level (10K pull-up, if dipswitch is open).
    // it takes some time until it reaches the high-level threshold, so read
    // the ports a couple of times to get a stable value.
    *(volatile uint8_t*)(0x06000000 | 0b1110'0001);
    porta_bits |= std::bitset<8> (dev::rx_gpio::pidr::pa.read ());
    port6_bits &= std::bitset<8> (dev::rx_gpio::pidr::p6.read ());

    *(volatile uint8_t*)(0x06000000 | 0b1110'0001);
    porta_bits |= std::bitset<8> (dev::rx_gpio::pidr::pa.read ());
    port6_bits &= std::bitset<8> (dev::rx_gpio::pidr::p6.read ());

    port6_bits.flip ();
    port6_bits.reset (0);  // ignore P60 bit

    std::bitset<8> port3_bits (dev::rx_gpio::pidr::p3.read ());
    std::bitset<8> portb_bits (dev::rx_gpio::pidr::pb.read ());
    std::bitset<8> portc_bits (dev::rx_gpio::pidr::pc.read ());
    std::bitset<8> mdmonr_bits (mdmonr.read ());

    uint8_t r = 0;

    if (port6_bits.any ())
    {
      // if any P6 (CS lines) bits were set during the PA read, assume
      // that something else was accessing the external bus.
      r |= m_last_value[0] << 0;
      r |= m_last_value[1] << 1;
      r |= m_last_value[2] << 2;
      r ^= 0b111;
    }
    else
    {
      r |= porta_bits[5] << 0;
      r |= porta_bits[6] << 1;
      r |= porta_bits[7] << 2;
    }

    r |= portc_bits[1] << 3;
    r |= portb_bits[2] << 4;
    r |= port3_bits[5] << 5;
    r |= portc_bits[7] << 6;
    r |= mdmonr_bits[0] << 7;

    // PC7 is a pull-down when off, all the others are pull-ups.
    // so need to invert the logic of all except PC7.
    r ^= 0b1011'1111;

    return m_last_value = { r };
  }

  virtual bool read_port (unsigned int n) const override
  {
    return n <= port_count ? read ()[n] : false;
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }


private:
  mutable std::bitset<port_count> m_last_value;
  static constexpr hw_reg_r <uint16_t, const_addr<0x00080000>> mdmonr = { };
};

} // namespace dev
#endif // includeguard_board_mcbv2_dev_dipswitch_inputs_includeguard

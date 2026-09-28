
/*
  the 4-pin dispswitch "S1"

  pin0: P93
  pin1: P35
  pin2: PC7
  pin3: MD (MDMONR)

  the resulting bitset will contain a '1' if a switch is in the logical
  on state and '0' for the off state.  for some switches this might not
  correspond to the electical state, since some of them are inverted.
*/

#ifndef includeguard_board_mcbv13_dev_dipswitch_inputs_includeguard
#define includeguard_board_mcbv13_dev_dipswitch_inputs_includeguard

#include <bitset>
#include <dev/hwreg.hpp>
#include <dev/digital_io_port.hpp>

namespace dev
{

class dipswitch_inputs : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 4;

  dipswitch_inputs (void) { }

  std::bitset<port_count> read (void) const
  {
    std::bitset<8> port3_bits (port3_pidr.read ());
    std::bitset<8> port9_bits (port9_pidr.read ());
    std::bitset<8> portc_bits (portc_pidr.read ());
    std::bitset<8> mdmonr_bits (mdmonr.read ());

    uint8_t r = 0;

    r |= port9_bits[3] << 0;
    r |= port3_bits[5] << 1;
    r |= portc_bits[7] << 2;
    r |= mdmonr_bits[0] << 3;

    r ^= 0b1011;

/*  (RX) GCC 5 produces branches for the following equivalent code...

    std::bitset<port_count> r (pca_bits.to_ulong () << 2);
    r[0] = portc_bits[4];
    r[1] = portc_bits[1];
    r[7] = portc_bits[7];
    r[6] = mdmonr_bits[0];
*/
    return { r };
  }

  virtual bool read_port (unsigned int n) const override
  {
    return n <= port_count ? read ()[n] : false;
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }


private:
  static constexpr hw_reg_r <uint8_t, const_addr<0x0008C043>> port3_pidr = { };
  static constexpr hw_reg_r <uint8_t, const_addr<0x0008C049>> port9_pidr = { };
  static constexpr hw_reg_r <uint8_t, const_addr<0x0008C04C>> portc_pidr = { };
  static constexpr hw_reg_r <uint16_t, const_addr<0x00080000>> mdmonr = { };
};

} // namespace dev
#endif // includeguard_board_mcbv13_dev_dipswitch_inputs_includeguard

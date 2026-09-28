
/*
  the 8-pin dispswitch "S1"

  pin0: PC4
  pin1: PC0
  pin2: PCA9698 (U24) port 4.0
  pin3: PCA9698 (U24) port 4.1
  pin4: PCA9698 (U24) port 4.2
  pin5: PCA9698 (U24) port 4.3
  pin6: MD (MDMONR)
  pin7: PC7

  the resulting bitset will contain a '1' if a switch is in the logical
  on state and '0' for the off state.  for some switches this might not
  correspond to the electical state, since some of them are inverted.
*/

#ifndef includeguard_board_mcbv1_dev_dipswitch_inputs_includeguard
#define includeguard_board_mcbv1_dev_dipswitch_inputs_includeguard

#include <bitset>

#include <dev/hwreg.hpp>
#include <dev/pca9698.hpp>
#include <dev/digital_io_port.hpp>

namespace dev
{

template < typename PCADev >
class dipswitch_inputs final : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 8;

  typedef PCADev pca9698_t;

  dipswitch_inputs (pca9698_t& p) : m_pca9698 (p) { }

  std::bitset<port_count> read (void) const
  {
    auto pca_bits = m_pca9698.template input_ports < 32, 36 > ();
    std::bitset<8> portc_bits (portc_pidr.read ());
    std::bitset<8> mdmonr_bits (mdmonr.read ());

    uint8_t r = (uint8_t)(pca_bits.to_ulong () << 2);
    r |= portc_bits[4] << 0;
    r |= portc_bits[0] << 1;
    r |= portc_bits[7] << 7;
    r |= mdmonr_bits[0] << 6;

    r ^= 0b01111111;

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
  static constexpr hw_reg_r <uint8_t, const_addr<0x0008C04C>> portc_pidr = { };
  static constexpr hw_reg_r <uint16_t, const_addr<0x00080000>> mdmonr = { };

  pca9698_t& m_pca9698;
};

} // namespace dev
#endif // includeguard_board_mcbv1_dev_dipswitch_inputs_includeguard

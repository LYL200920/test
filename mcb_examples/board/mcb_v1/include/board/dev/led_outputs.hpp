/*

there are 4 LEDs connected directly to the MCU IO ports:
  PJ5: LD1
  PJ3: LD2
  PF5: LD3
  P05: LD4

  FIXME: accessing single LED ports can be optimized.  should do something
	 like in pca9698 to allow port ranges to be read/written.
	 this would reduce code size in those particular cases.

  there is a conflict on the RX Port0 because there's also another output
  allocated.  for that it's better to use interrupt safe bit manipulation
  instructions, or else we'd need to disable interrupts during the access.
  PortJ and PortF can be overwritten safely in this case.

P00:  SCI6 TX
P01:  SCI6 RX
P02:  EXP_IN10 (input)
P03:  SCI6 RS485 transmit enable      <<<<<<<
P04:  -/-
P05:  LD4 (output)                    <<<<<<<
P06: -/-
P07:  EXP_IN11 (input)

PJ0: -/-
PJ1: -/-
PJ2: -/-
PJ3: LD2 (output)
PJ4: -/-
PJ5: LD1 (output)
PJ6: -/-
PJ7: -/-

PF0: -/-
PF1: -/-
PF2: -/-
PF3: -/-
PF4: -/-
PF5: LD3 (output)
PF6: -/-
PF7: -/-

*/

#ifndef includeguard_mcbv1_board_dev_led_outputs_includeguard
#define includeguard_mcbv1_board_dev_led_outputs_includeguard

#include <cstdint>
#include <bitset>
#include <array>

#include <dev/rx_gpio.hpp>
#include <dev/digital_io_port.hpp>

namespace dev
{

class led_outputs final : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 4;

  led_outputs (void)
  {
    m_outputs.fill (false);
  }

  led_outputs (const std::array<bool, port_count>& initval)
  {
    write (initval);
  }

  const std::array<bool, port_count>& read (void) const { return m_outputs; }

  virtual bool read_port (unsigned int n) const override
  {
    return n <= port_count ? m_outputs[n] : false;
  }

  void write (const std::array<bool, port_count>& val)
  {
    m_outputs = val;

    ld1_out (val[0]);
    ld2_out (val[1]);
    ld3_out (val[2]);
    ld4_out (val[3]);
  }

  // for compatibility
  void write (std::bitset<port_count> val)
  {
    for (unsigned int i = 0; i < port_count; ++i)
      write_port (i, val[i]);
  }

  virtual void write_port (unsigned int i, bool val) override
  {
    if (i <= m_outputs.size ())
    {
      m_outputs[i] = val;

      switch (i)
      {
	default:
	  break;

	case 0:
	  ld1_out (val);
	  break;

	case 1:
	  ld2_out (val);
	  break;

	case 2:
	  ld3_out (val);
	  break;

	case 3:
	  ld4_out (val);
	  break;
      }
    }
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  std::array<bool, port_count> m_outputs;

  dev::rx_gpio::shared_output_port<decltype (dev::rx_gpio::podr::pj), 5> ld1_out = { };
  dev::rx_gpio::shared_output_port<decltype (dev::rx_gpio::podr::pj), 3> ld2_out = { };
  dev::rx_gpio::exclusive_output_port<decltype (dev::rx_gpio::podr::pf), 0xFF, 0x00> ld3_out = { };
  dev::rx_gpio::shared_output_port<decltype (dev::rx_gpio::podr::p0), 5> ld4_out = { };
};

} // namespace dev

#endif // includeguard_mcbv1_board_dev_led_outputs_includeguard

/*

there are 4 green LEDs connected directly to the MCU IO ports:

  P02: LD1   (port write value = 0x02)
  P03: LD2   (port write value = 0x04)
  P05: LD3   (port write value = 0x20)
  P07: LD4   (port write value = 0x80)

(port0 has only the LED outputs)

in addition, there are 4 red LEDs connected in parallel.  the color can be
selected by switching the LED_COL_SEL line:
   P56 (144 pin MCU)
   P96 (176 pin MCU, port register shared with "ETH2 LED green" on P97)

and another 2 LEDs are connected for the ETH2 RJ45 jack:
   ETH2 LED green: P97 (176 pin MCU)
                   P60 (144 pin MCU, single output port)
   ETH2 LED orange: PC5 (single output port)


in order to use this setup effectively, a PWM has to be implemented.
this can be done with a timer-DTC combination.

to implement brightness control, a 1-bit DAC scheme can be used, which
does timeslicing.  every bit is output as a pulse of certain length.
the PWM carrier frequency is fixed and the pulse duration is adjusted based
on the bit position of the brightness value.

for example, let's say the PWM carrier frequency = 2048 timer clocks.  then
the following table

brightness bit 7:  512 clocks
brightness bit 6:  256 clocks
brightness bit 5:  128 clocks
brightness bit 4:   64 clocks
brightness bit 3:   32 clocks
brightness bit 2:   16 clocks
brightness bit 1:    8 clocks
brightness bit 0:    4 clocks

will result in a 50% duty cycle for the maximum brightness value.

the 8 brightness values for each LED are stored in RAM.  when the software
wants to control the LED it just writes the corresponding value in RAM.

the following can be used:

variant A)

uint16_t timer_values[] = {  512, 256, 128,  64,  32,  16,   8,   4 };

LD1 = 0xFF                  0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20
LD1 = 0xFE                  0x20,0x20,0x20,0x20,0x20,0x20,0x20,   0
  ...
LD1 = 0xF0                  0x20,0x20,0x20,0x20,   0,   0,   0,   0
  ...
LD1 = 0x00                     0,   0,   0,   0,   0,   0,   0,   0


to enable one LED out of the 10 LEDs ...
144 pin version:
  - write port 0 (LD1..LD3 select)
  - write port 5 (LED_COL_SEL select)
  - write port 6 (ETH2 green select)
  - write port c (ETH2 orange select)

176 pin version:
  - write port 0 (LD1..LD3 select)
  - write port 9 (ETH2 green / LED_COL_SEL select)
  - write port c (ETH2 orange select)

this can be merged to make a combined table for 144 and 176 pin versions:
  - write port 0 (LD1..LD3 select)
  - write port 5 (LED_COL_SEL select)
  - write port 6 (ETH2 green select)
  - write port 9 (ETH2 green / LED_COL_SEL select)
  - write port c (ETH2 orange select)

for every LED on/off state, there are 5 port register tables.


when starting one PWM cycle for one LED, the CPU prepares a DTC transfer chain:

{
  *port_reg_p0 = *table_row_ptr_port++;
  *port_reg_p5 = *table_row_ptr_port++;
  *port_reg_p6 = *table_row_ptr_port++;
  *port_reg_p9 = *table_row_ptr_port++;
  *port_reg_pc = *table_row_ptr_port++;

  timer_count_reg = *timer_values_ptr++;
}

and starts the timer.

at each PWM timer interrupt, the DTC steps through the two arrays and rewrites
the port output register and the timer values.

at the end/beginning of the PWM cycle, the CPU prepares the data for the
brightness row array.  the data could be pre-calculated in ROM or calculated
on-the-fly between each LED PWM cycle.  for example, the 8 bit brightness value
can be decomposed into the byte array for the output port register by the CPU,
while the timer values remain fixed (could be different for each LED though).

it would also be possible to pre-calculate all the data and make the DTC
rewrite and chase pointers into those tables.  however, the decomposition of
8 byte value to on/off bytes would require a 256 x 8 = 2 KByte table.  because
the DTC can't do address calculations, it can only rewrite the lower 16 bit or
8 bit value of an address to do pointer + offset addition.  for the 2 KByte
table, it would require using 16 bit offsets and thus the table needs to be
aligned on 64 KByte.  this seems not so nice.


*/

#ifndef includeguard_mcbv13_board_dev_led_outputs_includeguard
#define includeguard_mcbv13_board_dev_led_outputs_includeguard

#include <cstdint>
#include <array>
#include <bitset>
#include <chrono>

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>
#include <dev/rx_gpio.hpp>
#include <dev/digital_io_port.hpp>


// the PWM idea is nice, but does not work on this board.
// the current pulses are not strong enough to get a sufficient LED brightness.

#if 0
namespace dev
{

class led_outputs
{
public:
  static constexpr unsigned int port_count = 10;

  enum led_id
  {
    ld1_green = 0,
    ld2_green = 1,
    ld3_green = 2,
    ld4_green = 3,

    ld1_red = 4,
    ld2_red = 5,
    ld3_red = 6,
    ld4_red = 7,

    eth2_green = 8,
    eth2_orange = 9,
  };


  led_outputs (void)
  {
    m_last_update_time = std::chrono::high_resolution_clock::now ();
    m_cur_led = 0;

    for (auto& o : m_outputs)
      o = 0;
  }

  led_outputs (const std::array<uint8_t, port_count>& initval)
  {
    write (initval);
  }

  const std::array<uint8_t, port_count>& read (void) const { return m_outputs; }
  uint8_t read (unsigned int i) const { return m_outputs[i]; }

  void write (const std::array<uint8_t, port_count>& val) { m_outputs = val; }
  void write (unsigned int i, uint8_t val) { if (i <= m_outputs.size ()) m_outputs[i] = val; }

  // for compatibility
  void write (std::bitset<port_count> val)
  {
    for (unsigned int i = 0; i < port_count; ++i)
      write (i, val[i]);
  }


  void exec (void)
  {
    // for now, cycle through all LEDs and turn them on/off according to
    // the state.
    // to get an update frequency of 50 Hz on each LED, we have to cycle
    // "port_count" times faster, i.e. 50 * 10 = 500 Hz = 2 ms.
    static constexpr auto update_time = std::chrono::milliseconds (1);

    auto cur_time = std::chrono::high_resolution_clock::now ();
    if (cur_time - m_last_update_time >= update_time)
    {
      static constexpr uint8_t port_values[port_count][3] =
      {
	// port0, port9, portc
	{   0x04, 0x00,  0x00 },  // LD1 green
	{   0x08, 0x00,  0x00 },  // LD2 green
	{   0x20, 0x00,  0x00 },  // LD3 green
	{   0x80, 0x00,  0x00 },  // LD4 green

	{   0x04, 0x00,  0x00 },  // LD1 red
	{   0x08, 0x00,  0x00 },  // LD2 red
	{   0x20, 0x00,  0x00 },  // LD3 red
	{   0x80, 0x00,  0x00 },  // LD4 red

	{   0x00, 0xFF,  0x00 },  // ETH2 green
	{   0x00, 0x00,  0xFF },  // ETH2 orange
      };

      const uint8_t* cur_port_values = port_values[m_cur_led];
      const uint8_t on_off_mask = m_outputs[m_cur_led] != 0 ? 0xFF : 0x00;

      dev::rx_gpio::podr::p0 = cur_port_values[0] & on_off_mask;
      dev::rx_gpio::podr::p9 = (cur_port_values[1] & on_off_mask) ^ 0xFF;
      dev::rx_gpio::podr::pc = (cur_port_values[2] & on_off_mask) ^ 0xFF;

      m_last_update_time = cur_time;

      if (++m_cur_led == port_count)
        m_cur_led = 0;
    }
  }

private:
  std::chrono::high_resolution_clock::time_point m_last_update_time;
  unsigned int m_cur_led;
  std::array<uint8_t, port_count> m_outputs;

};

} // namespace dev

#endif


namespace dev
{

class led_outputs : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 10;

  enum led_id
  {
    ld1_green = 0,
    ld2_green = 1,
    ld3_green = 2,
    ld4_green = 3,

    ld1_red = 4, // dummy, do not exist on this board.
    ld2_red = 5,
    ld3_red = 6,
    ld4_red = 7,

    eth2_green = 8,
    eth2_orange = 9,
  };


  led_outputs (void)
  {
    for (auto& o : m_outputs)
      o = 0;
  }

  led_outputs (const std::array<uint8_t, port_count>& initval)
  {
    write (initval);
  }

  const std::array<uint8_t, port_count>& read (void) const { return m_outputs; }

  virtual bool read_port (unsigned int n) const override
  {
    return n <= port_count ? m_outputs[n] : false;
  }

  void write (const std::array<uint8_t, port_count>& val)
  {
    for (unsigned int i = 0; i < port_count; ++i)
      write_port (i, val[i]);
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

	case ld1_green:
	case ld1_red:
	  ld1_out (val);
	  break;

	case ld2_green:
	case ld2_red:
	  ld2_out (val);
	  break;

	case ld3_green:
	case ld3_red:
	  ld3_out (val);
	  break;

	case ld4_green:
	case ld4_red:
	  ld4_out (val);
	  break;

	case eth2_green:
	  eth2_green_out (val);
	  break;

	case eth2_orange:
	  eth2_orange_out (val);
	  break;
      }
    }
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

  void exec (void) { }

private:
//  std::chrono::high_resolution_clock::time_point m_last_update_time;
//  unsigned int m_cur_led;

  std::array<uint8_t, port_count> m_outputs;

  dev::rx_gpio::shared_output_port<decltype (dev::rx_gpio::podr::p0), 2> ld1_out = { };
  dev::rx_gpio::shared_output_port<decltype (dev::rx_gpio::podr::p0), 3> ld2_out = { };
  dev::rx_gpio::shared_output_port<decltype (dev::rx_gpio::podr::p0), 5> ld3_out = { };
  dev::rx_gpio::shared_output_port<decltype (dev::rx_gpio::podr::p0), 7> ld4_out = { };
  dev::rx_gpio::exclusive_output_port<decltype (dev::rx_gpio::podr::p9), 0x00, 0xFF> eth2_green_out = { };
  dev::rx_gpio::exclusive_output_port<decltype (dev::rx_gpio::podr::pc), 0x00, 0xFF> eth2_orange_out = { };
};

} // namespace dev

#endif // includeguard_mcbv13_board_dev_led_outputs_includeguard

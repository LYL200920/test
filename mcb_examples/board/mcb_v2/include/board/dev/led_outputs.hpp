/*

there are 4 green LEDs and 4 read LEDs,
connected via 5 lines to the MCU IO ports:

  P02: LD1   (port write value = 0b1111'1011)      1*2-1 = 1 -> max (2,1) = 2
  P03: LD2   (port write value = 0b1111'0111)      2*2-1 = 3
  P05: LD3   (port write value = 0b1101'1111)      3*2-1 = 5
  P07: LD4   (port write value = 0b0111'1111)      4*2-1 = 7

  P56: LD_COL_SEL (144 pin MCU)
  P11: LD_COL_SEL (176 pin MCU)

all ports are exclusive output ports, which allows using the DTC or DMAC
to periodically update the port data.

the LED wiring on the board has been designed specifically for PWM control.
if any LED is permanently turned on, it will burn due to over-current.

to turn on one specific LED, it requires setting the LD_COL_SEL output
to select the red or green color and the LED number in port0.

the easiest way of PWM control is cycling through all LED values and turning
the respective LED on or off.  since there are 8 LEDs, each LED will be turned
on with a duty cycle of 1/8.  however, this will not give a sufficient
brightness.  in order to get a brightness that is comparable to the green LED
at constant 2.5 mA, need to do all 4 green LEDs at once and then all 4 red
LEDs.  in order to avoid visible "cross talk" or "ghosting" effects dead
times have to be inserted.  if the switch over from green to red (and vice
versa) is done instantly, current will flow into the other LED for a short
amount of time, even if the other LED is then never switched on.

the brightness of green or red can be adjusted/compensated by adjusting
the ratio of the on and off cycles.

*/

#ifndef includeguard_mcbv2_board_dev_led_outputs_includeguard
#define includeguard_mcbv2_board_dev_led_outputs_includeguard

#include <cstdint>
#include <array>
#include <bitset>
#include <chrono>
#include <algorithm>

#include <dev/hwreg.hpp>
#include <utils/bits.hpp>
#include <utils/langcomp.hpp>
#include <dev/renesas/rx_gpio.hpp>
#include <dev/digital_io_port.hpp>

namespace dev
{

#if defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
  #define ld_col_sel_podr dev::rx_gpio::podr::p1
#else
  #define ld_col_sel_podr dev::rx_gpio::podr::p5
#endif

template <typename DtcFunc, typename TimerChannelFunc>
class led_outputs final : public digital_io_port::dev_if
{
  using timer = std::remove_reference_t<std::invoke_result_t<TimerChannelFunc>>;
  using timer_duration = typename timer::duration;
  static constexpr auto& timer_inst (void) { return std::invoke (TimerChannelFunc ()); }

  using dtc = std::remove_reference_t<std::invoke_result_t<DtcFunc>>;
  static constexpr auto& dtc_inst (void) { return std::invoke (DtcFunc ()); }

  using dtc_insn = typename dtc::insn;

  // 6 phases, each phase has 3 dtc instructions to update
  //   LED on/off output register
  //   LD_COL_SEL output register
  //   timer trigger point value (duration of the phase)
  // the DTC will step through those 3 arrays at each timer trigger point.

  // FIXME: except for the p0 values, all other arrays could be placed in ROM.

  using timer_trigger_point_value_type =
    typename std::remove_reference<decltype (timer_inst ().trigger_hwreg ())>::type::base_type;

  using p0_podr_value_type =
    typename std::remove_reference<decltype (dev::rx_gpio::podr::p0)>::type::base_type;

  using ld_col_sel_podr_value_type =
    typename std::remove_reference<decltype (ld_col_sel_podr)>::type::base_type;

  // could also make all the array elements volatile to tell the compiler
  // to actually write out the values to RAM.  but that would pessimize
  // CPU read-access of the values.  use soft_memory_fence instead.
  std::array<p0_podr_value_type, 6> m_p0_values;
  std::array<timer_trigger_point_value_type, 6> m_timer_ticks;
  std::array<ld_col_sel_podr_value_type, 6> m_ld_col_sel_values;
  std::array<dtc_insn, 3> m_dtc_insns;

  // use signed value for std::max to get better code on RX
  static constexpr int ld_bit (int i)
  {
    return std::max (2, (i+1)*2 - 1);
  }

  // convert usecs to timer ticks with timer scaling
  static constexpr auto clk_div = 32;

  // the max brightness of all led 
  // 255 == std::numeric_limits<decltype(uint8_t)>::max()
  static constexpr uint8_t brightness_max = 255;
  
public:
  static constexpr unsigned int port_count = 8;

  enum led_id
  {
    ld1_green = 0,
    ld2_green = 1,
    ld3_green = 2,
    ld4_green = 3,

    ld1_red = 4,
    ld2_red = 5,
    ld3_red = 6,
    ld4_red = 7
  };

  [[gnu::cold]]
  led_outputs (void)
  {
    auto& timer = timer_inst ();

    ld_col_sel_podr = 0xFF;

    //                    green bits        red bits
    //                       vv               vv
    m_p0_values         = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    m_ld_col_sel_values = { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF };


    // use 1/32 timer clock scale
    // with PCLKB = 48 MHz that's 48/32 = 1.5 MHz timer frequency.
    //   -> 1 timer tick = 0.6666 usec
    timer.set_timer_control (dev::rx_cmt::cmcr_t ()
	.set_clock_select (dev::rx_cmt::pclk_32)
	.set_match_interrupt_enabled ());

    set_brightness (220, 255);

    // each DTC instruction is
    //  1+1 + 3*1+1 + 2*1 + 1+1 + WR + 2 = 12 + WR ICLKs
    //  PODR register write takes 3 PCLKB cycles, i.e. normally 6 ICLKs
    //  TPU TRGA register write takes 3 PCLKB cycles, i.e. normally 6 ICLKs
    //  thus for each instruction WR = 6, so each instruction is 18 ICLKs
    //  18 x 3 = 54 ICLKs at each invocation

    // the alternative approach to modify the DTC vector table entry address
    // is also 3 instructions per invocation, but writes only 2 registers and
    // 1 RAM location.
    // thus, it's 12+6 + 12+6 + 12+1 = 49 ICLKs at each invocation.
    // however, it's more complex and requires more setup code in ROM and
    // more RAM variables.
    m_dtc_insns =
    {
      dtc_insn ()
	.set_src_addr (m_timer_ticks.data ()).set_src_addr_mode (dev::dtca::post_inc)
	.set_dst_addr (&timer.trigger_hwreg ()).set_dst_addr_mode (dev::dtca::fixed)
	.template set_element_size < timer_trigger_point_value_type > ()
	.set_transfer_mode (dev::dtca::repeat)
	.set_src_area_repeat (true)
	.set_repeat_transfer_current_count (m_timer_ticks.size ())
	.set_repeat_transfer_count (m_timer_ticks.size ())
	.set_chain_mode (dev::dtca::always_chain),

     dtc_insn ()
	.set_src_addr (m_p0_values.data ()).set_src_addr_mode (dev::dtca::post_inc)
	.set_dst_addr (&dev::rx_gpio::podr::p0).set_dst_addr_mode (dev::dtca::fixed)
	.template set_element_size < p0_podr_value_type > ()
	.set_transfer_mode (dev::dtca::repeat)
	.set_src_area_repeat (true)
	.set_repeat_transfer_current_count (m_p0_values.size ())
	.set_repeat_transfer_count (m_p0_values.size ())
	.set_chain_mode (dev::dtca::always_chain),

     dtc_insn ()
	.set_src_addr (m_ld_col_sel_values.data ()).set_src_addr_mode (dev::dtca::post_inc)
	.set_dst_addr (&ld_col_sel_podr).set_dst_addr_mode (dev::dtca::fixed)
	.template set_element_size < ld_col_sel_podr_value_type > ()
	.set_transfer_mode (dev::dtca::repeat)
	.set_src_area_repeat (true)
	.set_repeat_transfer_current_count (m_ld_col_sel_values.size ())
	.set_repeat_transfer_count (m_ld_col_sel_values.size ())
	.set_chain_mode (dev::dtca::no_chain)
    };

    dtc_inst ().template insns< typename timer::cmi_interrupt_line > ()
      = m_dtc_insns.data ();

    soft_memory_fence ();

    dtc_inst ().template enable< typename timer::cmi_interrupt_line > ();

    // start the timer with some initial trigger.
    // this will get the dtc going.
    timer.set_counter (timer_duration (0));
    timer.set_trigger (timer_duration (100), dev::timer::enable);
    timer.start ();

#if 0
    static const volatile uint8_t const_0x00 = 0x00;
    static const volatile uint8_t const_0xFF = 0xFF;
    auto& timer = timer_inst ();
    auto& timer_trigger_point = timer.trigger_a_hwreg ();

    // template dtc instruction.
    // for every instruction use a repeat transfer mode with the counts of 1.
    // this will always reload the transfer count CRAL from CRAH, so it
    // will always stay "1".  this means, the transfers will loop infinitely
    // and never trigger a CPU interrupt, which is what we want.
    // unfortunately, the DTCa does not allow avoiding the write-back
    // completely, even through effectively none of the values change
    // (the DTCb of RX65 does).
    // each dtc instruction is 1+1 + 3*1+1 + 1 + 1+1 + 1 + 2 = min 12 ICLKs
    // 3x 12 = 36 ICLKs
    constexpr dtc_insn tmpl = dtc_insn ()
	.set_transfer_mode (dev::dtca::repeat)
	.set_repeat_transfer_count (1)
	.set_repeat_transfer_current_count (1)
	.set_chain_mode (dev::dtca::always_chain);

    m_dtc_insns =
    {
      // green LEDs on
      dtc_insn::fixed_addr_copy (tmpl, &m_green_p0_values, &dev::rx_gpio::podr::p0),
      dtc_insn::fixed_addr_copy (tmpl, &m_timer_ticks[green_on], &timer_trigger_point),
      dtc_insn::fixed_addr_copy (tmpl, &m_dtc_insn_addr[green_off_a], &dtc_vector_table_entry)
	.set_chain_mode (dev::dtca::no_chain),

      // all LEDs off
      dtc_insn::fixed_addr_copy (tmpl, &const_0xFF, &dev::rx_gpio::podr::p0),
      dtc_insn::fixed_addr_copy (tmpl, &m_timer_ticks[green_off_a], &timer_trigger_point),
      dtc_insn::fixed_addr_copy (tmpl, &m_dtc_insn_addr[green_off_b], &dtc_vector_table_entry)
	.set_chain_mode (dev::dtca::no_chain),

      // switch LD_COL_SEL to red
      dtc_insn::fixed_addr_copy (tmpl, &const_0x00, &ld_col_sel_podr),
      dtc_insn::fixed_addr_copy (tmpl, &m_timer_ticks[green_off_b], &timer_trigger_point),
      dtc_insn::fixed_addr_copy (tmpl, &m_dtc_insn_addr[red_on], &dtc_vector_table_entry)
	.set_chain_mode (dev::dtca::no_chain),

      // red LEDs on
      dtc_insn::fixed_addr_copy (tmpl, &m_red_p0_values, &dev::rx_gpio::podr::p0),
      dtc_insn::fixed_addr_copy (tmpl, &m_timer_ticks[red_on], &timer_trigger_point),
      dtc_insn::fixed_addr_copy (tmpl, &m_dtc_insn_addr[red_off_a], &dtc_vector_table_entry)
	.set_chain_mode (dev::dtca::no_chain),

      // all LEDs off
      dtc_insn::fixed_addr_copy (tmpl, &const_0xFF, &dev::rx_gpio::podr::p0),
      dtc_insn::fixed_addr_copy (tmpl, &m_timer_ticks[red_off_a], &timer_trigger_point),
      dtc_insn::fixed_addr_copy (tmpl, &m_dtc_insn_addr[red_off_b], &dtc_vector_table_entry)
	.set_chain_mode (dev::dtca::no_chain),

      // switch LD_COL_SEL to green
      dtc_insn::fixed_addr_copy (tmpl, &const_0xFF, &ld_col_sel_podr),
      dtc_insn::fixed_addr_copy (tmpl, &m_timer_ticks[red_off_b], &timer_trigger_point),
      dtc_insn::fixed_addr_copy (tmpl, &m_dtc_insn_addr[green_on], &dtc_vector_table_entry)
	.set_chain_mode (dev::dtca::no_chain)
    };

    m_dtc_insn_addr[0] = &m_dtc_insns[3*0];
    m_dtc_insn_addr[1] = &m_dtc_insns[3*1];
    m_dtc_insn_addr[2] = &m_dtc_insns[3*2];
    m_dtc_insn_addr[3] = &m_dtc_insns[3*3];
    m_dtc_insn_addr[4] = &m_dtc_insns[3*4];
    m_dtc_insn_addr[5] = &m_dtc_insns[3*5];

    dtc_vector_table_entry = m_dtc_insns.data ();
#endif
    #undef ld_col_sel_podr
  }

  ~led_outputs (void)
  {
    // make sure that the LEDs are off and the timer and dtc are stopped.
    // notice that the DTC might be running right now, so we need to sync with it.
    timer_inst ().stop ();
    dtc_inst ().template enable< typename timer::cmi_interrupt_line > (false);
    dtc_inst ().template wait_for < typename timer::cmi_interrupt_line > ();

    // turn off all LEDs
    dev::rx_gpio::podr::p0 = 0xFF;
  }

  led_outputs (const std::array<bool, port_count>& initval)
  : led_outputs ()
  {
    write (initval);
  }

  void write (unsigned int i, bool val)
  {
    write_1 (i, val);
    soft_memory_fence ();
  }

  // brightness represents the brightness of the green and red LED.
  // the valid range for brightness is from 0 to brightness_max.
  // note: 0 is the lowest brightness level, not turning off the LED.
  void set_brightness (uint8_t brightness)
  {
    set_brightness (brightness, brightness);
  }

  // set the brightness of the red and green LED
  // note: 0 is the lowest brightness level, not turning off the LED
  void set_brightness (uint8_t brightness_green, uint8_t brightness_red)
  {
    static_assert (clk_div == 32);
    constexpr auto led_switch_us = std::chrono::microseconds (70);

    auto led_green_on_us = std::chrono::microseconds (brightness_green * 2);
    auto led_green_off_us = std::chrono::microseconds ((brightness_max - brightness_green) * 2 + 70);

    auto led_red_on_us = std::chrono::microseconds (brightness_red * 3);
    auto led_red_off_us = std::chrono::microseconds ((brightness_max - brightness_red) * 2 + 70);

    #define make_timer_tick_value(x) (static_cast<timer_trigger_point_value_type>(std::chrono::duration_cast<timer_duration> (x).count () / clk_div))

    m_timer_ticks =
    {
      make_timer_tick_value (led_green_on_us),
      make_timer_tick_value (led_green_off_us),
      make_timer_tick_value (led_switch_us),
      make_timer_tick_value (led_red_on_us),
      make_timer_tick_value (led_red_off_us),
      make_timer_tick_value (led_switch_us)
    };

    #undef make_timer_tick_value

    soft_memory_fence ();
  }

  bool read (unsigned int i) const
  {
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Winvalid-offsetof"

    constexpr unsigned int m_p0_values_off = offsetof (led_outputs, m_p0_values);

    #pragma GCC diagnostic pop

    unsigned int red_green_off = (i & 4) == 0 ? m_p0_values_off : (m_p0_values_off+3);

    return !utils::get_bit (*((uint8_t*)this + red_green_off), ld_bit (i & 0b11));
  }

  // for compatibility
  void write (std::bitset<port_count> val)
  {
    // iterating over a bitset doesn't work really well with GCC.
    // it will fail to unroll the loop or generate any kind of reasonable code.
    // however, if we manually unroll the loop, it can fold it and create
    // one memory read/write for the whole bitset.  if the input bitset is
    // a compile time constant, it will even figure out the constant bit
    // masks (GCC 7).

//    for (unsigned int i = 0; i < port_count; ++i)
//      write_1 (i, val[i]);

    write_1 (0, val[0]);
    write_1 (1, val[1]);
    write_1 (2, val[2]);
    write_1 (3, val[3]);
    write_1 (4, val[4]);
    write_1 (5, val[5]);
    write_1 (6, val[6]);
    write_1 (7, val[7]);

    soft_memory_fence ();
  }

  void write (const std::array<bool, port_count>& val)
  {
//    for (unsigned int i = 0; i < port_count; ++i)
//      write_1 (i, val[i]);

    write_1 (0, val[0]);
    write_1 (1, val[1]);
    write_1 (2, val[2]);
    write_1 (3, val[3]);
    write_1 (4, val[4]);
    write_1 (5, val[5]);
    write_1 (6, val[6]);
    write_1 (7, val[7]);

    soft_memory_fence ();
  }

  void exec (void)
  {
  }

  virtual bool read_port (unsigned int i) const override
  {
    return read (i);
  }

  virtual void write_port (unsigned int i, bool val) override
  {
    write (i, val);
  }
  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  [[gnu::noinline]]
  void write_1 (unsigned int i, bool val)
  {
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Winvalid-offsetof"

    constexpr unsigned int m_p0_values_off = offsetof (led_outputs, m_p0_values);

    #pragma GCC diagnostic pop

    unsigned int red_green_off = (i & 4) == 0 ? m_p0_values_off : (m_p0_values_off+3);

    utils::atomic_set_bit (!val, ld_bit (i & 0b11), (uint8_t*)this + red_green_off);
  }
};

} // namespace dev

#endif // includeguard_mcbv2_board_dev_led_outputs_includeguard

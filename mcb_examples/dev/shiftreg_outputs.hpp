/*

digital outputs device which consists of chained shift register
latches (e.g. 74xx595)

the number of latches devices is determined by the number of outputs.
the output bits order uses bits[7:0] for the first latch in the chain,
bits[15:8] for the second latch in the chaain and so on.

a timer device is used to periodically clock out the serial data and update
the outputs. each clock takes two timer ticks (clock low state + clock high state).
thus the actual output rate for each individual output is
    clk frequency / ((latch_count*8 + 2)*2)

for example, 16 outputs:
    100 kHz timer / (((2*8+2)*2) = 2.77 kHz IO update rate
    24 kHz timer / (((2*8+2)*2) = 666 Hz IO update rate
    10 kHz timer / (((2*8+2)*2) = 277 Hz IO update rate
*/

#ifndef includeguard_shiftreg_outputs_hpp_includeguard
#define includeguard_shiftreg_outputs_hpp_includeguard

#include <cstdint>
#include <bitset>
#include <array>

#include <utils/bits.hpp>
#include <utils/langcomp.hpp>
#include <utils/byte_order.hpp>
#include <dev/digital_io_port.hpp>

namespace dev
{

template <unsigned int OutputCount,
	  typename DO_OutputFunc,
	  typename CLK_OutputFunc,
	  typename SET_OutputFunc,
	  typename OE_OutputFunc,
	  typename TimerChannelFunc,
	  unsigned int ClkFrequencyHz>
class shiftreg_outputs final : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = OutputCount;

  static constexpr auto& timer_inst (void) { return std::invoke (TimerChannelFunc ()); }
  using base_timer = std::remove_reference_t<std::invoke_result_t<TimerChannelFunc>>;
  using rep = typename base_timer::duration::rep;

  // round the desired base frequency to something that can be represented
  // by the actual timer.
  using period_base_timer_ticks =
	std::ratio_divide< std::ratio<1, ClkFrequencyHz>,
			   typename base_timer::duration::period >;

  constexpr static unsigned int period_base_timer_ticks_i =
	period_base_timer_ticks::num/period_base_timer_ticks::den;

  using period = std::ratio_multiply < std::ratio<period_base_timer_ticks_i, 1>,
				       typename base_timer::duration::period >;

  using duration = std::chrono::duration < rep, period >;



  shiftreg_outputs (DO_OutputFunc do_out = DO_OutputFunc (),
		    CLK_OutputFunc clk_out = CLK_OutputFunc (),
		    SET_OutputFunc set_out = SET_OutputFunc (),
		    OE_OutputFunc oe_out = OE_OutputFunc ())
  : m_do_out (do_out), m_clk_out (clk_out), m_set_out (set_out), m_oe_out (oe_out)
  {
    m_outputs.fill (0);
    m_and_mask.fill (0xFF);
    m_or_mask.fill (0);
    m_xor_mask.fill (0);
    update_set_clear_val ();

    // initially disable the outputs.  they will be enabled after clocking
    // out the values.
    m_do_out ().write (0);
    m_clk_out ().write (0);
    m_set_out ().write (0);
    m_oe_out ().write (1);

    auto&& timer = timer_inst ();

    // FIXME: SP-51 Add uniform timer interface class
    timer.set_timer_control (typename base_timer::tcr_t ()
	.set_count_clock_type (dev::rx_tpua::pclk_1)
	.set_count_clock_edge_type (dev::rx_tpua::rising_edge)
	.set_counter_clear (dev::rx_tpua::by_trga));

    timer.set_counter (typename base_timer::duration (0));
    timer.set_trigger_a (typename base_timer::duration (period_base_timer_ticks_i),
			 [this] (void) { serial_out_tick (); });

    std::atomic_signal_fence (std::memory_order_release);
    timer.start ();

  }

  ~shiftreg_outputs (void)
  {
    timer_inst ().stop ();
  }

  std::bitset<port_count> and_mask (void) const { return to_bitset (m_and_mask); }
  std::bitset<port_count> or_mask (void) const { return to_bitset (m_or_mask); }
  std::bitset<port_count> xor_mask (void) const { return to_bitset (m_xor_mask); }

  void set_and_mask (const std::bitset<port_count>& val) { from_bitset (val, m_and_mask); update_set_clear_val (); }
  void set_or_mask (const std::bitset<port_count>& val) { from_bitset (val, m_or_mask); update_set_clear_val (); }
  void set_xor_mask (const std::bitset<port_count>& val) { from_bitset (val, m_xor_mask); update_set_clear_val (); }


  std::bitset<port_count> read (void) const
  {
    std::bitset<port_count> r;

    for (unsigned int i = 0; i < latch_count; ++i)
      r |= decltype (r) (m_outputs[i] ^ m_xor_mask[i]) << (i*8);

    return r;
  }

  void write (const std::bitset<port_count>& val)
  {
    auto b = val;

    for (unsigned int i = 0; i < latch_count; ++i)
    {
      m_outputs[i] = (((uint8_t)b.to_ulong () & m_and_mask[i]) | m_or_mask[i]) ^ m_xor_mask[i];
      b >>= 8;
    }
  }

  void exec (std::chrono::high_resolution_clock::time_point /* cur_time */)
  {
  }

  virtual void sync (void)
  {
    // wait for update cycle counter to change, which indicates completion of
    // one output update cycle.
    const auto x = m_serial_out_sync_count;

    while (m_serial_out_sync_count == x) { };
  }

  virtual bool read_port (unsigned int i) const override
  {
    return utils::get_bit (m_outputs[i / 8] ^ m_xor_mask[i / 8], i % 8);
  }

  virtual void write_port (unsigned int i, bool val) override
  {
    utils::atomic_copy_bit (i % 8, m_clear_set_val[val][i / 8], i % 8, &m_outputs[i / 8]);
  }

  digital_io_port operator [] (unsigned int n)
  {
    if (unlikely (n >= port_count))
      return { &digital_io_port::g_null_dev, n };

    return { this, n };
  }

private:
  static constexpr unsigned int latch_count = (port_count + 7) / 8;

  std::array<uint8_t, latch_count> m_outputs;
  std::array<uint8_t, latch_count> m_and_mask;
  std::array<uint8_t, latch_count> m_or_mask;
  std::array<uint8_t, latch_count> m_xor_mask;
  std::array<uint8_t, latch_count> m_clear_set_val[2];

  DO_OutputFunc m_do_out;
  CLK_OutputFunc m_clk_out;
  SET_OutputFunc m_set_out;
  OE_OutputFunc m_oe_out;

  enum struct serial_out_state
  {
    init,
    clock_out_data,
    set_data,
    enable_output
  };

  serial_out_state m_serial_out_state = serial_out_state::init;
  unsigned int m_serial_out_clock_count;
  volatile unsigned int m_serial_out_sync_count = 0;

  void update_set_clear_val (void)
  {
    for (unsigned int i = 0; i < latch_count; ++i)
    {
      m_clear_set_val[0][i] = ((0x00 & m_and_mask[i]) | m_or_mask[i]) ^ m_xor_mask[i];
      m_clear_set_val[1][i] = ((0xFF & m_and_mask[i]) | m_or_mask[i]) ^ m_xor_mask[i];
    }
  }

  static std::bitset<port_count> to_bitset (const std::array<uint8_t, latch_count>& val)
  {
    std::bitset<port_count> r;

    for (unsigned int i = 0; i < latch_count; ++i)
      r |= decltype (r) (val[i]) << (i*8);

    return r;
  }

  static void from_bitset (const std::bitset<port_count>& val_in, std::array<uint8_t, latch_count>& val_out)
  {
    auto b = val_in;

    for (unsigned int i = 0; i < latch_count; ++i)
    {
      val_out[i] = (uint8_t)b.to_ulong ();
      b >>= 8;
    }
  }

  void serial_out_tick (void)
  {
    switch (m_serial_out_state)
    {
    default:
      unreachable;
      break;

    case serial_out_state::init:
      m_set_out ().write (0);
      m_do_out ().write (0);
      m_serial_out_clock_count = 0;
      m_serial_out_state = serial_out_state::clock_out_data;
      break;

    case serial_out_state::clock_out_data:
    {
      unsigned int bit_i = m_serial_out_clock_count / 2;
      unsigned int latch_i = latch_count - (bit_i / 8) - 1;
      bool bit_val = utils::get_bit (m_outputs[latch_i], (~bit_i % 8));

      m_do_out ().write (bit_val);
      m_clk_out ().write (m_serial_out_clock_count & 1);

      if (++m_serial_out_clock_count == latch_count * 8 * 2)
	m_serial_out_state = serial_out_state::set_data;

      break;
    }

    case serial_out_state::set_data:
      m_set_out ().write (1);
      m_serial_out_state = serial_out_state::enable_output;
      break;

    case serial_out_state::enable_output:
      m_set_out ().write (0);
      m_oe_out ().write (0);
      m_serial_out_state = serial_out_state::init;
      ++m_serial_out_sync_count;
      break;
    }
  }
};

}
#endif // includeguard_shiftreg_outputs_hpp_includeguard

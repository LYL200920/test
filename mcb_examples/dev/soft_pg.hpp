
/*

a software pulse generator, which uses one (hardware) timer to generate
up to N channels of pulse outputs.

each channel consists of a 20 bit up-counter.  when the counter overflows,
it is reloaded with the next value from a linked list of counter values.
this allows constructing arbitrary types of pulse outputs, like frequency
dithering for PTO motor control or PWM with duty cycle and phase shift.

each channel can operate in a different mode like free running toggle mode
or edge counting toggle mode.

all channels in a soft-pg device are synchronized.  an external timer device
is used to trigger the channel processing at the specified carrier frequency.

*/

#ifndef includeguard_dev_soft_pg_includeguard
#define includeguard_dev_soft_pg_includeguard

#include <chrono>
#include <type_traits>
#include <functional>
#include <tuple>

#include <utils/bits.hpp>
#include <utils/byte_order.hpp>
#include <utils/value_range.hpp>

namespace dev
{
namespace soft_pg_impl
{

struct channel_base
{
  // bit 7: current table index (0 or 1)
  // bit 6: channel enable/disable
  // bit [5:0] mode (used as function snippet number)
  uint8_t m_mode;

  // channel number
  int8_t m_ch_num;

  // offset of the processing function in the code buffer for all channels.
  int16_t m_code_offset;

  // [0]: first 4 bytes of code in disabled-mode (bra.a pcdsp:24 insn).
  //      can also be used to obtain the function code size.
  //
  // [1]: first 4 bytes of code in enabled-mode.
  uint32_t m_first_off_on_code_bytes[2];

  // bit [31:12] - 20 bit up-counter.  overflow = carry
  // bit [11]    - flag
  // bit [10]    - disable flag
  // bit [9:0]   - 10 bit next table index
  uint32_t m_count_table_idx_flg;

  // table is always 8 byte aligned.
  // lowest bits are optional flags.
  // bit [31:3]  - table base address
  // bit [2]     - flag
  // bit [1]     - flag
  // bit [0]     - flag
  uint32_t* m_cur_table_ptr;

  // edge counter in case it is used by the channel mode
  uint32_t m_edge_counter;

  // two tables for changing pulse configurations on the fly
  constexpr static unsigned int table_size = 32;

  std::array<uint32_t, table_size> m_table[2];
};

#ifndef __RX__
  #warning implemented only on RX
#else

// the channel processing functions will get copied into RAM.
// each function is specialized for the specific output channel.
//
// the run time generated code is invoked through a special inline asm
// snippet, to tell the compiler about the registers used/clobbered:
//   r1: channel_base*
//   r2: toggle outputs
//   r3: tmp0
//   r4: tmp1
//   r5: tmp2
//
// n.b. can't use local explicit register variables in templates due to gcc bug.

template < unsigned int sizeof_ch >inline void
process_channel_inc_ch_ptr (struct channel_base* ch, uint32_t toggle_outputs)
{
  asm volatile (
"	.short	1f - 0f"	"\n"
"0:\n"
	"add	%1,%0"		"\n"
"1:\n"
  : "+r" (ch)
  : "i" (sizeof_ch)
  : "cc");
}

template< unsigned int ch_num > inline void
process_channel_free_run_output_toggle (struct channel_base* ch, uint32_t toggle_outputs)
{
  asm volatile (
"	.short	2f - 0f"	"\n"
"0:\n"
"	mov.l	%A3[%1], r3"	"\n"
"	add	#(1<<12), r3"	"\n"
"	bnc	1f"		"\n"

	// counter overflow
"	bset	%2, %0"		"\n"

	// reload counter with new value from table
"	mov.l	%A4[%1], r4"	"\n"
"	and	#0x3FF, r3"	"\n"
"	mov.l	[r3, r4], r3"	"\n"
"1:\n"
"	mov.l	r3, %A3[%1]"	"\n"
"2:\n"

  : "+&r" (toggle_outputs) // 0
  : "r" (ch), // 1
    "i" (ch_num), // 2
    "i" (offsetof (struct channel_base, m_count_table_idx_flg)), // 3
    "i" (offsetof (struct channel_base, m_cur_table_ptr)) // 4
  : "cc", "memory");
}

template< unsigned int ch_num > inline void
process_channel_edge_count_output_toggle (struct channel_base* ch, uint32_t toggle_outputs)
{
  asm volatile (
	// if bit 10 flag is set, assume that edge counter has reached
	// zero and the channel is supposed to stop.
"	.short	2f - 0f"	"\n"
"0:\n"
"	mov.l	%A3[%1], r3"	"\n"
"	btst	#10, r3"	"\n"
"	bnz	2f"		"\n"

"	add	#(1<<12), r3"	"\n"
"	bnc	1f"		"\n"

	// counter overflow

	// decrement & test edge counter
	// set bit 10 flag if edge counter has reached zero
"	mov.l	%A5[%1], r4"	"\n"
"	bset	%2, %0"		"\n"
"	sub	#1, r4"		"\n"
"	bmz	#10, r3"	"\n"
"	mov.l	r4, %A5[%1]"	"\n"
"	bz	1f"		"\n"

	// reload counter with new value from table
"	mov.l	%A4[%1], r4"	"\n"
"	and	#0x3FF, r3"	"\n"
"	mov.l	[r3, r4], r3"	"\n"
"1:\n"
"	mov.l	r3, %A3[%1]"	"\n"
"2:\n"

  : "+&r" (toggle_outputs) // 0
  : "r" (ch), // 1
    "i" (ch_num), // 2
    "i" (offsetof (struct channel_base, m_count_table_idx_flg)), // 3
    "i" (offsetof (struct channel_base, m_cur_table_ptr)), // 4
    "i" (offsetof (struct channel_base, m_edge_counter)) // 5
  : "cc", "memory");
}

struct dev_base
{
  static unsigned int constexpr max_channel_count = 32;

  template <size_t... I>
  constexpr static inline std::array<const void*, sizeof... (I)>
  process_channel_free_run_output_toggle_funcs (std::index_sequence<I...>)
  {
    return {{ ((const void*)&soft_pg_impl::process_channel_free_run_output_toggle<I>)... }};
  }

  template <size_t... I>
  constexpr static inline std::array<const void*, sizeof... (I)>
  process_channel_edge_count_output_toggle_funcs (std::index_sequence<I...>)
  {
    return {{ ((const void*)&soft_pg_impl::process_channel_edge_count_output_toggle<I>)... }};
  }

  static std::pair<uint8_t*, unsigned int>
  copy_func (uint8_t* out_ptr, unsigned int func_id, unsigned int ch_count)
  {
    static constexpr auto funcs0 = process_channel_free_run_output_toggle_funcs (std::make_index_sequence< max_channel_count > ());
    static constexpr auto funcs1 = process_channel_edge_count_output_toggle_funcs (std::make_index_sequence< max_channel_count > ());

    static constexpr std::array<const void*, max_channel_count> funcs[] = { funcs0, funcs1 };

    return copy_func (out_ptr, funcs[func_id][ch_count]);
  }

  static std::pair<uint8_t*, unsigned int>
  copy_func (uint8_t* out_ptr, const void* func_ptr)
  {
    const int16_t* func_ptr16 = (const int16_t*)func_ptr;

    auto func_sz = *func_ptr16;
    std::memcpy (out_ptr, func_ptr16 + 1, func_sz);
    return { out_ptr + func_sz, func_sz };
  }

  static void gen_proc_code (uint8_t* code_out_begin, uint8_t* code_out_end,
			     channel_base* ch_begin, channel_base* ch_end, unsigned int ch_sz,
			     const void* inc_ch_ptr_func)
  {
    uint8_t* code_out = code_out_begin;
    unsigned int i = 0;

    for (auto* c = ch_begin; c != ch_end; c = (channel_base*)((uintptr_t)c + ch_sz), ++i)
    {
      if (c != ch_begin)
	code_out = copy_func (code_out, inc_ch_ptr_func).first;

      if (unsigned int mode_num = c->m_mode & 0b111111)
      {
	c->m_code_offset = code_out - code_out_begin;
	auto code_out_n_sz = copy_func (code_out, mode_num - 1, i);

	// to quickly enable/disable (start/stop) a channel, replace its first
	// 4 bytes with the opcode "bra pcdsp:24" to jump over the whole code block
	// for that channel when it is turned off.  when it is turned on, run
	// the normal code of the function snippet.

	//  opcode: 0b00000100 dddddddd dddddddd dddddddd

	//  example: bra.a 0x827E = 04 7E 82 00

	c->m_first_off_on_code_bytes[0] = (0x04 << 0) | code_out_n_sz.second << 8; // "bra.a pcdsp:24" insn
	c->m_first_off_on_code_bytes[1] = *(uint32_t*)code_out;

	//set_channel_enable (i, utils::get_bit (c.m_mode, 6));
	// initially the channel code is enabled.  if enabled flag is not
	// set, overwrite with disabled-mode code.
	if (!utils::get_bit (c->m_mode, 6))
	  *(uint32_t*)code_out = c->m_first_off_on_code_bytes[0];

	code_out = code_out_n_sz.first;
      }
    }

    // append "mov.l r2,r1" insn to get the return value into place.
    *code_out++ = 0xEF;
    *code_out++ = 0x21;

    // append "rts" insn for function return
    *code_out++ = 0x02;
  }

  template <unsigned int table_size>
  static inline std::pair<unsigned int, unsigned int>
  gen_freq_dither_table (uint32_t* table_out, uint32_t alpha,
			 uint32_t val0, uint32_t val1)
  {
    // use the carry overflow of the addition operation as an indicator
    // when to output a "tc" value.  the step count (slope of linear interpolation)
    // is always <= 1 and we store only the 32 bit fraction of it.
    uint32_t step = ((uint64_t)alpha << 32) / table_size;

    uint32_t x = 0;
    uint32_t tmp, tmp1;
    unsigned int val0_count = 0;
    unsigned int val1_count = 0;
/*
    for (unsigned int i = 0; i < table_size; ++i)
    {
      x += step;

      if (x >= (1ll << 32))
      {
	x -= (1ll << 32);
	table_out[i] = val0 | ((i + 1) % table_size);
	++val0_count;
      }
      else
      {
	table_out[i] = val1 | ((i + 1) % table_size);
	++val1_count;
      }
    }
*/
    for (unsigned int i = 0; i < table_size; ++i)
    {
      #if __RXv2__

      asm volatile (
"	add	%[step],%[x]"	"\n"	// carry = overflow
					// can't use carry bit for conditional store, must use zero flag
"	sbb	%[tmp],%[tmp]"	"\n"	//    overflow: tmp = 0x00000000, z = 1
					// no overflow: tmp = 0xFFFFFFFF, z = 0
"	stnz	%[val1],%[tmp]"	"\n"	// no overflow: use val1
"	stz	%[val0],%[tmp]"	"\n"	//    overflow: use val0
"	or	%[tag],%[tmp]"	"\n"
"	mov.l	%[tmp],[%[table_out]+]"	"\n"

	: [x] "+r" (x), [tmp] "+r" (tmp), [table_out] "+r" (table_out)
	: [step] "ri" (step), [tag] "ri" ((i+1) % table_size),
	  [val0] "ri" (val0), [val1] "ri" (val1)
	: "cc", "memory"
      );

      #else

      asm volatile (
"	add	%[step],%[x]"		"\n"	// carry = overflow
"	sbb	%[tmp],%[tmp]"		"\n"	// overflow: tmp = 0x00000000, no overflow: tmp = 0xFFFFFFFF
"	and	%[tmp],%[val1],%[tmp1]"	"\n"
"	not	%[tmp]"			"\n"
"	and	%[val0],%[tmp]"		"\n"
"	or	%[tmp1],%[tmp]"		"\n"
"	or	%[tag],%[tmp]"		"\n"
"	mov.l	%[tmp],[%[table_out]+]"	"\n"

	: [x] "+r" (x), [tmp] "+r" (tmp), [tmp1] "+r" (tmp1), [table_out] "+r" (table_out)
	: [step] "ri" (step), [tag] "ri" ((i+1) % table_size),
	  [val0] "ri" (val0), [val1] "ri" (val1)
	: "cc", "memory"
      );

      #endif
    }

    return { val0_count, val1_count };
  }

};


#endif // __RX__

} // namespace soft_pg_impl


template < unsigned int BaseFrequencyHz, typename TimerChannelFunc,
	   unsigned int CtrlCallbackFreqDiv, typename CtrlCallbackFunc,
	   typename... ChannelOutputFuncs >
class soft_pg : protected soft_pg_impl::dev_base
{
public:
  enum channel_mode : int8_t
  {
    disabled,
    free_run_output_toggle,
    edge_count_output_toggle
  };

  using channel_output_funcs = std::tuple<ChannelOutputFuncs... >;

  static constexpr unsigned int channel_count = std::tuple_size_v<channel_output_funcs>;

  static constexpr auto& timer_inst (void) { return std::invoke (TimerChannelFunc ()); }
  using base_timer = std::remove_reference_t<std::invoke_result_t<TimerChannelFunc>>;
  using rep = typename base_timer::duration::rep;

  // round the desired base frequency to something that can be represented
  // by the actual timer.
  using period_base_timer_ticks =
	std::ratio_divide< std::ratio<1, BaseFrequencyHz>,
			   typename base_timer::duration::period >;

  constexpr static unsigned int period_base_timer_ticks_i =
	period_base_timer_ticks::num/period_base_timer_ticks::den;

  using period = std::ratio_multiply < std::ratio<period_base_timer_ticks_i, 1>,
				       typename base_timer::duration::period >;

  using duration = std::chrono::duration < rep, period >;

  using ctrl_period = std::ratio_divide < period, std::ratio<CtrlCallbackFreqDiv> >;
  using ctrl_duration = std::chrono::duration < rep, ctrl_period >;

  class channel : protected soft_pg_impl::channel_base
  {
  public:
    constexpr static uint32_t counter_max_value = (1u << 20) - 1;

    using duration = soft_pg::duration;
    using period = typename duration::period;
    using rep = typename duration::rep;

    using soft_pg_dev_t = soft_pg;

    channel_mode mode (void) const { return (channel_mode)(m_mode & 0b111111); }

    void set_mode (channel_mode m)
    {
      if (m == mode ())
	return;

      m_mode = (m_mode & 0b11000000) | ((unsigned int)m & 0b111111);
      outer ().gen_proc_code ();
    }

    void enable (bool val = true)
    {
      // could also use xchg instruction to swap 4 bytes and save 4 bytes of RAM.
      auto& p = *(uint32_t*)(outer ().m_codebuffer.begin () + m_code_offset);
      p = m_first_off_on_code_bytes[val];
      m_mode = utils::set_bit (m_mode, 6, val);
    }

    void disable (void) { enable (false); }

    bool is_enabled (void) const
    {
      // if edge_count_output_toggle mode, then also need to check the bit in
      // counter variable.  in other modes the flag in the counter will always
      // be zero.
      return utils::get_bit (m_mode, 6) & !utils::get_bit (m_count_table_idx_flg, 10);
    }

    void reset (void)
    {
      m_count_table_idx_flg = m_cur_table_ptr[0];
    }

    using edge_counter_type = decltype (m_edge_counter);

    void set_edge_counter (edge_counter_type val) { m_edge_counter = val; }
    edge_counter_type edge_counter (void) const { return m_edge_counter; }

    // simple pwm pulse shape with two counter values
    // this assumes that it is already in simple pwm mode.
    template <typename RepA, typename PeriodA, typename RepB, typename PeriodB>
    void set_pulse_shape_simple_pwm (const std::chrono::duration<RepA, PeriodA>& count_a_,
				     const std::chrono::duration<RepB, PeriodB>& count_b_)
    {
      auto count_a = std::chrono::duration_cast<duration> (count_a_);
      auto count_b = std::chrono::duration_cast<duration> (count_b_);

      // generate new values into other table
      auto& t = m_table[(m_mode >> 7) ^ 1];

      t[0] = conv_counter_value (count_a.count ()) | 1;
      t[1] = conv_counter_value (count_b.count ()) | 0;

      // swap tables
      m_cur_table_ptr = t.data ();
      m_mode ^= 1 << 7;
    }

    // ~50% duty cycle pulse with frequency dithering
    // returns the actual resulting pps value.
    static constexpr unsigned int pps_min_value = 1;
    static constexpr unsigned int pps_max_value = period::den / (period::num * 2);

    [[gnu::noinline, gnu::noclone]]
    unsigned int set_pulse_shape_pps (utils::clamped_value<unsigned int, pps_min_value, pps_max_value> pps)
    {
      // 400 pps = 400 hz = 1/400
      // 1/400 / (1/75000) = 187.5
      // 1/ (187 * (1/75000)) = 401.069518717
      constexpr unsigned int pd = period::den;
      constexpr unsigned int pn = period::num;
      constexpr unsigned int tz = table_size;

      // this function uses 32 bit calculations, but the timer constants might be
      // 64 bit values.
      static_assert (pd == period::den);
      static_assert (pn == period::num);

      // we need to generate rising edge and falling edge, so edge frequency
      // is double of the pulse frequency.
      unsigned int pps_2 = pps * 2;

      // calculate the number of ticks as a fixed point number.
      // the number of fraction bits depends on the table size.
      // if table size is pow2 it will get optimized by the compiler.
      unsigned int tz_tf = (tz * pd) / (pps_2 * pn);

      // linear interpolation blend factor is the fraction.
      unsigned int alpha_0 = tz_tf % tz;

      [[maybe_unused]] unsigned int alpha_1 = tz - alpha_0;

      unsigned int tf = tz_tf / tz;
      unsigned int tc = tf + 1;

      // the final output frequency depends on the number of "tf" values
      // and the "tc" values in the table, i.e. the average tick frequency.
      // the following calculation can be simplified by substition.
      // unsigned int pps_out = (pd * tz) / (pn * 2 * (tf*alpha_1 + tc*alpha_0));
      unsigned int pps_out = ((pd*tz) / (2*pn)) / (tf*tz + alpha_0);

      // generate new table values
      // output tf and tc tick values in an alternating way.

      auto& t = m_table[(m_mode >> 7) ^ 1];

      [[maybe_unused]] auto alpha0_alpha1_count =
	gen_freq_dither_table <tz> (t.data (), alpha_0, conv_counter_value (tc), conv_counter_value (tf));

//      std::printf ("set_pulse_shape_pps %u (%u)  tc: %u tf: %u  alpha: %u / %u (%u / %u), pps out: %u\n",
//	(unsigned int)pps, pps_2, tc, tf, alpha_0, alpha_1, alpha0_alpha1_count.first, alpha0_alpha1_count.second, pps_out);

      // swap tables
      m_cur_table_ptr = t.data ();
      m_mode ^= 1 << 7;

      return pps_out;
    }

    channel (void) { }
    channel (const channel&) = delete;
    channel& operator = (const channel&) = delete;

    uint32_t counter (void) const { return m_count_table_idx_flg >> 12; }

    bool output (void) const { return utils::get_bit (m_ch_num, m_outer->m_outputs); }

    void set_output (bool val)
    {
      constexpr bool is_be = utils::native_byte_order () == utils::big_endian;
      const unsigned int n = (m_ch_num / 8u) ^ (is_be ? 0b11 : 0b00);

      utils::atomic_set_bit (val, m_ch_num, (int8_t*)&(m_outer->m_outputs) + n);
    }


  private:
    // convert count-down value (to trigger point on underflow) to
    // count-up value (to trigger point on overflow)
    static uint32_t conv_counter_value (uint32_t val)
    {
      return (counter_max_value -
	      (uint32_t)std::min ((int)counter_max_value, (int)val) + 1) << 12;
    }

    soft_pg& outer (void) const { return *m_outer; }

    soft_pg* m_outer;

    friend class soft_pg;
  };

  template <typename... Args >
  soft_pg (Args&&... args)
  : m_channel_output_funcs (std::forward<Args> (args)...)
  {
    for (unsigned int i = 0; i < m_channels.size (); ++i)
    {
      auto& c = m_channels[i];

      c.m_outer = this;
      c.m_ch_num = i;
      c.m_mode = disabled;
      c.m_code_offset = 0;
      c.m_count_table_idx_flg = 0;
      c.m_cur_table_ptr = c.m_table[0].data ();
      c.m_edge_counter = 0;
      c.m_table[0][0] = 0;
      c.m_table[1][0] = 0;
    }

    m_outputs = 0;
    gen_proc_code ();
  }

  const auto& channel (unsigned int i) const { return m_channels[i]; }
  auto& channel (unsigned int i) { return m_channels[i]; }


  void start (void)
  {
    m_ctrl_clb_mod_counter = 0;

    write_outputs ();

    auto&& timer = timer_inst ();

    timer.set_timer_control (typename base_timer::tcr_t ()
	.set_count_clock_type (dev::rx_tpua::pclk_1)
	.set_count_clock_edge_type (dev::rx_tpua::rising_edge)
	.set_counter_clear (dev::rx_tpua::by_trga));

    timer.set_counter (typename base_timer::duration (0));
    timer.set_trigger_a (typename base_timer::duration (period_base_timer_ticks_i),
			 [this] (void) { process_channels (); });

    timer.start ();
  }

  void stop (void)
  {
    timer_inst ().stop ();
  }

private:
  static constexpr unsigned int max_channel_count = 32;

  static_assert (channel_count <= max_channel_count);
  volatile uint32_t m_outputs;

  std::array<class channel, channel_count> m_channels;

  channel_output_funcs m_channel_output_funcs;

  volatile unsigned int m_ctrl_clb_mod_counter;

  std::array<uint8_t, 64 * channel_count> m_codebuffer;

  void process_channels (void)
  {
    uint32_t toggle_outputs = 0;

    toggle_outputs = ((uint32_t(*)(soft_pg_impl::channel_base*, uint32_t))m_codebuffer.data ()) (&m_channels.front (), toggle_outputs);

    m_outputs ^= toggle_outputs;
    write_outputs ();

    // allow invoking a user callback every N ticks
    // this can be useful for updating motor control parameters to implement
    // speed ramps, check limit sensors and so on.
    auto ctrl_clb_mod_counter_n = m_ctrl_clb_mod_counter + 1;

    if (ctrl_clb_mod_counter_n == CtrlCallbackFreqDiv)
    {
      ctrl_clb_mod_counter_n = 0;
      if constexpr (!std::is_void_v <CtrlCallbackFunc >)
	std::invoke (CtrlCallbackFunc ());
    }
    m_ctrl_clb_mod_counter = ctrl_clb_mod_counter_n;
  }

  void gen_proc_code (void)
  {
    auto& timer = timer_inst ();

    const bool timer_was_running = timer.running ();
    timer.stop ();

    dev_base::gen_proc_code (m_codebuffer.begin (), m_codebuffer.end (),
		   m_channels.begin (), m_channels.end (), sizeof (class channel),
		   (const void*)&soft_pg_impl::process_channel_inc_ch_ptr < sizeof (class channel) >);

    if (timer_was_running)
      timer.start ();
  }

  void write_outputs (void)
  {
    // FIXME SP-55: do write-combining on outputs that are in the same device.
    // also, if this is invoked from an ISR context, there is no need to use
    // atomic bit operations.
    write_outputs_1<> (m_outputs);
  }

  template <unsigned int I = 0> std::enable_if_t < I < channel_count, void>
  write_outputs_1 (uint32_t output_values)
  {
    std::invoke (std::get< I > (m_channel_output_funcs), utils::get_bit (output_values, I));
    write_outputs_1< I + 1 > (output_values);
  }

  template <unsigned int I = 0> std::enable_if_t < I >= channel_count, void>
  write_outputs_1 ([[maybe_unused]] uint32_t output_values)
  {
  }

};

} // namespace dev
#endif // includeguard_dev_soft_pg_includeguard

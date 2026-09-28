
/*

pulse counter device

it receives a bitstream input of one or more channels, does some signal
decoding and updates the counter.

the following pulse signal formats are supported

- single ended pulse (1 wire)
  always increments the counter on rising edge

- pulse + direction (2 wire)
  increments the counter in positive direction on rising edge
  decrements the counter in negative direction on rising edge

- double pulse (2 wire)
  one pulse increments the counter on rising edge
  other pulse decrements the counter on rising edge

- AB phase single edge (2 wire)

- AB phase double edge (2 wire)

- AB phase quad edge (2 wire)

*/

#ifndef includeguard_dev_pulse_counter_hpp_includeguard
#define includeguard_dev_pulse_counter_hpp_includeguard

#include <type_traits>
#include <utils/bits.hpp>
#include <dev/pulse_type.hpp>

namespace dev
{

template <typename InputsSampler>
class pulse_counter
{
  using sample_block_t = typename InputsSampler::sample_block_t;
  constexpr static unsigned int bits_per_sample = InputsSampler::bits_per_sample;
  constexpr static unsigned int samples_per_block = InputsSampler::samples_per_block;
  constexpr static unsigned int sample_blocks_per_clb = InputsSampler::sample_blocks_per_clb;

  constexpr static unsigned int channel_bitpos_a = 0;
  constexpr static unsigned int channel_bitpos_b = channel_bitpos_a + 1;

  static_assert (channel_bitpos_a < bits_per_sample);

  using counter_value = uint32_t;

public:
  pulse_counter (InputsSampler& sampler, unsigned int channel)
  : m_sampler (sampler), m_sampler_channel (channel)
  {
  }

  const auto& pulse_type (void) const { return m_pulse_type; }
  bool is_reverse_direction (void) const { return m_rev_dir; }

  [[gnu::noinline]]
  void set_pulse_type (dev::pulse_type val, bool rev_dir = false)
  {
    m_pulse_type = val;
    m_rev_dir = rev_dir;

    if (val == pulse_type::disable)
      m_sampler.set_buffer_full_clb (m_sampler_channel, nullptr);

    // careful -- do not use std::bind here.  the lambda will result in a
    // direct function call invocation, whereas the thing from bind will
    // always go through one indirection.
#define set_pulse_type_clb(type, and_cond, func)\
	else if (val == type && and_cond)\
	  m_sampler.set_buffer_full_clb (m_sampler_channel, \
		[this] (const sample_block_t* b, unsigned int c) { decode_pulse<&pulse_counter::func> (b, c); })

    set_pulse_type_clb (pulse_type::single_pulse, rev_dir == false, decode_single_pulse);
    set_pulse_type_clb (pulse_type::single_pulse, rev_dir == true, decode_single_pulse_rev);

    set_pulse_type_clb (pulse_type::single_pulse_direction, rev_dir == false, decode_pulse_direction);
    set_pulse_type_clb (pulse_type::single_pulse_direction, rev_dir == true, decode_pulse_direction_rev);

    set_pulse_type_clb (pulse_type::double_pulse, rev_dir == false, decode_double_pulse);
    set_pulse_type_clb (pulse_type::double_pulse, rev_dir == true, decode_double_pulse_rev);

    set_pulse_type_clb (pulse_type::ab_phase_single_edge, rev_dir == false, decode_ab_phase_single_edge);
    set_pulse_type_clb (pulse_type::ab_phase_single_edge, rev_dir == true, decode_ab_phase_single_edge_rev);

    set_pulse_type_clb (pulse_type::ab_phase_double_edge, rev_dir == false, decode_ab_phase_double_edge);
    set_pulse_type_clb (pulse_type::ab_phase_double_edge, rev_dir == true, decode_ab_phase_double_edge_rev);

    set_pulse_type_clb (pulse_type::ab_phase_quad_edge, rev_dir == false, decode_ab_phase_quad_edge);
    set_pulse_type_clb (pulse_type::ab_phase_quad_edge, rev_dir == true, decode_ab_phase_quad_edge_rev);

#undef set_pulse_type_clb
  }

  void start (void) { m_sampler.start (m_sampler_channel); }
  void stop (void) { m_sampler.stop (m_sampler_channel); }
  bool is_running (void) const { return m_sampler.is_running (m_sampler_channel); }

  void set_counter (counter_value value) { m_counter = value; }
  counter_value counter (void) const { return m_counter; }

private:
  volatile counter_value m_counter = 0;
  sample_block_t m_prev_block;

  InputsSampler& m_sampler;
  unsigned int m_sampler_channel;

  dev::pulse_type m_pulse_type = dev::pulse_type::disable;
  bool m_rev_dir = false;


  template <void (pulse_counter::*Func)(sample_block_t, sample_block_t, sample_block_t)>
  void decode_pulse (const sample_block_t* blocks, [[maybe_unused]] unsigned int block_count)
  {
    sample_block_t prev_block = m_prev_block;

    for (const sample_block_t* src_ptr = blocks; src_ptr != blocks + sample_blocks_per_clb; ++src_ptr)
    {
      auto cur_val = *src_ptr;

      auto prev_val = (prev_block >> (bits_per_sample * (samples_per_block-1)))
		      | (cur_val << bits_per_sample);

      prev_block = cur_val;

      auto is_rising_edge = (prev_val ^ cur_val) & cur_val;
      auto is_falling_edge = (prev_val ^ cur_val) & prev_val;

      (this->*Func) (cur_val, is_rising_edge, is_falling_edge);
    }

    m_prev_block = prev_block;
  }

  void decode_single_pulse ([[maybe_unused]] sample_block_t cur_val,
			    sample_block_t is_rising_edge,
			    [[maybe_unused]] sample_block_t is_falling_edge)
  {
    // single ended count, src channel = 0
    // 11 insns = 46 ns @ 240 mhz

    m_counter += utils::popcount_odd_shl24 (is_rising_edge) >> 24;
  }

  void decode_single_pulse_rev ([[maybe_unused]] sample_block_t cur_val,
				sample_block_t is_rising_edge,
				[[maybe_unused]] sample_block_t is_falling_edge)
  {
    // single ended count, src channel = 0
    // 11 insns = 46 ns @ 240 mhz

    m_counter -= utils::popcount_odd_shl24 (is_rising_edge) >> 24;
  }

  void decode_double_pulse ([[maybe_unused]] sample_block_t cur_val,
			    sample_block_t is_rising_edge,
			    [[maybe_unused]] sample_block_t is_falling_edge)
  {
    // double pulse
    // 22 insns = 92 ns @ 240 mhz

    m_counter += (int32_t)(utils::popcount_odd_shl24 (is_rising_edge)
			   - (utils::popcount_even_shl24 (is_rising_edge) & 0xFF000000)) >> 24;
  }

  void decode_double_pulse_rev ([[maybe_unused]] sample_block_t cur_val,
				sample_block_t is_rising_edge,
				[[maybe_unused]] sample_block_t is_falling_edge)
  {
    // double pulse
    // 22 insns = 92 ns @ 240 mhz

    m_counter += (int32_t)(utils::popcount_even_shl24 (is_rising_edge)
			   - (utils::popcount_odd_shl24 (is_rising_edge) & 0xFF000000)) >> 24;
  }

  void decode_pulse_direction (sample_block_t cur_val,
			       sample_block_t is_rising_edge,
			       [[maybe_unused]] sample_block_t is_falling_edge)
  {
    // pulse + direction
    static_assert (channel_bitpos_b == channel_bitpos_a + 1);
    static_assert (std::is_same<sample_block_t, uint32_t>::value);

    sample_block_t is_low = ~cur_val;
    sample_block_t is_high = cur_val;
    sample_block_t inc_cond = is_rising_edge & (is_high >> 1);
    sample_block_t dec_cond = is_rising_edge & (is_low >> 1);

    m_counter += (int32_t)(utils::popcount_odd_shl24 (inc_cond)
			   - (utils::popcount_odd_shl24 (dec_cond) & 0xFF000000)) >> 24;
  }

  void decode_pulse_direction_rev (sample_block_t cur_val,
				   sample_block_t is_rising_edge,
				   [[maybe_unused]] sample_block_t is_falling_edge)
  {
    // pulse + direction
    static_assert (channel_bitpos_b == channel_bitpos_a + 1);
    static_assert (std::is_same<sample_block_t, uint32_t>::value);

    sample_block_t is_low = ~cur_val;
    sample_block_t is_high = cur_val;
    sample_block_t dec_cond = is_rising_edge & (is_high >> 1);
    sample_block_t inc_cond = is_rising_edge & (is_low >> 1);

    m_counter += (int32_t)(utils::popcount_odd_shl24 (inc_cond)
			   - (utils::popcount_odd_shl24 (dec_cond) & 0xFF000000)) >> 24;
  }

  void decode_ab_phase_single_edge (sample_block_t cur_val,
				    sample_block_t is_rising_edge,
				    sample_block_t is_falling_edge)
  {
    // AB phase single edge evaluation
    static_assert (channel_bitpos_b == channel_bitpos_a + 1);
    static_assert (std::is_same<sample_block_t, uint32_t>::value);

    sample_block_t is_low = ~cur_val;
    sample_block_t is_high = cur_val;
    sample_block_t inc_cond = is_rising_edge & (is_low >> 1);
    sample_block_t dec_cond = is_falling_edge & (is_high >> 1);

    m_counter += (int32_t)(utils::popcount_odd_shl24 (inc_cond)
			   - (utils::popcount_odd_shl24 (dec_cond) & 0xFF000000)) >> 24;
  }

  void decode_ab_phase_single_edge_rev (sample_block_t cur_val,
					sample_block_t is_rising_edge,
					sample_block_t is_falling_edge)
  {
    // AB phase single edge evaluation
    static_assert (channel_bitpos_b == channel_bitpos_a + 1);
    static_assert (std::is_same<sample_block_t, uint32_t>::value);

    sample_block_t is_low = ~cur_val;
    sample_block_t is_high = cur_val;
    sample_block_t dec_cond = is_rising_edge & (is_low >> 1);
    sample_block_t inc_cond = is_falling_edge & (is_high >> 1);

    m_counter += (int32_t)(utils::popcount_odd_shl24 (inc_cond)
			   - (utils::popcount_odd_shl24 (dec_cond) & 0xFF000000)) >> 24;
  }

  void decode_ab_phase_double_edge (sample_block_t cur_val,
				    sample_block_t is_rising_edge,
				    sample_block_t is_falling_edge)
  {
    // AB phase double edge evaluation
    // 32 insns = 134 ns @ 240 mhz
    static_assert (channel_bitpos_b == channel_bitpos_a + 1);
    static_assert (std::is_same<sample_block_t, uint32_t>::value);

    sample_block_t is_low = ~cur_val;
    sample_block_t is_high = cur_val;
    sample_block_t inc_cond = (is_rising_edge & (is_low >> 1)) | (is_falling_edge & (is_high >> 1));
    sample_block_t dec_cond = ((is_rising_edge >> 1) & is_low) | ((is_falling_edge >> 1) & is_high);

    m_counter += (int32_t)(utils::popcount_odd_shl24 (inc_cond)
			   - (utils::popcount_odd_shl24 (dec_cond) & 0xFF000000)) >> 24;
  }

  void decode_ab_phase_double_edge_rev (sample_block_t cur_val,
					sample_block_t is_rising_edge,
					sample_block_t is_falling_edge)
  {
    // AB phase double edge evaluation
    // 32 insns = 134 ns @ 240 mhz
    static_assert (channel_bitpos_b == channel_bitpos_a + 1);
    static_assert (std::is_same<sample_block_t, uint32_t>::value);

    sample_block_t is_low = ~cur_val;
    sample_block_t is_high = cur_val;
    sample_block_t dec_cond = (is_rising_edge & (is_low >> 1)) | (is_falling_edge & (is_high >> 1));
    sample_block_t inc_cond = ((is_rising_edge >> 1) & is_low) | ((is_falling_edge >> 1) & is_high);

    m_counter += (int32_t)(utils::popcount_odd_shl24 (inc_cond)
			   - (utils::popcount_odd_shl24 (dec_cond) & 0xFF000000)) >> 24;
  }

  void decode_ab_phase_quad_edge (sample_block_t cur_val,
				  sample_block_t is_rising_edge,
				  sample_block_t is_falling_edge)
  {
    // AB phase quad edge evaluation
    // 41 insns = 171 ns @ 240 mhz
    static_assert (channel_bitpos_b == channel_bitpos_a + 1);
    static_assert (std::is_same<sample_block_t, uint32_t>::value);

    sample_block_t is_low = ~cur_val;
    sample_block_t is_high = cur_val;

    sample_block_t inc_cond =     (is_rising_edge & (is_low >> 1))
				| ((is_rising_edge >> 1) & is_high)
				| (is_falling_edge & (is_high >> 1))
				| ((is_falling_edge >> 1) & is_low);

    sample_block_t dec_cond =     ((is_rising_edge >> 1) & is_low)
				| (is_rising_edge & (is_high >> 1))
				| ((is_falling_edge >> 1) & is_high)
				| (is_falling_edge & (is_low >> 1));

    m_counter += (int32_t)(utils::popcount_odd_shl24 (inc_cond)
			   - (utils::popcount_odd_shl24 (dec_cond) & 0xFF000000)) >> 24;
  }

  void decode_ab_phase_quad_edge_rev (sample_block_t cur_val,
				      sample_block_t is_rising_edge,
				      sample_block_t is_falling_edge)
  {
    // AB phase quad edge evaluation
    // 41 insns = 171 ns @ 240 mhz
    static_assert (channel_bitpos_b == channel_bitpos_a + 1);
    static_assert (std::is_same<sample_block_t, uint32_t>::value);

    sample_block_t is_low = ~cur_val;
    sample_block_t is_high = cur_val;

    sample_block_t dec_cond =     (is_rising_edge & (is_low >> 1))
				| ((is_rising_edge >> 1) & is_high)
				| (is_falling_edge & (is_high >> 1))
				| ((is_falling_edge >> 1) & is_low);

    sample_block_t inc_cond =     ((is_rising_edge >> 1) & is_low)
				| (is_rising_edge & (is_high >> 1))
				| ((is_falling_edge >> 1) & is_high)
				| (is_falling_edge & (is_low >> 1));

    m_counter += (int32_t)(utils::popcount_odd_shl24 (inc_cond)
			   - (utils::popcount_odd_shl24 (dec_cond) & 0xFF000000)) >> 24;
  }

};

} // namespace dev
#endif // includeguard_dev_pulse_counter_hpp_includeguard

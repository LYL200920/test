/*
  MCB TPU Pulse Gen
*/

#include <cstdio>
#include <chrono>
#include <atomic>

#include <board/board.hpp>
#include <board/board_info.hpp>
#include <net/net.hpp>
#include <fs/flash_bfs.hpp>
#include <fs/romfs.hpp>
#include <net/http_server.hpp>
#include <utils/text.hpp>

#define log_all
#include <logging/logging.hpp>

#include "app_http_server.hpp"

//--------------------------------------------------------------------------

template <typename TpuDev> class tpu_pulse_gen : public pulse_gen
{
public:
  tpu_pulse_gen (TpuDev& dev) : m_tpu_dev (dev)
  {
    m_tpu_dev.stop ();
    m_tpu_dev.set_timer_control (m_tpu_dev.timer_control ()
	.set_count_clock_type (dev::rx_tpua::pclk_1)
	.set_count_clock_edge_type (dev::rx_tpua::rising_edge)
	.set_counter_clear (dev::rx_tpua::clear_disable));

    m_tpu_dev.set_timer_mode (dev::rx_tpua::tmdr_t ()
	.set_mode (dev::rx_tpua::normal)
	.set_buffer_operation_a (false)
	.set_buffer_operation_b (false));

    m_tpu_dev.set_trigger_b (dev::rx_tpua::disable);
    m_tpu_dev.set_trigger_a (m_tpu_dev.max_time_point ().time_since_epoch (),
    [this] (void)
    {
      on_timer_triggered ();
    });

    update_timer_params ();
    reset ();
  }

  virtual void start (void) override
  {
    m_tpu_dev.start ();
  }

  virtual void stop (void) override
  {
    m_tpu_dev.stop ();
  }

  virtual void reset (void) override
  {
    m_tpu_dev.stop ();

    m_bit_pattern_i = 0;
    m_pulse_phase_i = 0;
    m_cur_count = m_pulse_phase[0];

    m_tpu_dev.set_counter (timer_duration (0));

    soft_memory_fence ();
    m_tpu_dev.start ();
  }

  virtual bool running (void) const override
  {
    return m_tpu_dev.running ();
  }

  virtual void set_frequency (float val) override
  {
    pulse_gen::set_frequency (val);
    update_timer_params ();
  }

  virtual void set_duty_cycle (float val) override
  {
    pulse_gen::set_duty_cycle (val);
    update_timer_params ();
  }

  virtual void set_phase_offset (float val) override
  {
    pulse_gen::set_phase_offset (val);
    update_timer_params ();
  }

  virtual void set_bit_pattern (std::vector<bool> val) override
  {
    guarded_parameter_change ([&] (void)
    {
      pulse_gen::set_bit_pattern (std::move (val));
    });
  }

  virtual void set_output_port (dev::digital_io_port val) override
  {
    guarded_parameter_change ([&] (void)
    {
      pulse_gen::set_output_port (std::move (val));
    });
  }

private:
  // frequency hz -> timer ticks
  // the frequency is represented by n * max_timer_ticks + timer_ticks.

  // could use DTC repeat transfers for that.  max repeat transfer count is
  // 256 tough, which is ~3.5 Hz min.

  using timer_duration = typename TpuDev::duration;
  using timer_time_point = typename TpuDev::time_point;
  using timer_duration_long = std::chrono::duration<uint64_t, typename timer_duration::period>;

  struct timer_count
  {
    constexpr timer_count (void) : n (0), d (0) { }
    constexpr timer_count (unsigned int n_, timer_duration d_) : n (n_), d (d_) { }
    constexpr timer_count (timer_duration_long d_)
    {
      const auto max_timer_duration = TpuDev::max_time_point ().time_since_epoch ();

      n = d_ / max_timer_duration;
      d = d_ - (n * max_timer_duration);
    }

    unsigned int n;
    timer_duration d;
  };

  static constexpr timer_duration_long freq_to_timer_duration (float freq_hz)
  {
    // allow 1/100 Hz input resolution.
    // do all calculations with integer types.
    const auto freq_hz_i = (uint32_t)(freq_hz * 100);

    return timer_duration_long (
	freq_hz_i > 0
	? ((uint64_t)timer_duration_long::period::den * 100) / freq_hz_i
	: 0);
  }

  void update_timer_params (void)
  {
    const auto max_timer_duration = TpuDev::max_time_point ().time_since_epoch ();

    // allow 1/100 Hz input resolution.
    // do all calculations with integer types.
    const auto cycle_duration = freq_to_timer_duration (m_frequency_hz);

    const auto turn_on_duration = (cycle_duration * (unsigned int)(m_duty_cycle_percent*100)) / 10000;
    const auto turn_off_duration = cycle_duration - turn_on_duration;

    const auto phase_offset_duration = timer_count (
	(cycle_duration * (unsigned int)(m_phase_offset_percent*100)) / 10000);

    const auto turn_on_duration_1 = timer_count (turn_on_duration);
    const auto turn_off_duration_1 = timer_count (turn_off_duration);
/*
    std::printf ("turn on duration  = %llu = (%u, %u)\n", turn_on_duration.count (),
	(unsigned int)turn_on_duration_1.n, (unsigned int)turn_on_duration_1.d.count ());

    std::printf ("turn off duration = %llu = (%u, %u)\n", turn_off_duration.count (),
	(unsigned int)turn_off_duration_1.n, (unsigned int)turn_off_duration_1.d.count ());
*/
    guarded_parameter_change ([&] (void)
    {
      m_pulse_phase[0] = phase_offset_duration;
      m_pulse_phase[1] = turn_on_duration_1;
      m_pulse_phase[2] = turn_off_duration_1;
    });
  }

  template <typename F> void guarded_parameter_change (F&& f)
  {
    auto prev_val0 = m_in_parameter_change.exchange (1);
    f ();
    auto prev_val1 = m_in_parameter_change.exchange (0);

    if (prev_val0 == 2 || prev_val1 == 2)
      on_timer_triggered ();
  }

  void on_timer_triggered (void)
  {
    if (m_in_parameter_change.exchange (2))
    {
      // we have interrupted the user context, while it was modifying the
      // parameters.
      return;
    }

    if (m_cur_count.n > 0)
    {
      // assume that the max value has been set already.
      // the timer counter wraps around and we just need to do the count down.
      m_cur_count.n -= 1;
    }
    else if (m_cur_count.d.count () > 0)
    {
      m_tpu_dev.set_counter (m_tpu_dev.max_time_point () - m_cur_count.d
		+ m_tpu_dev.now ().time_since_epoch () + timer_duration (6));
      m_cur_count.d = timer_duration (0);
    }
    else
      next_pulse_phase ();

    m_in_parameter_change.store (0);
  }

  void next_pulse_phase (void)
  {
    if (++m_pulse_phase_i == m_pulse_phase.size ())
      m_pulse_phase_i = 0;

    m_cur_count = m_pulse_phase[m_pulse_phase_i];

    if (m_cur_count.n > 0)
      m_tpu_dev.set_counter (m_tpu_dev.max_time_point ());
    else if (m_cur_count.d.count () > 0)
    {
      m_tpu_dev.set_counter (m_tpu_dev.max_time_point () - m_cur_count.d
		+ m_tpu_dev.now ().time_since_epoch () + timer_duration (6));
      m_cur_count.d = timer_duration (0);
    }
    else
    {
      // this might result in an infinite loop
      next_pulse_phase ();
      return;
    }

    if (m_pulse_phase_i != 0)
    {
      if (m_bit_pattern.empty ())
      {
	// phase a (0b00):  off
	// phase b (0b01):   on
	// phase c (0b10):  off
	m_output_port.write ((m_pulse_phase_i & 0b01) != 0);
      }
      else
      {
	m_output_port.write (m_bit_pattern[m_bit_pattern_i++]);
	if (m_bit_pattern_i == m_bit_pattern.size ())
	  m_bit_pattern_i = 0;
      }
    }
  }


  TpuDev& m_tpu_dev;
  std::atomic<int> m_in_parameter_change = ATOMIC_VAR_INIT (0);

  // current read position of the bit pattern vector.
  unsigned int m_bit_pattern_i = 0;

  // current read position of the pulse phase array.
  unsigned int m_pulse_phase_i = 0;

  //    |‾‾‾‾‾‾|             |‾‾‾‾‾‾|
  //____|      |________.____|      |_____________
  // A     B        C      A    B        C
  //
  // A:  phase offset
  // B:  on duration   = period duration * duty cycle
  // C:  off duration  = period duration - on duration
  //
  // during phase A the bitpattern value is not read
  // and the output is not set.
  //
  // phase B and C will each read from the bitpattern vector and write the
  // corresponding value to the output port when the phase is entered
  // (rising edge).

  std::array<timer_count, 3> m_pulse_phase;

  timer_count m_cur_count;
};

static tpu_pulse_gen<this_board::type::tpu0_t> g_pulse_gen_tpu0 (this_board::inst ().tpu0);
static tpu_pulse_gen<this_board::type::tpu1_t> g_pulse_gen_tpu1 (this_board::inst ().tpu1);
static tpu_pulse_gen<this_board::type::tpu2_t> g_pulse_gen_tpu2 (this_board::inst ().tpu2);
static tpu_pulse_gen<this_board::type::tpu3_t> g_pulse_gen_tpu3 (this_board::inst ().tpu3);

static std::reference_wrapper<pulse_gen> g_all_pulse_gens[] =
{
  g_pulse_gen_tpu0,
  g_pulse_gen_tpu1,
  g_pulse_gen_tpu2,
  g_pulse_gen_tpu3
};

//--------------------------------------------------------------------------

template <typename MCXAxis>
class mcx_direction_output : public dev::digital_io_port::dev_if
{
public:
  mcx_direction_output (MCXAxis& axis) : m_axis (axis)
  {
    m_axis.set_mode3 (m_axis.mode3 ()
	.set_pulse_output_mode (dev::mcx51x::single_pulse_single_direction)
	.set_direction_output_logic (dev::mcx51x::positive)
	.set_swap_pulse_output_signals (false));
  }

  virtual void write_port (unsigned int, bool val) override
  {
    if (val)
      m_axis.set_pos_dir ();
    else
      m_axis.set_neg_dir ();
  }

  operator dev::digital_io_port (void)
  {
    return dev::digital_io_port (this, 0);
  }

private:
  MCXAxis& m_axis;
};

//--------------------------------------------------------------------------

static auto g_romfs = fs::romfs::mount_this_image_partition ();

//--------------------------------------------------------------------------

// FIXME MCB-97: this should be shared.
// the ipv4 config file with fileid = 0 has a fixed layout.
static constexpr unsigned int ipv4_config_fileid = 0;
struct ipv4_config
{
  uint8_t static_addr[4];
  uint8_t static_netmask[4];
  uint8_t static_gateway_addr[4];
  uint8_t static_dns_addr[4];
  //bool use_dhcp : 1;
  //... other flags
};

//--------------------------------------------------------------------------

int main (void)
{
  log_level::enable (log_level::warn);
  log_level::enable (log_level::error);

  const auto& board_ifcfg = this_board::board_info ().ifconfigs ()[0];
  ipv4_config use_ifcfg;
  std::memcpy (use_ifcfg.static_addr, board_ifcfg.ipv4_addr ().data (), 4);
  std::memcpy (use_ifcfg.static_netmask, board_ifcfg.ipv4_netmask ().data (), 4);
  std::memcpy (use_ifcfg.static_gateway_addr, board_ifcfg.ipv4_addr ().data (), 4);

  // try to overwrite the ipv4 config from the configuration file in the
  // data flash.
  if (auto p = fs::flash_bfs::mount_partition (this_board::inst ().data_flash (), "config"))
    if (auto f = p->open_file (ipv4_config_fileid))
    {
      ipv4_config c;
      if (f->read (&c, sizeof (c)) == sizeof (c))
	std::memcpy (&use_ifcfg, &c, sizeof (c));
    }

  // set MCX pulse mode to allow controlling the direction pin as a general
  // purpose output.
/*
  for (auto& a : this_board::inst ().mcx51x.axes ())
  {
  }
*/
  mcx_direction_output<this_board::type::mcx51x_t::axis_t> mcx_x_out (this_board::inst ().mcx51x.axis (0));




  net::init (this_board::inst ().eth0,
	     use_ifcfg.static_addr, use_ifcfg.static_gateway_addr,
	     use_ifcfg.static_netmask);

  app_http_server httpsrv_delegate (g_romfs.get (),
	std::begin (g_all_pulse_gens), std::end (g_all_pulse_gens));

  net::http::server httpsrv (80, httpsrv_delegate);

  auto prev_time = std::chrono::high_resolution_clock::now ();
  auto cur_time = prev_time;
  auto last_button_sync_time = prev_time;
  auto last_print_stat_time = prev_time;

  for (unsigned int i = 1; i < 4 && false; ++i)
  {
    pulse_gen& pg = g_all_pulse_gens[i];
    pg.set_output_port (this_board::inst ().led_outputs[i]);
    pg.set_duty_cycle (50);
    pg.set_phase_offset (0);
    pg.set_frequency (100'000);
    pg.start ();
  }

  {
    pulse_gen& pg = g_all_pulse_gens[0];
    pg.set_output_port (mcx_x_out);
    pg.set_duty_cycle (0.1f);
    pg.set_phase_offset (0);
    pg.set_frequency (100);
    pg.start ();
  }

  for (unsigned int main_loop_count = 0; ; ++main_loop_count)
  {
    prev_time = cur_time;
    cur_time = std::chrono::high_resolution_clock::now ();

    this_board::inst ().exec ();

    if (cur_time - last_button_sync_time >= std::chrono::milliseconds (10))
    {
      last_button_sync_time = cur_time;

    }

    net::exec ();
    httpsrv.exec ();
  }

  return 0;
}

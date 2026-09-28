/*
  MCB motor and browser example
*/

#include <cstdio>
#include <chrono>

#include <board/board.hpp>
#include <board/board_info.hpp>
#include <net/net.hpp>
#include <fs/flash_bfs.hpp>
#include <fs/romfs.hpp>
#include <net/http_server.hpp>
#include <utils/text.hpp>
#include <dev/pulse_counter.hpp>

#define log_all
#include <logging/logging.hpp>

#include <dev/soft_pg.hpp>
#include <dev/soft_pg_motor.hpp>

//--------------------------------------------------------------------------
// we would like to treat all axes in the GUI equally.  for that we have
// a base class interface for the GUI and implementations for the particular
// axes.

class motor_axis
{
public:
  const std::string_view& name (void) const { return m_name; }

  virtual void drive_on (bool val) = 0;

  virtual uint32_t max_speed (void) const = 0;
  virtual uint32_t max_feed (void) const = 0;
  virtual uint32_t max_accel (void) const = 0;
  virtual uint32_t max_scurve_accel (void) const = 0;

  virtual uint32_t current_speed (void) const = 0;
  virtual uint32_t current_feed (void) const = 0;
  virtual void reset_current_feed (void) = 0;

  virtual void set_speed (uint32_t val) = 0;
  virtual uint32_t speed (void) const = 0;

  virtual void set_feed (uint32_t val) = 0;
  virtual uint32_t feed (void) const = 0;

  virtual uint32_t encoder_feed (void) const = 0;
  virtual void reset_encoder_feed (void) = 0;

  virtual bool pulse_output_mode_supported (dev::pulse_type pt) const = 0;
  virtual dev::pulse_type pulse_output_mode (void) const = 0;
  virtual void set_pulse_output_mode (dev::pulse_type pt) = 0;

  virtual bool pulse_input_mode_supported (dev::pulse_type pt) const = 0;
  virtual dev::pulse_type pulse_input_mode (void) const = 0;
  virtual void set_pulse_input_mode (dev::pulse_type pt) = 0;

  enum direction_t
  {
    fwd,
    rev
  };

  virtual direction_t direction (void) const = 0;
  virtual void set_direction (direction_t val) = 0;

  enum mode_t
  {
    continuous,
    feed_count
  };

  virtual mode_t mode (void) const = 0;
  virtual void set_mode (mode_t val) = 0;

  enum accel_decel_t
  {
    accel_decel_disable,
    linear,
    scurve
  };

  virtual accel_decel_t accel_decel (void) const = 0;
  virtual void set_accel_decel (accel_decel_t val) = 0;

  virtual uint32_t accel (void) const = 0;
  virtual void set_accel (uint32_t val) = 0;

  virtual uint32_t scurve_accel (void) const = 0;
  virtual void set_scurve_accel (uint32_t val) = 0;

  virtual void start (void) = 0;
  virtual void stop (void) = 0;
  virtual void stop_immediately (void) = 0;

  virtual void start_fwd (void)
  {
    stop_immediately ();
    while (is_driving ()) { }
    set_direction (fwd);
    start ();
  }

  virtual void start_rev (void)
  {
    stop_immediately ();
    while (is_driving ()) { }
    set_direction (rev);
    start ();
  }

  virtual void do_homing (void) = 0;

  virtual bool is_driving (void) const = 0;

  virtual bool alarm_status (void) const = 0;
  virtual bool servo_on_status (void) const = 0;

  struct sensor_status_t
  {
    bool limit_p;
    bool limit_n;
    bool org;
  };

  virtual sensor_status_t sensor_status (void) const = 0;

protected:
  motor_axis (std::string_view name) : m_name (name) { }

  std::string_view m_name;
};

const char* to_string (motor_axis::direction_t val)
{
  switch (val)
  {
    case motor_axis::fwd: return "fwd";
    case motor_axis::rev: return "rev";
    default: return "";
  }
}

const char* to_string (motor_axis::mode_t val)
{
  switch (val)
  {
    case motor_axis::continuous: return "cont";
    case motor_axis::feed_count: return "feed";
    default: return "";
  }
}

const char* to_string (motor_axis::accel_decel_t val)
{
  switch (val)
  {
    case motor_axis::accel_decel_disable: return "off";
    case motor_axis::linear: return "linear";
    case motor_axis::scurve: return "scurve";
    default: return "";
  }
}

const char* to_string (dev::pulse_type pt)
{
  switch (pt)
  {
    case dev::pulse_type::disable: return "Disabled";
    case dev::pulse_type::single_pulse: return "Single Pulse";
    case dev::pulse_type::single_pulse_direction: return "Single Pulse + Direction";
    case dev::pulse_type::double_pulse: return "Double Pulse";
    case dev::pulse_type::ab_phase_single_edge: return "AB Phase (single edge)";
    case dev::pulse_type::ab_phase_double_edge: return "AB Phase (double edge)";
    case dev::pulse_type::ab_phase_quad_edge: return "AB Phase (quad edge)";
    default: return "";
  }
}

//--------------------------------------------------------------------------
// MCX51x motor axis

struct mcx_motor_axis : public motor_axis
{
  using mcx51x_t = this_board::type::mcx51x_t;

  mcx51x_t::axis_t& m_axis;

  direction_t m_dir = fwd;
  mode_t m_mode = continuous;
  accel_decel_t m_accel_decel = linear;

  uint32_t m_jerk = 18000000;
  uint32_t m_feed = 0;

  dev::pulse_type m_pulse_output_mode = dev::pulse_type::single_pulse_direction;
  dev::pulse_type m_pulse_input_mode = dev::pulse_type::ab_phase_quad_edge;

  static constexpr unsigned int amp_enable_output = 0;
  static constexpr unsigned int alarm_clear_output = 2;
  static constexpr unsigned int amp_alarm_input = 0;

  mcx_motor_axis (mcx51x_t::axis_t& a,
		  std::string_view name)
  : motor_axis (name), m_axis (a)
  {
    m_axis.container ().set_interpolation_mode (dev::mcx51x::interpolation_mode_t (0));

    m_axis.outputs ()[amp_enable_output].write (false);
    m_axis.outputs ()[alarm_clear_output].write (false);

    m_axis.drive_stop ();
    m_axis.clear_finish_status ();
    m_axis.set_input_signal_filter_mode (dev::mcx51x::input_signal_filter_t ()
	.set_signals_filter_time (10)
	.set_encoder_filter_time (0)
	.set_limitp_limitn_filter_enable ()
	.set_stop0_stop1_filter_enable ()
	.set_inpos_alarm_filter_enable ()
	.set_pio0123_filter_enable ()
	.set_pio4567_filter_enable ()
	.set_stop2_filter_enable ()
	.set_eca_ecb_filter_enable ());

    m_axis.set_jerk (m_jerk);
    m_axis.set_accel_pps (300000);
    m_axis.set_decel_pps (300000);
    m_axis.set_initial_speed_pps (1);
    m_axis.set_drive_speed_pps (8000);
    m_axis.set_drive_pulse_number (0);
    m_axis.set_logical_position_counter (0);
    m_axis.set_real_position_counter (0);

    m_axis.set_mode2 (dev::mcx51x::mode2_t ()
	.set_stop0_logic (dev::mcx51x::active_low)
	.set_stop1_logic (dev::mcx51x::active_low)
	.set_stop1_enable (false)
	.set_stop2_logic (dev::mcx51x::active_low)
	.set_stop2_enable (false)
	.set_in_position_logic (dev::mcx51x::active_low)
	.set_in_position_enable (false)
	.set_alarm_logic (dev::mcx51x::active_high)
	.set_alarm_enable (false)
	.set_stop0_enable (false)
	.set_hw_limit_logic (dev::mcx51x::active_low)
	.set_hw_limit_enable (true)
	.set_hw_limit_stop_mode (dev::mcx51x::stop_immediately)
	.set_sw_limit_enable (false));

    m_axis.set_mode3 (dev::mcx51x::mode3_t ()
	.set_decel_mode (dev::mcx51x::automatic)
	.set_accel_decel_symmetry (dev::mcx51x::symmetric)
	.set_accel_decel_mode (dev::mcx51x::linear)
	.set_pulse_output_mode (dev::mcx51x::single_pulse_single_direction)
	.set_pulse_output_logic (dev::mcx51x::positive)
	.set_direction_output_logic (dev::mcx51x::positive)
	.set_swap_pulse_output_signals (false)
	.set_encoder_pulse_input (dev::mcx51x::quad_pulse_input_quad_edge_eval)
	.set_encoder_input_logic (dev::mcx51x::positive)
	.set_swap_encoder_input_signals (false)
	.set_swap_hw_limit_input_signals (false)
	.set_enable_triangle_form_prevention (true));

    drive_on (false);
  }

  virtual void drive_on (bool val) override
  {
    if (val)
    {
      m_axis.outputs ()[amp_enable_output].write (true);
      m_axis.outputs ()[alarm_clear_output].write (false);
      m_axis.clear_finish_status ();
    }
    else
    {
      m_axis.outputs ()[amp_enable_output].write (false);
      m_axis.outputs ()[alarm_clear_output].write (true);
    }
  }

  virtual uint32_t max_speed (void) const override { return m_axis.drive_speed_max_value_pps; }
  virtual uint32_t max_feed (void) const override { return m_axis.drive_pulse_number_max_value; }
  virtual uint32_t max_accel (void) const override { return m_axis.accel_max_value_pps; }
  virtual uint32_t max_scurve_accel (void) const override { return m_axis.jerk_max_value_pps; }

  virtual uint32_t current_speed (void) const override { return m_axis.current_drive_speed_pps (); }
  virtual uint32_t current_feed (void) const override { return std::abs (m_axis.logical_position_counter ()); }
  virtual void reset_current_feed (void) override { m_axis.set_logical_position_counter (0); }

  virtual void set_speed (uint32_t val) override
  {
    if (m_accel_decel == accel_decel_disable)
      m_axis.set_initial_speed_pps (val);

    m_axis.set_drive_speed_pps (val);
  }
  virtual uint32_t speed (void) const override { return m_axis.drive_speed_pps (); }

  virtual void set_feed (uint32_t val) override { m_feed = val; }
  virtual uint32_t feed (void) const override { return m_feed; }

  virtual direction_t direction (void) const override { return m_dir; }
  virtual void set_direction (direction_t val) override
  {
    m_dir = val;
    if (m_dir == fwd)
      m_axis.set_pos_dir ();
    else
      m_axis.set_neg_dir ();
  }

  virtual mode_t mode (void) const override { return m_mode; }
  virtual void set_mode (mode_t val) override { m_mode = val; }

  virtual accel_decel_t accel_decel (void) const override { return m_accel_decel; }
  virtual void set_accel_decel (accel_decel_t val) override { m_accel_decel = val; }

  virtual uint32_t accel (void) const override { return m_axis.accel_pps (); }
  virtual void set_accel (uint32_t val) override
  {
    m_axis.set_accel_pps (val);
    m_axis.set_decel_pps (val);
  }

  virtual uint32_t scurve_accel (void) const override { return m_jerk; }
  virtual void set_scurve_accel (uint32_t val) override
  {
    m_jerk = val;
    m_axis.set_jerk_pps (m_jerk);
  }

  virtual void start (void) override
  {
    m_axis.clear_finish_status ();

    if (m_accel_decel == accel_decel_disable)
      m_axis.set_initial_speed (m_axis.drive_speed ());
    else
    {
      m_axis.set_initial_speed (10);

      if (m_accel_decel == linear)
        m_axis.set_mode3 (m_axis.mode3 ().set_accel_decel_mode (dev::mcx51x::linear));
      else // if (m_accel_decel == scurve)
        m_axis.set_mode3 (m_axis.mode3 ().set_accel_decel_mode (dev::mcx51x::scurve));
    }

    if (m_mode == continuous)
    {
      if (m_dir == fwd)
	m_axis.drive_pos_dir_pulse_continuous ();
      else if (m_dir == rev)
	m_axis.drive_neg_dir_pulse_continuous ();
    }
    else
    {
      m_axis.set_drive_pulse_number (m_dir == fwd ? m_feed : -(int32_t)m_feed);
      m_axis.drive_relative ();
    }
  }

  virtual void stop (void) override
  {
    m_axis.drive_decel_stop ();
  }
  virtual void stop_immediately (void) override
  {
    m_axis.drive_stop ();
  }

  virtual bool alarm_status (void) const override
  {
    return m_axis.signal_status ().alarm ();
  }

  virtual bool servo_on_status (void) const override
  {
    return m_axis.outputs ()[amp_enable_output].read ()
	   && !m_axis.inputs ()[amp_alarm_input].read ();
  }

  virtual sensor_status_t sensor_status (void) const override
  {
    auto s = m_axis.signal_status ();
    return { s.hw_limit_pos (), s.hw_limit_neg (), s.stop1 () };
  }

  virtual bool is_driving (void) const override
  {
    return m_axis.is_driving ();
  }

  virtual void do_homing (void) override
  {
    m_axis.set_auto_home_search_mode1 (dev::mcx51x::auto_home_search_mode1_t ()
	.set_high_speed_search (false)

	.set_low_speed_search (true)
	.set_low_speed_search_dir (m_dir == fwd ? dev::mcx51x::positive : dev::mcx51x::negative)
	.set_low_speed_search_use_limit (dev::mcx51x::s2_use_stop1)
	.set_low_speed_search_clear_deviation_counter (true)
	.set_low_speed_search_clear_real_position_counter (true)
	.set_low_speed_search_clear_logical_position_counter (true)

	.set_z_phase_search (false)
	.set_high_speed_offset_drive (false));

    m_axis.set_auto_home_search_mode2 (dev::mcx51x::auto_home_search_mode2_t ()
	.set_sand (false)
	.set_clear_real_position_counter (false)
	.set_clear_logical_position_counter (true)
	.set_clear_deviation_counter (true)
	.set_deviation_counter_clear_pulse_width (dev::mcx51x::dcc_10msec)
	.set_use_inter_step_timer (true)
	.set_inter_step_timer (dev::mcx51x::tm_500msec));

    // m_axis.set_jerk (config ().jerk ());  // use current axis setting
    m_axis.set_drive_pulse_number (0);		// homing offset drive position

    // m_axis->set_initial_speed (m_initial_speed);
    m_axis.set_home_search_speed (m_axis.drive_speed ());
    m_axis.set_drive_speed (m_axis.drive_speed ());
    m_axis.set_accel (m_axis.accel ());
    m_axis.set_decel (m_axis.accel ());

    m_axis.drive_auto_home_search ();
  }


  virtual uint32_t encoder_feed (void) const override { return m_axis.real_position_counter (); }
  virtual void reset_encoder_feed (void) override { m_axis.set_real_position_counter (0); }

  virtual bool pulse_output_mode_supported (dev::pulse_type pt) const override
  {
    return dev::mcx51x::pulse_output_type_supported (pt);
  }

  virtual dev::pulse_type pulse_output_mode (void) const override
  {
    return m_pulse_output_mode;
  }

  virtual void set_pulse_output_mode (dev::pulse_type pt) override
  {
    if (dev::mcx51x::pulse_output_type_supported (pt) && m_pulse_output_mode != pt)
    {
      m_pulse_output_mode = pt;
      m_axis.set_mode3 (m_axis.mode3 ().set_pulse_output_mode (pt));
    }
  }

  virtual bool pulse_input_mode_supported (dev::pulse_type pt) const override
  {
    return dev::mcx51x::pulse_input_type_supported (pt);
  }

  virtual dev::pulse_type pulse_input_mode (void) const override
  {
    return m_pulse_input_mode;
  }

  virtual void set_pulse_input_mode (dev::pulse_type pt) override
  {
    if (dev::mcx51x::pulse_input_type_supported (pt) && m_pulse_input_mode != pt)
    {
      m_pulse_input_mode = pt;
      m_axis.set_mode3 (m_axis.mode3 ().set_encoder_pulse_input (pt));
    }
  }


};


//--------------------------------------------------------------------------
// PCD4641 motor axis

struct pcd_motor_axis : public motor_axis
{
  using pcd4641_t = this_board::type::pcd4641_t;

  pcd4641_t::axis_t& m_axis;

  mode_t m_mode = continuous;
  accel_decel_t m_accel_decel = linear;

  uint32_t m_feed = 0;
  uint32_t m_speed = 8000;

  static constexpr unsigned int amp_enable_output = this_board::type::pcd_axis_outputs_dev_t::ots;
  static constexpr unsigned int alarm_clear_output = this_board::type::pcd_axis_outputs_dev_t::p1;
  static constexpr unsigned int amp_alarm_input = this_board::type::pcd_axis_inputs_dev_t::stp;

  dev::pulse_type m_pulse_output_mode = dev::pulse_type::single_pulse_direction;

  #ifdef MCB_HAVE_PCD_AXIS_INPUTS_SAMPLER
    dev::pulse_type m_pulse_input_mode = dev::pulse_type::ab_phase_quad_edge;
    dev::pulse_counter <this_board::type::pcd_axis_inputs_sampler_t> m_encoder;
  #endif


  pcd_motor_axis (pcd4641_t::axis_t& a, std::string_view name)
  : motor_axis (name), m_axis (a)
    #ifdef MCB_HAVE_PCD_AXIS_INPUTS_SAMPLER
      , m_encoder (this_board::inst ().pcd_axis_inputs_sampler, a.num ())
    #endif
  {
    m_axis.set_output_mode (dev::pcd4641::output_mode_t ()
	.set_output_logic (dev::pcd4641::negative_logic)
	.set_pulse_output (false)
	.set_sequence_signal_output (true)
	.set_stop_during_accel_decel (false)  // if true it will not do FL <-> FH accel/decel, only drive at FL
	.set_sensor_sensitivity (dev::pcd4641::high_sensitivity));

    m_axis.set_control_mode (dev::pcd4641::control_mode_t ()
	.set_org_input_signal (false)
	.set_sd_input_signal (false)
	.set_positioning_mode (false)
	.set_direction (dev::pcd4641::positive_dir)
	.set_ots_output (false)
	.set_accel_decel_mode (dev::pcd4641::linear));

    m_axis.outputs ()[amp_enable_output].write (false);
    m_axis.outputs ()[alarm_clear_output].write (false);

    m_axis.enable_ramp_down_point_irq (false);
    m_axis.enable_exernal_start_irq (false);

    m_axis.set_feed (m_axis.feed_max_value);
    m_axis.set_speed (dev::pcd4641::fl_speed, 1);
    m_axis.set_speed_pps (dev::pcd4641::fh_speed, m_speed);

    m_axis.set_accel_decel_rate (100);

    m_axis.set_ramp_down_point_u24 (0);
    m_axis.set_idling_pulses (0);

    m_axis.set_env (m_axis.env ()
	.set_pulse_output_mode (dev::pcd4641::single_pulse_single_direction)
	.set_positioning_down_counter (true)
	.set_ramp_down_mode (dev::pcd4641::automatic)
	.set_stp_stop_mode (dev::pcd4641::sensor_stop_decel)
	.set_el_stop_mode (dev::pcd4641::sensor_stop_immediately)
	.set_org_stop_mode (dev::pcd4641::sensor_stop_decel)
	.set_position_counter_auto_reset (true)
	.set_position_counter_enable (true)
	.set_position_counter_dir (dev::pcd4641::count_inc));

    m_axis.set_current_position_u24 (0);

    #ifdef MCB_HAVE_PCD_AXIS_INPUTS_SAMPLER
      m_encoder.set_pulse_type (m_pulse_input_mode);
      m_encoder.start ();

      std::this_thread::sleep_for (std::chrono::milliseconds (1));
      m_encoder.set_counter (0);
    #endif
  }

  virtual void drive_on (bool val) override
  {
    if (val)
    {
      m_axis.outputs ()[amp_enable_output].write (true);
      m_axis.outputs ()[alarm_clear_output].write (false);
    }
    else
    {
      m_axis.outputs ()[amp_enable_output].write (false);
      m_axis.outputs ()[alarm_clear_output].write (true);
    }
  }


  virtual uint32_t max_speed (void) const override { return m_axis.speed_max_value_pps; }
  virtual uint32_t max_feed (void) const override { return m_axis.feed_max_value; }
  virtual uint32_t max_accel (void) const override { return 65535; }
  virtual uint32_t max_scurve_accel (void) const override { return 0; }

  virtual uint32_t current_speed (void) const override
  {
    return m_axis.current_speed_pps ();
  }

  virtual uint32_t current_feed (void) const override { return m_axis.current_position_u24 (); }
  virtual void reset_current_feed (void) override { m_axis.set_current_position_u24 (0); }

  virtual void set_speed (uint32_t val) override
  {
    m_speed = val;
    m_axis.set_speed_pps (dev::pcd4641::fh_speed, val);
/*
    auto s = pcd4641_t::calc_speed (val);
    m_axis.set_speed (dev::pcd4641::fh_speed, s.fract);
    m_axis.set_speed_magnification (s.exp);

    std::printf ("pcd set speed %u -> %u.%u\n", val, s.exp, s.fract);
*/
  }

  virtual uint32_t speed (void) const override { return m_speed; }

  virtual void set_feed (uint32_t val) override { m_feed = val; }
  virtual uint32_t feed (void) const override { return m_feed; }

  virtual direction_t direction (void) const override
  {
    auto d = m_axis.control_mode ().direction ();
    if (d == dev::pcd4641::positive_dir)
      return fwd;
    else if (d == dev::pcd4641::negative_dir)
      return rev;
    else
      assert_unreachable ();
  }

  virtual void set_direction (direction_t val) override
  {
     m_axis.set_control_mode (m_axis.control_mode ()
	.set_direction (val == fwd ? dev::pcd4641::positive_dir : dev::pcd4641::negative_dir));
  }

  virtual mode_t mode (void) const override { return m_mode; }
  virtual void set_mode (mode_t val) override { m_mode = val; }

  virtual accel_decel_t accel_decel (void) const override { return m_accel_decel; }
  virtual void set_accel_decel (accel_decel_t val) override { m_accel_decel = val; }

  virtual uint32_t accel (void) const override { return m_axis.accel_decel_rate (); }
  virtual void set_accel (uint32_t val) override { m_axis.set_accel_decel_rate (val); }

  virtual uint32_t scurve_accel (void) const override { return 0; }
  virtual void set_scurve_accel (uint32_t val) override { }

  virtual void start (void) override
  {
    set_speed (m_speed);

    m_axis.set_output_mode (m_axis.output_mode ()
	.set_pulse_output (true)
	.set_stop_during_accel_decel (false));

    m_axis.set_control_mode (m_axis.control_mode ()
	.set_accel_decel_mode (m_accel_decel == scurve ? dev::pcd4641::scurve : dev::pcd4641::linear)
	.set_positioning_mode (m_mode == feed_count)
	.set_org_input_signal (false));

    m_axis.set_feed (m_feed);

    m_axis.drive (dev::pcd4641::drive_mode_t ()
		.set_speed_sel (dev::pcd4641::fh_speed)
		.set_hold_start (false)
		.set_speed_mode (m_accel_decel == accel_decel_disable
				 ? dev::pcd4641::constant_speed
				 : dev::pcd4641::high_speed_accel_decel)
		.set_cmd (dev::pcd4641::start));
  }

  virtual void stop (void) override
  {
    m_axis.drive (m_axis.start_mode ().set_cmd (dev::pcd4641::decel_stop));
  }

  virtual void stop_immediately (void) override
  {
    m_axis.drive (m_axis.start_mode ().set_cmd (dev::pcd4641::stop_immediately));
  }

  virtual bool alarm_status (void) const override
  {
    return m_axis.inputs ()[amp_alarm_input].read ();
  }

  virtual bool servo_on_status (void) const override
  {
    return m_axis.outputs ()[amp_enable_output].read ()
	   && !m_axis.inputs ()[amp_alarm_input].read ();
  }

  virtual sensor_status_t sensor_status (void) const override
  {
    auto s = m_axis.ext_status ();
    return { s.pel (), s.mel (), s.org () };
  }

  virtual bool is_driving (void) const override
  {
    return m_axis.status ().is_driving ();
  }

  virtual void do_homing (void) override
  {
    set_speed (m_speed);

    m_axis.set_output_mode (m_axis.output_mode ()
	.set_pulse_output (true)
	.set_stop_during_accel_decel (false));

    // use the current direction setting of the axis for the origin return drive
    m_axis.set_control_mode (m_axis.control_mode ()
	.set_accel_decel_mode (m_accel_decel == scurve ? dev::pcd4641::scurve : dev::pcd4641::linear)
	.set_positioning_mode (false)
	.set_org_input_signal (true));

    m_axis.set_feed (m_feed);

    m_axis.drive (dev::pcd4641::drive_mode_t ()
		.set_speed_sel (dev::pcd4641::fh_speed)
		.set_hold_start (false)
		.set_speed_mode (m_accel_decel == accel_decel_disable
				 ? dev::pcd4641::constant_speed
				 : dev::pcd4641::high_speed_accel_decel)
		.set_cmd (dev::pcd4641::start));

  }


  virtual uint32_t encoder_feed (void) const override
  {
    #ifdef MCB_HAVE_PCD_AXIS_INPUTS_SAMPLER
      return m_encoder.counter ();
    #else
      return 0;
    #endif
  }

  virtual void reset_encoder_feed (void) override
  {
    #ifdef MCB_HAVE_PCD_AXIS_INPUTS_SAMPLER
      return m_encoder.set_counter (0);
    #endif
  }

  virtual bool pulse_output_mode_supported (dev::pulse_type pt) const override
  {
    return dev::pcd4641::pulse_output_type_supported (pt);
  }

  virtual dev::pulse_type pulse_output_mode (void) const override
  {
    return m_pulse_output_mode;
  }

  virtual void set_pulse_output_mode (dev::pulse_type pt) override
  {
    if (dev::pcd4641::pulse_output_type_supported (pt) && m_pulse_output_mode != pt)
    {
      m_pulse_output_mode = pt;
      m_axis.set_env (m_axis.env ().set_pulse_output_mode (pt));
    }
  }

  virtual bool pulse_input_mode_supported (dev::pulse_type pt) const override
  {
    #ifdef MCB_HAVE_PCD_AXIS_INPUTS_SAMPLER
      return true;
    #else
      return pt == dev::pulse_type::disable;
    #endif
  }

  virtual dev::pulse_type pulse_input_mode (void) const override
  {
    #ifdef MCB_HAVE_PCD_AXIS_INPUTS_SAMPLER
      return m_pulse_input_mode;
    #else
      return dev::pulse_type::disable;
    #endif
  }

  virtual void set_pulse_input_mode (dev::pulse_type pt) override
  {
    #ifdef MCB_HAVE_PCD_AXIS_INPUTS_SAMPLER
      if (m_pulse_input_mode != pt)
      {
	m_pulse_input_mode = pt;
	m_encoder.set_pulse_type (pt);
      }
    #endif
  }
};

//--------------------------------------------------------------------------
// soft-pg axes

struct soft_pg_control_func
{
  void operator () (void);
};

using soft_pg_t = dev::soft_pg < 75'000, this_board::type::tpu0_inst,
				 5, soft_pg_control_func,

  // 2 IOs for one axis (pulse + direction or double-pulse mode)
  dev::bind_digital_output_port_write_t < this_board::type::trigger_outputs_inst, 0 >,
  dev::bind_digital_output_port_write_t < this_board::type::trigger_outputs_inst, 1 >,

  // 2 IOs for one axis (pulse + direction or double-pulse mode)
  dev::bind_digital_output_port_write_t < this_board::type::trigger_outputs_inst, 2 >,
  dev::bind_digital_output_port_write_t < this_board::type::trigger_outputs_inst, 3 >,

  // 2 IOs for one axis (pulse + direction or double-pulse mode)
  dev::bind_digital_output_port_write_t < this_board::type::led_outputs_inst, 0 >,
  dev::bind_digital_output_port_write_t < this_board::type::led_outputs_inst, 1 >,

  // 2 IOs for one axis (pulse + direction or double-pulse mode)
  dev::bind_digital_output_port_write_t < this_board::type::led_outputs_inst, 2 >,
  dev::bind_digital_output_port_write_t < this_board::type::led_outputs_inst, 3 >,

  // ch 8,9
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 0 >,
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 1 >,

  // ch 10,11
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 2 >,
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 3 >,

  // ch 12,13
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 4 >,
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 5 >,

  // ch 14,15
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 6 >,
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 7 >,

  // ch 16,17
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 8 >,
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 9 >,

  // ch 18,19
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 10 >,
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 11 >,

  // ch 20,21
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 12 >,
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 13 >,

  // ch 22,23
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 14 >,
  dev::bind_digital_output_port_write_t < this_board::type::digital_outputs_inst, 15 >

>;

soft_pg_t g_soft_pg;

template <unsigned int N> struct bind_soft_pg_channel_t
{
  auto& operator () (void) const
  {
    static_assert (N < g_soft_pg.channel_count);
    return g_soft_pg.channel (N);
  }
};

using soft_pg_axis0_t = dev::soft_pg_motor::dev_inst < soft_pg_t::ctrl_duration,
						bind_soft_pg_channel_t < 0 >,
						bind_soft_pg_channel_t < 1 > >;
soft_pg_axis0_t g_soft_pg_axis0;

using soft_pg_axis1_t = dev::soft_pg_motor::dev_inst < soft_pg_t::ctrl_duration,
						bind_soft_pg_channel_t < 2 >,
						bind_soft_pg_channel_t < 3 > >;
soft_pg_axis1_t g_soft_pg_axis1;

using soft_pg_axis2_t = dev::soft_pg_motor::dev_inst < soft_pg_t::ctrl_duration,
						bind_soft_pg_channel_t < 4 >,
						bind_soft_pg_channel_t < 5 > >;
soft_pg_axis2_t g_soft_pg_axis2;

using soft_pg_axis3_t = dev::soft_pg_motor::dev_inst < soft_pg_t::ctrl_duration,
						bind_soft_pg_channel_t < 6 >,
						bind_soft_pg_channel_t < 7 > >;
soft_pg_axis3_t g_soft_pg_axis3;


using soft_pg_axis4_t = dev::soft_pg_motor::dev_inst < soft_pg_t::ctrl_duration,
						bind_soft_pg_channel_t < 8 >,
						bind_soft_pg_channel_t < 9 > >;
soft_pg_axis4_t g_soft_pg_axis4;


using soft_pg_axis5_t = dev::soft_pg_motor::dev_inst < soft_pg_t::ctrl_duration,
						bind_soft_pg_channel_t < 10 >,
						bind_soft_pg_channel_t < 11 > >;
soft_pg_axis5_t g_soft_pg_axis5;

using soft_pg_axis6_t = dev::soft_pg_motor::dev_inst < soft_pg_t::ctrl_duration,
						bind_soft_pg_channel_t < 12 >,
						bind_soft_pg_channel_t < 13 > >;
soft_pg_axis6_t g_soft_pg_axis6;

using soft_pg_axis7_t = dev::soft_pg_motor::dev_inst < soft_pg_t::ctrl_duration,
						bind_soft_pg_channel_t < 14 >,
						bind_soft_pg_channel_t < 15 > >;
soft_pg_axis7_t g_soft_pg_axis7;

using soft_pg_axis8_t = dev::soft_pg_motor::dev_inst < soft_pg_t::ctrl_duration,
						bind_soft_pg_channel_t < 16 >,
						bind_soft_pg_channel_t < 17 > >;
soft_pg_axis8_t g_soft_pg_axis8;

using soft_pg_axis9_t = dev::soft_pg_motor::dev_inst < soft_pg_t::ctrl_duration,
						bind_soft_pg_channel_t < 18 >,
						bind_soft_pg_channel_t < 19 > >;
soft_pg_axis9_t g_soft_pg_axis9;

using soft_pg_axis10_t = dev::soft_pg_motor::dev_inst < soft_pg_t::ctrl_duration,
						bind_soft_pg_channel_t < 20 >,
						bind_soft_pg_channel_t < 21 > >;
soft_pg_axis10_t g_soft_pg_axis10;

using soft_pg_axis11_t = dev::soft_pg_motor::dev_inst < soft_pg_t::ctrl_duration,
						bind_soft_pg_channel_t < 22 >,
						bind_soft_pg_channel_t < 23 > >;
soft_pg_axis11_t g_soft_pg_axis11;


void soft_pg_control_func::operator () (void)
{
  g_soft_pg_axis0.exec ();
  g_soft_pg_axis1.exec ();
  g_soft_pg_axis2.exec ();
  g_soft_pg_axis3.exec ();

  g_soft_pg_axis4.exec ();
  g_soft_pg_axis5.exec ();
  g_soft_pg_axis6.exec ();
  g_soft_pg_axis7.exec ();
  g_soft_pg_axis8.exec ();
  g_soft_pg_axis9.exec ();
  g_soft_pg_axis10.exec ();
  g_soft_pg_axis11.exec ();
}

template <typename SoftPGAxis> class soft_pg_axis : public motor_axis
{
public:
  soft_pg_axis (SoftPGAxis& axis, std::string_view name)
  : motor_axis (name), m_axis (axis)
  {
    set_speed (8000);
  }

  virtual void drive_on (bool val) override { }
  virtual uint32_t max_speed (void) const override { return m_axis.speed_max_value_pps; }
  virtual uint32_t max_feed (void) const override { return m_axis.feed_max_value; }
  virtual uint32_t max_accel (void) const override { return 0; }
  virtual uint32_t max_scurve_accel (void) const override { return 0; }
  virtual uint32_t current_speed (void) const override { return m_axis.current_speed_pps (); }
  virtual uint32_t current_feed (void) const override { return m_axis.feed_count (); }
  virtual void reset_current_feed (void) override { }
  virtual void set_speed (uint32_t val) override { m_axis.set_speed_pps (val); }
  virtual uint32_t speed (void) const override { return m_axis.speed_pps (); }
  virtual void set_feed (uint32_t val) override { m_set_feed_count = val; }
  virtual uint32_t feed (void) const override { return m_set_feed_count; }

  virtual uint32_t encoder_feed (void) const override { return 0; }
  virtual void reset_encoder_feed (void) override { }

  virtual bool pulse_output_mode_supported (dev::pulse_type pt) const override { return m_axis.pulse_output_type_supported (pt); }
  virtual dev::pulse_type pulse_output_mode (void) const override { return m_axis.pulse_output_type (); }
  virtual void set_pulse_output_mode (dev::pulse_type pt) override { m_axis.set_pulse_output_type (pt); }

  virtual bool pulse_input_mode_supported (dev::pulse_type pt) const override { return false; }
  virtual dev::pulse_type pulse_input_mode (void) const override { return dev::pulse_type::disable; }
  virtual void set_pulse_input_mode (dev::pulse_type pt) override { }

  virtual direction_t direction (void) const override
  {
    return m_axis.direction () == dev::soft_pg_motor::fwd ? fwd : rev;
  }
  virtual void set_direction (direction_t val) override
  {
    m_axis.set_direction (val == fwd ? dev::soft_pg_motor::fwd : dev::soft_pg_motor::rev);
  }

  virtual mode_t mode (void) const override
  {
    switch (m_axis.drive_mode ())
    {
      case dev::soft_pg_motor::drive_mode::continuous: return continuous;
      case dev::soft_pg_motor::drive_mode::feed_count: return feed_count;
      default: assert_unreachable ();
    }
  }
  virtual void set_mode (mode_t val) override
  {
    switch (val)
    {
      case continuous: m_axis.set_drive_mode (dev::soft_pg_motor::drive_mode::continuous); break;
      case feed_count: m_axis.set_drive_mode (dev::soft_pg_motor::drive_mode::feed_count); break;
      default: break;
    }
  }

  virtual accel_decel_t accel_decel (void) const override { return accel_decel_disable; }
  virtual void set_accel_decel (accel_decel_t val) override { }

  virtual uint32_t accel (void) const override { return 0; }
  virtual void set_accel (uint32_t val) override { }

  virtual uint32_t scurve_accel (void) const override { return 0; }
  virtual void set_scurve_accel (uint32_t val) override { }

  virtual void start (void) override
  {
    if (mode () == feed_count)
      m_axis.set_feed_count (m_set_feed_count);

    m_axis.start ();
  }

  virtual void stop (void) override { m_axis.stop (); }
  virtual void stop_immediately (void) override { m_axis.stop_immediately (); }

  virtual void do_homing (void) override { }

  virtual bool is_driving (void) const override { return m_axis.is_driving (); }

  virtual bool alarm_status (void) const override { return false; }
  virtual bool servo_on_status (void) const override { return false; }

  virtual motor_axis::sensor_status_t sensor_status (void) const override
  {
    auto r = m_axis.sensor_status ();
    return { r.pos_limit, r.neg_limit, r.origin };
  }


private:
  SoftPGAxis& m_axis;
  uint32_t m_set_feed_count = 0;
};

//--------------------------------------------------------------------------
// global instances of the axes.

#if defined (MCB_USE_MCX514)
std::array<std::unique_ptr<motor_axis>, 8 + 4 + 8> g_axes =
{
  std::make_unique<mcx_motor_axis> (this_board::inst ().mcx51x.axis (dev::mcx51x::axis_x), "CN14A (MCX514 X)"),
  std::make_unique<mcx_motor_axis> (this_board::inst ().mcx51x.axis (dev::mcx51x::axis_y), "CN14B (MCX514 Y)"),
  std::make_unique<mcx_motor_axis> (this_board::inst ().mcx51x.axis (dev::mcx51x::axis_z), "CN15A (MCX514 Z)"),
  std::make_unique<mcx_motor_axis> (this_board::inst ().mcx51x.axis (dev::mcx51x::axis_u), "CN15B (MCX514 U)"),

#elif defined (MCB_USE_MCX512)
std::array<std::unique_ptr<motor_axis>, 6 + 4 + 8> g_axes =
{
  std::make_unique<mcx_motor_axis> (this_board::inst ().mcx51x.axis (dev::mcx51x::axis_x), "CN14A (MCX512 X)"),
  std::make_unique<mcx_motor_axis> (this_board::inst ().mcx51x.axis (dev::mcx51x::axis_y), "CN14B (MCX512 Y)"),
#else
std::array<std::unique_ptr<motor_axis>, 4 + 4 + 8> g_axes =
{
#endif

  std::make_unique<pcd_motor_axis> (this_board::inst ().pcd4641.axis (dev::pcd4641::axis_x), "CN13A (PCD4641 X)"),
  std::make_unique<pcd_motor_axis> (this_board::inst ().pcd4641.axis (dev::pcd4641::axis_y), "CN13B (PCD4641 Y)"),
  std::make_unique<pcd_motor_axis> (this_board::inst ().pcd4641.axis (dev::pcd4641::axis_z), "CN11A (PCD4641 Z)"),
  std::make_unique<pcd_motor_axis> (this_board::inst ().pcd4641.axis (dev::pcd4641::axis_u), "CN11B (PCD4641 U)"),

  std::make_unique< soft_pg_axis < soft_pg_axis0_t >> (g_soft_pg_axis0, "SoftPG TrgOut0,TrgOut1"),
  std::make_unique< soft_pg_axis < soft_pg_axis1_t >> (g_soft_pg_axis1, "SoftPG TrgOut2,TrgOut3"),
  std::make_unique< soft_pg_axis < soft_pg_axis2_t >> (g_soft_pg_axis2, "SoftPG LED0,LED1"),
  std::make_unique< soft_pg_axis < soft_pg_axis3_t >> (g_soft_pg_axis3, "SoftPG LED2,LED3"),

  std::make_unique< soft_pg_axis < soft_pg_axis4_t >> (g_soft_pg_axis4, "SoftPG DO0,DO1"),
  std::make_unique< soft_pg_axis < soft_pg_axis5_t >> (g_soft_pg_axis5, "SoftPG DO2,DO3"),
  std::make_unique< soft_pg_axis < soft_pg_axis6_t >> (g_soft_pg_axis6, "SoftPG DO4,DO5"),
  std::make_unique< soft_pg_axis < soft_pg_axis7_t >> (g_soft_pg_axis7, "SoftPG DO6,DO7"),
  std::make_unique< soft_pg_axis < soft_pg_axis8_t >> (g_soft_pg_axis8, "SoftPG DO8,DO9"),
  std::make_unique< soft_pg_axis < soft_pg_axis9_t >> (g_soft_pg_axis9, "SoftPG DO10,DO11"),
  std::make_unique< soft_pg_axis < soft_pg_axis10_t >> (g_soft_pg_axis10, "SoftPG DO12,DO13"),
  std::make_unique< soft_pg_axis < soft_pg_axis11_t >> (g_soft_pg_axis11, "SoftPG DO14,DO15"),
};

// DD motor browser control configuration.
#if defined (MCB_USE_MCX514)
static constexpr unsigned int g_dd_motor_axis_count = 4;

static constexpr std::array<uint32_t, g_dd_motor_axis_count> g_dd_motor_pulses_per_revolution = 
{
    360000,
    360000,
    360000,
    360000
};

static constexpr std::array<bool, g_dd_motor_axis_count> g_dd_motor_home_is_forward = 
{
    true,
    true,
    true,
    true
};

static std::array<bool, g_dd_motor_axis_count> g_dd_motor_software_home_is_set = 
{
  false,
  false,
  false,
  false
};

#endif


//--------------------------------------------------------------------------

static auto g_romfs = fs::romfs::mount_this_image_partition ();

struct http_server_delegate : public net::http::server::delegate
{
  virtual std::unique_ptr<fs::file>
  handle_file_request (const net::http::server::request& req, std::string_view req_uri) override
  {
    if (g_romfs != nullptr)
      return g_romfs->open_file (req_uri);

    return { };
  }

  virtual net::http::response
  handle_request (const net::http::server::request& req, std::string_view req_uri) override
  {
    if (req_uri == "/" && req.request_line ().method () == "GET")
      return html_root (req);

    if (req_uri == "/axis_param" && req.request_line ().method () == "POST")
      return set_axis_param (req);

    if (req_uri == "/axis_status" && req.request_line ().method () == "GET")
      return get_axis_status (req);

    if (req_uri == "/axis_cmd" && req.request_line ().method () == "POST")
      return do_axis_cmd (req);

    if (req_uri == "/dd_motor_cmd" && req.request_line().method() == "POST")
        return do_dd_motor_cmd(req);

    std::printf ("%s  %s\n", std::string (req.request_line ().method ()).c_str (),
			       std::string (req_uri).c_str ());

    return { };
  }

  net::http::response get_axis_status (const net::http::request& req)
  {
    std::string out;
    out.reserve ();

    bool first = true;

    for (auto&& a : g_axes)
    {
      auto sensor_st = a->sensor_status ();

      out += utils::printf_format ("%saxis=%u"
				"&cur_speed=%u"
				"&cur_feed=%u"
				"&enc_feed=%u"
				"&set_speed_val=%u"
				"&set_feed_val=%u"
				"&direction_val=%s"
				"&mode_val=%s"
				"&accel_decel_val=%s"
				"&accel_val=%u"
				"&scurve_accel_val=%u"
				"&alarm=%d"
				"&srvon=%d"
				"&lmtp=%d"
				"&lmtn=%d"
				"&org=%d"
				"&pulse_out_mode=%d"
				"&pulse_in_mode=%d",

	first ? "" : "&",
	(unsigned int)(&a - &g_axes.front ()),
	a->current_speed (),
	a->current_feed (),
	a->encoder_feed (),
	a->speed (),
	a->feed (),
	to_string (a->direction ()),
	to_string (a->mode ()),
	to_string (a->accel_decel ()),
	a->accel (),
	a->scurve_accel (),
	a->alarm_status (),
	a->servo_on_status (),
	sensor_st.limit_p,
	sensor_st.limit_n,
	sensor_st.org,
	(int)a->pulse_output_mode (),
	(int)a->pulse_input_mode ());

      first = false;
    }

    return net::http::make_200_ok_response (req, "application/x-www-form-urlencoded", std::move (out));
  }

  net::http::response do_axis_cmd (const net::http::request& req)
  {
    const std::string& content = req.content_data ();
    if (content.find ("axis=") == 0)

    try
    {
      int axis_num = std::stoi (content.substr (std::strlen ("axis=")));

      if (axis_num < 0 || axis_num >= (int)g_axes.size ())
	throw std::invalid_argument ("axis number out of range");

      auto& axis = *(g_axes[axis_num]);

      auto i0 = content.find (' ');
      auto i1 = content.size ();

      std::string_view cmd_name (content.data () + i0 + 1, i1 - i0 - 1);
      // std::printf ("axis %d  cmd %s (%zu)\n", axis_num, cmd_name.to_string ().c_str (), cmd_name.size ());

      if (cmd_name == "start")
	axis.start ();
      else if (cmd_name == "stop")
	axis.stop ();
      else if (cmd_name == "stop_imm")
	axis.stop_immediately ();
      else if (cmd_name == "fwd")
	axis.start_fwd ();
      else if (cmd_name == "rev")
	axis.start_rev ();
      else if (cmd_name == "on")
	axis.drive_on (true);
      else if (cmd_name == "off")
	axis.drive_on (false);
      else if (cmd_name == "home")
	axis.do_homing ();
      else if (cmd_name == "reset_feed")
	axis.reset_current_feed ();
      else if (cmd_name == "reset_enc")
	axis.reset_encoder_feed ();
    }
    catch (const std::exception&)
    {
    }

    return net::http::make_200_ok_response (req);
  }

    static bool dd_get_form_value (const std::string& form,
                                   const char* name,
                                   std::string_view& value)
    {
        const std::string_view key (name);
        std::size_t begin = 0;

        while (begin <= form.size ())
        {
            std::size_t end = form.find ('&', begin);
            if (end == std::string::npos)
                end = form.size ();

            std::string_view field (form.data () + begin, end - begin);

            if (field.size() > key.size ()
                && field.substr (0, key.size ()) == key
                && field[key.size ()] == '=')
            {
                value = field.substr (key.size () + 1);
                return true;
            }

            if (end == form.size ())
                break;

            begin = end + 1;
        }

        return false;
    }

    static bool dd_parse_u32 (std::string_view text, uint32_t& value)
    {
        if(text.empty ())
            return false;

        for (char c : text)
            if (c < '0' || c > '9')
                return false;

        try
        {
            std::size_t consumed = 0;
            const unsigned long long parsed = 
                std::stoull (std::string (text), &consumed);

            if (consumed != text.size () || parsed > 0xffffffffULL)
                return false;

            value = static_cast<uint32_t> (parsed);
            return true;
        }
        catch (const std::exception&)
        {
            return false;
        }
    }

    static bool dd_get_u32 (const std::string& form,
                            const char* name,
                            uint32_t& value)
    {
        std::string_view text;

        return dd_get_form_value (form, name, text)
                && dd_parse_u32 (text, value);
    }

    net::http::response make_dd_motor_response (const net::http::request& req, const char* result)
    {
      std::string content = "result=";
      content += result;

      return net::http::make_200_ok_response(req, "application/x-www-form-urlencoded", std::move (content));
    }

    net::http::response do_dd_motor_cmd (const net::http::request& req)
    {
        const std::string& content = req.content_data ();

        std::string_view action;
        if (!dd_get_form_value (content, "action", action))
            return net::http::make_200_ok_response (req);

        if (action == "stop")
        {
            for (auto&& axis : g_axes)
                axis->stop_immediately ();

            return make_dd_motor_response (req, "ok");
        }

  #if defined (MCB_USE_MCX514)
        uint32_t axis_num = 0;
        if (!dd_get_u32 (content, "axis", axis_num))
            return make_dd_motor_response (req, "invalid_axis");

        if (axis_num >= g_dd_motor_axis_count || 
            axis_num >= g_axes.size ())
            return make_dd_motor_response (req, "invalid_axis");

        auto& axis = *(g_axes[axis_num]);

        auto& mcx_axis = this_board::inst ().mcx51x.axis (axis_num);

        if (action != "set_home" &&
            action != "home" &&
            action != "find_origin" &&
            action != "forward" &&
            action != "reverse")
            return make_dd_motor_response (req, "invalid_action");

        if (axis.alarm_status ())
            return make_dd_motor_response (req, "alarm");

        if (!axis.servo_on_status ())
            return make_dd_motor_response (req, "servo_off");

        if (action == "set_home")
        {
          if (axis.is_driving ())
            return make_dd_motor_response (req, "axis_busy");

          mcx_axis.set_logical_position_counter (0);

          mcx_axis.set_real_position_counter (0);

          g_dd_motor_software_home_is_set[axis_num] = true;

          return make_dd_motor_response (req, "ok");
        }
        
        if (action == "find_origin")
        {
          if (axis.is_driving ())
            return make_dd_motor_response (req, "axis_busy");

          axis.set_direction (
              g_dd_motor_home_is_forward[axis_num]
                  ? motor_axis::fwd
                  : motor_axis::rev
          );

          axis.do_homing ();

          g_dd_motor_software_home_is_set[axis_num] = false;

          return make_dd_motor_response (req, "ok");
        }

        if (action == "home")
        {
          if (!g_dd_motor_software_home_is_set[axis_num])
            return make_dd_motor_response (req, "home_not_set");

          if (axis.is_driving ())
            return make_dd_motor_response (req, "axis_busy");

          const int32_t current_position = mcx_axis.logical_position_counter ();

          if (current_position == 0)
            return make_dd_motor_response (req, "ok");

          const uint64_t distance = current_position < 0 ? 
                                        static_cast<uint64_t> (-static_cast<int64_t> (current_position)) :
                                        static_cast<uint64_t> (current_position);

          if (distance == 0 || distance > axis.max_feed ())
            return make_dd_motor_response (req, "invalid_parameter");

          axis.set_mode (motor_axis::feed_count);
          axis.set_feed (static_cast<uint32_t> (distance));

          if (current_position > 0)
            axis.start_rev ();
          else
            axis.start_fwd ();

          return make_dd_motor_response (req, "ok");
        }

        // if (action != "forward" && action != "reverse")
        //     return net::http::make_200_ok_response (req);

        const bool is_forward = action == "forward";

        std::string_view run_mode;
        if (!dd_get_form_value (content, "run_mode", run_mode))
            return make_dd_motor_response (req, "invalid_parameter");

        if (run_mode == "speed")
        {
            uint32_t speed = 0;

            if (!dd_get_u32(
                    content,
                    is_forward ? "forward_speed" : "reverse_speed",
                    speed))
                return make_dd_motor_response (req, "invalid_parameter");

            if (speed == 0 || speed > axis.max_speed ())
                return make_dd_motor_response (req, "invalid_parameter");

            axis.set_mode (motor_axis::continuous);
            axis.set_speed (speed);

            if (is_forward)
                axis.start_fwd ();
            else 
                axis.start_rev ();

            return make_dd_motor_response (req, "ok");
        }
        else if (run_mode == "pulse")
        {
          uint32_t speed = 0;
          uint32_t pulse_count =0;

          if (!dd_get_u32 (
                content,
                is_forward ? "forward_speed" : "reverse_speed",
                speed))
            return make_dd_motor_response (req, "invalid_parameter");

          if (!dd_get_u32 (
              content,
              "pulse_count",
              pulse_count))
          return make_dd_motor_response (req, "invalid_parameter");

          if (speed == 0 || speed > axis.max_speed ())
            return make_dd_motor_response (req, "invalid_parameter");

          if (pulse_count == 0 ||
              pulse_count > axis.max_feed())
            return make_dd_motor_response (req, "invalid_parameter");

          axis.set_mode (motor_axis::feed_count);
          axis.set_speed (speed);
          axis.set_feed (pulse_count);

          if (is_forward)
            axis.start_fwd ();
          else
            axis.start_rev ();

          return make_dd_motor_response (req, "ok");
        }
        else if (run_mode == "angle")
        {
            uint32_t angle_mdeg = 0;
            uint32_t angle_speed = 0;

            if (!dd_get_u32 (content, "angle_mdeg", angle_mdeg) ||
                !dd_get_u32 (content, "angle_speed", angle_speed))
                return net::http::make_200_ok_response (req);

            if (angle_mdeg == 0 ||
                angle_speed == 0 ||
                angle_speed > axis.max_speed ())
                return make_dd_motor_response (req, "invalid_parameter");

            const uint64_t pulses_per_revolution = g_dd_motor_pulses_per_revolution[axis_num];

            const uint64_t feed64 = (static_cast<uint64_t> (angle_mdeg) * pulses_per_revolution + 180000ULL) / 360000ULL;

            if (feed64 == 0 || feed64 > axis.max_feed ())
                return make_dd_motor_response (req, "invalid_parameter");

            axis.set_mode (motor_axis::feed_count);
            axis.set_speed (angle_speed);
            axis.set_feed (static_cast<uint32_t> (feed64));

            if (is_forward)
                axis.start_fwd ();
            else 
                axis.start_rev ();

            return make_dd_motor_response (req, "ok");
        }
#endif
        return net::http::make_200_ok_response (req);
    }

  net::http::response set_axis_param (const net::http::request& req)
  {
    const std::string& content = req.content_data ();
    if (content.find ("axis=") == 0)
    {
      try
      {
	// FIXME: std::stoi causes an LTO error sometimes ...
	// (depends on the total program size)

	int axis_num = std::stoi (content.substr (std::strlen ("axis=")));

	if (axis_num < 0 || axis_num >= (int)g_axes.size ())
	  throw std::invalid_argument ("axis number out of range");

	auto& axis = *(g_axes[axis_num]);

	auto i0 = content.find (' ');
	auto i1 = content.find ('=', i0);

	std::string_view param_name (content.data () + i0 + 1, i1 - i0 - 1);
	std::string_view param_val (content.data () + i1 + 1, content.size () - i1 - 1);

	if (param_name == "set_speed_val")
	{
	  int val = std::stoi (std::string (param_val));
	  //std::printf ("set speed = %d\n", val);

	  axis.set_speed (val);
	}
	else if (param_name == "set_feed_val")
	{
	  int val = std::stoi (std::string (param_val));
	  // std::printf ("set feed = %d\n", val);

	  axis.set_feed (val);
	}
	else if (param_name == "direction_val")
	{
	  //std::printf ("direction = %s\n", param_val.to_string ().c_str ())

	  if (param_val == "fwd")
	    axis.set_direction (motor_axis::fwd);
	  else if (param_val == "rev")
	    axis.set_direction (motor_axis::rev);
	}
	else if (param_name == "mode_val")
	{
	  //std::printf ("mode = %s\n", param_val.to_string ().c_str ());

	  if (param_val == "cont")
	    axis.set_mode (motor_axis::continuous);
	  else if (param_val == "feed")
	    axis.set_mode (motor_axis::feed_count);
	}
	else if (param_name == "accel_decel_val")
	{
	  //std::printf ("accel_decel_val = %s\n", param_val.to_string ().c_str ());

	  if (param_val == "off")
	    axis.set_accel_decel (motor_axis::accel_decel_disable);
	  else if (param_val == "linear")
	    axis.set_accel_decel (motor_axis::linear);
	  else if (param_val == "scurve")
	    axis.set_accel_decel (motor_axis::scurve);
	}
	else if (param_name == "accel_val")
	{
	  int val = std::stoi (std::string (param_val));
	  //std::printf ("accel_val = %d\n", val);

	  axis.set_accel (val);
	}
	else if (param_name == "scurve_accel_val")
	{
	  int val = std::stoi (std::string (param_val));
	  //std::printf ("scurve_accel_val = %d\n", val);

	  axis.set_scurve_accel (val);
	}
	else if (param_name == "pulse_out_mode")
	{
	  axis.set_pulse_output_mode ((dev::pulse_type)std::stoi (std::string (param_val)));
	}
	else if (param_name == "pulse_in_mode")
	{
	  axis.set_pulse_input_mode ((dev::pulse_type)std::stoi (std::string (param_val)));
	}
      }
      catch (const std::exception&)
      {
      }
    }

    return net::http::make_200_ok_response (req);
  }


  net::http::response html_root (const net::http::request& req)
  {
    std::string out;
    out.reserve (1024*8);

    out += R"raw(

<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN" "DTD/xhtml1-strict.dtd">
<html xmlns="http://www.w3.org/1999/xhtml" xml:lang="en" lang="en">
<head><title>MCB Motor Test Example</title></head>

<style type="text/css">

div.axis_box
{
  border: 1px solid black;
  margin: 4px;
  padding: 4px;
  display: inline-block;
}

thead
{
  background-color: lightgray;
}

select, input.number_param
{
  width: 100%;
  text-align: right;
}

.stat_disp
{
  font-weight: bold;
}

.action_button
{
  margin: 4px;
}

#dd_motor_control
{
    display:block;
    width:calc(100% - 18px);
    max-width:600px;
    box-sizing:border-box;
    margin-top:16px;
}

#dd_motor_control table
{
    width:100%;
    table-layout:fixed;
}

#dd_motor_control td:first-child
{
    width:180px;
}

#dd_motor_control select,
#dd_motor_control input.number_param
{
  width: 100%;
  box-sizing: border-box;
}

.dd_motor_buttons
{
  display: table;
  width: 100%;
  border-spacing: 6px;
}

.dd_motor_button_row
{
  display: table-row;
}

.dd_motor_button_cell
{
  display: table-cell;
  width: 50%;
}

.dd_motor_button
{
  width: 100%;
  box-sizing: border-box;
}

.stat_led
{
  font-weight: bold;
  color: #E0E0E0;
  text-shadow: 0 0 0.125px #000000
}

.stat_led[state=on][color=red]
{
  color: #F00000;
  text-shadow: 0 0 0.125px #000000, 0 0 16px #F05050, 0 0 8px #F00000
}

.stat_led[state=on][color=green]
{
  color: #00F000;
  text-shadow: 0 0 0.125px #000000, 0 0 16px #50F050, 0 0 8px #00F000
}

</style>


<script language="javascript">

function on_page_load ()
{
  update_status ();
  dd_run_mode_changed ();
}

var http_req;
var http_req_timeout_obj;

function update_status ()
{
  if (!http_req)
    http_req = new XMLHttpRequest ();

  http_req.open ("GET", "axis_status?t=" + Math.random (), true);
  try
  {
    http_req.timeout = 500;
  }
  catch (err) { }

  http_req.onreadystatechange = function ()
  {
    if (http_req.readyState == 4 && http_req.status == 200)
    {
      var linkst_e = document.getElementById ("link_status");
      if (linkst_e.getAttribute ("state") !== "on")
	    linkst_e.setAttribute ("state", "on");

      //document.getElementById ("link_status").setAttribute ("state", "on");
      clearTimeout (http_req_timeout_obj);

      var key_values = http_req.responseText.split ('&');
      for (var i = 0; i < key_values.length; ++i)
      {
        var kv = key_values[i].split ('=');

        var current_axis_box;

        if (kv[0] == "axis")
        {
        current_axis_box = document.getElementById ("axis_" + kv[1]);
        // print_debug ("current axis box = " + current_axis_box);
        }
        else if (current_axis_box)
        {
        var e = current_axis_box.querySelector ("#" + kv[0]);
        // print_debug (current_axis_box.id + " " + kv[0] + " = " + e.type);

            if (e)
            {
                e.name = e.id;

                // do not change the value of an element which is currently
                // being changed by the user.
                if (e !== document.activeElement)
                {
                    if (e.className === "stat_led")
                    {
                        if (kv[1] == "1")
                        {
                            if (e.getAttribute ("state") !== "on")
                                e.setAttribute ("state", "on");
                        }
                        else if (e.getAttribute ("state") !== "off")
                            e.setAttribute ("state", "off");
                    }
                    else if (e.type === "checkbox")
                    {
                        if (e.checked != (kv[1] == "1"))
                            e.checked = kv[1] == "1";
                    }
                    else if (e.type === "number")
                        e.value = kv[1];
                    else if (e.type === "select-one")
                        e.value = kv[1];
                    else if (e.type === "text")
                    {
                        if (e.value != kv[1])
                            e.value = kv[1];
                    }
                    else if (e.innerHTML != kv[1])
                        e.innerHTML = kv[1];
                }
            }
        }
      }
      dd_update_ready_state ();
    }

    

    // the request has finished (successfully or not).
    // send the next request after some time.
    if (http_req.readyState == 4)
      setTimeout (update_status, 100);
  }

  http_req.send ();
  http_req_timeout_obj = setTimeout (function ()
  {
    document.getElementById ("link_status").setAttribute ("state", "off");
  }, 500)
}


function send_params (e)
{
  var axis_num = get_axis (e);
  if (axis_num < 0)
    return;

  var param_name = e.id;
  var param_val = e.value;

  //print_debug ("send_params axis " + axis_num + " " + param_name + " = " + param_val);

  var req = new XMLHttpRequest ();
  req.open ("POST", "axis_param", true);
  req.setRequestHeader ("Content-Type", "text/plain");
  req.send ("axis=" + axis_num + " " + param_name + "=" + param_val);
}

function axis_cmd (e, cmdstr)
{
  var axis_num = get_axis (e);

  if (axis_num < 0)
    return;

  // print_debug ("axis_cmd " + axis_num + " " + cmdstr);

  var req = new XMLHttpRequest ();
  req.open ("POST", "axis_cmd", true);
  req.setRequestHeader ("Content-Type", "text/plain");
  req.send ("axis=" + axis_num + " " + cmdstr);
}

function dd_run_mode_changed ()
{
    var run_mode = document.getElementById ("dd_run_mode").value;

    var uses_direction_speed = run_mode === "speed" || run_mode === "pulse";

    var is_pulse_mode = run_mode === "pulse";
    var is_angle_mode = run_mode === "angle";

    document.getElementById ("dd_forward_speed").disabled = !uses_direction_speed;
    document.getElementById ("dd_reverse_speed").disabled = !uses_direction_speed;
    document.getElementById ("dd_pulse_count").disabled = !is_pulse_mode;
    document.getElementById ("dd_angle").disabled = !is_angle_mode;
    document.getElementById ("dd_angle_speed").disabled = !is_angle_mode;
}

function dd_selected_axis_is_servo_on ()
{
    var axis_num = document.getElementById ("dd_bind").value;
    var axis_box = document.getElementById ("axis_" + axis_num);

    if (!axis_box)
        return false;

    var servo_status = axis_box.querySelector ("#srvon");

    return servo_status && servo_status.getAttribute ("state") === "on";
}

function dd_update_ready_state ()
{
    var ready = dd_selected_axis_is_servo_on ();

    document.getElementById ("dd_set_home_button").disabled = !ready;
    document.getElementById ("dd_home_button").disabled = !ready;
    document.getElementById ("dd_find_origin_button").disabled = !ready;
    document.getElementById ("dd_reverse_button").disabled = !ready;
    document.getElementById ("dd_forward_button").disabled = !ready;
}

function dd_number_value (element_id)
{
    var value = Number (document.getElementById (element_id).value);

    if (!Number.isFinite (value) || value < 0)
        return -1;

    return value;
}

function dd_motor_cmd (action)
{
    if (action !== "stop" && !dd_selected_axis_is_servo_on ())
    {
        alert ("请先在所绑定的轴控制框中点击 On。");
        return;
    }

  var body = "action=" + encodeURIComponent (action);

  if (action !== "stop")
  {
    var axis = document.getElementById ("dd_bind").value;
    body += "&axis=" + encodeURIComponent (axis);
  }

  if (action === "forward" || action === "reverse")
  {
    var run_mode = document.getElementById ("dd_run_mode").value;
    body += "&run_mode=" + encodeURIComponent (run_mode);

    if (run_mode === "speed" || run_mode === "pulse")
    {
      var speed_element_id = action === "forward" ? "dd_forward_speed" : "dd_reverse_speed";
      var speed_parameter_name = action === "forward" ? "forward_speed" : "reverse_speed";

      var speed = dd_number_value (speed_element_id);
      if (speed <= 0 || !Number.isInteger (speed))
      {
        alert ("The selected direction speed must be a positive integer.");
        return;
      }
      body += "&" + speed_parameter_name + "=" + Math.round (speed);

      if (run_mode === "pulse")
      {
        var pulse_count = dd_number_value ("dd_pulse_count");

        if (pulse_count <= 0 || !Number.isInteger (pulse_count))
        {
          alert ("Pulse Count must be a positive integer.");
          return;
        }

        body += "&pulse_count=" + Math.round (pulse_count);
      }
    }
    else if (run_mode === "angle")
    {
      var angle = dd_number_value ("dd_angle");
      var angle_speed = dd_number_value ("dd_angle_speed");

      if (angle <= 0 || angle_speed <= 0)
      {
        alert ("Angle and Angle Speed must be greater than zero.");
        return;
      }

      if (!Number.isInteger (angle_speed))
      {
        alert ("Angle Speed must be a positive integer.");
        return;
      }

      body +=
        "&angle_mdeg=" + Math.round (angle * 1000) +
        "&angle_speed=" + Math.round (angle_speed);
    }
    else
    {
      alert ("Unknown DD motor run mode.");
      return;
    }
  }

  var req = new XMLHttpRequest ();

  req.open ("POST", "dd_motor_cmd", true);
  req.setRequestHeader (
    "Content-Type",
    "application/x-www-form-urlencoded");

  req.onreadystatechange = function ()
  {
    if (req.readyState !== 4)
      return;

    if (req.status !== 200)
    {
      alert ("DD motor command failed: network error.");
      return;
    }

    if (req.responseText === "result=servo_off")
    {
      alert (
        "The selected axis is not enabled. " +
        "Please click On in the bound axis control box first.");
    }
    else if (req.responseText === "result=alarm")
    {
      alert (
        "The selected axis has an alarm. " +
        "Please clear the servo alarm before operation.");
    }
    else if (req.responseText === "result=invalid_axis")
    {
      alert ("The selected DD motor axis is invalid.");
    }
    else if (req.responseText === "result=invalid_parameter")
    {
      alert ("One or more DD motor parameters are invalid.");
    }
    else if (req.responseText === "result=unsupported")
    {
      alert ("DD motor control is not supported by this board.");
    }
    else if (req.responseText === "result=home_not_set")
    {
      alert ("Software Home has not been set. " +
             "Move the motor to the desired position and press Set Home first.");
    }
    else if (req.responseText === "result=axis_busy")
    {
      alert ("The selected axis is moving. " +
             "Press Stop and wait for the axis to stop first.");
    }
    else if (req.responseText === "result=ok" && 
             action === "set_home")
    {
      alert ("The current position has been recorded as Software Home.");
    }

  };

  req.send (body);
}

function get_axis (e)
{
  for (var pe = e.parentElement; pe; pe = pe.parentElement)
  {
    var s = pe.id.substring (0, 5);

    if (s === "axis_")
    {
      var i = parseInt (pe.id.substring (5));
      return i >= 0 ? i : -1;
    }
  }

  return -1;
}

function print_debug (msg)
{
  var e = document.getElementById ("debug");
  e.style.display = "block";

  e.innerHTML += "<br/>" + msg;
}

</script>

<body onload="javascript:on_page_load()">
<div class="content"><div class="centered">

<div class="stat_led" id="link_status" state="off" color="green">LINK</div>

)raw";

    // ------------------------------------------------------------------

    for (auto&& a : g_axes)
      out += make_axis_controls_box (*a, &a - &g_axes.front ());

#if defined (MCB_USE_MCX514)
    out += make_dd_motor_controls_box ();
#endif
    // ------------------------------------------------------------------
    out += R"raw(
</div>

 <div id="debug" style="display:none">debug</div>

</body></html>

)raw";

    return net::http::make_200_ok_response (req, "text/html", std::move (out));
  }


  std::string make_axis_controls_box (const motor_axis& a, int html_id)
  {
    dev::pulse_type all_pulse_types[] =
    {
      dev::pulse_type::disable,
      dev::pulse_type::single_pulse,
      dev::pulse_type::single_pulse_direction,
      dev::pulse_type::double_pulse,
      dev::pulse_type::ab_phase_single_edge,
      dev::pulse_type::ab_phase_double_edge,
      dev::pulse_type::ab_phase_quad_edge
    };

    std::string pulse_out_modes;
    std::string pulse_in_modes;

    for (auto& tp : all_pulse_types)
    {
      pulse_out_modes += utils::printf_format (
	"<option value=\"%d\" %s>%s</option>",
	(int)tp, a.pulse_output_mode_supported (tp) ? "" : "disabled", to_string (tp));

      pulse_in_modes +=  utils::printf_format (
	"<option value=\"%d\" %s>%s</option>",
	(int)tp, a.pulse_input_mode_supported (tp) ? "" : "disabled", to_string (tp));
    }

    return utils::printf_format (R"raw(

<div class="axis_box" id="axis_%d">
  <table style="width:100%%">
  <thead><tr><th colspan="2"> %s </th></tr> </thread>
  <tbody>

  <tr><td colspan="2">
    <input type="button" value="On" onclick="axis_cmd(this, 'on')" style="width:120px;">
    <input type="button" value="Off" onclick="axis_cmd(this, 'off')" style="width:120px;">
  </td></tr>

  <tr><td colspan="2">
    <span class="stat_led" color="red" id="alarm" state="off">ALM</span>
    <span class="stat_led" color="green" id="srvon" state="off">ON</span>
    <span class="stat_led" color="red" id="lmtp" state="off">LMT+</span>
    <span class="stat_led" color="red" id="lmtn" state="off">LMT-</span>
    <span class="stat_led" color="green" id="org" state="off">ORG</span>
  </td></tr>

  <tr class="stat_disp"><td>Current Speed</td><td id="cur_speed"></td></tr>
  <tr class="stat_disp"><td>Current Feed</td><td id="cur_feed"></td></tr>
  <tr class="stat_disp"><td>Encoder Feed</td><td id="enc_feed"></td></tr>

  <tr><td>Set Speed</td>
  <td><input class="number_param" type="number" id="set_speed_val" min="0" max="%d"
	 onchange="" oninput="send_params(this)" onwheel="" /></td></tr>

  <tr><td>Set Feed</td>
  <td><input class="number_param" type="number" id="set_feed_val" min="0" max="%d"
	 onchange="" oninput="send_params(this)" onwheel="" /></td></tr>

  <tr><td>Direction</td>
  <td><select id="direction_val" oninput="send_params(this)">
	<option value="fwd">Forward</option>
	<option value="rev">Reverse</option> </select></td></tr>

  <tr><td>Mode</td>
  <td><select id="mode_val" oninput="send_params(this)">
	<option value="cont">Continuous</option>
	<option value="feed">Feed Count</option> </select></td></tr>

  <tr><td>Accel/Decel</td>
  <td><select id="accel_decel_val" oninput="send_params(this)">
	<option value="off">Disable</option>
	<option value="linear">Linear</option>
	<option value="scurve">S-Curve</option> </select></td></tr>

  <tr><td>Accel</td>
  <td>
  <input class="number_param" type="number" id="accel_val" min="0" max="%d"
	 onchange="" oninput="send_params(this)" onwheel="" /></td></tr>

  <tr><td>S-Curve Accel</td>
  <td>
  <input class="number_param" type="number" id="scurve_accel_val" min="0" max="%d"
	 onchange="" oninput="send_params(this)" onwheel="" /></td></tr>

  <tr><td>Pulse Output</td>
  <td><select id="pulse_out_mode" oninput="send_params(this)">%s</select></td></tr>

  <tr><td>Pulse Input</td>
  <td><select id="pulse_in_mode" oninput="send_params(this)">%s</select></td></tr>

  </tbody>
  </table>

  <br>

  <input type="button" value="Start" onclick="axis_cmd(this, 'start')" style="width:120px;"/><br>
  <input type="button" value="Stop" onclick="axis_cmd(this, 'stop')" style="width:120px;"/>
  <input type="button" value="Stop Imm." onclick="axis_cmd(this, 'stop_imm')" style="width:120px;"/><br>
  <input type="button" value="Rev" onclick="axis_cmd(this, 'rev')" style="width:120px;"/>
  <input type="button" value="Fwd" onclick="axis_cmd(this, 'fwd')" style="width:120px;"/><br>
  <input type="button" value="Home" onclick="axis_cmd(this, 'home')" style="width:120px;"/><br>
  <input type="button" value="Reset Feed" onclick="axis_cmd(this, 'reset_feed')" style="width:120px;"/>
  <input type="button" value="Reset Enc" onclick="axis_cmd(this, 'reset_enc')" style="width:120px;"/><br>

</div>

)raw",

	html_id, a.name ().data (), a.max_speed (), a.max_feed (),
	a.max_accel (), a.max_scurve_accel (),
	pulse_out_modes.c_str (), pulse_in_modes.c_str ());
  }


std::string make_dd_motor_controls_box (void)
{
    std::string bind_options;

    for (unsigned int i = 0; i < g_dd_motor_axis_count; ++i)
    {
        bind_options += utils::printf_format ("<option value=\"%u\">%s</option>", i,
                                              g_axes[i]->name ().data ());
    }

    return utils::printf_format (R"raw(
        
<div class="axis_box" id="dd_motor_control">
    <table>
        <thead>
            <tr><th colspan="2">DD Motor Control</th></tr>
        </thead>

        <tbody>
            <tr>
                <td>Bind</td>
                <td>
                    <select id="dd_bind" onchange="dd_update_ready_state()">
                        %s
                    </select>
                </td>
            </tr>

            <tr>
                <td>Forward Speed</td>
                <td>
                    <input class="number_param"
                           id="dd_forward_speed"
                           type="number"
                           min="1"
                           step="1"
                           value="8000" />
                </td>
            </tr>

            <tr>
                <td>Reverse Speed</td>
                <td>
                    <input class="number_param"
                           id="dd_reverse_speed"
                           type="number"
                           min="1"
                           step="1"
                           value="8000" />
                </td>
            </tr>

            <tr>
                <td>Pulse Count</td>
                <td>
                    <input class="number_param"
                           id="dd_pulse_count"
                           type="number"
                           min="1"
                           max="2147483646"
                           step="1"
                           value="10000" />
                </td>
            </tr>

            <tr>
                <td>Angle (deg)</td>
                <td>
                    <input class="number_param"
                           id="dd_angle"
                           type="number"
                           min="0.001"
                           step="0.001"
                           value="360" />
                </td>
            </tr>

            <tr>
                <td>Angle Speed</td>
                <td>
                    <input class="number_param"
                           id="dd_angle_speed"
                           type="number"
                           min="1"
                           step="1"
                           value="8000" />
                </td>
            </tr>

            <tr>
                <td>Run Mode</td>
                <td>
                    <select id="dd_run_mode"
                            onchange="dd_run_mode_changed()">
                        <option value="speed">Speed</option>
                        <option value="pulse">Pulse Count</option>
                        <option value="angle">Angle</option>
                    </select>
                </td>
            </tr>
        </tbody>
    </table>

    <br>

    <div class="dd_motor_buttons">
      <div class="dd_motor_button_row">
        <div class="dd_motor_button_cell">
          <input class="dd_motor_button"
                id="dd_set_home_button"
                type="button"
                value="Set Home"
                onclick="dd_motor_cmd('set_home')" />
        </div>

        <div class="dd_motor_button_cell">
          <input class="dd_motor_button"
                id="dd_home_button"
                type="button"
                value="Home"
                onclick="dd_motor_cmd('home')" />
        </div>

        <div class="dd_motor_button_cell">
            <input class="dd_motor_button"
                  id="dd_find_origin_button"
                  type="button"
                  value="Find ORG"
                  onclick="dd_motor_cmd('find_origin')" />
        </div>
      </div>

      <div class="dd_motor_button_row">
        <div class="dd_motor_button_cell">
          <input class="dd_motor_button"
                id="dd_reverse_button"
                type="button"
                value="Reverse"
                onclick="dd_motor_cmd('reverse')" />
        </div>

        <div class="dd_motor_button_cell">
          <input class="dd_motor_button"
                id="dd_forward_button"
                type="button"
                value="Forward"
                onclick="dd_motor_cmd('forward')" />
        </div>

        <div class="dd_motor_button_cell">
          <input class="dd_motor_button"
                type="button"
                value="Stop"
                onclick="dd_motor_cmd('stop')" />
        </div>
      </div>
    </div>
  </div>     
        )raw", bind_options.c_str ());
}

};



//--------------------------------------------------------------------------
// each axis has 4 buttons assigned to it.

static auto&& digital_inputs = this_board::inst ().digital_inputs;
static auto&& digital_outputs = this_board::inst ().digital_outputs;

class button
{
public:
  constexpr button (unsigned int num)
  : m_num (num), m_down (false), m_prev_down (false), m_down_time (), m_up_time () { }

  bool is_down (void) const { return m_down; }
  bool is_up (void) const { return !is_down (); }

  bool was_down (void) const { return m_prev_down; }
  bool was_up (void) const { return !was_down (); }

  void exec (std::chrono::high_resolution_clock::time_point cur_time)
  {
    m_prev_down = m_down;
    m_down = digital_inputs[m_num].read ();
    if (m_prev_down != m_down)
    {
      if (m_down)
        m_down_time = cur_time;
      else
        m_up_time = cur_time;
    }
  }

  const std::chrono::high_resolution_clock::time_point& down_time (void) const { return m_down_time; }
  const std::chrono::high_resolution_clock::time_point& up_time (void) const { return m_up_time; }

private:
  unsigned int m_num;
  bool m_down;
  bool m_prev_down;
  std::chrono::high_resolution_clock::time_point m_down_time;
  std::chrono::high_resolution_clock::time_point m_up_time;
};

std::array<button, 32> g_buttons =
{{
  {  0 }, {  1 }, {  2 }, {  3 }, {  4 }, {  5 }, {  6 }, {  7 },
  {  8 }, {  9 }, { 10 }, { 11 }, { 12 }, { 13 }, { 14 }, { 15 },
  { 16 }, { 17 }, { 18 }, { 19 }, { 20 }, { 21 }, { 22 }, { 23 },
  { 24 }, { 25 }, { 26 }, { 27 }, { 28 }, { 29 }, { 30 }, { 31 }
}};

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

static struct mcx_axis_isr_counts
{
  volatile unsigned int mr_compare_match[4];
  volatile unsigned int drive_start;
  volatile unsigned int constant_speed_start;
  volatile unsigned int constant_speed_end;
  volatile unsigned int drive_stop;
  volatile unsigned int auto_homing_end;
  volatile unsigned int timer;
  volatile unsigned int split_pulse;
  volatile unsigned int split_pulse_end;

  mcx_axis_isr_counts (void) { std::memset (this, 0, sizeof (mcx_axis_isr_counts)); }
} g_mcx_axis_isr_counts[4];


template <typename Rep, typename Period>
auto to_freq (const std::chrono::duration<Rep, Period>& val)
{
  //return std::chrono::duration_cast< std::chrono::duration <float, std::ratio<1,1> > > (val).count ();
  return (float)Period::den / ((float)Period::num * val.count ());
}

int main (void)
{
  std::printf ("soft pg freq hz = %f %f   period = %u / %u\n",
	to_freq (soft_pg_t::base_timer::duration (1)),
	to_freq (soft_pg_t::duration (1)),
	(unsigned int)soft_pg_t::period::num,
	(unsigned int)soft_pg_t::period::den
  );

  g_soft_pg.start ();


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

  net::init (this_board::inst ().eth0,
	     use_ifcfg.static_addr, use_ifcfg.static_gateway_addr,
	     use_ifcfg.static_netmask);

  http_server_delegate httpsrv_delegate;
  net::http::server httpsrv (80, httpsrv_delegate);

  // MCX synchronous action test/demo
  #if 0
  {
    auto& x = this_board::inst ().mcx51x.axes ()[0];
    auto& y = this_board::inst ().mcx51x.axes ()[1];
    auto& z = this_board::inst ().mcx51x.axes ()[2];
    auto& u = this_board::inst ().mcx51x.axes ()[3];

    x.sync_action_1 ()
	.set_trigger (dev::mcx51x::mr_comp_match,
			dev::mcx51x::greater_equal,
			dev::mcx51x::logical_position_counter)
	.set_mr (50000)
	.set_action (dev::mcx51x::drive_decel_stop)
//	.set_action_link_sync_set (0)
	.set_repeat ()
	.enable ();

    x.sync_action_0 ()
	.set_trigger (dev::mcx51x::drive_stopped)
	.set_action (dev::mcx51x::load_mr_to_logical_position_counter)
	.set_mr (0)
	.set_repeat ()
	.set_action_link_other_axis_sync0_set (dev::mcx51x::axis_y)
	.enable ();

    y.sync_action_0 ()
	.set_trigger (dev::mcx51x::no_trigger)
	.set_action (dev::mcx51x::drive_pos_dir_pulse_continuous)
	.set_repeat ()
	.enable ();

    y.sync_action_1 ()
	.set_trigger (dev::mcx51x::drive_stopped)
	.set_action (dev::mcx51x::no_action)
	.set_action_link_other_axis_sync0_set (dev::mcx51x::axis_z)
	.set_repeat ()
	.enable ();

    z.sync_action_0 ()
	.set_trigger (dev::mcx51x::no_trigger)
	.set_action (dev::mcx51x::drive_pos_dir_pulse_continuous)
	.set_repeat ()
	.enable ();

    z.sync_action_1 ()
	.set_trigger (dev::mcx51x::drive_stopped)
	.set_action (dev::mcx51x::no_action)
	.set_action_link_other_axis_sync0_set (dev::mcx51x::axis_u)
	.set_repeat ()
	.enable ();

    u.sync_action_0 ()
	.set_trigger (dev::mcx51x::no_trigger)
	.set_action (dev::mcx51x::drive_pos_dir_pulse_continuous)
	.set_repeat ()
	.enable ();

    u.sync_action_1 ()
	.set_trigger (dev::mcx51x::drive_stopped)
	.set_action (dev::mcx51x::no_action)
	.set_repeat ()
	.set_trigger_func ([] ()
	{
	  // here we're in an ISR context.
	  // for more complex things, need to post a message to the main loop.
	  g_axes[0]->start ();
	})
	.enable ();

    x.set_mr_compare_match (0, dev::mcx51x::greater_equal,
			    dev::mcx51x::logical_position_counter);
    x.set_mr (0, 500);
  }

  for (auto& a : this_board::inst ().mcx51x.axes ())
  {
    const unsigned int num = a.num ();
    a.set_mr_compare_match_func (0, [=] { g_mcx_axis_isr_counts[num].mr_compare_match[0] += 1; });
    a.set_mr_compare_match_func (1, [=] { g_mcx_axis_isr_counts[num].mr_compare_match[1] += 1; });
    a.set_mr_compare_match_func (2, [=] { g_mcx_axis_isr_counts[num].mr_compare_match[2] += 1; });
    a.set_mr_compare_match_func (3, [=] { g_mcx_axis_isr_counts[num].mr_compare_match[3] += 1; });

    a.set_drive_start_func ([=] { g_mcx_axis_isr_counts[num].drive_start += 1; });
    a.set_constant_speed_start_func ([=] { g_mcx_axis_isr_counts[num].constant_speed_start += 1; });
    a.set_constant_speed_end_func ([=] { g_mcx_axis_isr_counts[num].constant_speed_end += 1; });
    a.set_drive_stop_func ([=] { g_mcx_axis_isr_counts[num].drive_stop += 1; });
    a.set_auto_homing_end_func ([=] { g_mcx_axis_isr_counts[num].auto_homing_end += 1; });
    a.set_timer_func ([=] { g_mcx_axis_isr_counts[num].timer += 1; });
    a.set_split_pulse_func ([=] { g_mcx_axis_isr_counts[num].split_pulse += 1; });
    a.set_split_pulse_end_func ([=] { g_mcx_axis_isr_counts[num].split_pulse_end += 1; });
  }

  #endif

//  this_board::inst ().led_outputs.write (0b11110000);
  this_board::inst ().led_outputs.write (0b11000011);

  auto prev_time = std::chrono::high_resolution_clock::now ();
  auto cur_time = prev_time;
  auto last_button_sync_time = prev_time;
  auto last_print_stat_time = prev_time;

  for (unsigned int main_loop_count = 0; ; ++main_loop_count)
  {
    prev_time = cur_time;
    cur_time = std::chrono::high_resolution_clock::now ();

    this_board::inst ().exec ();

    if (cur_time - last_print_stat_time >= std::chrono::milliseconds (1000))
    {
      last_print_stat_time = cur_time;

      #if 0
      for (const auto& i : g_mcx_axis_isr_counts)
      {
	printf (
	"\nmr_compare_match[0]   %u"
	"\nmr_compare_match[1]   %u"
	"\nmr_compare_match[2]   %u"
	"\nmr_compare_match[3]   %u"
	"\ndrive_start           %u"
	"\nconstant_speed_start  %u"
	"\nconstant_speed_end    %u"
	"\ndrive_stop            %u"
	"\nauto_homing_end       %u"
	"\ntimer                 %u"
	"\nsplit_pulse           %u"
	"\nsplit_pulse_end       %u"
	"\n",
	i.mr_compare_match[0], i.mr_compare_match[1], i.mr_compare_match[2], i.mr_compare_match[3],
	i.drive_start, i.constant_speed_start, i.constant_speed_end, i.drive_stop,
	i.auto_homing_end, i.timer, i.split_pulse, i.split_pulse_end);
      }

      printf ("\n-------------\n");
      #endif
    }

    {
      static std::array<uint8_t, 256> read_buffer;

      auto& debug_port = this_board::inst ().debug_usart;

      auto buffer_st = debug_port.buffer_stat ();

      if (buffer_st.rx_error)
      {
        debug_port.reset_receiver ();
        debug_port.reset_rx_buffer ();
      }

      if (!debug_port.rx_fifo_empty ())
      {
        unsigned int sz = debug_port.read (read_buffer.data (), read_buffer.size ());
        debug_port.write (read_buffer.data (), sz);
      }
    }

#if 0
    if (cur_time - last_button_sync_time >= std::chrono::milliseconds (10))
    {
      last_button_sync_time = cur_time;

      for (auto&& b : g_buttons)
	b.exec (cur_time);

      static_assert (std::tuple_size<decltype (g_buttons)>::value >= 4 * std::tuple_size<decltype (g_axes)>::value, "");

      for (unsigned int a = 0; a < g_axes.size (); ++a)
      {
	const auto& speed_down_button = g_buttons[a*4 + 0];
	const auto& speed_up_button = g_buttons[a*4 + 1];
	const auto& start_stop_button = g_buttons[a*4 + 2];
	const auto& dir_button = g_buttons[a*4 + 3];

	if (start_stop_button.is_down () && start_stop_button.was_up ())
	{
	  if (g_axes[a]->current_speed () > 0)
	    g_axes[a]->stop ();
	  else
	    g_axes[a]->start ();
	}


	if (dir_button.is_down () && dir_button.was_up ())
	{
	  g_axes[a]->set_direction (g_axes[a]->direction () == motor_axis::fwd
				    ? motor_axis::rev
				    : motor_axis::fwd);
	}

	auto cur_speed_val = g_axes[a]->speed ();

	if (speed_up_button.is_down () && speed_down_button.is_up ())
	{
	  unsigned int val_mod = 1;
	  auto button_hold_time = cur_time - speed_up_button.down_time ();
	  if (button_hold_time > std::chrono::seconds (1))
	    val_mod = 10;
	  if (button_hold_time > std::chrono::seconds (2))
	    val_mod = 100;
	  if (button_hold_time > std::chrono::seconds (3))
	    val_mod = 1000;

	  g_axes[a]->set_speed (std::min (cur_speed_val + val_mod, g_axes[a]->max_speed ()));
	}
	if (speed_down_button.is_down () && speed_up_button.is_up ())
	{
	  unsigned int val_mod = 1;
	  auto button_hold_time = cur_time - speed_down_button.down_time ();
	  if (button_hold_time > std::chrono::seconds (1))
	    val_mod = 10;
	  if (button_hold_time > std::chrono::seconds (2))
	    val_mod = 100;
	  if (button_hold_time > std::chrono::seconds (3))
	    val_mod = 1000;

	  g_axes[a]->set_speed (cur_speed_val > val_mod ? cur_speed_val - val_mod : 1);
	}
      }
    }
#endif

    net::exec ();
    httpsrv.exec ();
  }

  return 0;
}

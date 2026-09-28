
/*

a simple motor controller that uses the software pulse generator to
output pulses for step and direction.

*/

#ifndef includeguard_dev_soft_pg_motor_includeguard
#define includeguard_dev_soft_pg_motor_includeguard

#include <chrono>
#include <functional>
#include <limits>
#include <type_traits>
#include <utils/value_range.hpp>
#include <dev/pulse_type.hpp>

namespace dev
{
namespace soft_pg_motor
{

enum direction_t
{
  fwd = 0,
  rev = 1
};

enum struct drive_mode
{
  continuous,
  feed_count
};

struct sensor_status_t
{
  bool pos_limit;
  bool neg_limit;
  bool origin;
};

template < typename CtrlDuration,  //template <std::intmax_t Num, std::intmax_t Denom> typename CtrlPeriod,
	   typename PulseOut_SoftPGChannelFunc,
	   typename DirOut_SoftPGChannelFunc,
	   typename PosLimitInputFunc = void,
	   typename NegLimitInputFunc = void,
	   typename OriginInputFunc = void >
class dev_inst
{
public:
  static constexpr auto& pulse_out_channel_inst (void) { return std::invoke (PulseOut_SoftPGChannelFunc ()); }
  using pulse_out_channel_t = std::remove_reference_t<std::invoke_result_t<PulseOut_SoftPGChannelFunc>>;
  using pulse_out_channel_dev_t = typename pulse_out_channel_t::soft_pg_dev_t;

  static constexpr auto& dir_out_channel_inst (void) { return std::invoke (DirOut_SoftPGChannelFunc ()); }
  using dir_out_channel_t = std::remove_reference_t<std::invoke_result_t<DirOut_SoftPGChannelFunc>>;
  using dir_out_channel_dev_t = typename dir_out_channel_t::soft_pg_dev_t;

  using duration = CtrlDuration;
  using rep = typename duration::rep;
  using period = typename duration::period;

  static constexpr bool pulse_output_type_supported (pulse_type pt)
  {
    switch (pt)
    {
      default:
        return false;

      case pulse_type::disable:
      case pulse_type::single_pulse:
      case pulse_type::single_pulse_direction:
      case pulse_type::double_pulse:
        return true;
    }
  }

  dev_inst (void)
  {
    read_sensor_inputs ();

    auto& p0 = pulse_out_channel_inst ();
    auto& p1 = dir_out_channel_inst ();

    m_drive_mode = drive_mode::continuous;
    m_stop_at_origin = false;
    p0.set_mode (pulse_out_channel_dev_t::free_run_output_toggle);
    p1.set_mode (dir_out_channel_dev_t::free_run_output_toggle);

    m_dir = fwd;

    set_speed_pps (1);

    p0.set_output (false);
    p1.set_output (false);

    set_pulse_output_type (pulse_type::single_pulse_direction);
  }

  void set_pulse_output_type (pulse_type pt)
  {
    auto& p0 = pulse_out_channel_inst ();
    auto& p1 = dir_out_channel_inst ();

    bool en = is_driving ();

    switch (pt)
    {
      default:
        return;

      case pulse_type::disable:
	p0.disable ();
	p1.disable ();
	break;

      case pulse_type::single_pulse:
	p0.enable (en);
	p1.disable ();
        break;

      case pulse_type::single_pulse_direction:
	p0.enable (en);
	p1.disable ();
	p1.set_output (m_dir == rev);
        break;

      case pulse_type::double_pulse:
	p0.enable ((m_dir == fwd) & en);
	p1.enable ((m_dir == rev) & en);
        break;
    }

    m_pulse_output_type = pt;
  }

  pulse_type pulse_output_type (void) const { return m_pulse_output_type; }

  using feed_counter_type = typename pulse_out_channel_t::edge_counter_type;
  
  static constexpr feed_counter_type feed_max_value = std::numeric_limits<feed_counter_type>::max () / 2;
  static constexpr feed_counter_type feed_min_value = 1;
 
  feed_counter_type feed_count (void) const
  {
    if (m_pulse_output_type == pulse_type::double_pulse)
    {
      if (m_dir == fwd)
	return pulse_out_channel_inst ().edge_counter () / 2;
      else
	return dir_out_channel_inst ().edge_counter () / 2;
    }
    else
      return pulse_out_channel_inst ().edge_counter () / 2;
  }

  void set_feed_count (utils::clamped_value<feed_counter_type, feed_min_value, feed_max_value> val)
  {
    pulse_out_channel_inst ().set_edge_counter (val * 2);
    dir_out_channel_inst ().set_edge_counter (val * 2);
  }

  static constexpr unsigned int speed_min_value_pps = pulse_out_channel_t::pps_min_value;
  static constexpr unsigned int speed_max_value_pps = pulse_out_channel_t::pps_max_value;

  unsigned int speed_pps (void) const
  {
    return m_set_speed_pps;
  }
  unsigned int current_speed_pps (void) const
  {
    return is_driving () ? m_cur_speed_pps : 0;
  }
  void set_speed_pps (utils::clamped_value<unsigned int, speed_min_value_pps, speed_max_value_pps> val)
  {
    // always set the speed to both pulse outputs for double pulse output mode.

    auto& p0 = pulse_out_channel_inst ();
    auto& p1 = dir_out_channel_inst ();

    // FIXME: this should gradually change the speed based on accel/decel setting.

    m_set_speed_pps = val;
    m_cur_speed_pps = p0.set_pulse_shape_pps (val);
    p1.set_pulse_shape_pps (val);

    if (!is_driving ())
    {
      // it could be not driving because it has stopped due to feed counter.
      // if we reset it, it will re-start again, which we don't want here.
      p0.disable ();
      p1.disable ();

      p0.reset ();
      p1.reset ();
    }
  }

  direction_t direction (void) const { return m_dir; }
  void set_direction (direction_t val)
  {
    if (m_dir == val)
      return;

    m_dir = val;

    auto& p0 = pulse_out_channel_inst ();
    auto& p1 = dir_out_channel_inst ();

    switch (m_pulse_output_type)
    {
      default:
      case pulse_type::disable:
      case pulse_type::single_pulse:
        break;

      case pulse_type::single_pulse_direction:
	p1.set_output (val == rev);
        break;

      case pulse_type::double_pulse:
	if (m_drive_mode == drive_mode::feed_count)
	{
	  // synchronize feed counter
	  if (val == fwd)
	    p0.set_edge_counter (p1.edge_counter ());
	  else
	    p1.set_edge_counter (p0.edge_counter ());
	}

	if (is_driving ())
	{
	  p0.enable (val == fwd);
	  p1.enable (val == rev);
	}
        break;
     }
  }

  bool is_driving (void) const
  {
    return pulse_out_channel_inst ().is_enabled ()
	   | dir_out_channel_inst ().is_enabled ();
  }

  void start (void)
  {
    auto& p0 = pulse_out_channel_inst ();
    auto& p1 = dir_out_channel_inst ();

    int8_t s = read_sensor_inputs ();

    // refuse to start if one of the limit sensors is on
    if (is_stop_condition (s))
      return;

    p0.reset ();
    p1.reset ();

    switch (m_pulse_output_type)
    {
      default:
      case pulse_type::disable:
        break;

      case pulse_type::single_pulse:
      case pulse_type::single_pulse_direction:
	p0.enable ();
        break;

      case pulse_type::double_pulse:
	if (m_drive_mode == drive_mode::feed_count)
	{
	  // synchronize feed counter
	  if (m_dir == fwd)
	    p0.set_edge_counter (p1.edge_counter ());
	  else
	    p1.set_edge_counter (p0.edge_counter ());
	}

	p0.enable (m_dir == fwd);
	p1.enable (m_dir == rev);
        break;
     }
  }

  void stop (void)
  {
    // FIXME: implement speed ramp, set speed to 0 and let it run
    stop_immediately ();
  }

  void stop_immediately (void)
  {
    auto& p0 = pulse_out_channel_inst ();
    auto& p1 = dir_out_channel_inst ();

    p0.disable ();
    p1.disable ();

    switch (m_pulse_output_type)
    {
      default:
      case pulse_type::disable:
        break;

      case pulse_type::single_pulse:
      case pulse_type::single_pulse_direction:
	p0.set_output (false);
        break;

      case pulse_type::double_pulse:
	p0.set_output (false);
	p1.set_output (false);
        break;
     }
  }

  enum drive_mode drive_mode (void) const { return m_drive_mode; }

  void set_drive_mode (enum drive_mode val)
  {
    auto& p0 = pulse_out_channel_inst ();
    auto& p1 = dir_out_channel_inst ();

    m_drive_mode = val;
    if (val == drive_mode::continuous)
    {
      p0.set_mode (pulse_out_channel_dev_t::free_run_output_toggle);
      p1.set_mode (dir_out_channel_dev_t::free_run_output_toggle);
    }
    else if (val == drive_mode::feed_count)
    {
      p0.set_mode (pulse_out_channel_dev_t::edge_count_output_toggle);
      p1.set_mode (dir_out_channel_dev_t::edge_count_output_toggle);
    }
  }

  // if set, driving will stop when the origin sensor is on.
  bool stop_at_origin (void) const { return m_stop_at_origin; }
  void set_stop_at_origin (bool val) { m_stop_at_origin = val; }

  sensor_status_t sensor_status (void) const
  {
    // return the last sampled value, that is what this software device sees
    // and uses.
    auto val = m_sensor_inputs;
    return
    {
      utils::get_bit (val, pos_limit_bit_i),
      utils::get_bit (val, neg_limit_bit_i),
      utils::get_bit (val, origin_bit_i)
    };
  }

  void exec (/*rep tick_count*/)
  {
    // FIXME: implement speed ramps

    if (is_driving () && is_stop_condition (read_sensor_inputs ()))
      stop_immediately ();
  }

private:
  bool is_stop_condition (int8_t sensors) const
  {
    return (utils::get_bit (sensors, pos_limit_bit_i) && m_dir == fwd)
	    || (utils::get_bit (sensors, neg_limit_bit_i) && m_dir == rev)
	    || (m_stop_at_origin && utils::get_bit (sensors, origin_bit_i));
  }

  static constexpr bool read_pos_limit_input (void)
  {
    if constexpr (!std::is_void_v <PosLimitInputFunc>)
      return PosLimitInputFunc ()();
    else
      return false;
  }
  static constexpr bool read_neg_limit_input (void)
  {
    if constexpr (!std::is_void_v <NegLimitInputFunc>)
      return NegLimitInputFunc ()();
    else
      return false;
  }
  static constexpr bool read_org_input (void)
  {
    if constexpr (!std::is_void_v <OriginInputFunc>)
      return OriginInputFunc ()();
    else
      return false;
  }

  static constexpr unsigned int pos_limit_bit_i = 2;
  static constexpr unsigned int neg_limit_bit_i = 1;
  static constexpr unsigned int origin_bit_i = 0;

  int8_t read_sensor_inputs (void)
  {
    int8_t val =   (read_pos_limit_input () << pos_limit_bit_i)
		 | (read_neg_limit_input () << neg_limit_bit_i)
		 | (read_org_input () << origin_bit_i);

    m_sensor_inputs = val;
    return val;
  }

  pulse_type m_pulse_output_type;
  direction_t m_dir;
  bool m_stop_at_origin;
  enum drive_mode m_drive_mode;

  // the speed values will be modifed by the exec function, which might be
  // running in an ISR context.
  volatile unsigned int m_set_speed_pps;
  volatile unsigned int m_cur_speed_pps;

  volatile int8_t m_sensor_inputs;
};


} // namespace soft_pg_motor
} // namespace dev
#endif // includeguard_dev_soft_pg_motor_includeguard



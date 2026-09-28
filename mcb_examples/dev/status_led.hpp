/*

a device that is normally used to control a status LED by displaying
various on/off patterns.

actually it's not limited to an LED, the output function can be anything
as specified by the user.

FIXME: make it possible to use this on top of a hardware timer, like RX TPU
or MTU and DTC.  maybe provide some default implementations for the
SetStatusFunc type and then use some specializations to automagically use
hardware acceleration if available.

*/

#ifndef includeguard_dev_status_led_hpp_includeguard
#define includeguard_dev_status_led_hpp_includeguard

#include <cstdint>
#include <chrono>
#include <vector>

namespace dev
{

template <typename OutputPortFn, typename Container = std::vector<bool>>
class status_led : private OutputPortFn
{
public:
  status_led (const OutputPortFn& f = { }, unsigned int reserve_size = 32)
  : OutputPortFn (f)
  {
    // FIXME MCB-172: could do without a free-store allocation by having a small
    // bit array stored by value in this class and using a custom allocator
    // for vector<bool>
    m_bit_pattern.reserve (reserve_size);
    reset ();
  }

  status_led (OutputPortFn&& f, unsigned int reserve_size = 32)
  : OutputPortFn (std::move (f))
  {
    // FIXME MCB-172: could do without a free-store allocation by having a small
    // bit array stored by value in this class and using a custom allocator
    // for vector<bool>
    m_bit_pattern.reserve (reserve_size);
    reset ();
  }

  void exec (void)
  {
    exec (std::chrono::high_resolution_clock::now ());
  }

  void exec (const std::chrono::high_resolution_clock::time_point& now)
  {
    if (m_bit_pattern.empty ())
    {
      m_cur_bit_pos = 0;
      m_last_time = now;
      return;
    }

    if (m_bit_pattern.size () == 1)
    {
      if (m_cur_output_state != m_bit_pattern.front ())
      {
	m_cur_output_state = m_bit_pattern.front ();
	write_port (m_cur_output_state);
      }
      return;
    }

    bool s = m_cur_output_state;

    for (auto delta_time = now - m_last_time; delta_time >= m_bit_freq;
	 delta_time -= m_bit_freq)
    {
      s = m_bit_pattern[m_cur_bit_pos++];

      if (m_cur_bit_pos >= m_bit_pattern.size ())
	m_cur_bit_pos = 0;

      delta_time -= m_bit_freq;
      m_last_time += m_bit_freq;
    }

    if (s != m_cur_output_state)
    {
      write_port (s);
      m_cur_output_state = s;
    }
  }

  void reset (void)
  {
    m_last_time = std::chrono::high_resolution_clock::now ();
    m_cur_bit_pos = 0;
    m_cur_output_state = m_bit_pattern.empty () ? false : (bool)m_bit_pattern[m_cur_bit_pos];
    write_port (m_cur_output_state);
  }

  template <typename Allocator, typename Rep, typename Period> 
  void set_pattern (const std::vector<bool, Allocator>& bit_pattern,
		    const std::chrono::duration<Rep, Period>& freq)
  {
    m_bit_pattern = bit_pattern;
    m_bit_freq = { freq };

    check_fix_cur_bitpos ();
  }

  template <typename Allocator, typename Rep, typename Period> 
  void set_pattern (std::vector<bool, Allocator>&& bit_pattern,
		    const std::chrono::duration<Rep, Period>& freq)
  {
    m_bit_pattern = std::move (bit_pattern);
    m_bit_freq = { freq };

    check_fix_cur_bitpos ();
  }

  template <typename Allocator>
  void set_pattern (const std::vector<bool, Allocator>& bit_pattern)
  {
    m_bit_pattern = bit_pattern;
    check_fix_cur_bitpos ();
  }

  template <typename Allocator>
  void set_pattern (std::vector<bool, Allocator>&& bit_pattern)
  {
    m_bit_pattern = std::move (bit_pattern);
    check_fix_cur_bitpos ();
  }

  template <typename Rep, typename Period>
  void set_pattern (const std::initializer_list<bool>& bit_pattern,
		    const std::chrono::duration<Rep, Period>& freq)
  {
    m_bit_pattern = { bit_pattern };
    m_bit_freq = { freq };
    check_fix_cur_bitpos ();
  }

  template <typename Rep, typename Period>
  void set_pattern (std::initializer_list<bool>&& bit_pattern,
		    const std::chrono::duration<Rep, Period>& freq)
  {
    m_bit_pattern = { std::move (bit_pattern) };
    m_bit_freq = { freq };
    check_fix_cur_bitpos ();
  }

  void set_pattern (const std::initializer_list<bool>& bit_pattern)
  {
    m_bit_pattern = { bit_pattern };
    check_fix_cur_bitpos ();
  }

  void set_pattern (std::initializer_list<bool>&& bit_pattern)
  {
    m_bit_pattern = { std::move (bit_pattern) };
    check_fix_cur_bitpos ();
  }

  template <typename A, typename B>
  void set_pattern (const std::pair<A, B>& p)
  {
    set_pattern (p.first, p.second);
  }

  template <typename A, typename B>
  void set_pattern (std::pair<A, B>&& p)
  {
    set_pattern (std::move (p.first), std::move (p.second));
  }

private:
  bool m_cur_output_state;
  unsigned int m_cur_bit_pos;
  Container m_bit_pattern;
  std::chrono::high_resolution_clock::duration m_bit_freq = std::chrono::seconds (1);
  std::chrono::high_resolution_clock::time_point m_last_time;

  void check_fix_cur_bitpos (void)
  {
    if (m_cur_bit_pos >= m_bit_pattern.size ())
      m_cur_bit_pos = 0;
  }

  void write_port (bool val) { (*(OutputPortFn*)this)().write (val); }
};

template <typename PortFn>
inline status_led<PortFn> make_status_led (PortFn f)
{
  return status_led<PortFn> (f);
}

template <typename Container, typename PortFn>
inline status_led<PortFn, Container> make_status_led (PortFn f)
{
  return status_led<PortFn, Container> (f);
}


} // namespace dev
#endif // includeguard_dev_status_led_hpp_includeguard

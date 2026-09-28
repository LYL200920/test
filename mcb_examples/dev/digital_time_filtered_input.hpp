/*

*/

#ifndef includeguard_dev_digital_time_filtered_input_includeguard
#define includeguard_dev_digital_time_filtered_input_includeguard

#include <chrono>
#include <dev/digital_io_port.hpp>

namespace dev
{


class digital_time_filtered_input : public digital_io_port
{
public:
  digital_time_filtered_input (digital_io_port p)
  : digital_io_port (&m_dev_if_impl, 0, positive_logic)
  {
    m_input = p;
    m_last_state = p.read ();
    m_last_time = std::chrono::high_resolution_clock::now ();
    m_dev_if_impl.m_state = m_last_state;
    m_triggered = true;
  }

  digital_time_filtered_input (const digital_time_filtered_input&) = delete;
  digital_time_filtered_input& operator = (const digital_time_filtered_input&) = delete;

  std::chrono::high_resolution_clock::duration on_filter_time (void) const { return m_on_filter_time; }
  void set_on_filter_time (std::chrono::high_resolution_clock::duration val) { m_on_filter_time = val; }

  std::chrono::high_resolution_clock::duration off_filter_time (void) const { return m_off_filter_time; }
  void set_off_filter_time (std::chrono::high_resolution_clock::duration val) { m_off_filter_time = val; }

  void set_on_off_filter_time (std::chrono::high_resolution_clock::duration val)
  {
    m_on_filter_time = val;
    m_off_filter_time = val;
  }

  void exec (std::chrono::high_resolution_clock::time_point now)
  {
    const bool cur_state = m_input.read ();
    if (cur_state != m_last_state)
    {
      m_last_time = now;
      m_last_state = cur_state;
      m_triggered = true;
    }
    else if (m_triggered)
    {
      if (m_last_state == false
	  && now - m_last_time >= m_off_filter_time)
      {
	m_dev_if_impl.m_state = false;
	m_triggered = false;
      }

      else if (m_last_state == true
	       && now - m_last_time >= m_on_filter_time)
      {
	m_dev_if_impl.m_state = true;
	m_triggered = false;
      }
    }
  }


private:
  struct dev_if_impl final : public digital_io_port::dev_if
  {
    bool m_state = false;

    virtual bool read_port ([[gnu::unused]] unsigned int n) const { return m_state; }
  } m_dev_if_impl;

  digital_io_port m_input;
  bool m_triggered;
  bool m_last_state;
  std::chrono::high_resolution_clock::time_point m_last_time;
  std::chrono::high_resolution_clock::duration m_on_filter_time = std::chrono::milliseconds (1000);
  std::chrono::high_resolution_clock::duration m_off_filter_time = std::chrono::milliseconds (1000);
};


} // namespace dev
#endif // includeguard_dev_digital_time_filtered_input_includeguard

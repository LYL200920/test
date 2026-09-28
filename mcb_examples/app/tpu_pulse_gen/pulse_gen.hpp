#ifndef includeguard_pulse_gen_hpp_includeguard
#define includeguard_pulse_gen_hpp_includeguard

#include <dev/digital_io_port.hpp>
#include <vector>

class pulse_gen
{
public:
  virtual ~pulse_gen (void) { }

  // start/continue, stop/pause and reset control of the pulse generator.
  // reset will ensure that it will restart the bitpattern sequence and pulse
  // phase.
  virtual void start (void) = 0;
  virtual void stop (void) = 0;
  virtual void reset (void) = 0;

  virtual bool running (void) const = 0;

  const auto& frequency (void) const { return m_frequency_hz; }
  virtual void set_frequency (float val) { m_frequency_hz = val; }

  const auto& duty_cycle (void) const { return m_duty_cycle_percent; }
  virtual void set_duty_cycle (float val) { m_duty_cycle_percent = val; }

  const auto& phase_offset (void) const { return m_phase_offset_percent; }
  virtual void set_phase_offset (float val) { m_phase_offset_percent = val; }

  const auto& bit_pattern (void) const { return m_bit_pattern; }
  virtual void set_bit_pattern (std::vector<bool> val) { m_bit_pattern = std::move (val); }

  const auto& output_port (void) const { return m_output_port; }
  virtual void set_output_port (dev::digital_io_port val) { m_output_port = std::move (val); }


protected:
  virtual void on_bit_pattern_changed (void) { }
  virtual void on_output_port_changed (void) { }

  float m_frequency_hz = 1;
  float m_duty_cycle_percent = 50;
  float m_phase_offset_percent = 0;
  std::vector<bool> m_bit_pattern;
  dev::digital_io_port m_output_port;
};

#endif // includeguard_pulse_gen_hpp_includeguard

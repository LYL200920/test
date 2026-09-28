/*

NLD board specific LED output.

white LED uses TIM14 CH1
red LED uses TIM17 CH1

timer is used in PWM 1 mode.  each LED output is instantiated individually
and the brightness is adjusted via the PWM duty cycle.

*/

#ifndef includeguard_board_nld_dev_led_outputs_hpp_includeguard
#define includeguard_board_nld_dev_led_outputs_hpp_includeguard

#include <dev/digital_io_port.hpp>
#include <dev/stm32f0/stm32f0_tim.hpp>
#include <utils/value_range.hpp>

namespace dev
{

template <typename TimerInstFunc, unsigned int MaxValue, unsigned int PeriodValue>
class led_output
{
public:
  led_output (void)
  {
    // this works for TIM14 and TIM17 CH1
    auto& regs = TimerInstFunc () ().regs ();

    regs.psc = 0;
    regs.arr = PeriodValue;
    regs.ccr[0] = 0;

    regs.cr2 = 0;

    // CC1CE = 0
    // OC1M = 110 (output compare mode = PWM mode 1)
    // OC1PE = 0 (preload enable)
    // OC1FE = 0 (fast disable)
    // CC1S = 00 (channel configured as output)
    regs.ccmr1 = 0b0'110'1'0'00;

    // CC1NP = 0 (active high)
    // CC1P  = 0 (active high)
    // CC1E  = 1 (output enable)
    regs.ccer = 0b0001;

    // BDTR MOE = 1 (OC and OCN outputs are enabled)
    regs.bdtr = 1 << 15;

    regs.egr = 1;
    regs.cr1 = 1;
  }

  static constexpr unsigned int max_value (void) { return MaxValue; }

  void set_value (utils::clamped_value<unsigned int, 0, MaxValue> val)
  {
    m_cur_value = (m_cur_value & 1) | (val << 1);
    if (m_cur_value & 1)
      TimerInstFunc () ().regs ().ccr[0] = m_cur_value >> 1;
  }

  // implement concept of digital_io_port::read, ::write, ::set_trigger_func
  // if a real instance of digital_io_port is required, need to create
  // a wrapper object around it.
  operator digital_io_port (void) const;

  bool read (void) const
  {
    return m_cur_value & 1;
  }

  void write (bool val)
  {
    // set compare match output to pwm mode 1 or force inactive.
    // TimerInstFunc ().regs ().ccmr1 = val ? 0b0'110'1'0'00 : 0b0'100'1'0'00;

    m_cur_value = (m_cur_value & ~1u) | val;
    TimerInstFunc () ().regs ().ccr[0] = val ? (m_cur_value >> 1) : 0;
  }

  template <typename F>
  void set_trigger_func (digital_io_port::trigger_type, F&&) { }

private:
  // use the LSB as the enable flag
  static_assert (MaxValue < (1 << 15));
  uint32_t m_cur_value = MaxValue << 1;
};

} // namespace dev

#endif // includeguard_board_nld_dev_led_outputs_hpp_includeguard

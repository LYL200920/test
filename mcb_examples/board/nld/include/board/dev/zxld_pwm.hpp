/*

MCU -> ZXLD PWM output

uses TIM1 CH1 to output either a constant off, constant on or ready-pulse
to the ZXLD.  the output can be cut off by the comparator as a over-voltage
protection (break event/function: output low).

the ready pulse is used to keep the ZXLD out of standby mode and to allow
quick response to LED-ON signals.  if the ZXLD goes into standby it takes a
long time (100 - 200 µsec) for it to wake up and resume operation.

unfortunately, the threshold when the ZXLD will enter standby mode depends
on each part and the device temperature.  this makes it difficult to find
the appropriate timing to avoid the standby mode and without actually turning
on the LED.  if the ready pulse is too frequent, the LED will turn.

when it goes into standby mode, the analog STATUS output will drop to 0V.
when it comes out of standby the STATUS output will be around 1.7V
(3.6V on ZXLD output), which indicates the "driver stalled" state.
during that time, the FLAG output will be also pulled low.

when PWM is low and pulled high, the ZXLD will respond to that by immediately
outputting a high on the GATE to turn on the power FET and start switching.
to avoid that during the ready pulse, additional external gating hardware
for the GATE output is used which is controlled by another timer channel.
usually both timer channels will output a synchronized ready pulse, although
the gate-off channel could also simply output a constant high during ready mode.
*/

#ifndef includeguard_board_nld_dev_zxld_pwm_hpp_includeguard
#define includeguard_board_nld_dev_zxld_pwm_hpp_includeguard

#include <dev/stm32f0/stm32f0_tim.hpp>
#include <utils/value_range.hpp>
#include <algorithm>
#include <type_traits>

namespace dev
{

template <typename PWM_TimerInstFunc,
	  typename GATE_OFF_TimerInstFunc>
class zxld_pwm_A02
{
  using pwm_timer = typename std::remove_reference<decltype (PWM_TimerInstFunc ()())>::type;
  using gate_off_timer = typename std::remove_reference<decltype (GATE_OFF_TimerInstFunc ()())>::type;

  static_assert (pwm_timer::tick_frequency == gate_off_timer::tick_frequency, "");

  static constexpr unsigned int tick_frequency = pwm_timer::tick_frequency / 8;
  static constexpr unsigned int psc_div = (pwm_timer::tick_frequency / tick_frequency) - 1;

  typedef std::chrono::duration<int32_t, std::ratio<1, tick_frequency>> duration;
  typedef typename duration::rep rep;
  typedef typename duration::period period;

public:
  enum status_t
  {
    off,
    ready,
    on,
    break_cond,
  };

  zxld_pwm_A02 (void)
  {
    auto&& regs = PWM_TimerInstFunc ()().regs ();
    auto&& regs2 = GATE_OFF_TimerInstFunc ()().regs ();
    regs.psc = psc_div;
    regs2.psc = psc_div;

    // use shortest possible pulse that is seen by the ZXLD.
    constexpr auto pulse_duration = std::chrono::microseconds (3);

    // according to the datasheet, 10 ms is the minimum time when it might
    // start entering standby mode.  so set the pulse period to something
    // shorter than that.
    // if the ZXLD GATE stays at the same level (hgih or low) for > 100 usec
    // it will go into fault state 2 (driver stalled).  it can be avoided
    // by outputting the ready pulse at a < 100 usec interval.  however, the
    // fault state doesn't seem to have an impact on the response time.
    constexpr auto period = std::chrono::microseconds (5000);

    constexpr auto arr_val = std::chrono::duration_cast<duration> (period).count ();
    constexpr auto ccr_val = std::chrono::duration_cast<duration> (period - pulse_duration).count ();

    static_assert (arr_val > 0 && arr_val < 65535, "");
    static_assert (ccr_val > 0 && ccr_val < 65535, "");

    regs.arr = arr_val;
    regs.ccr[0] = ccr_val;

    regs2.arr = arr_val;
    regs2.ccr[0] = ccr_val;

    // start with OC1REF forced low
    regs.cr2 = 0;
    regs.ccmr1 = 0b0'100'1'0'00;
    regs.ccer = 0b0001 << 0;

    regs2.cr2 = 0;
    regs2.ccmr1 = 0b0'100'1'0'00;
    regs2.ccer = 0b0001 << 0;

    reset_break_condition ();

 // smcr.ts = 010 (TRGI = TIM3 selection)
 // smcr.sms = 100 (reset mode, re-init & start on TRGI rising edge)

    // UG = 1
    regs2.egr = 1;
    regs2.cr1 = 1;

    regs.egr = 1;
    regs.cr1 = 1;

    m_status = off;
  }

  status_t status (void) const
  {
    return is_break_condition () ? break_cond : m_status;
  }

  void set_ready (void)
  {
    // pwm mode 2
    auto& regs = PWM_TimerInstFunc ()().regs ();
    auto&& regs2 = GATE_OFF_TimerInstFunc ()().regs ();

    // turn output off
    regs.ccmr1 = 0b0'100'1'0'00;
//    regs2.ccmr1 = 0b0'100'1'0'00;

    // restart ready timer if it was on before
    regs2.egr = 1;
    regs.egr = 1;

    // enable timer pulse output
    regs2.ccmr1 = 0b0'111'1'0'00;
    regs.ccmr1 = 0b0'111'1'0'00;

    m_status = ready;
  }

  void set_off (void)
  {
    // force OC1REF low
    PWM_TimerInstFunc ()().regs ().ccmr1 = 0b0'100'1'0'00;
    GATE_OFF_TimerInstFunc ()().regs ().ccmr1 = 0b0'100'1'0'00;

    m_status = off;
  }

  void set_on (void)
  {
    // force OC1REF high
    PWM_TimerInstFunc ()().regs ().ccmr1 = 0b0'101'1'0'00;

    // force OC1REF low
    GATE_OFF_TimerInstFunc ()().regs ().ccmr1 = 0b0'100'1'0'00;

    m_status = on;
  }

  void reset_break_condition (void)
  {
    set_off ();
    PWM_TimerInstFunc ()().regs ().sr &= ~(1 << 7);
    PWM_TimerInstFunc ()().regs ()
   	.bdtr = 0
	| (1 << 15)	// MOE = 1, main output enable. cleared in hardware on break
	| (1 << 13)	// BKP = 1, break input BRK is active high
	| (1 << 12)	// BKE = 1, break enable
	| (1 << 11)	// OSSR = 1, run mode off-state: output inactive level
	| (1 << 10)	// OSSI = 1, idle mode off-state: output inactive level
	| 0;

    GATE_OFF_TimerInstFunc ()().regs ()
   	.bdtr = 0
	| (1 << 15)	// MOE = 1, main output enable. cleared in hardware on break
	| (1 << 13)	// BKP = 1, break input BRK is active high
	| (1 << 12)	// BKE = 1, break enable
	| (1 << 11)	// OSSR = 1, run mode off-state: output inactive level
	| (1 << 10)	// OSSI = 1, idle mode off-state: output inactive level
	| 0;
  }

  bool is_break_condition (void) const
  {
    return (PWM_TimerInstFunc ()().regs ().sr & (1 << 7)) != 0;
  }

private:
  status_t m_status;
};

// ------------------------------------------------------------------------------
// A-03 board version has a LEDK_ENABLE signal instead of a GATE_OFF.
// the logic is inverted.

// as it turned out during further field tests, if LEDK is opened (Q3) it will
// prevent the LED from lighting up in the ready-pulse state, but it has a problem
// of large inrush currents on the LED when LEDK Q3 is closed.
// the inrush current can be avoided with a ramp-up but slows down response time.
// thus keep LEDK on all the time.

template <typename PWM_TimerInstFunc,
	  typename LEDK_ENABLE_TimerInstFunc>
class zxld_pwm_A03
{
  using pwm_timer = typename std::remove_reference<decltype (PWM_TimerInstFunc ()())>::type;
  using ledk_enable_timer = typename std::remove_reference<decltype (LEDK_ENABLE_TimerInstFunc ()())>::type;

  static_assert (pwm_timer::tick_frequency == ledk_enable_timer::tick_frequency, "");

  static constexpr unsigned int tick_frequency = pwm_timer::tick_frequency / 8;
  static constexpr unsigned int psc_div = (pwm_timer::tick_frequency / tick_frequency) - 1;

  static_assert (tick_frequency == 48'000'000/8);

  typedef std::chrono::duration<int32_t, std::ratio<1, tick_frequency>> duration;
  typedef typename duration::rep rep;
  typedef typename duration::period period;

  static inline void ledk_on (void)  { LEDK_ENABLE_TimerInstFunc ()().regs ().ccmr1 = 0b0'101'1'0'00; }
  static inline void ledk_off (void) { LEDK_ENABLE_TimerInstFunc ()().regs ().ccmr1 = 0b0'100'1'0'00; }

  // permanently on (for test)
//  static inline void ledk_off (void)  { LEDK_ENABLE_TimerInstFunc ()().regs ().ccmr1 = 0b0'101'1'0'00; }

  // OC1M = 110: PWM mode 1
  //      up count:   TIMx_CNT  < TIMx_CCR1 -> active
  //                  TIMx_CNT >= TIMx_CCR1 -> inactive
  //
  //      down count: TIMx_CNT <= TIMx_CCR1 -> active
  //                  TIMx_CNT  > TIMx_CCR1 -> inactive


  // OC1M = 111: PWM mode 2
  //      up count:   TIMx_CNT  < TIMx_CCR1 -> inactive
  //                  TIMx_CNT >= TIMx_CCR1 -> active
  //
  //      down count: TIMx_CNT <= TIMx_CCR1 -> inactive
  //                  TIMx_CNT  > TIMx_CCR1 -> active

  static inline void ledk_pulse (void) { LEDK_ENABLE_TimerInstFunc ()().regs ().ccmr1 = 0b0'111'1'0'00; }

  static inline void pwm_on (void)    { PWM_TimerInstFunc ()().regs ().ccmr1 = 0b0'101'1'0'00; }
  static inline void pwm_off (void)   { PWM_TimerInstFunc ()().regs ().ccmr1 = 0b0'100'1'0'00; }
  static inline void pwm_pulse (void) { PWM_TimerInstFunc ()().regs ().ccmr1 = 0b0'111'1'0'00; }


public:
  enum status_t
  {
    off,
    ready,
    on,
    break_cond,
  };

  zxld_pwm_A03 (void)
  {
    auto&& regs = PWM_TimerInstFunc ()().regs ();
    auto&& regs2 = LEDK_ENABLE_TimerInstFunc ()().regs ();
    regs.psc = psc_div;
    regs2.psc = psc_div;

    // start with OC1REF forced low (ZXLD PWM off, LEDK off)
    regs.cr2 = 0;
    regs.ccer = 0b0001 << 0;

    regs2.cr2 = 0;
    regs2.ccer = 0b0001 << 0;

    reset_break_condition ();

    m_status = off;
    pwm_off ();
    ledk_off ();

 // smcr.ts = 010 (TRGI = TIM3 selection)
 // smcr.sms = 100 (reset mode, re-init & start on TRGI rising edge)

    // UG = 1
    regs2.egr = 1;
    regs2.cr1 = 1;

    regs.egr = 1;
    regs.cr1 = 1;

//    set_on_duty_permille (1000);
  }

  void enable_ledk (void)
  {
    // ramp-up LEDK

    // 1 iteration (256 output writes = ~64 µsec)
    // 256 iterations = ~17 ms
    for (unsigned int j = 0; j < 256; ++j)
    {
      for (unsigned int i = 0; i < j; ++i)
      {
        ledk_on ();
      }

      for (unsigned int i = 0; i < 256 - j; ++i)
      {
        ledk_off ();
      }
    }

    ledk_on ();
  }

  status_t status (void) const
  {
    return is_break_condition () ? break_cond : m_status;
  }

  void set_ready (void)
  {
    // pwm mode 2
    auto& regs = PWM_TimerInstFunc ()().regs ();
    auto&& regs2 = LEDK_ENABLE_TimerInstFunc ()().regs ();

    // disable PWM output, keep LEDK enabled
    pwm_off ();

    // use shortest possible pulse that is seen by the ZXLD.
    constexpr auto pulse_duration = std::chrono::microseconds (3);

    // according to the datasheet, 10 ms is the minimum time when it might
    // start entering standby mode.  so set the pulse period to something
    // shorter than that.
    // if the ZXLD GATE stays at the same level (high or low) for > 100 usec
    // it will go into fault state 2 (driver stalled).  it can be avoided
    // by outputting the ready pulse at a < 100 usec interval.  however, the
    // fault state doesn't seem to have an impact on the response time.
    constexpr auto period = std::chrono::microseconds (5000);

    constexpr auto arr_val = std::chrono::duration_cast<duration> (period).count ();
    constexpr auto ccr_val = std::chrono::duration_cast<duration> (period - pulse_duration).count ();

    static_assert (arr_val > 0 && arr_val < 65535, "");
    static_assert (ccr_val > 0 && ccr_val < 65535, "");

    regs.arr = arr_val;
    regs2.arr = arr_val;

    regs.ccr[0] = ccr_val;
    regs2.ccr[0] = ccr_val;

    // restart ready timer if it was on before
    regs2.egr = 1;
    regs.egr = 1;

    // enable ZXLD PWM timer pulse output

    // keep LEDK on.
    // ledk_off ();

    pwm_pulse ();

    m_status = ready;
  }
#if 0
  void set_on_duty_permille (uint16_t val_in)
  {
    // min zxld pwm dimming pulse width = 2 usec (500 khz)
    // recommended dimming frequency = 1 khz (1000 usec), but can be higher
    //
    // period = 2 usec * 100 steps = 200 usec
    constexpr unsigned int max_val = 1000;
    constexpr unsigned int min_val = 0;

    unsigned int val = std::clamp ((unsigned int)val_in, min_val, max_val);

    // period is the max pulse width (constant on).
    constexpr auto period = std::chrono::microseconds (2) * 100;
    constexpr auto period_ticks = std::chrono::duration_cast<duration> (period).count ();

    // example calculation with period count = 1200 ticks
    // want to map 1000 -> 1200 (period_count)
    // (1200 * 1024) / 1000 = 1228.8
    // (((1200 * 1024) / 1000) * 10000 + 9999) / 10000  = 1229
    // (1000 * 1229) / 1024 = 1200

    constexpr unsigned int scale_factor = ((((uint64_t)period_ticks * 1024 * 10000) / max_val) + 9999) / 10000;

    static_assert (((max_val * scale_factor) / 1024u) == period_ticks);

    unsigned int pulse_width_ticks = (val * scale_factor) / 1024u;

    m_on_arr_val = period_ticks;
    m_on_ccr_val = period_ticks - pulse_width_ticks;

    if (m_status == on)
    {
      auto& regs = PWM_TimerInstFunc ()().regs ();
      regs.arr = m_on_arr_val;
      regs.ccr[0] = m_on_ccr_val;
    }
  }
#endif
  void set_off (void)
  {
    pwm_off ();
//    ledk_off ();

    m_status = off;
  }

  void set_on (void)
  {

#if 1
    pwm_on ();
//    ledk_on ();
#endif

    // when turning on the LED we must be careful not to cause a overvoltage
    // or overcurrent.

    // turn on LEDK first to drain any accumulated voltage that might have
    // built up across LEDA and LEDK.

#if 0
// ramp 11

    static constexpr uint8_t pwm_on_ramp[3] =
    {
      1, 2, 3
    };

    for (auto ramp_val : pwm_on_ramp)
    {
      // force OC1REF high
      for (unsigned int i = 0; i < ramp_val; ++i)
      {
        ledk_on ();
      }

      // force OC1REF low
      for (unsigned int i = 0; i < 4 - ramp_val; ++i)
      {
        ledk_off ();
      }
    }

    pwm_on ();
    ledk_on ();
#endif

#if 0
// ramp 10

    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();

    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();

    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();

    pwm_on ();
    ledk_on ();

#endif


#if 0
// ramp 9

    ledk_on ();
    ledk_on ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();

    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_off ();
    ledk_off ();
    ledk_off ();
    ledk_off ();

    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_on ();
    ledk_off ();
    ledk_off ();

    pwm_on ();
    ledk_on ();

#endif


#if 0
// ramp 7
    pwm_on ();

    static constexpr uint8_t pwm_on_ramp[4] =
    {
      1, 2, 3, 4
    };

    for (auto ramp_val : pwm_on_ramp)
    {
      // force OC1REF high
      for (unsigned int i = 0; i < ramp_val; ++i)
      {
        ledk_on ();
      }

      // force OC1REF low
      for (unsigned int i = 0; i < 4 - ramp_val; ++i)
      {
        ledk_off ();
      }
    }

    ledk_on;
#endif


#if 0
// ramp 6
    static constexpr uint8_t pwm_on_ramp[4] =
    {
      1, 2, 3, 4
    };

    for (auto ramp_val : pwm_on_ramp)
    {
      // force OC1REF high
      for (unsigned int i = 0; i < ramp_val; ++i)
      {
        ledk_on ();
      }

      // force OC1REF low
      for (unsigned int i = 0; i < 4 - ramp_val; ++i)
      {
        ledk_off ();
      }
    }

    ledk_on;
    pwm_on;
#endif

#if 0
// ramp 3
    ledk_on;

    // then do a ramp-up on LEDA
    static constexpr uint8_t pwm_on_ramp[8] =
    {
      1, 2, 3, 4, 7, 10, 12, 16
    };

    // 1 iteration (16 outputs = 4 µsec)
    // 8 iterations = 32 µsec
    for (auto ramp_val : pwm_on_ramp)
    {
      // force OC1REF high
      for (unsigned int i = 0; i < ramp_val; ++i)
      {
        pwm_on ();
        ledk_on ();
      }

      // force OC1REF low
      for (unsigned int i = 0; i < 16 - ramp_val; ++i)
      {
        pwm_off ();
        ledk_off ();
      }
    }
#endif

#if 0
    // enable ZXLD PWM timer pulse output
    auto& regs = PWM_TimerInstFunc ()().regs ();
    regs.arr = m_on_arr_val;
    regs.ccr[0] = m_on_ccr_val;

    // do not restart timer ...

    // restart timer
    regs.egr = 1;
    regs.ccmr1 = 0b0'111'1'0'00;
#endif
    m_status = on;
  }

  void reset_break_condition (void)
  {
    set_off ();
    PWM_TimerInstFunc ()().regs ().sr &= ~(1 << 7);
    PWM_TimerInstFunc ()().regs ()
   	.bdtr = 0
	| (1 << 15)	// MOE = 1, main output enable. cleared in hardware on break
	| (1 << 13)	// BKP = 1, break input BRK is active high
	| (1 << 12)	// BKE = 1, break enable
	| (1 << 11)	// OSSR = 1, run mode off-state: output inactive level
	| (1 << 10)	// OSSI = 1, idle mode off-state: output inactive level
	| 0;

    LEDK_ENABLE_TimerInstFunc ()().regs ()
   	.bdtr = 0
	| (1 << 15)	// MOE = 1, main output enable. cleared in hardware on break
	| (1 << 13)	// BKP = 1, break input BRK is active high
	| (1 << 12)	// BKE = 1, break enable
	| (1 << 11)	// OSSR = 1, run mode off-state: output inactive level
	| (1 << 10)	// OSSI = 1, idle mode off-state: output inactive level
	| 0;
  }

  bool is_break_condition (void) const
  {
    return (PWM_TimerInstFunc ()().regs ().sr & (1 << 7)) != 0;
  }

private:
  status_t m_status;
//  uint16_t m_on_arr_val;
//  uint16_t m_on_ccr_val;
};


} // namespace dev

#endif // includeguard_board_nld_dev_zxld_pwm_output_hpp_includeguard

/*

MCU -> ZXLD ADJ output

uses two channel outputs of a timer CH1 and CH2 with extern RC filter as a DAC
to set the LED output current for the ZXLD GI and ADJ pins.

the ADJ output value is limited to 50% duty cycle, which is about 1.4V
the actual working range of the ZXLD ADJ input is 0.125V..1.25V.
the working range of the ZXLD GI input is 0.2 x ADJ to 0.5 x ADJ

but we don't care about the actual PWM output voltage.  instead
the calibration stores the relationship of PWM value -> LED output current.
this compensates for the other component tolerances and variances as well.

try to read-back the actual voltage:
  * switch pin to analog input
  * use ADC
*/

#ifndef includeguard_board_nld_dev_zxld_adj_hpp_includeguard
#define includeguard_board_nld_dev_zxld_adj_hpp_includeguard

#include <dev/stm32f0/stm32f0_tim.hpp>
#include <utils/value_range.hpp>
#include <type_traits>

namespace dev
{

template <typename TimerInstFunc, unsigned int CarrierFreqHz>
class zxld_adj
{
public:
  using timer_type = typename std::remove_reference<decltype (TimerInstFunc ()())>::type;

  static constexpr unsigned int carrier_frequency = CarrierFreqHz;
  static constexpr unsigned int tick_frequency = timer_type::tick_frequency;

  typedef std::chrono::duration<int32_t, std::ratio<1, tick_frequency>> duration;
  typedef typename duration::rep rep;
  typedef typename duration::period period;

  static constexpr unsigned int period_counter_value = tick_frequency / carrier_frequency;

  static constexpr unsigned int max_value (void) { return period_counter_value/2; }

  #if defined (BOARD_VARIANT_A02)
    static constexpr unsigned int gi_timer_ch = 0;
    static constexpr unsigned int adj_timer_ch = 1;
  #elif defined (BOARD_VARIANT_A03) ||defined (BOARD_VARIANT_A04)
    static constexpr unsigned int gi_timer_ch = 1;
    static constexpr unsigned int adj_timer_ch = 0;
  #endif

  zxld_adj (void)
  {
    static_assert (period_counter_value <= 65535, "");

    auto&& regs = TimerInstFunc ()().regs ();
    regs.psc = 0;
    regs.arr = period_counter_value;
    regs.ccr[gi_timer_ch] = 0;
    regs.ccr[adj_timer_ch] = 0;

    regs.cr2 = 0;

    // OC2M = 110 (output compare mode = PWM mode 1)
    // OC2PE = 0 (preload enable)
    // OC2FE = 0 (fast disable)
    // CC2S = 00 (channel configured as output)
    regs.ccmr1 = (0b0'110'0'0'00 << 8) | (0b0'110'0'0'00 << 0);

    // CC2NP = 0 (active high)
    // CC2P  = 0 (active high)
    // CC2E  = 1 (output enable)
    regs.ccer = (0b0001 << 4) | (0b0001 << 0);

    // BDTR MOE = 1 (OC and OCN outputs are enabled)
    regs.bdtr = 1 << 15;

    regs.egr = 1;
    regs.cr1 = 1;
  }

  using adj_input_param_t = utils::clamped_value<unsigned int, 0, max_value ()>;
  using gi_input_param_t = utils::clamped_value<unsigned int, 0, max_value ()/2>;

  void set_value (adj_input_param_t adj_val, gi_input_param_t gi_val)
  {
    auto&& regs = TimerInstFunc ()().regs ();
    regs.ccr[gi_timer_ch] = gi_val;   // GI output
    regs.ccr[adj_timer_ch] = adj_val;  // ADJ output
  }

  void set_adj_value (adj_input_param_t adj_val)
  {
    TimerInstFunc ()().regs ().ccr[adj_timer_ch] = adj_val;
  }

  void set_adj_output_mode (unsigned int ccmr_ocm_bits)
  {
    constexpr unsigned int sh = adj_timer_ch * 8;
    auto&& regs = TimerInstFunc ()().regs ();

    regs.ccmr1 = (regs.ccmr1 & ~(0b0'111'0'0'00 << sh)) | ((ccmr_ocm_bits & 0b111) << (4 + sh));
  }

  void set_gi_output_mode (unsigned int ccmr_ocm_bits)
  {
    constexpr unsigned int sh = gi_timer_ch * 8;
    auto&& regs = TimerInstFunc ()().regs ();

    regs.ccmr1 = (regs.ccmr1 & ~(0b0'111'0'0'00 << sh)) | ((ccmr_ocm_bits & 0b111) << (4 + sh));
  }


  void set_adj_output_const_high (void)
  {
    set_adj_output_mode (0b101);
  }
  void set_adj_output_const_low (void)
  {
    set_adj_output_mode (0b100);
  }
  void set_adj_output_pwm (void)
  {
    set_adj_output_mode (0b110);
  }

  void set_gi_value (gi_input_param_t gi_val)
  {
    TimerInstFunc ()().regs ().ccr[gi_timer_ch] = gi_val;
  }
  void set_gi_output_const_high (void)
  {
    set_gi_output_mode (0b101);
  }
  void set_gi_output_const_low (void)
  {
    set_gi_output_mode (0b100);
  }
  void set_gi_output_pwm (void)
  {
    set_gi_output_mode (0b110);
  }

  unsigned int adj_value (void) const
  {
    return TimerInstFunc ()().regs ().ccr[adj_timer_ch];
  }

  unsigned int gi_value (void) const
  {
    return TimerInstFunc ()().regs ().ccr[gi_timer_ch];
  }

private:
};


} // namespace dev

#endif // includeguard_board_nld_dev_zxld_adj_output_hpp_includeguard

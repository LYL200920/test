/*
  GD32F30xx TIM timer device

  this driver can be used for all the timers

     Type    | timer    | reg base address
-------------+----------+---------------------
   Advanced  |  TIM0    | 0x4001'2C00
   Advanced  |  TIM7    | 0x4001'3400
-------------+----------+---------------------
  General-L0 |  TIM1    | 0x4000'0000
  General-L0 |  TIM2    | 0x4000'0400
  General-L0 |  TIM3    | 0x4000'0800
  General-L0 |  TIM4    | 0x4000'0C00
-------------+----------+---------------------
  General-L1 |  TIM8    | 0x4001'4C00
  General-L1 |  TIM11   | 0x4000'1800
-------------+----------+---------------------
  General-L2 |  TIM9    | 0x4001'5000
  General-L2 |  TIM10   | 0x4001'5400
  General-L2 |  TIM12   | 0x4000'1C00
  General-L2 |  TIM13   | 0x4000'2000
-------------+----------+---------------------
    Basic    |  TIM5    | 0x4000'1000
    Basic    |  TIM6    | 0x4000'1400
-------------+----------+---------------------

  Note: not all hardware timers support all functions.
*/

#ifndef includeguard_gd32f30_tim_hpp_includeguard
#define includeguard_gd32f30_tim_hpp_includeguard

#include <cstdint>
#include <functional>
#include <chrono>
#include <array>
#include <dev/hwreg.hpp>
#include <dev/interrupt.hpp>
#include <utils/bits.hpp>
// #include <dev/timer.hpp>

namespace dev
{

namespace gd32f30_tim
{

typedef std::function<void (void)> trigger_callback_func_t;


enum trigger_disable_tag { disable };
enum trigger_enable_tag { enable };

enum capture_compare_output_mode : uint8_t
{
  // The OxCPRE (the the output prepare signal of CHx) signal
  // keeps stable, independent of the comparison between
  // the register TIMERx_CHxCV and the counter TIMERx_CNT.
  timing = 0b000,

  // OxCPRE signal is forced high when the counter is
  // equals to the output compare register TIMERx_CHxCV
  set_if_match = 0b001,

  // OxCPRE signal is forced low when the counter is
  // equals to the output compare register TIMERx_CHxCV
  clear_if_match  = 0b010,

  // OxCPRE toggles when the counter is equals to the output
  // compare register TIMERx_CHxCV.
  toggle_if_match = 0b011,

  // O0CPRE is forced to low level OR high level.
  force_low  = 0b100,
  force_high = 0b101,

  // When counting up, O0CPRE is high when the counter is
  // smaller than TIMERx_CH0CV, and low otherwise.
  // When counting down, O0CPRE is low when the counter is
  // larger than TIMERx_CH0CV, and high otherwise.
  pwm_0 = 0b110,
  // contrary to pwm_0
  pwm_1 = 0b111,
};

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -


template <uintptr_t RegBaseAddr,
	  unsigned int ChannelCount,
	  unsigned int PeripheralClockHz,
	  typename BreakInterruptLine,
	  typename TriggerInterruptLine,
	  typename CommutationInterruptLine,
	  typename CaptureCompare_InterruptLine,
	  typename UpdateInterruptLine>
class hw_inst
{
public:
  struct regs_t
  {
    // all timer are 16-bit counter.

    // most of regs are 16-bit wide but aligned on 32 bit and can
    // be accessed by half-word(16-bit) or word(32-bit)
    // smcfg is 32-bit wide and only can be accessed word (32-bit)
						// offset
    dev::hw_reg_rw<uint32_t> ctl0;		// 0x00                      Control 0
    dev::hw_reg_rw<uint32_t> ctl1;		// 0x04                      Control 1
    dev::hw_reg_rw<uint32_t> smcfg;		// 0x08                      Slave mode configuration
    dev::hw_reg_rw<uint32_t> dmainten;		// 0x0C                      DMA and interrupt enable
    dev::hw_reg_rw<uint32_t> intf;		// 0x10                      Interrupt flag
    dev::hw_reg_rw<uint32_t> swevg;		// 0x14                      Software event generation
    dev::hw_reg_rw<uint32_t> chctl0;		// 0x18                      Channel control 0
    dev::hw_reg_rw<uint32_t> chctl1;		// 0x1C                      Channel control 1
    dev::hw_reg_rw<uint32_t> chctl2;		// 0x20                      Channel control 2
    dev::hw_reg_rw<uint32_t> cnt;		// 0x24                      Counter
    dev::hw_reg_rw<uint32_t> psc;		// 0x28                      Prescaler
    dev::hw_reg_rw<uint32_t> car;		// 0x2C                      Counter auto reload
    dev::hw_reg_rw<uint32_t> crep;		// 0x30                      Counter repetition
    dev::hw_reg_rw<uint32_t> chcv[4];		// 0x34, 0x38, 0x3C, 0x40    Channel 0, 1, 2, 3 capture/compare value
    dev::hw_reg_rw<uint32_t> cchp;		// 0x44                      Complementary channel protection
    dev::hw_reg_rw<uint32_t> dmacfg;		// 0x48                      DMA configuration
    dev::hw_reg_rw<uint32_t> dmatb;		// 0x4C                      DMA transfer buffer
    dev::hw_reg_rw<uint32_t> reserve[42];	// ...                       0xFC-0x4C-sizeof (uint32_t)
    dev::hw_reg_rw<uint32_t> cfg;		// 0xFC                      Configuration
  };

  static_assert (sizeof (regs_t) == 0x100 /*0xFC+4*/, "gd32f30_tim regs_t size mismatch");

  static constexpr regs_t& regs (void) { return *(regs_t*)RegBaseAddr; }

  typedef BreakInterruptLine           break_interrupt_line;
  typedef TriggerInterruptLine         trigger_interrupt_line;
  typedef CommutationInterruptLine     commutation_interrupt_line;
  typedef CaptureCompare_InterruptLine capture_compare_interrupt_line;
  typedef UpdateInterruptLine          update_interrupt_line;

  static constexpr auto interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int interrupt_priority = interrupt::priority_1;

  static constexpr unsigned int channel_count = ChannelCount;

  // use the highest clock possible to represent time values of this timer.
  // if clock scaling is used, the counts are shifted.
  static constexpr unsigned int tick_frequency = PeripheralClockHz;

  typedef std::chrono::duration<int32_t, std::ratio<1, tick_frequency>> duration;
  typedef typename duration::rep rep;
  typedef typename duration::period period;
  typedef std::chrono::time_point<hw_inst, duration> time_point;

  static time_point now (void)
  {
    // FIXME: this should use the current time scaling
    return time_point (duration (regs ().cnt));
  }

  // we can't tie the time point to uint16 because if clock scaling is used,
  // we need to be able to represent much larger values.
  static constexpr time_point max_time_point (void)
  {
    // FIXME: this should use the current time scaling
    return time_point (duration (std::numeric_limits<uint16_t>::max ()));
  }

  uint16_t prescaler (void) const { return regs ().psc; }
  void set_prescaler (uint16_t val) { regs ().psc = val; }

  // FIXME: this should use the current time scaling
  uint16_t counter (void) { return regs ().cnt; }
  void set_counter (duration val) { regs ().cnt = val.count (); }
  void set_counter (time_point val) { regs ().cnt = val.time_since_epoch ().count (); }

  uint16_t capture_compare_value (unsigned int channel) const { return regs ().chcv[channel]; }
  void set_capture_compare_value (unsigned int channel, uint16_t val) { regs ().chcv[channel] = val; }

  uint16_t auto_reload_value (void) const { return regs ().car; }
  void set_auto_reload_value (uint16_t val) { regs ().car = val; }

  [[gnu::cold]] hw_inst (void)
  {
    break_isr_t (this);
    trigger_isr_t (this);
    commutation_isr_t (this);
    capture_compare_isr_t (this);
    update_isr_t (this);

    set_interrupt_enable (true);
  }

  [[gnu::cold]] ~hw_inst (void)
  {
    set_interrupt_enable (false);
  }

  void set_interrupt_enable (bool val)
  {
    if (!val)
    {
      // disable all interrut requests in the timer unit
      // regs ().tier = 0;

      break_isr_t::disable ();
      trigger_isr_t::disable ();
      commutation_isr_t::disable ();
      capture_compare_isr_t::disable ();
      update_isr_t::disable ();
    }
    else
    {
      break_isr_t::enable (interrupt_type, interrupt_priority);
      trigger_isr_t::enable (interrupt_type, interrupt_priority);
      commutation_isr_t::enable (interrupt_type, interrupt_priority);
      capture_compare_isr_t::enable (interrupt_type, interrupt_priority);
      update_isr_t::enable (interrupt_type, interrupt_priority);
    }
  }

  // set counter value only.  keep trigger callback function as it is.
  // this can be used to use the trigger for counter-clearing only (no callback)
  // or to modify the current counter value.
  void set_capture_compare (unsigned int channel, duration counter_value)
  {
    if (channel >= channel_count)
      return;

    if (!m_capture_compare_func[channel])
      return;

    // FIXME: this should use the current time scaling
    regs ().chcv[channel] = counter_value.count ();
  }

  // set capture/compare callback function for the specified channel.
  // if the channel is greater than the number of channels available,
  // it will do nothing.
  template <typename Func>
  void set_capture_compare_interupt_func (unsigned int channel, Func f)
  {
    if (channel >= channel_count)
      return;

    m_capture_compare_func[channel] = f;
    if (m_capture_compare_func[channel])
    {
      // enable interrupts for that channel capture/compare
      regs ().dmainten = utils::set_bit (regs ().dmainten.read (), channel + 1);
    }
    else
    {
      // disable interrupts for that channel capture/compare
      regs ().dmainten = utils::clear_bit (regs ().dmainten.read (), channel + 1);
    }
  }

  void set_capture_compare_interrupt_enable (unsigned int channel, bool val)
  {
    if (channel >= channel_count)
      return;

    if (!m_capture_compare_func[channel])
      return;

    if (!val)
      regs ().dmainten = utils::clear_bit (regs ().dmainten.read (), channel + 1);
    else
      regs ().dmainten = utils::set_bit (regs ().dmainten.read (), channel + 1);
  }

  // set channel capture compare mode to output.
  void set_capture_compare_channel_mode (unsigned int channel, capture_compare_output_mode mode)
  {
    if (channel >= channel_count)
      return;

    // disable write protection
    regs ().cchp &= ~(uint16_t)(0b11 << 8);

    if (channel == 0)
    {
      // clear channel 0.
      regs ().chctl0 &= ~(uint16_t)(0b1111'1111 << 0);

      regs ().chctl0 |= (uint16_t)0
		| (0 << 7)     // CHxCOMCEN     : output compare clear disable
		| (mode << 4)  // CHxCOMCTL[2:0]: compare output control
		| (0 << 3)     // CHxCOMSEN     : output compare shadow disable
		| (0 << 2)     // CHxCOMFEN     : output quickly compare disable.
		| (0b00 << 0); // CHxMS[1:0]    : I/O mode selection
    }

    else if (channel == 1)
    {
      // clear channel 1.
      regs ().chctl0 &= ~(uint16_t)(0b1111'1111 << 7);

      regs ().chctl0 |= ((uint16_t)0
		| (0 << 7)    // CHxCOMCEN     : output compare clear disable
		| (mode << 4) // CHxCOMCTL[2:0]: compare output control
		| (0 << 3)    // CHxCOMSEN     : output compare shadow disable
		| (0 << 2)    // CHxCOMFEN     : output quickly compare disable.
		| (0b00 << 0) // CHxMS[1:0]    : I/O mode selection
		) << 7;
    }

    else if (channel == 2)
    {
      // clear channel 2.
      regs ().chctl1 &= ~(uint16_t)(0b1111'1111 << 0);

      regs ().chctl1 |= (uint16_t)0
		| (0 << 7)     // CHxCOMCEN     : output compare clear disable
		| (mode << 4)  // CHxCOMCTL[2:0]: compare output control
		| (0 << 3)     // CHxCOMSEN     : output compare shadow disable
		| (0 << 2)     // CHxCOMFEN     : output quickly compare disable.
		| (0b00 << 0); // CHxMS[1:0]    : I/O mode selection
    }

    else if (channel == 3)
    {
      // clear channel 3.
      regs ().chctl1 &= ~(uint16_t)(0b1111'1111 << 7);

      regs ().chctl1 |= ((uint16_t)0
		| (0 << 7)    // CHxCOMCEN     : output compare clear disable
		| (mode << 4) // CHxCOMCTL[2:0]: compare output control
		| (0 << 3)    // CHxCOMSEN     : output compare shadow disable
		| (0 << 2)    // CHxCOMFEN     : output quickly compare disable.
		| (0b00 << 0) // CHxMS[1:0]    : I/O mode selection
		) << 7;
    }
  }

  void set_capture_compare_channel_enable (unsigned int channel, bool val)
  {
    if (channel >= channel_count)
      return;

    // disable write protection
    regs ().cchp &= ~(uint16_t)(0b11 << 8);

    // FIXME: we just add the channel enable if func here,
    //        we are using the default option now, the other
    //        options set funcs should be added later...
    regs ().chctl2 &= ~(uint16_t)(0b1111 << channel);
    regs ().chctl2 |= ((uint16_t)0
		| (0 << 3) // CH0NP  : complementary output high level is active level
		| (0 << 2) // CH0NEN : complementary output disabled
		| (0 << 1) // CH0P   : high level is active level
		| (0 << 0) // CHxEN  : disable
		) << 4 * channel;

    if (!val)
    {
      // Primary output enable: disable channel outputs (CHxO or CHxON)
      regs ().cchp = utils::clear_bit (regs ().cchp.read (), 15);
      // CHxEN: capture/compare function disable
      regs ().chctl2 = utils::clear_bit (regs ().chctl2.read (), channel*4);
    }
    else
    {
      // Primary output enable: enabled channel outputs (CHxO or CHxON)
      regs ().cchp = utils::set_bit (regs ().cchp.read (), 15);
      // CHxEN: capture/compare function enable
      regs ().chctl2 = utils::set_bit (regs ().chctl2.read (), channel*4);
    }
  }

  void start (void)
  {
    regs ().ctl0 = utils::set_bit (regs ().ctl0.read (), 0);
  }

  void stop (void)
  {
    regs ().ctl0 = utils::clear_bit (regs ().ctl0.read (), 0);
  }

  bool running (void) const
  {
    return utils::get_bit (regs ().ctl0.read (), 0);
  }


public:
  typedef interrupt::connected_isr<break_interrupt_line,
	interrupt::func<decltype (&hw_inst::break_isr), &hw_inst::break_isr>> break_isr_t;

  typedef interrupt::connected_isr<trigger_interrupt_line,
	interrupt::func<decltype (&hw_inst::trigger_isr), &hw_inst::trigger_isr>> trigger_isr_t;

  typedef interrupt::connected_isr<commutation_interrupt_line,
	interrupt::func<decltype (&hw_inst::commutation_isr), &hw_inst::commutation_isr>> commutation_isr_t;

  typedef interrupt::connected_isr<capture_compare_interrupt_line,
	interrupt::func<decltype (&hw_inst::capture_compare_isr), &hw_inst::capture_compare_isr>> capture_compare_isr_t;

  typedef interrupt::connected_isr<update_interrupt_line,
	interrupt::func<decltype (&hw_inst::update_isr), &hw_inst::update_isr>> update_isr_t;

private:
  void break_isr (void)
  {
  }

  void trigger_isr (void)
  {
  }

  void commutation_isr (void)
  {
  }

  void capture_compare_isr (void)
  {
    // This flag is set by hardware and cleared by software.

    // When channel 0 is in input  mode, this flag is set when a "capture" event occurs.
    // When channel 0 is in output mode, this flag is set when a "compare" event occurs.

    // If channel0 is set to input mode, this bit will be reset by reading regs.chcv[channel].

    uint32_t interrupt_flag = regs ().intf;

    constexpr unsigned int channel_0_int_f  = 1 << 1;
    constexpr unsigned int channel_1_int_f  = 1 << 2;
    constexpr unsigned int channel_2_int_f  = 1 << 3;
    constexpr unsigned int channel_3_int_f  = 1 << 4;

    // channnel0 interrupt occurred
    if (interrupt_flag & channel_0_int_f)
    {
      if (m_capture_compare_func[0])
        m_capture_compare_func[0] ();

      // clear flag.
      regs ().intf = utils::clear_bit (regs ().intf.read (), channel_0_int_f);
    }

    // channnel1 interrupt occurred
    if (interrupt_flag & channel_1_int_f)
    {
      if (m_capture_compare_func[1])
        m_capture_compare_func[1] ();

      // clear flag.
      regs ().intf = utils::clear_bit (regs ().intf.read (), channel_1_int_f);
    }

    // channnel2 interrupt occurred
    if (interrupt_flag & channel_2_int_f)
    {
      if (m_capture_compare_func[2])
        m_capture_compare_func[2] ();

      // clear flag.
      regs ().intf = utils::clear_bit (regs ().intf.read (), channel_2_int_f);
    }

    // channnel3 interrupt occurred
    if (interrupt_flag & channel_3_int_f)
    {
      if (m_capture_compare_func[3])
        m_capture_compare_func[3] ();

      // clear flag.
      regs ().intf = utils::clear_bit (regs ().intf.read (), channel_3_int_f);
    }
  }

  void update_isr (void)
  {
    // check which timer channels have fired and mask them with the interrupt
    // enable setting.  call user defined function
  }

private:
  std::array<trigger_callback_func_t, channel_count> m_capture_compare_func;
};


} // namespace gd32f30_tim
} // namespace dev
#endif // includeguard_gd32f30_tim_hpp_includeguard

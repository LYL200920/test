/*
  STM32F0xx TIM timer device

  this driver can be used for all the timers

   timer    | reg base address
  ----------+---------------------------
    TIM2    | 0x40000000
    TIM3    | 0x40000400
    TIM6    | 0x40001000
    TIM7    | 0x40001400
    TIM14   | 0x40002000
    TIM1    | 0x40012C00
    TIM15   | 0x40014000
    TIM16   | 0x40014400
    TIM17   | 0x40014800

  but not all hardware timers support all functions.
*/

#ifndef includeguard_stm32f0_tim_hpp_includeguard
#define includeguard_stm32f0_tim_hpp_includeguard

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

namespace stm32f0_tim
{

typedef std::function<void (void)> trigger_callback_func_t;


enum trigger_disable_tag { disable };
enum trigger_enable_tag { enable };

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -


template <uintptr_t RegBaseAddr,
	  unsigned int ChannelCount,
	  unsigned int PeripheralClockHz,
	  typename BreakUpdateTriggerCom_InterruptLine,
	  typename CaptureCompare_InterruptLine,
	  typename Global_InterruptLine>
	  // typename ModuleEnableDisableFunc>
class hw_inst
{
public:
  typedef BreakUpdateTriggerCom_InterruptLine break_update_trigger_com_interrupt_line;
  typedef CaptureCompare_InterruptLine capture_compare_interrupt_line;
  typedef Global_InterruptLine global_interrupt_line;

  static constexpr auto interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int interrupt_priority = 1;

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

  // FIXME: this should use the current time scaling
  void set_counter (time_point val) { regs ().cnt = val.time_since_epoch ().count (); }
  void set_counter (duration val) { regs ().cnt = val.count (); }

  uint16_t prescaler (void) const { return regs ().psc; }
  void set_prescaler (uint16_t val) { regs ().psc = val; }

  [[gnu::cold]] hw_inst (void)
  : m_break_update_trigger_com_isr (this), m_capture_compare_isr (this),
    m_global_isr (this)
  {
    set_device_enable (true);
  }

  [[gnu::cold]] ~hw_inst (void)
  {
    set_device_enable (false);
  }

  void set_device_enable (bool val)
  {
    if (!val)
    {
      // disable all interrut requests in the timer unit
      // regs ().tier = 0;

      m_break_update_trigger_com_isr.disable ();
      m_capture_compare_isr.disable ();
      m_global_isr.disable ();
    }
    else
    {
      m_break_update_trigger_com_isr.enable (interrupt_type, interrupt_priority);
      m_capture_compare_isr.enable (interrupt_type, interrupt_priority);
      m_global_isr.enable (interrupt_type, interrupt_priority);
    }

    //ModuleEnableDisableFunc () (val);
  }


  // set counter value and trigger callback function for the specified channel.
  // if the channel is greater than the number of channels available,
  // it will do nothing.
  template <typename Func>
  void set_trigger (unsigned int channel, duration counter_value, Func f)
  {
    if (channel >= channel_count)
      return;

    m_trigger_func[channel] = f;
    if (m_trigger_func[channel])
    {
      // enable interrupts for that channel capture/compare
      m_imask = utils::set_bit (m_imask, channel + 1);
      regs ().dier = utils::set_bit (regs ().dier.read (), channel + 1);
    }
    else
    {
      // disable interrupts for that channel capture/compare
      m_imask = utils::clear_bit (m_imask, channel + 1);
      regs ().dier = utils::clear_bit (regs ().dier.read (), channel + 1);
    }

    // FIXME: this should use the current time scaling
    regs ().ccr[channel] = counter_value.count ();
  }

  // set counter value only.  keep trigger callback function as it is.
  // this can be used to use the trigger for counter-clearing only (no callback)
  // or to modify the current counter value.
  void set_trigger (unsigned int channel, duration counter_value)
  {
    if (channel >= channel_count)
      return;

    // FIXME: this should use the current time scaling
    regs ().ccr[channel] = counter_value.count ();
  }

/*
  // explicitly enable the trigger point.  this is useful in combination with
  // DTC scripts which don't set or use the CPU callback function.
  void set_trigger_a (duration counter_value, trigger_enable_tag)
  {
    m_tgi_a_isr.enable (tgi_a_interrupt_type, tgi_a_interrupt_priority);
    regs ().tier |= tgiea | tier_reserved_set_bits;

    // FIXME: this should use the current time scaling
    regs ().trga = counter_value.count ();
  }
*/

  // disable the trigger point and clear the trigger callback function.
  void set_trigger (unsigned int channel, trigger_disable_tag)
  {
    // trigger point can't be really disabled.  it's always there.

    if (channel >= channel_count)
      return;

    m_imask = utils::clear_bit (m_imask, channel + 1);
    regs ().dier = utils::clear_bit (regs ().dier.read (), channel + 1);
  }

  // trigger hardware register.  e.g. for using in DMA scripts
  auto& trigger_hwreg (unsigned int channel)
  {
    if (channel >= channel_count)
      return m_dummy_ccr_hwreg;

    regs ().ccr[channel];
  }

  auto& counter_hwreg (void) { return regs ().cnt; }

  void start (void)
  {
    // utils::atomic_set_bit (TpuChannelNumber % 8u, &TSTR);
  }

  void stop (void)
  {
    // utils::atomic_clear_bit (TpuChannelNumber % 8u, &TSTR);
  }

  bool running (void) const
  {
    // return utils::get_bit (*&TSTR, TpuChannelNumber);
    return false;
  }




  struct regs_t
  {
    // all regs are actually only 16 bit wide but aligned on 32 bit.
    // manual says the "registers are 16-bit addressable" for TIM1.
    // but not sure about the endianess.
    // TIM2 has a 32 bit counter.
					// offset
    dev::hw_reg_rw<uint32_t> cr1;	// 0x00
    dev::hw_reg_rw<uint32_t> cr2;	// 0x04
    dev::hw_reg_rw<uint32_t> smcr;	// 0x08
    dev::hw_reg_rw<uint32_t> dier;	// 0x0C
    dev::hw_reg_rw<uint32_t> sr;	// 0x10
    dev::hw_reg_rw<uint32_t> egr;	// 0x14
    dev::hw_reg_rw<uint32_t> ccmr1;	// 0x18
    dev::hw_reg_rw<uint32_t> ccmr2;	// 0x1C
    dev::hw_reg_rw<uint32_t> ccer;	// 0x20
    dev::hw_reg_rw<uint32_t> cnt;	// 0x24
    dev::hw_reg_rw<uint32_t> psc;	// 0x28
    dev::hw_reg_rw<uint32_t> arr;	// 0x2C
    dev::hw_reg_rw<uint32_t> rcr;	// 0x30
    dev::hw_reg_rw<uint32_t> ccr[4];	// 0x34, 0x38, 0x3C, 0x40
    dev::hw_reg_rw<uint32_t> bdtr;	// 0x44
    dev::hw_reg_rw<uint32_t> dcr;	// 0x48
    dev::hw_reg_rw<uint32_t> dmar;	// 0x4C
    dev::hw_reg_rw<uint32_t> optr;	// 0x50 (TIM14 only, "OR")
  };

  static_assert (sizeof (regs_t) == 0x54, "");

  static constexpr regs_t& regs (void) { return *(regs_t*)RegBaseAddr; }

private:

  // TIM1 has 2 different ISRs
  void break_update_trigger_com_isr (void)
  {
  }

  void capture_compare_isr (void)
  {
    // check which timer channels have fired and mask them with the interrupt
    // enable setting.  call user defined function
  }

  // all other timers use only one global ISR
  void global_isr (void)
  {
    // check which timer channels have fired and mask them with the interrupt
    // enable setting.  call user defined function
  }

public:
  typedef interrupt::connected_isr<break_update_trigger_com_interrupt_line,
	interrupt::func<decltype (&hw_inst::break_update_trigger_com_isr), &hw_inst::break_update_trigger_com_isr>> break_update_trigger_com_isr_t;

  typedef interrupt::connected_isr<capture_compare_interrupt_line,
	interrupt::func<decltype (&hw_inst::capture_compare_isr), &hw_inst::break_update_trigger_com_isr>> capture_compare_isr_t;

  typedef interrupt::connected_isr<global_interrupt_line,
	interrupt::func<decltype (&hw_inst::global_isr), &hw_inst::global_isr>> global_isr_t;

private:
  break_update_trigger_com_isr_t m_break_update_trigger_com_isr;
  capture_compare_isr_t m_capture_compare_isr;
  global_isr_t m_global_isr;

  uint32_t m_imask = 0;
  std::array<trigger_callback_func_t, channel_count> m_trigger_func;

  hw_reg_rw<uint32_t> m_dummy_ccr_hwreg;
};


} // namespace stm32f0_tim
} // namespace dev
#endif // includeguard_stm32f0_tim_hpp_includeguard

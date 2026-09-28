/*
  RX CMT timer device

  each device has 2 channels.
  channels are instantiated individually.

*/

#ifndef includeguard_rx_cmt_hpp_includeguard
#define includeguard_rx_cmt_hpp_includeguard

#include <cstdint>
#include <functional>
#include <chrono>
#include <dev/hwreg.hpp>
#include <dev/interrupt.hpp>
#include <utils/bits.hpp>
#include <dev/timer.hpp>

namespace dev
{

namespace rx_cmt
{

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

enum cks_value_t
{
  pclk_8 = 0b00,
  pclk_32 = 0b01,
  pclk_128 = 0b10,
  pclk_512 = 0b11
};

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

class cmcr_t
{
public:
  constexpr cmcr_t (void) : m_value (0) { }
  explicit constexpr cmcr_t (uint8_t v) : m_value (v) { }

  constexpr uint16_t value (void) const { return m_value; }

  constexpr cks_value_t clock_select (void) const { return (cks_value_t)(m_value & 0b11); }
  cmcr_t& set_clock_select (cks_value_t val)
  {
    m_value = (m_value & ~0b11) | (unsigned int)val;
    return *this;
  }

  constexpr bool match_interrupt_enabled (void) const { return utils::get_bit (m_value, 6); }
  cmcr_t& set_match_interrupt_enabled (bool val = true) { m_value = utils::set_bit (m_value, 6, val); return *this; }

private:
  uint16_t m_value;
};

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -


template <uintptr_t RegBaseAddr,	// address of CMSTRx for the channel group
	  unsigned int ChannelNumber,
	  unsigned int PeripheralClockHz,
	  typename CMI_InterruptLine,	// compare match
	  typename ModuleEnableDisableFunc,
	  typename TriggerCallBackFunc = timer::trigger_callback_func_t>
class hw_chn_inst
{
public:
  typedef CMI_InterruptLine cmi_interrupt_line;
  static constexpr auto cmi_interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int cmi_interrupt_priority = interrupt::priority_8;

  // use the highest clock possible to represent time values of this timer.
  // if clock scaling is used, the counts are shifted.
  static constexpr unsigned int tick_frequency = PeripheralClockHz / 8;

  typedef std::chrono::duration<int32_t, std::ratio<1, tick_frequency>> duration;
  typedef typename duration::rep rep;
  typedef typename duration::period period;
  typedef std::chrono::time_point<hw_chn_inst, duration> time_point;

  static time_point now (void)
  {
    // FIXME: this should use the current time scaling
    return time_point (duration (regs ().cmcnt));
  }

  // we can't tie the time point to uint16 because if clock scaling is used,
  // we need to be able to represent much larger values.
  static constexpr time_point max_time_point (void)
  {
    // FIXME: this should use the current time scaling
    return time_point (duration (std::numeric_limits<uint16_t>::max ()));
  }

  // FIXME: this should use the current time scaling
  void set_counter (time_point val) { regs ().cmcnt = val.time_since_epoch ().count (); }
  void set_counter (duration val) { regs ().cmcnt = val.count (); }

  [[gnu::cold]] hw_chn_inst (void)
  : m_cmi_isr (this)
  {
    set_device_enable (true);
  }

  [[gnu::cold]] ~hw_chn_inst (void)
  {
    set_device_enable (false);
  }

  void set_device_enable (bool val)
  {
    if (!val)
    {
      m_cmi_isr.disable ();
    }

    ModuleEnableDisableFunc () (val);
  }


  // set counter value and trigger callback function
  template <typename Func>
  void set_trigger (duration counter_value, Func f)
  {
    m_cmi_func = f;
    if (m_cmi_func)
    {
      m_cmi_isr.enable (cmi_interrupt_type, cmi_interrupt_priority);
    }
    else
    {
      m_cmi_isr.disable ();
    }

    // FIXME: this should use the current time scaling
    regs ().cmcor = counter_value.count ();
  }

  // set counter value only.  keep trigger callback function as it is.
  // this can be used to use the trigger for counter-clearing only (no callback)
  // or to modify the current counter value.
  void set_trigger (duration counter_value)
  {
    // FIXME: this should use the current time scaling
    regs ().cmcor = counter_value.count ();
  }

  // explicitly enable the trigger point.  this is useful in combination with
  // DTC scripts which don't set or use the CPU callback function.
  void set_trigger (duration counter_value, timer::trigger_enable_tag)
  {
    m_cmi_isr.enable (cmi_interrupt_type, cmi_interrupt_priority);

    // FIXME: this should use the current time scaling
    regs ().cmcor = counter_value.count ();
  }

  // disable the trigger point and clear the trigger callback function.
  void set_trigger (timer::trigger_disable_tag)
  {
    // trigger point can't be really disabled.  it's always there.
    // if trigger point a is used as counter clear, counter clearing is disabled.
    // clear the interrupt enable (and all associated DTC/DMAC triggers).
    m_cmi_isr.disable ();
  }


  // trigger_a hardware register.  e.g. for using in DTC instructions
  auto& trigger_hwreg (void) { return regs ().cmcor; }

  auto& counter_hwreg (void) { return regs ().cmcnt; }

  void start (void)
  {
    //utils::atomic_set_bit (ChannelNumber % 2u, &regs ().cmstr);
    regs ().cmstr |= 1 << (ChannelNumber % 2u);
  }

  void stop (void)
  {
    //utils::atomic_clear_bit (ChannelNumber % 2u, &regs ().cmstr);
    regs ().cmstr &= ~(1 << (ChannelNumber % 2u));
  }

  bool running (void) const
  {
    return utils::get_bit (*&regs ().cmstr, ChannelNumber % 2u);
  }


  cmcr_t timer_control (void) const { return cmcr_t (regs ().cmcr); }
  void set_timer_control (cmcr_t val) { regs ().cmcr = val.value (); }

private:

  struct regs_t
  {
    dev::hw_reg_rw<uint16_t, dev::const_addr<RegBaseAddr>> cmstr;
    dev::hw_reg_rw<uint16_t, dev::const_addr<RegBaseAddr + 2 + (ChannelNumber % 2u) * 6>> cmcr;
    dev::hw_reg_rw<uint16_t, dev::const_addr<RegBaseAddr + 4 + (ChannelNumber % 2u) * 6>> cmcnt;
    dev::hw_reg_rw<uint16_t, dev::const_addr<RegBaseAddr + 6 + (ChannelNumber % 2u) * 6>> cmcor;
  };

  // use any dummy address for the register struct.  it doesn't matter because
  // we use const addresses for each individual reg.  but like that we can
  // return a valid reference.
  static constexpr regs_t& regs (void) { return *(regs_t*)0x1000; }

  void cmi_func (void) { if (m_cmi_func) m_cmi_func (); }

public:
  typedef interrupt::connected_isr<cmi_interrupt_line,
	interrupt::func<decltype (&hw_chn_inst::cmi_func), &hw_chn_inst::cmi_func>> cmi_isr_t;

private:
  cmi_isr_t m_cmi_isr;
  TriggerCallBackFunc m_cmi_func;
};


} // namespace rx_cmt
} // namespace dev
#endif // includeguard_rx_cmt_hpp_includeguard

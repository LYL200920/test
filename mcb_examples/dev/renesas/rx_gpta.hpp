/*
  RX GPTa timer device

  for simplicity sake, expose only 2 trigger points.
  if needed, the driver can be extended later.
*/

#ifndef includeguard_rx_gpta_hpp_includeguard
#define includeguard_rx_gpta_hpp_includeguard

#include <dev/timer.hpp>
#include <dev/hwreg.hpp>
#include <utils/bits.hpp>

namespace dev
{

namespace rx_gpta
{

template < uintptr_t RegBaseAddr,
	   unsigned int ChannelNumber,
	   uintptr_t CommonRegBaseAddr,
	   unsigned int PeripheralClockHz,

	   typename GTCI_A_InterruptLine,
	   typename GTCI_B_InterruptLine,
	   typename GTCI_V_InterruptLine,
	   typename GTCI_U_InterruptLine,

	   typename ModuleEnableDisableFunc >
class hw_chn_inst
{
public:
  using gtci_a_interrupt_line = GTCI_A_InterruptLine;
  static constexpr auto gtci_a_interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int gtci_a_interrupt_priority = interrupt::priority_8;

  using gtci_b_interrupt_line = GTCI_B_InterruptLine;
  static constexpr auto gtci_b_interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int gtci_b_interrupt_priority = interrupt::priority_8;

  using gtci_v_interrupt_line = GTCI_V_InterruptLine;
  static constexpr auto gtci_v_interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int gtci_v_interrupt_priority = interrupt::priority_8;

  using gtci_u_interrupt_line = GTCI_U_InterruptLine;
  static constexpr auto gtci_u_interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int gtci_u_interrupt_priority = interrupt::priority_8;

  // highest clock possible
  static constexpr unsigned int tick_frequency = PeripheralClockHz;


  typedef std::chrono::duration<int32_t, std::ratio<1, tick_frequency>> duration;
  typedef typename duration::rep rep;
  typedef typename duration::period period;
  typedef std::chrono::time_point<hw_chn_inst, duration> time_point;

  static time_point now (void)
  {
    // FIXME: this should use the current time scaling
    return time_point (duration (regs ().gtcnt));
  }

  // we can't tie the time point to uint16 because if clock scaling is used,
  // we need to be able to represent much larger values.
  static constexpr time_point max_time_point (void)
  {
    // FIXME: this should use the current time scaling
    return time_point (duration (std::numeric_limits<uint16_t>::max ()));
  }

  void set_simple_trigger_mode (void)
  {
    auto&& r = regs ();

    r.gtcr = 0
	| (0b000 << 0)	// saw-wave pwm mode 0b000
	| (0b00 << 8)	// timer prescaler = pclka/1
	| (0b00<< 12)	// counter clearing disable.
	| 0;

    r.gtudc = 1; // count up
    r.gtpr = max_time_point ().time_since_epoch ().count (); // counter period
  }


  // FIXME: this should use the current time scaling
  void set_counter (time_point val) { regs ().gtcnt = val.time_since_epoch ().count (); }
  void set_counter (duration val) { regs ().gtcnt = val.count (); }


  // set counter value and trigger callback function
  void set_trigger_a (duration counter_value, timer::trigger_callback_func_t f)
  {
    if (f != nullptr)
    {
      m_gtci_a_func = std::move (f);
      std::atomic_signal_fence (std::memory_order_release);
      gtci_a_isr_t::enable (gtci_a_interrupt_type, gtci_a_interrupt_priority);
      regs ().gtintad |= gtinta;
    }
    else
    {
      gtci_a_isr_t::disable ();
      m_gtci_a_func = &timer::empty_func;
      std::atomic_signal_fence (std::memory_order_release);
      regs ().gtintad &= (uint16_t)~gtinta;
    }

    // FIXME: this should use the current time scaling
    regs ().gtccra = counter_value.count ();
  }


  // set counter value only.  keep trigger callback function as it is.
  // this can be used to use the trigger for counter-clearing only (no callback)
  // or to modify the current counter value.
  void set_trigger_a (duration counter_value)
  {
    // FIXME: this should use the current time scaling
    regs ().gtccra = counter_value.count ();
  }

  // explicitly enable the trigger point.  this is useful in combination with
  // DTC scripts which don't set or use the CPU callback function.
  void set_trigger_a (duration counter_value, timer::trigger_enable_tag)
  {
    gtci_a_isr_t::enable (gtci_a_interrupt_type, gtci_a_interrupt_priority);
    regs ().gtintad |= gtinta;

    // FIXME: this should use the current time scaling
    regs ().gtccra = counter_value.count ();
  }

  // disable the trigger point and clear the trigger callback function.
  void set_trigger_a (timer::trigger_disable_tag)
  {
    gtci_a_isr_t::disable ();
    regs ().gtintad &= ~gtinta;
  }

  // trigger_a hardware register.  e.g. for using in DTC instructions
  auto& trigger_a_hwreg (void) { return regs ().gtccra; }

  // FIXME: add another trigger point

  auto& counter_hwreg (void) { return regs ().gtcnt; }

  void start (void)
  {
    // FIXME: must use 16-bit access for gtstr.
    // must use atomic 8-bit shadow variable to ensure atomic access.
    auto i = dev::this_cpu::save_disable_interrupts ();
    common_regs ().gtstr |= 1 << (ChannelNumber % 4u);
    dev::this_cpu::restore_interrupts (i);
  }

  void stop (void)
  {
    auto i = dev::this_cpu::save_disable_interrupts ();
    common_regs ().gtstr &= (uint16_t)~(1 << (ChannelNumber % 4u));
    dev::this_cpu::restore_interrupts (i);
  }

  bool running (void) const
  {
    return (common_regs ().gtstr & (1 << (ChannelNumber % 4u))) != 0;
  }

  [[gnu::cold]] hw_chn_inst (void)
  {
    gtci_a_isr_t (this);
    gtci_b_isr_t (this);
    gtci_v_isr_t (this);
    gtci_u_isr_t (this);

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
      // disable all interrupts
      regs ().gtintad = 0;

      gtci_a_isr_t::disable ();
      gtci_b_isr_t::disable ();
      gtci_v_isr_t::disable ();
      gtci_u_isr_t::disable ();
    }

    ModuleEnableDisableFunc () (val);
  }

private:
  struct common_regs_t
  {
    dev::hw_reg_rw<uint16_t> gtstr;
    dev::hw_reg_rw<uint16_t> nfcr;
    dev::hw_reg_rw<uint16_t> gthscr;
    dev::hw_reg_rw<uint16_t> gthccr;
    dev::hw_reg_rw<uint16_t> gthssr;
    dev::hw_reg_rw<uint16_t> gthpsr;
    dev::hw_reg_rw<uint16_t> gtwp;
    dev::hw_reg_rw<uint16_t> gtsync;
    dev::hw_reg_rw<uint16_t> gtetint;
    dev::hw_reg_rw<uint16_t> res0;
    dev::hw_reg_rw<uint16_t> gtbdr;
    dev::hw_reg_rw<uint16_t> res1;
    dev::hw_reg_rw<uint16_t> gtswp;
  };

  static_assert (sizeof (common_regs_t) == 0x18 + 2);
  static constexpr common_regs_t& common_regs (void) { return *(common_regs_t*)CommonRegBaseAddr; }


  struct regs_t
  {
    dev::hw_reg_rw<uint16_t> gtior;
    dev::hw_reg_rw<uint16_t> gtintad;
    dev::hw_reg_rw<uint16_t> gtcr;
    dev::hw_reg_rw<uint16_t> gtber;
    dev::hw_reg_rw<uint16_t> gtudc;
    dev::hw_reg_rw<uint16_t> gtitc;
    dev::hw_reg_r<uint16_t> gtst;
    dev::hw_reg_rw<uint16_t> gtcnt;
    dev::hw_reg_rw<uint16_t> gtccra;
    dev::hw_reg_rw<uint16_t> gtccrb;
    dev::hw_reg_rw<uint16_t> gtccrc;
    dev::hw_reg_rw<uint16_t> gtccrd;
    dev::hw_reg_rw<uint16_t> gtccre;
    dev::hw_reg_rw<uint16_t> gtccrf;
    dev::hw_reg_rw<uint16_t> gtpr;
    dev::hw_reg_rw<uint16_t> gtpbr;
    dev::hw_reg_rw<uint16_t> gtpdbr;
    dev::hw_reg_rw<uint16_t> res0;
    dev::hw_reg_rw<uint16_t> gtadtra;
    dev::hw_reg_rw<uint16_t> gtadtbra;
    dev::hw_reg_rw<uint16_t> gtadtdbra;
    dev::hw_reg_rw<uint16_t> res1;
    dev::hw_reg_rw<uint16_t> gtadtrb;
    dev::hw_reg_rw<uint16_t> gtadtbrb;
    dev::hw_reg_rw<uint16_t> gtadtdbrb;
    dev::hw_reg_rw<uint16_t> res2;
    dev::hw_reg_rw<uint16_t> gtoncr;
    dev::hw_reg_rw<uint16_t> gtdtcr;
    dev::hw_reg_rw<uint16_t> gtdvu;
    dev::hw_reg_rw<uint16_t> gtdvd;
    dev::hw_reg_rw<uint16_t> gtdbu;
    dev::hw_reg_rw<uint16_t> gtdbd;
    dev::hw_reg_r<uint16_t> gtsos;
    dev::hw_reg_rw<uint16_t> gtsotr;
  };

  static_assert (sizeof (regs_t) == 0x44);
  static constexpr regs_t& regs (void) { return *(regs_t*)RegBaseAddr; }

  static constexpr unsigned int gtinta = 1 << 0;
  static constexpr unsigned int gtintb = 1 << 1;
  static constexpr unsigned int gtintc = 1 << 2;
  static constexpr unsigned int gtintd = 1 << 3;
  static constexpr unsigned int gtinte = 1 << 4;
  static constexpr unsigned int gtintf = 1 << 5;

  void gtci_a_func (void) { assume_always_true (m_gtci_a_func); m_gtci_a_func (); }
  void gtci_b_func (void) { assume_always_true (m_gtci_b_func); m_gtci_b_func (); }
  void gtci_v_func (void) { assume_always_true (m_gtci_v_func); m_gtci_v_func (); }
  void gtci_u_func (void) { assume_always_true (m_gtci_u_func); m_gtci_u_func (); }

public:
  typedef interrupt::connected_isr<gtci_a_interrupt_line,
	interrupt::func<decltype (&hw_chn_inst::gtci_a_func), &hw_chn_inst::gtci_a_func>> gtci_a_isr_t;

  typedef interrupt::connected_isr<gtci_b_interrupt_line,
	interrupt::func<decltype (&hw_chn_inst::gtci_b_func), &hw_chn_inst::gtci_b_func>> gtci_b_isr_t;

  typedef interrupt::connected_isr<gtci_v_interrupt_line,
	interrupt::func<decltype (&hw_chn_inst::gtci_v_func), &hw_chn_inst::gtci_v_func>> gtci_v_isr_t;

  typedef interrupt::connected_isr<gtci_u_interrupt_line,
	interrupt::func<decltype (&hw_chn_inst::gtci_u_func), &hw_chn_inst::gtci_u_func>> gtci_u_isr_t;

private:
  timer::trigger_callback_func_t m_gtci_a_func = &timer::empty_func;
  timer::trigger_callback_func_t m_gtci_b_func = &timer::empty_func;
  timer::trigger_callback_func_t m_gtci_v_func = &timer::empty_func;
  timer::trigger_callback_func_t m_gtci_u_func = &timer::empty_func;
};


} // namespace rx_gpta
} // namespace dev
#endif // includeguard_rx_gpta_hpp_includeguard

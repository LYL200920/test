/*

MCB v2 board specific driver for the emergency stop trigger input.

*/

#ifndef includeguard_mcbv2_board_dev_emg_stop_inputs_includeguard
#define includeguard_mcbv2_board_dev_emg_stop_inputs_includeguard

#include <cstdint>
#include <bitset>
#include <array>
#include <utils/langcomp.hpp>
#include <dev/hwreg.hpp>
#include <dev/digital_io_port.hpp>
#include <dev/renesas/rx_gpio.hpp>

#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)

  #include <dev/renesas/rx63_interrupt.hpp>

#elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
      || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

  #include <dev/renesas/rx64_interrupt.hpp>

#endif


namespace dev
{

// ---------------------------------------------------------------------------
// the emg stop input is connected directly to the MCU input port
// and can also be used as an interrupt source.

class emg_stop_inputs final : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 1;

  using trigger_callback_func = digital_io_port::trigger_callback_func;
  using trigger_type = digital_io_port::trigger_type;

  static constexpr auto falling_edge = digital_io_port::falling_edge;
  static constexpr auto rising_edge = digital_io_port::rising_edge;
  static constexpr auto any_edge = digital_io_port::any_edge;


private:
  struct isr_func
  {
    static void invoke (void* p)
    {
      auto* thiss = (emg_stop_inputs*)p;
      if (thiss->m_clb)
	thiss->m_clb (read_1 (), 0);
    }
  };

  static bool read_1 (void)
  {
    // input is inverted so we test for zero, not for non-zero.
    return (dev::rx_gpio::pidr::p3.read () & (1 << 2)) == 0;
  }

  // if the input is inverted, the trigger edge type also has to be inverted
  // to keep the logical meaning of "rising edge" = signal goes from off to on.
  static constexpr interrupt::trigger_type
  convert_trigger_type (trigger_type t, bool invert)
  {
    t = invert ? digital_io_port::invert_trigger_type (t) : t;
    switch (t)
    {
      default: return interrupt::any_edge;
      case digital_io_port::falling_edge: return interrupt::falling_edge;
      case digital_io_port::rising_edge: return interrupt::rising_edge;
      case digital_io_port::any_edge: return interrupt::any_edge;
    }
  }

public:
  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    typedef interrupt::connected_isr<rx63_interrupt::line< rx63_interrupt::icu_irq2>, isr_func> isr_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
	|| defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
    typedef interrupt::connected_isr<rx64_interrupt::line< rx64_interrupt::icu_irq2>, isr_func> isr_t;
  #endif

  emg_stop_inputs (void)
  {
    // assume that the interrupt is initially disabled.
    // when a trigger callback function is set, the interrupt will be
    // enabled.
  }

  virtual void
  set_trigger_func (unsigned int port, trigger_type t,
		    trigger_callback_func f) override
  {
    auto tt = convert_trigger_type (t, true);

    switch (port)
    {
      default: return;
      case 0: set_trigger_func_1<isr_t> (tt, f); return;
    }
  }

  virtual bool read_port (unsigned int n) const override
  {
    return n <= port_count ? read_1 () : false;
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

  std::bitset<port_count> read (void) const
  {
    return { read_1 () };
  }

private:
  trigger_callback_func m_clb;

  template <typename Interrupt> void
  set_trigger_func_1 (interrupt::trigger_type t, trigger_callback_func f)
  {
    // interrupt lines have actually global state.  the connected_isr is
    // just a proxy object to access those and we don't need to keep it
    // around all the time, only when enabling/disabling interrupt lines.
    // thus we temporarily construct the object here.
    Interrupt i (this);
    if (f == nullptr)
      i.disable ();
    else
    {
      m_clb = f;
      soft_memory_fence ();
      i.enable (t, 6);  // priority is fixed to 6 for now.
    }
  }


};

} // namespace dev
#endif // includeguard_mcbv2_board_dev_emg_stop_inputs_includeguard

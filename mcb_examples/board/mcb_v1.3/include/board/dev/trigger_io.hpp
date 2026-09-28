/*

MCB v1.3 board specific driver for the trigger inputs and trigger outputs.

*/

#ifndef includeguard_mcbv13_board_dev_trigger_io_includeguard
#define includeguard_mcbv13_board_dev_trigger_io_includeguard

#include <cstdint>
#include <bitset>
#include <array>
#include <utils/langcomp.hpp>
#include <dev/hwreg.hpp>
#include <dev/digital_io_port.hpp>

#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)

  #include <dev/rx63_interrupt.hpp>

#elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
      || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

  #include <dev/rx64_interrupt.hpp>

#endif


namespace dev
{

// ---------------------------------------------------------------------------
// the 8 trigger inputs are connected directly to the MCU input ports
// and can also be used as interrupt sources.

class trigger_inputs : public digital_io_port::dev_if
{
  template <unsigned int N> struct isr_func
  {
    static void invoke (void* p)
    {
      trigger_inputs* thiss = (trigger_inputs*)p;
      if (thiss->m_clb[N])
	thiss->m_clb[N] (thiss->read ()[N], N);
    }
  };

public:
  static constexpr unsigned int port_count = 8;

  using trigger_callback_func = digital_io_port::trigger_callback_func;
  using trigger_type = digital_io_port::trigger_type;

  static constexpr auto falling_edge = digital_io_port::falling_edge;
  static constexpr auto rising_edge = digital_io_port::rising_edge;
  static constexpr auto any_edge = digital_io_port::any_edge;

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

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    typedef interrupt::connected_isr<rx63_interrupt::line< rx63_interrupt::icu_irq8>, isr_func<0>> isr0_t;
    typedef interrupt::connected_isr<rx63_interrupt::line< rx63_interrupt::icu_irq9>, isr_func<1>> isr1_t;
    typedef interrupt::connected_isr<rx63_interrupt::line< rx63_interrupt::icu_irq10>, isr_func<2>> isr2_t;
    typedef interrupt::connected_isr<rx63_interrupt::line< rx63_interrupt::icu_irq11>, isr_func<3>> isr3_t;
    typedef interrupt::connected_isr<rx63_interrupt::line< rx63_interrupt::icu_irq12>, isr_func<4>> isr4_t;
    typedef interrupt::connected_isr<rx63_interrupt::line< rx63_interrupt::icu_irq13>, isr_func<5>> isr5_t;
    typedef interrupt::connected_isr<rx63_interrupt::line< rx63_interrupt::icu_irq14>, isr_func<6>> isr6_t;
    typedef interrupt::connected_isr<rx63_interrupt::line< rx63_interrupt::icu_irq15>, isr_func<7>> isr7_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
	|| defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
    typedef interrupt::connected_isr<rx64_interrupt::line< rx64_interrupt::icu_irq8>, isr_func<0>> isr0_t;
    typedef interrupt::connected_isr<rx64_interrupt::line< rx64_interrupt::icu_irq9>, isr_func<1>> isr1_t;
    typedef interrupt::connected_isr<rx64_interrupt::line< rx64_interrupt::icu_irq10>, isr_func<2>> isr2_t;
    typedef interrupt::connected_isr<rx64_interrupt::line< rx64_interrupt::icu_irq11>, isr_func<3>> isr3_t;
    typedef interrupt::connected_isr<rx64_interrupt::line< rx64_interrupt::icu_irq12>, isr_func<4>> isr4_t;
    typedef interrupt::connected_isr<rx64_interrupt::line< rx64_interrupt::icu_irq13>, isr_func<5>> isr5_t;
    typedef interrupt::connected_isr<rx64_interrupt::line< rx64_interrupt::icu_irq14>, isr_func<6>> isr6_t;
    typedef interrupt::connected_isr<rx64_interrupt::line< rx64_interrupt::icu_irq15>, isr_func<7>> isr7_t;
  #endif

  static std::bitset<port_count> default_xor_mask (void)
  {
    return { };
  }

  trigger_inputs (void)
  {
    // assume that the interrupts are initially disabled.
    // when a trigger callback function is set, the interrupt will be
    // enabled.
    m_and_mask.set ();
    m_or_mask.reset ();
    m_xor_mask.reset ();
  }

  std::bitset<port_count> read (void) const
  {
    std::bitset<port_count> bits (port4_pidr.read ());
    return ((bits & m_and_mask) | m_or_mask) ^ m_xor_mask;
  }

  void write (const std::bitset<port_count>& /*val*/)
  {
    // writing an input has no effect.
  }

  std::bitset<port_count> and_mask (void) const { return m_and_mask; }
  std::bitset<port_count> or_mask (void) const { return m_or_mask; }
  std::bitset<port_count> xor_mask (void) const { return m_xor_mask; }

  void set_and_mask (std::bitset<port_count> val) { m_and_mask = val; }
  void set_or_mask (std::bitset<port_count> val) { m_or_mask = val; }
  void set_xor_mask (std::bitset<port_count> val) { m_xor_mask = val; }

  virtual void
  set_trigger_func (unsigned int port, trigger_type t,
		    trigger_callback_func f) override
  {
    auto tt = convert_trigger_type (t, m_xor_mask[port]);

    switch (port)
    {
      default: return;
      case 0: set_trigger_func_1<isr0_t, 0> (tt, f); return;
      case 1: set_trigger_func_1<isr1_t, 1> (tt, f); return;
      case 2: set_trigger_func_1<isr2_t, 2> (tt, f); return;
      case 3: set_trigger_func_1<isr3_t, 3> (tt, f); return;
      case 4: set_trigger_func_1<isr4_t, 4> (tt, f); return;
      case 5: set_trigger_func_1<isr5_t, 5> (tt, f); return;
      case 6: set_trigger_func_1<isr6_t, 6> (tt, f); return;
      case 7: set_trigger_func_1<isr7_t, 7> (tt, f); return;
    }
  }

  virtual bool read_port (unsigned int n) const override
  {
    return n <= port_count ? read ()[n] : false;
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

private:
  // address of P4 port register is the same on RX63, RX64, RX71.
  static constexpr hw_reg_r <uint8_t, const_addr<0x0008C044>> port4_pidr = { };

  std::bitset<port_count> m_and_mask;
  std::bitset<port_count> m_or_mask;
  std::bitset<port_count> m_xor_mask;

  std::array<trigger_callback_func, port_count> m_clb;

  template <typename Interrupt, unsigned int N> void
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
      m_clb[N] = f;
      soft_memory_fence ();
      i.enable (t, 6);  // priority is fixed to 6 for now.
    }
  }
};

// ---------------------------------------------------------------------------
// the 8 trigger outputs are connected to the 16 bit AD bus as CS7.
// any bus access in the CS7 area will latch the current data lines as
// trigger outputs.

class trigger_outputs : public digital_io_port::dev_if
{
public:
  static constexpr unsigned int port_count = 8;

  trigger_outputs (void)
  {
    m_and_mask.set ();
    m_or_mask.reset ();
    m_xor_mask.reset ();

    m_data_port_cache = 0;
    data_port = 0;
  }

  std::bitset<port_count> and_mask (void) const { return m_and_mask; }
  std::bitset<port_count> or_mask (void) const { return m_or_mask; }
  std::bitset<port_count> xor_mask (void) const { return m_xor_mask; }

  void set_and_mask (std::bitset<port_count> val) { m_and_mask = val; }
  void set_or_mask (std::bitset<port_count> val) { m_or_mask = val; }
  void set_xor_mask (std::bitset<port_count> val) { m_xor_mask = val; }

  // when reading the outputs, re-apply the AOX masks so that
  // write (read ()) will result in a no-operation.
  std::bitset<port_count> read (void) const
  {
    std::bitset<port_count> val (m_data_port_cache);

    // no need to apply the other masks.
    return val ^ m_xor_mask;
  }

  void write (std::bitset<port_count> val)
  {
    data_port = m_data_port_cache = make_raw_write_value (val).to_ulong ();
  }

  virtual bool read_port (unsigned int n) const override
  {
    return n <= port_count ? read ()[n] : false;
  }

  virtual void write_port (unsigned int n, bool val) override
  {
    if (n < port_count)
      write (read ().set (n, val));
  }

  digital_io_port operator [] (unsigned int n) { return { this, n }; }

  // allow direct output register writes from DTC.
  // if this is used, the AOX mask has to be computed separately.
  auto& outputs_hwreg (void) { return data_port; }

  std::bitset<port_count>
  make_raw_write_value (std::bitset<port_count> val) const
  {
    return ((val & m_and_mask) | m_or_mask) ^ m_xor_mask;
  }

private:
  // just use any address in the CS7 area.  because it's a write-only latch,
  // have to buffer the value in RAM for software read-back.

  static constexpr hw_reg_w <uint8_t, const_addr<0x01000000>> data_port = { };

  uint8_t m_data_port_cache;
  std::bitset<port_count> m_and_mask;
  std::bitset<port_count> m_or_mask;
  std::bitset<port_count> m_xor_mask;
};


} // namespace dev
#endif // includeguard_mcbv13_board_dev_trigger_io_includeguard

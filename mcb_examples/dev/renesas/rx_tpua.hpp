/*
  RX TPUa timer device

  each device has 6 channels.
  each timer channel has one 16 bit counter and 4 general registers
  which can be used to implement 4 compare-match trigger points.
  in addition some channels have overflow and underflow trigger points.
  each trigger point can trigger a CPU interrupt.  the first trigger point
  can trigger the DMAC.  at least the first two or more trigger points can
  trigger the DTC or A/D converter.

  the RX63 has two devices, which results in a total of 12 channels.
  however, the interrupts for the 2nd device are shared with the MTU.

  to use this class on different MCUs the interrupt lines are specified
  as template arguments.

  the peripheral clock parameter is the peripheral clock on which the TPU
  runs.  on RX63 there is only one peripheral clock PCLK.  on RX64 and RX71
  it uses PCLKB.

  to simplify driver implemenation, each timer channel is exposed as an
  individual device.  not all channels are equal in hardware, though.
  in particular the per-channel interrupt capabilities differ.


  using two trigger points a PWM pattern can be constructed.
  the first trigger point does not clear the counter and turns the pulse on.
  the second trigger point clears the counter.  both trigger points define
  the duty cycle and the cycle time.
          _____
  _______|     |
    A       B

  A: first trigger point
  B: second trigger point

  the first trigger point can be set up to do a DTC transfer to write an
  "on value" to a virtual output (memory variable) or output port.
  the second trigger point can be set up to do a DTC transfer to write an
  "off value".

  if the timer peripheral clock is 48 MHz, one tick is about 20.83 ns.
  with a 16 bit timer, this gives a max. period of 1365 usec, which is
  about 732 kHz.

  if the DTC is used, each transfer takes 13 cycles.  so the maximum DTC
  issue rate is ICLK / 13 = 96/13 = 7.38 MHz.

  for simplicity sake, expose only the first 2 trigger points.
  if needed, the driver can be extended later.
*/

#ifndef includeguard_rx_tpua_hpp_includeguard
#define includeguard_rx_tpua_hpp_includeguard

#include <dev/timer.hpp>
#include <dev/hwreg.hpp>
#include <utils/bits.hpp>
#include <utils/langcomp.hpp>

namespace dev
{

namespace rx_tpua
{

typedef std::function<void (void)> trigger_callback_func_t;


// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// the timer counting clock source selections of the TPU are assymetrical.
// not all channels have the same capabilities and the bit patterns for the
// selections are overlapping.  try to straighten this a little by mapping
// a common global set of options to channel-specific bit patterns.

enum tpsc_value_t
{
  pclk_1,
  pclk_4,
  pclk_16,
  pclk_64,
  pclk_256,
  pclk_1024,
  pclk_4096,

  ext_tclka_tclke,
  ext_tclkb_tclkf,
  ext_tclkc_tclkg,
  ext_tclkd_tclkh,

  tpu2_tpu8_tcnt,
  tpu5_tpu11_tcnt,
};

template <unsigned int TpuChannelNumber>
constexpr unsigned int tpsc_hw_value (tpsc_value_t);

template <unsigned int TpuChannelNumber>
constexpr tpsc_value_t tpsc_sw_value (unsigned int);

template <> inline constexpr unsigned int tpsc_hw_value<0> (tpsc_value_t v)
{
  switch (v)
  {
    case pclk_1: return 0b000;
    case pclk_4: return 0b001;
    case pclk_16: return 0b010;
    case pclk_64: return 0b011;

    case ext_tclka_tclke: return 0b100;
    case ext_tclkb_tclkf: return 0b101;
    case ext_tclkc_tclkg: return 0b110;
    case ext_tclkd_tclkh: return 0b111;

    // unsupported values just map to something
    default:
      return 0b000;
  };
}

template <> inline constexpr tpsc_value_t tpsc_sw_value<0> (unsigned int v)
{
  switch (v)
  {
    case 0b000: return pclk_1;
    case 0b001: return pclk_4;
    case 0b010: return pclk_16;
    case 0b011: return pclk_64;

    case 0b100: return ext_tclka_tclke;
    case 0b101: return ext_tclkb_tclkf;
    case 0b110: return ext_tclkc_tclkg;
    case 0b111: return ext_tclkd_tclkh;

    // unsupported values just map to something
    default:
      return pclk_1;
  };
};

template <> inline constexpr unsigned int tpsc_hw_value<6> (tpsc_value_t v)
{
  return tpsc_hw_value<0> (v);
}

template <> inline constexpr tpsc_value_t tpsc_sw_value<6> (unsigned int v)
{
  return tpsc_sw_value<0> (v);
}


template <> inline constexpr unsigned int tpsc_hw_value<1> (tpsc_value_t v)
{
  switch (v)
  {
    case pclk_1: return 0b000;
    case pclk_4: return 0b001;
    case pclk_16: return 0b010;
    case pclk_64: return 0b011;
    case pclk_256: return 0b110;

    case ext_tclka_tclke: return 0b100;
    case ext_tclkb_tclkf: return 0b101;

    case tpu2_tpu8_tcnt: return 0b111;

    // unsupported values just map to something
    default:
      return 0b000;
  };
}

template <> inline constexpr tpsc_value_t tpsc_sw_value<1> (unsigned int v)
{
  switch (v)
  {
    case 0b000: return pclk_1;
    case 0b001: return pclk_4;
    case 0b010: return pclk_16;
    case 0b011: return pclk_64;
    case 0b110: return pclk_256;

    case 0b100: return ext_tclka_tclke;
    case 0b101: return ext_tclkb_tclkf;

    case 0b111: return tpu2_tpu8_tcnt;

    // unsupported values just map to something
    default:
      return pclk_1;
  };
}

template <> inline constexpr unsigned int tpsc_hw_value<7> (tpsc_value_t v)
{
  return tpsc_hw_value<1> (v);
}

template <> inline constexpr tpsc_value_t tpsc_sw_value<7> (unsigned int v)
{
  return tpsc_sw_value<1> (v);
}

template <> inline constexpr unsigned int tpsc_hw_value<2> (tpsc_value_t v)
{
  switch (v)
  {
    case pclk_1: return 0b000;
    case pclk_4: return 0b001;
    case pclk_16: return 0b010;
    case pclk_64: return 0b011;
    case pclk_1024: return 0b111;

    case ext_tclka_tclke: return 0b100;
    case ext_tclkb_tclkf: return 0b101;
    case ext_tclkc_tclkg: return 0b110;

    // unsupported values just map to something
    default:
      return 0b000;
  };
}

template <> inline constexpr tpsc_value_t tpsc_sw_value<2> (unsigned int v)
{
  switch (v)
  {
    case 0b000: return pclk_1;
    case 0b001: return pclk_4;
    case 0b010: return pclk_16;
    case 0b011: return pclk_64;
    case 0b111: return pclk_1024;

    case 0b100: return ext_tclka_tclke;
    case 0b101: return ext_tclkb_tclkf;
    case 0b110: return ext_tclkc_tclkg;

    // unsupported values just map to something
    default:
      return pclk_1;
  };
}

template <> inline constexpr unsigned int tpsc_hw_value<8> (tpsc_value_t v)
{
  return tpsc_hw_value<2> (v);
}

template <> inline constexpr tpsc_value_t tpsc_sw_value<8> (unsigned int v)
{
  return tpsc_sw_value<2> (v);
}


template <> inline constexpr unsigned int tpsc_hw_value<3> (tpsc_value_t v)
{
  switch (v)
  {
    case pclk_1: return 0b000;
    case pclk_4: return 0b001;
    case pclk_16: return 0b010;
    case pclk_64: return 0b011;
    case pclk_256: return 0b110;
    case pclk_1024: return 0b101;
    case pclk_4096: return 0b111;

    case ext_tclka_tclke: return 0b100;

    // unsupported values just map to something
    default:
      return 0b000;
  };
}

template <> inline constexpr tpsc_value_t tpsc_sw_value<3> (unsigned int v)
{
  switch (v)
  {
    case 0b000: return pclk_1;
    case 0b001: return pclk_4;
    case 0b010: return pclk_16;
    case 0b011: return pclk_64;
    case 0b110: return pclk_256;
    case 0b101: return pclk_1024;
    case 0b111: return pclk_4096;

    case 0b100: return ext_tclka_tclke;

    // unsupported values just map to something
    default:
      return pclk_1;
  };
}

template <> inline constexpr unsigned int tpsc_hw_value<9> (tpsc_value_t v)
{
  return tpsc_hw_value<3> (v);
}

template <> inline constexpr tpsc_value_t tpsc_sw_value<9> (unsigned int v)
{
  return tpsc_sw_value<3> (v);
}

template <> inline constexpr unsigned int tpsc_hw_value<4> (tpsc_value_t v)
{
  switch (v)
  {
    case pclk_1: return 0b000;
    case pclk_4: return 0b001;
    case pclk_16: return 0b010;
    case pclk_64: return 0b011;
    case pclk_1024: return 0b110;

    case ext_tclka_tclke: return 0b100;
    case ext_tclkc_tclkg: return 0b101;

    case tpu5_tpu11_tcnt: return 0b111;

    // unsupported values just map to something
    default:
      return 0b000;
  };
}

template <> inline constexpr tpsc_value_t tpsc_sw_value<4> (unsigned int v)
{
  switch (v)
  {
    case 0b000: return pclk_1;
    case 0b001: return pclk_4;
    case 0b010: return pclk_16;
    case 0b011: return pclk_64;
    case 0b110: return pclk_1024;

    case 0b100: return ext_tclka_tclke;
    case 0b101: return ext_tclkc_tclkg;

    case 0b111: return tpu5_tpu11_tcnt;

    // unsupported values just map to something
    default:
      return pclk_1;
  };
}

template <> inline constexpr unsigned int tpsc_hw_value<10> (tpsc_value_t v)
{
  return tpsc_hw_value<4> (v);
}

template <> inline constexpr tpsc_value_t tpsc_sw_value<10> (unsigned int v)
{
  return tpsc_sw_value<4> (v);
}

template <> inline constexpr unsigned int tpsc_hw_value<5> (tpsc_value_t v)
{
  switch (v)
  {
    case pclk_1: return 0b000;
    case pclk_4: return 0b001;
    case pclk_16: return 0b010;
    case pclk_64: return 0b011;
    case pclk_256: return 0b110;

    case ext_tclka_tclke: return 0b100;
    case ext_tclkc_tclkg: return 0b101;
    case ext_tclkd_tclkh: return 0b111;

    // unsupported values just map to something
    default:
      return 0b000;
  };
}

template <> inline constexpr tpsc_value_t tpsc_sw_value<5> (unsigned int v)
{
  switch (v)
  {
    case 0b000: return pclk_1;
    case 0b001: return pclk_4;
    case 0b010: return pclk_16;
    case 0b011: return pclk_64;
    case 0b110: return pclk_256;

    case 0b100: return ext_tclka_tclke;
    case 0b101: return ext_tclkc_tclkg;
    case 0b111: return ext_tclkd_tclkh;

    // unsupported values just map to something
    default:
      return pclk_1;
  };
}

template <> inline constexpr unsigned int tpsc_hw_value<11> (tpsc_value_t v)
{
  return tpsc_hw_value<5> (v);
}

template <> inline constexpr tpsc_value_t tpsc_sw_value<11> (unsigned int v)
{
  return tpsc_sw_value<5> (v);
}

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

enum ckeg_value_t
{
  falling_edge = 0b00,
  rising_edge  = 0b01,
  any_edge = falling_edge | rising_edge
};


// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

enum cclr_value_t
{
  clear_disable = 0b000,
  by_trga = 0b001,
  by_trgb = 0b010,
  by_other_sync_channel = 0b011,
  by_trgc = 0b101,
  by_trgd = 0b110,
};

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

enum tmdr_mode_t
{
  normal = 0b0000,
  pwm_mode_1 = 0b0010,
  pwm_mode_2 = 0b0011,
  phase_counting_mode_1 = 0b0100,
  phase_counting_mode_2 = 0b0101,
  phase_counting_mode_3 = 0b0110,
  phase_counting_mode_4 = 0b0111
};

enum tmdr_icselb_t
{
  tiocb_pin = 0,
  tioca_pin = 1
};

enum tmdr_icseld_t
{
  tiocd_pin = 0,
  tiocc_pin = 1
};

class tmdr_t
{
public:
  constexpr tmdr_t (void) : m_value (0) { }
  explicit constexpr tmdr_t (uint8_t v) : m_value (v) { }

  constexpr uint8_t value (void) const { return m_value; }

  constexpr tmdr_mode_t mode (void) const { return (tmdr_mode_t)(m_value & 0b1111); }
  tmdr_t& set_mode (tmdr_mode_t val)
  {
    m_value = (m_value & 0b1111'0000) | (unsigned int)val;
    return *this;
  }

  constexpr bool buffer_operation_a (void) const { return utils::get_bit (m_value, 4); }
  tmdr_t& set_buffer_operation_a (bool val) { m_value = utils::set_bit (m_value, 4, val); return *this; }

  constexpr bool buffer_operation_b (void) const { return utils::get_bit (m_value, 5); }
  tmdr_t& set_buffer_operation_b (bool val) { m_value = utils::set_bit (m_value, 5, val); return *this; }

  constexpr tmdr_icselb_t trgb_input_capture_source (void) const { return (tmdr_icselb_t)utils::get_bit (m_value, 6); }
  tmdr_t& set_trgb_input_capture_source (tmdr_icselb_t val) { m_value = utils::set_bit (m_value, 6, (unsigned int)val); return *this; }

  constexpr tmdr_icseld_t grd_input_capture_source (void) const  { return (tmdr_icseld_t)utils::get_bit (m_value, 7); }
  tmdr_t& set_grd_input_capture_source (tmdr_icseld_t val) { m_value = utils::set_bit (m_value, 7, (unsigned int)val); return *this; }

private:
  uint8_t m_value;
};


// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -


template <uintptr_t RegBaseAddr,
	  unsigned int TpuChannelNumber,  // 0 - 5, needed for TSTR, TSYR, ... bits
	  uintptr_t TSTR_RegAddr,
	  uintptr_t TSYR_RegAddr,
	  uintptr_t NFCR_RegAddr,
	  unsigned int PeripheralClockHz,

	  typename TGI_A_InterruptLine,	// input capture/compare match
	  typename TGI_B_InterruptLine, // input capture/compare match
	  typename TGI_C_InterruptLine, // input capture/compare match
	  typename TGI_D_InterruptLine, // input capture/compare match
	  typename TCI_V_InterruptLine, // counter overflow
	  typename TCI_U_InterruptLine, // counter underflow

	  typename ModuleEnableDisableFunc>
class hw_chn_inst
{
public:
  typedef TGI_A_InterruptLine tgi_a_interrupt_line;
  static constexpr auto tgi_a_interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int tgi_a_interrupt_priority = interrupt::priority_8;

  typedef TGI_B_InterruptLine tgi_b_interrupt_line;
  static constexpr auto tgi_b_interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int tgi_b_interrupt_priority = interrupt::priority_8;

  typedef TGI_C_InterruptLine tgi_c_interrupt_line;
  static constexpr auto tgi_c_interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int tgi_c_interrupt_priority = interrupt::priority_8;

  typedef TGI_D_InterruptLine tgi_d_interrupt_line;
  static constexpr auto tgi_d_interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int tgi_d_interrupt_priority = interrupt::priority_8;

  typedef TCI_V_InterruptLine tci_v_interrupt_line;
  static constexpr auto tci_v_interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int tci_v_interrupt_priority = interrupt::priority_8;

  typedef TCI_U_InterruptLine tci_u_interrupt_line;
  static constexpr auto tci_u_interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int tci_u_interrupt_priority = interrupt::priority_8;


  // use the highest clock possible to represent time values of this timer.
  // if clock scaling is used, the counts are shifted.
  static constexpr unsigned int tick_frequency = PeripheralClockHz;

  typedef std::chrono::duration<int32_t, std::ratio<1, tick_frequency>> duration;
  typedef typename duration::rep rep;
  typedef typename duration::period period;
  typedef std::chrono::time_point<hw_chn_inst, duration> time_point;

  static time_point now (void)
  {
    // FIXME: this should use the current time scaling
    return time_point (duration (regs ().tcnt));
  }

  // we can't tie the time point to uint16 because if clock scaling is used,
  // we need to be able to represent much larger values.
  static constexpr time_point max_time_point (void)
  {
    // FIXME: this should use the current time scaling
    return time_point (duration (std::numeric_limits<uint16_t>::max ()));
  }

  // FIXME: this should use the current time scaling
  void set_counter (time_point val) { regs ().tcnt = val.time_since_epoch ().count (); }
  void set_counter (duration val) { regs ().tcnt = val.count (); }

  [[gnu::cold]] hw_chn_inst (void)
  {
    tgi_a_isr_t (this);
    tgi_b_isr_t (this);
    tgi_c_isr_t (this);
    tgi_d_isr_t (this);
    tci_v_isr_t (this);
    tci_u_isr_t (this);

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
      // disable all interrut requests in the timer unit
      regs ().tier = 0;

      tgi_a_isr_t::disable ();
      tgi_b_isr_t::disable ();
      tgi_c_isr_t::disable ();
      tgi_d_isr_t::disable ();
      tci_v_isr_t::disable ();
      tci_u_isr_t::disable ();
    }

    ModuleEnableDisableFunc () (val);
  }

  void set_simple_trigger_mode (void)
  {
    set_timer_control (timer_control ()
	.set_count_clock_type (pclk_1)
	.set_count_clock_edge_type (rising_edge)
	.set_counter_clear (clear_disable));

    set_timer_mode (tmdr_t ()
	.set_mode (normal)
	.set_buffer_operation_a (false)
	.set_buffer_operation_b (false));
  }

  // set counter value and trigger callback function
  void set_trigger_a (duration counter_value, timer::trigger_callback_func_t f)
  {
    if (f != nullptr)
    {
      m_tgi_a_func = std::move (f);
      std::atomic_signal_fence (std::memory_order_release);
      tgi_a_isr_t::enable (tgi_a_interrupt_type, tgi_a_interrupt_priority);
      regs ().tier |= tgiea | tier_reserved_set_bits;
    }
    else
    {
      tgi_a_isr_t::disable ();
      m_tgi_a_func = &timer::empty_func;
      std::atomic_signal_fence (std::memory_order_release);
      regs ().tier &= ~tgiea;
    }

    // FIXME: this should use the current time scaling
    regs ().trga = counter_value.count ();
  }

  // set counter value only.  keep trigger callback function as it is.
  // this can be used to use the trigger for counter-clearing only (no callback)
  // or to modify the current counter value.
  void set_trigger_a (duration counter_value)
  {
    // FIXME: this should use the current time scaling
    regs ().trga = counter_value.count ();
  }

  // explicitly enable the trigger point.  this is useful in combination with
  // DTC scripts which don't set or use the CPU callback function.
  void set_trigger_a (duration counter_value, timer::trigger_enable_tag)
  {
    tgi_a_isr_t::enable (tgi_a_interrupt_type, tgi_a_interrupt_priority);
    regs ().tier |= tgiea | tier_reserved_set_bits;

    // FIXME: this should use the current time scaling
    regs ().trga = counter_value.count ();
  }

  // disable the trigger point and clear the trigger callback function.
  void set_trigger_a (timer::trigger_disable_tag)
  {
    // trigger point can't be really disabled.  it's always there.
    // if trigger point a is used as counter clear, counter clearing is disabled.
    // clear the interrupt enable (and all associated DTC/DMAC triggers).
    tgi_a_isr_t::disable ();
    regs ().tier &= ~tgiea;
  }


  // trigger_a hardware register.  e.g. for using in DTC instructions
  auto& trigger_a_hwreg (void) { return regs ().trga; }

  void set_trigger_b (duration counter_value, timer::trigger_callback_func_t f)
  {
    if (f)
    {
      m_tgi_b_func = std::move (f);
      std::atomic_signal_fence (std::memory_order_release);
      tgi_b_isr_t::enable (tgi_b_interrupt_type, tgi_b_interrupt_priority);
      regs ().tier |= tgieb | tier_reserved_set_bits;
    }
    else
    {
      tgi_b_isr_t::disable ();
      m_tgi_b_func = &timer::empty_func;
      std::atomic_signal_fence (std::memory_order_release);
      regs ().tier &= ~tgieb;
    }

    // FIXME: this should use the current time scaling
    regs ().trgb = counter_value.count ();
  }

  void set_trigger_b (duration counter_value)
  {
    regs ().trgb = counter_value.count ();
  }

  void set_trigger_b (duration counter_value, timer::trigger_enable_tag)
  {
    tgi_b_isr_t::enable (tgi_b_interrupt_type, tgi_b_interrupt_priority);
    regs ().tier |= tgieb | tier_reserved_set_bits;

    // FIXME: this should use the current time scaling
    regs ().trgb = counter_value.count ();
  }

  void set_trigger_b (timer::trigger_disable_tag)
  {
    tgi_b_isr_t::disable ();
    regs ().tier &= ~tgieb;
  }

  // trigger_b hardware register.  e.g. for using in DTC instructions
  auto& trigger_b_hwreg (void) { return regs ().trgb; }

  auto& counter_hwreg (void) { return regs ().tcnt; }

  void start (void)
  {
    utils::atomic_set_bit (TpuChannelNumber % 8u, &TSTR);
  }

  void stop (void)
  {
    utils::atomic_clear_bit (TpuChannelNumber % 8u, &TSTR);
  }

  bool running (void) const
  {
    return utils::get_bit (*&TSTR, TpuChannelNumber % 8u);
  }



  // the timer control register value type depends on the timer channel
  // number.  so it has to be in here.
  class tcr_t
  {
  public:
    constexpr tcr_t (void) : m_value (0) { }
    explicit constexpr tcr_t (uint8_t value) : m_value (value) { }

    constexpr uint8_t value (void) const { return m_value; }

    constexpr tpsc_value_t count_clock_type (void) const
    {
      return tpsc_sw_value<TpuChannelNumber> (m_value & 0b111);
    }

    tcr_t& set_count_clock_type (tpsc_value_t v)
    {
      m_value = (m_value & 0b11111'000) | tpsc_hw_value<TpuChannelNumber> (v);
      return *this;
    }

    constexpr ckeg_value_t count_clock_edge_type (void) const
    {
      return (ckeg_value_t)((m_value >> 3) & 0b11);
    }

    tcr_t& set_count_clock_edge_type (ckeg_value_t v)
    {
      m_value = (m_value & 0b111'00'111) | ((unsigned int)v << 3);
      return *this;
    }

    constexpr cclr_value_t counter_clear (void) const
    {
      return (cclr_value_t)((m_value >> 5) & 0b111);
    }

    tcr_t& set_counter_clear (cclr_value_t v)
    {
      m_value = (m_value & 0b000'11111) | ((unsigned int)v << 5);
      return *this;
    }

  private:
    uint8_t m_value;
  };


  tcr_t timer_control (void) const { return tcr_t (regs ().tcr); }
  void set_timer_control (tcr_t val) { regs ().tcr = val.value (); }

  tmdr_t timer_mode (void) const { return tmdr_t (regs ().tmdr); }
  void set_timer_mode (tmdr_t val) { regs ().tmdr = val.value (); }

private:
  enum
  {
    tgiea = 0b00000001,
    tgieb = 0b00000010,
    tgiec = 0b00000100,
    tgied = 0b00001000,
    tciev = 0b00010000,
    tcieu = 0b00100000,
    ttge =  0b10000000,

    tier_reserved_set_bits = 0b01000000
  };

  struct regs_t
  {
    dev::hw_reg_rw<uint8_t> tcr;	// tpu0: 0x00088110
    dev::hw_reg_rw<uint8_t> tmdr;	// tpu0: 0x00088111
    dev::hw_reg_rw<uint8_t> tiorh;	// tpu0: 0x00088112
    dev::hw_reg_rw<uint8_t> tiorl;	// tpu0: 0x00088113
    dev::hw_reg_rw<uint8_t> tier;	// tpu0: 0x00088114
    dev::hw_reg_rw<uint8_t> tsr;	// tpu0: 0x00088115
    dev::hw_reg_rw<uint16_t> tcnt;	// tpu0: 0x00088116
    dev::hw_reg_rw<uint16_t> trga;	// tpu0: 0x00088118
    dev::hw_reg_rw<uint16_t> trgb;	// tpu0: 0x0008811A
    dev::hw_reg_rw<uint16_t> trgc;	// tpu0: 0x0008811C
    dev::hw_reg_rw<uint16_t> trgd;	// tpu0: 0x0008811E
  };

  static_assert (sizeof (regs_t) == 16, "");

  static constexpr regs_t& regs (void) { return *(regs_t*)RegBaseAddr; }

  static constexpr hw_reg_rw<uint8_t, const_addr<TSTR_RegAddr>> TSTR = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<TSYR_RegAddr>> TSYR = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<NFCR_RegAddr>> NFCR = { };

  void tgi_a_func (void) { assume_always_true (m_tgi_a_func); m_tgi_a_func (); }
  void tgi_b_func (void) { assume_always_true (m_tgi_b_func); m_tgi_b_func (); }
  void tgi_c_func (void) { assume_always_true (m_tgi_c_func); m_tgi_c_func (); }
  void tgi_d_func (void) { assume_always_true (m_tgi_d_func); m_tgi_d_func (); }
  void tci_v_func (void) { assume_always_true (m_tci_v_func); m_tci_v_func (); }
  void tci_u_func (void) { assume_always_true (m_tci_u_func); m_tci_u_func (); }

public:
  typedef interrupt::connected_isr<tgi_a_interrupt_line,
	interrupt::func<decltype (&hw_chn_inst::tgi_a_func), &hw_chn_inst::tgi_a_func>> tgi_a_isr_t;

  typedef interrupt::connected_isr<tgi_b_interrupt_line,
	interrupt::func<decltype (&hw_chn_inst::tgi_b_func), &hw_chn_inst::tgi_b_func>> tgi_b_isr_t;

  typedef interrupt::connected_isr<tgi_c_interrupt_line,
	interrupt::func<decltype (&hw_chn_inst::tgi_c_func), &hw_chn_inst::tgi_c_func>> tgi_c_isr_t;

  typedef interrupt::connected_isr<tgi_d_interrupt_line,
	interrupt::func<decltype (&hw_chn_inst::tgi_d_func), &hw_chn_inst::tgi_d_func>> tgi_d_isr_t;

  typedef interrupt::connected_isr<tci_v_interrupt_line,
	interrupt::func<decltype (&hw_chn_inst::tci_v_func), &hw_chn_inst::tci_v_func>> tci_v_isr_t;

  typedef interrupt::connected_isr<tci_u_interrupt_line,
	interrupt::func<decltype (&hw_chn_inst::tci_u_func), &hw_chn_inst::tci_u_func>> tci_u_isr_t;

private:
  timer::trigger_callback_func_t m_tgi_a_func = &timer::empty_func;
  timer::trigger_callback_func_t m_tgi_b_func = &timer::empty_func;
  timer::trigger_callback_func_t m_tgi_c_func = &timer::empty_func;
  timer::trigger_callback_func_t m_tgi_d_func = &timer::empty_func;
  timer::trigger_callback_func_t m_tci_v_func = &timer::empty_func;
  timer::trigger_callback_func_t m_tci_u_func = &timer::empty_func;
};


} // namespace rx_tpua
} // namespace dev
#endif // includeguard_rx_tpua_hpp_includeguard

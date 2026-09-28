/*

digital input port implementation for STM32-like GPIO

on those MCUs the GPIO inputs have to be routed into the EXTI external lines.
this has to be done outside somehow.  the EXTI interrupt also has to
be wired externally to the instance of the input port.

*/

#ifndef includeguard_dev_stm32f0_digital_input_hpp_includeguard
#define includeguard_dev_stm32f0_digital_input_hpp_includeguard

#include <dev/digital_io_port.hpp>
#include <dev/interrupt.hpp>

namespace dev
{

template < typename PortRegs, unsigned int PortBitNum, unsigned int ExtiBitNum,
           typename InterruptLine,
           enum digital_io_port::logic Logic = digital_io_port::positive_logic >
class stm32f0_digital_input
{
private:
  static constexpr PortRegs port (void) { return PortRegs (); }

  static constexpr uintptr_t exti_regbase = 0x40010400;
  static constexpr hw_reg_rw<uint32_t, const_addr<exti_regbase + 0x00>> exti_imr = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<exti_regbase + 0x04>> exti_emr = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<exti_regbase + 0x08>> exti_rtsr = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<exti_regbase + 0x0C>> exti_ftsr = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<exti_regbase + 0x10>> exti_swier = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<exti_regbase + 0x14>> exti_pr = { };

  // PA0, PB0, ... -> EXIT0
  // PA1, PB1, ... -> EXIT1
  // ....
  // number of used exti lines depends on the MCU

  void isr (void)
  {
    // some of the exti interrupts are mapped onto shared interrupt vectors
    // (multiplexed interrupts). when this function is called from the demultiplexer
    // it could be that the actual interrupt has not fired (but some other multiplexed
    // interrupt).  so we need to check that our interrupt actually fired.
    if (exti_pr & (1u << ExtiBitNum))
    {
      // clear pending interrupt flag
      exti_pr = (1u << ExtiBitNum);

      // FIXME: this doesn't eliminate the nullptr check in the generated code
      assume_always_true (m_clb_func != nullptr);
      m_clb_func (read (), 0);
    }
  }

public:
  using interrupt_line = InterruptLine;
  using isr_t = typename interrupt::connected_isr<interrupt_line,
	interrupt::func<decltype (&stm32f0_digital_input::isr), &stm32f0_digital_input::isr>>;

  // use highest priority for input triggers.
  static constexpr unsigned int interrupt_priority = dev::interrupt::max_priority;

  using trigger_callback_func = digital_io_port::trigger_callback_func;

private:
  trigger_callback_func m_clb_func;

public:

  stm32f0_digital_input (void)
  {
    isr_t (this);
  }

  // implement the functions of digital_io_port::read, ::write, ::set_trigger_func
  // if a real instance of digital_io_port is required, need to create
  // a wrapper object around it.
  //operator digital_io_port (void) const;

  bool read (void) const
  {
    if constexpr (Logic == digital_io_port::positive_logic)
      return (port ().idr & (1 << PortBitNum)) != 0;
    else
      return (port ().idr & (1 << PortBitNum)) == 0;
  }

  void write (bool) { }
  void set (void) { }
  void clear (void) { }

  template <typename F>
  void set_trigger_func (digital_io_port::trigger_type t, F&& f)
  {
    dev::interrupt::trigger_type tt = dev::interrupt::any_edge;

    if constexpr (Logic == digital_io_port::negative_logic)
      t = digital_io_port::invert_trigger_type (t);

    if (t == digital_io_port::falling_edge)
    {
      exti_rtsr &= ~(1u << ExtiBitNum);
      exti_ftsr |= (1u << ExtiBitNum);
      tt = dev::interrupt::falling_edge;
    }
    else if (t == digital_io_port::rising_edge)
    {
      exti_rtsr |= (1u << ExtiBitNum);
      exti_ftsr &= ~(1u << ExtiBitNum);
      tt = dev::interrupt::rising_edge;
    }
    else //if (t == digital_io_port::any_edge)
    {
      exti_rtsr |= (1u << ExtiBitNum);
      exti_ftsr |= (1u << ExtiBitNum);
      tt = dev::interrupt::any_edge;
    }

    // if the passed function is known to be non-null at compile time
    // (e.g. lambda), the compiler might complain about the nullptr check.
    // silence it.
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Waddress"
    if (f != nullptr)
    #pragma GCC diagnostic pop
    {
      m_clb_func = std::forward<F> (f);
      std::atomic_signal_fence (std::memory_order_release);

      exti_imr |= 1u << ExtiBitNum;
      isr_t::enable (tt, interrupt_priority);
    }
    else
      exti_imr &= ~(1u << ExtiBitNum);
  }

  void sync (void) { }
  enum digital_io_port::logic logic  (void) const { return Logic; }
};


} // namespace dev

#endif // includeguard_dev_stm32f0_digital_input_hpp_includeguard

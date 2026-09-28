
#ifndef includeguard_dev_virtual_interrupt_demux_hpp_includeguard
#define includeguard_dev_virtual_interrupt_demux_hpp_includeguard

#include <dev/interrupt.hpp>
#include <utils/tuple.hpp>

namespace dev
{

template <typename MuxedInterruptLine, unsigned int InterruptPriority,
	  typename... DemuxedInterruptLines>
class virtual_interrupt_demux
{
private:
  static void isr ()
  {
    // FIXME: this doesn't get inlined at all
    // maybe it's better to write a specialized EXTI demux (for STM32-like devices)
    utils::tuple_for_each (std::tuple<DemuxedInterruptLines...> (),
	[] (std::size_t, auto&& x)
	{
	  x.trigger ();
	});
  }

public:
  static constexpr unsigned int interrupt_priority = InterruptPriority;

  using interrupt_line = MuxedInterruptLine;
  using isr_t = typename interrupt::connected_isr<interrupt_line,
	interrupt::func<decltype (&virtual_interrupt_demux::isr), &virtual_interrupt_demux::isr>>;

  virtual_interrupt_demux (void) : m_isr (nullptr)
  {
    m_isr.enable (interrupt::any_edge, interrupt_priority);
  }

private:
  isr_t m_isr;
};

} // namespace dev

#endif // includeguard_dev_virtual_interrupt_demux_hpp_includeguard

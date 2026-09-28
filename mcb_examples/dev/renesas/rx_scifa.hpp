
// the SCIg (RX64, RX71) is register compatible with SCIc (RX63), but with
// slightly different capabilities.

#ifndef includeguard_dev_rx_scifa_hpp_includeguard
#define includeguard_dev_rx_scifa_hpp_includeguard

#include <cstdint>

#include <dev/interrupt.hpp>

#include <utils/value_range.hpp>
#include <utils/bits.hpp>

namespace dev
{

template <uintptr_t RegAddress, unsigned int SciNum,
	  unsigned int PeripheralClock, unsigned int MaxBitRate,
	  typename BRI_InterruptLine,
	  typename ERI_InterruptLine,
	  typename RXI_InterruptLine,
	  typename TXI_InterruptLine,
	  typename TEI_InterruptLine,
	  typename DRI_InterruptLine,
	  typename ModuleEnableDisableFunc >
class rx_scifa
{
public:
  // break or overrun
  typedef BRI_InterruptLine bri_interrupt_line;
  static constexpr interrupt::trigger_type bri_interrupt_type = interrupt::low_level;
  static constexpr unsigned int bri_interrupt_priority = interrupt::priority_7;

  // framing error or parity error
  typedef ERI_InterruptLine eri_interrupt_line;
  static constexpr interrupt::trigger_type eri_interrupt_type = interrupt::low_level;
  static constexpr unsigned int eri_interrupt_priority = interrupt::priority_7;

  // receive fifo data full
  typedef RXI_InterruptLine rxi_interrupt_line;
  static constexpr interrupt::trigger_type rxi_interrupt_type = interrupt::low_level;
  static constexpr unsigned int rxi_interrupt_priority = interrupt::priority_7;

  // transmit fifo data empty
  typedef TXI_InterruptLine txi_interrupt_line;
  static constexpr interrupt::trigger_type txi_interrupt_type = interrupt::low_level;
  static constexpr unsigned int txi_interrupt_priority = interrupt::priority_7;

  // transmit end
  typedef TEI_InterruptLine tei_interrupt_line;
  static constexpr interrupt::trigger_type tei_interrupt_type = interrupt::low_level;
  static constexpr unsigned int tei_interrupt_priority = interrupt::priority_7;

  // receive data ready
  typedef DRI_InterruptLine dri_interrupt_line;
  static constexpr interrupt::trigger_type dri_interrupt_type = interrupt::low_level;
  static constexpr unsigned int dri_interrupt_priority = interrupt::priority_7;


protected:
};

} // namespace dev
#endif // includeguard_dev_rx_scifa_hpp_includeguard

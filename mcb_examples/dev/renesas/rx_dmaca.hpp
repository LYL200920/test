/*

Renesas RX DMACA(a) driver.


on RX63 DMACA has only 4 channels.

on RX71M DMACA has 8 channels, but channel 4-7 can't trigger DTC,
only CPU interrupt.


each channel is basically independent of each other.
the hardware channel priority relationship is ignored in this case.

scatter/gather DMA transfers are implemented by using the DTC to walk
the pointer/size arrays.  this is only possible for channels 0-3, which have
non-multiplexed interrupts.  channels 4-7 have to be done by CPU.

*/

#ifndef includeguard_dev_rx_dmaca_hpp_includeguard
#define includeguard_dev_rx_dmaca_hpp_includeguard

#include <dev/hwreg.hpp>
#include <dev/interrupt.hpp>
#include <dev/dma.hpp>
#include <utils/bits.hpp>
#include <dev/renesas/rx_dtca.hpp>

namespace dev
{
namespace rx_dmaca
{


static constexpr hw_reg_rw<uint8_t, const_addr<0x00082200>> dmast = { };
static constexpr hw_reg_r<uint8_t, const_addr<0x00082204>> dmac74_dmist = { };

template <unsigned int ChannelNumber, uintptr_t RegBaseAddr,
	  typename InterruptLine,
	  template<unsigned int> class ModuleEnableFunc,
	  typename DtcFunc >
class hw_chn_inst final : public dev::dma::channel
{
public:
  using dtc = std::remove_reference_t<std::invoke_result_t<DtcFunc>>;
  static constexpr auto& dtc_inst (void) { return std::invoke (DtcFunc ()); }

  typedef InterruptLine interrupt_line;
  static constexpr auto interrupt_type = interrupt::rising_edge;
  static constexpr unsigned int interrupt_priority = interrupt::priority_9;
  static constexpr bool has_offset_register = ChannelNumber == 0;

  [[gnu::cold]] hw_chn_inst (void)
  : m_isr (this)
  {
    set_device_enable (true);
  }

  [[gnu::cold]] ~hw_chn_inst (void)
  {
    set_device_enable (false);
  }

  void set_device_enable (bool val)
  {
    if (val)
      m_isr.enable (interrupt_type, interrupt_priority);
    else
      m_isr.disable ();

    ModuleEnableFunc<ChannelNumber> () (val);
  }

  virtual void lock (void) override { }
  virtual void unlock (void) override { }
  virtual bool try_lock (void) override { return false; }

  virtual std::future<dev::dma::transfer_status>
  transfer (const dev::dma::transfer_desc& desc,
	    std::function<void (void)> completion_clb) override
  {
    return { };
  }

private:
  struct regs_t
  {					// DMAC0 addr
    dev::hw_reg_rw<uint32_t> dmsar;	// 0x00082000
    dev::hw_reg_rw<uint32_t> dmdar;	// 0x00082004
    dev::hw_reg_rw<uint32_t> dmcra;	// 0x00082008
    dev::hw_reg_rw<uint16_t> dmcrb;	// 0x0008200C
    dev::hw_reg_rw<uint16_t> dmtmd;	// 0x00082010
    dev::hw_reg_rw<uint8_t> res11;
    dev::hw_reg_rw<uint8_t> res12;
    dev::hw_reg_rw<uint8_t> dmint;	// 0x00082013
    dev::hw_reg_rw<uint16_t> dmamd;	// 0x00082014
    dev::hw_reg_rw<uint32_t> dmofr;	// 0x00082018 (DMAC0 only)
    dev::hw_reg_rw<uint8_t> dmcnt;	// 0x0008201C
    dev::hw_reg_rw<uint8_t> dmreq;	// 0x0008201D
    dev::hw_reg_rw<uint8_t> dmsts;	// 0x0008201E
    dev::hw_reg_rw<uint8_t> dmcsl;	// 0x0008201F
  };

  static_assert (sizeof (regs_t) == 32, "");

  static constexpr regs_t& regs (void) { return *(regs_t*)RegBaseAddr; }

  static constexpr dev::hw_reg_rw<uint8_t, const_addr<0x00087400 + ChannelNumber * 4>> icu_dmrsr = { };

  void isr_func (void)
  {

  }

public:
  typedef interrupt::connected_isr<interrupt_line,
	interrupt::func<decltype (&hw_chn_inst::isr_func), &hw_chn_inst::isr_func>> isr_t;

private:
  isr_t m_isr;
};


} // namespace rx_dmaca
} // namespace dev
#endif // includeguard_dev_rx_dmaca_hpp_includeguard

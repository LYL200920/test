
#ifndef includeguard_dev_rx_sci_spi_hpp_includeguard
#define includeguard_dev_rx_sci_spi_hpp_includeguard

#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <atomic>

#include <utils/langcomp.hpp>

#include <dev/spi.hpp>
#include <dev/renesas/rx_gpio.hpp>
#include <dev/renesas/rx_dtca.hpp>
#include <dev/dma.hpp>

namespace dev
{

template <typename DtcFunc,
	  typename SciDev, typename SS0,
	  typename SS1 = void,
	  typename SS2 = void,
	  typename SS3 = void,
	  typename SS4 = void,
	  typename SS5 = void>

class rx_sci_spi_master final : public SciDev, public spi::master
{
public:
  using sci_dev = SciDev;

  using dtc = std::remove_reference_t<std::invoke_result_t<DtcFunc>>;
  static constexpr auto& dtc_inst (void) { return std::invoke (DtcFunc ()); }

  using dtc_insn = typename dtc::insn;

   static constexpr unsigned int max_bitrate = SciDev::max_bitrate;

  [[gnu::cold]] rx_sci_spi_master (dma::dispatcher& dma_dispatcher)
  : m_dma_dispatcher (dma_dispatcher),
    m_txi_isr (this), m_rxi_isr (this), m_tei_isr (this), m_eri_isr (this)
  {

  }

  virtual std::shared_ptr<spi::prepared_transfer>
  prepare_transfer (const spi::transfer_config& cfg) const override
  {
    /*
	cfg.bitrate_hz			SMR.CKS, BRR, MDDR, SEMR.BRME
	cfg.phase.bitrate_div

	cfg.phase.cs_line			software
	cfg.phase.transfer_count		software
	cfg.phase.clock_delay_cycles		software (only roughly with DTC/DMA)
	cfg.phase.cs_delay_cycles		software (only roughly with DTC/DMA)
	cfg.phase.next_phase_delay_cycles	software (only roughly with DTC/DMA)

	cfg.phase.allow_trailing_clock		software
	cfg.phase.data_dir			software
	cfg.phase.clock_phase			SPMR.CKPH
	cfg.phase.clock_polarity		SPMR.CKPOL
	cfg.phase.byte_order			software
	cfg.phase.bit_order			SCMR.SDIR
	cfg.phase.lane_mode			software + TX/RX control
						+ DTC/DMAC transfer control
						(single_lane_full_duplex, single_lane_half_duplex only)

	cfg.phase.data_idle_output_mode		software (only output_low, output_high)
						default: SCR.TE = 0: Txd = high-z
						switch pin to GPIO when TE becomes 0 (TIE ISR)

	cfg.phase.data2_unused_output_mode	not supported / ignored
	cfg.phase.data3_unused_output_mode	not supported / ignored
	cfg.phase.cs_idle_output_mode		software
	cfg.phase.clk_idle_output_mode		SPMR.SSE, SCR.TE

	generic SS type:
		invoke SS::set by CPU function (in DMA start callback)
		use DMA for SPI transfer
		invoke SS::clear in CPU completion function

	SS type dev::rx_gpio::exclusive_output_port:
		could use DTC, but it is actually slower than using the CPU.
		the CPU has to be used to be used to setup the
		SCI and DMA registers, so it can do also one GPIO write (CS on).
		the CPU will run an ISR for the DMA completion, so it can
		also do one GPIO write (CS off).

		for multi-phase transfers could use the DTC to rewrite SCI
		registers and start DMA.  however, each DTC instruction takes
		about 18 ICLKs, which makes it a lot slower than the CPU.

	when using DTC:
	  10 MBit = 1250000 bytes/sec
		  = 1250000 * 18 ICLK = 22.5 MHz (9.3% @ 240 MHz)


	scatter/gather buffers with DTC:
		use conditional insn to modify the address + count of the
		DTC insn that does the actual transfer

	scatter/gather with DMAC:
		DMAC end interrupt triggers DTC, DTC rewrites DMAC address
		and transfer count registers and DMCNT.DTE = 1 to re-start
		the transfer.
		this should be done automatically by the DMAC driver.
		DTC insn time: (2 + 1) + (4*2+1) + (3*2) + (2+1) + (2) + 2 = 25 ICLK

    */

    return nullptr;
  }

  virtual std::future<spi::transfer_status>
  transfer (const std::shared_ptr<spi::prepared_transfer>& tr,
	    spi::buffer_sequence_t&& send_buf,
	    spi::buffer_sequence_t&& recv_buf,
	    std::function<void (void)> completion_clb = nullptr) override
  {
    /*
	request a dma channel lock.  if it is not available immediately,
	it will do a callback later.
	once we have the channel lock
	  * setup the write the DMA transfer on the DMA channel and start the
	    transfer (will be triggered by SCI)
	  * assert the CS line
	  * start SCI rx/tx operation

	for send and receive need 2 DMA channel locks.

	for receive-only phases/transfers, disable CTS and set RE = 1 will
	start clock output.  no need to transmit dummy data.

	if there is no send buffer or send data is disabled, setup fixed
	source address DMA transfer to send a fixed byte high/low depending
	on use cfg.data_idle_output_mode.

	if there is no recv buffer or recv data is disabled, do not enable
	SCI RX and do not request a dma channel lock for receiving data.


	if the data buffers are small, don't bother setting up DMA
	transfers, it will be slower than the actual transfer itself.

	if there are multiple transfer phases and the data buffers are small,
	and CS-disable timing is not critical, and SS lines are all exclusive
	output ports, can make one long DTC insn list to do everything without
	interrupting the CPU.

	- cpu: write cs port (enable cs)
	- cpu: start sci tx on cpu (will trigger an txi)
	  - dtc (normal transfer): copy bytes for phase X
	  - dtc (cond. chain, normal transfer): write cs port (disable cs)        | if CS port is the same,
	  - dtc (always chain, normal transfer): write cs port (enable next cs)   | can also use only 1 write
	  - dtc (always chain, block transfer): re-write 1st dtc insn for next phase
    */

    auto t = std::make_shared<transfer_impl> (
	std::static_pointer_cast<prepared_transfer_impl> (tr),
	std::move (send_buf),
	std::move (recv_buf),
	std::move (completion_clb));



    return { };
  }


private:
  struct prepared_transfer_impl : spi::prepared_transfer
  {

  };

  struct transfer_impl : std::enable_shared_from_this<transfer_impl>
  {
    transfer_impl (std::shared_ptr<prepared_transfer_impl>&& pt_,
		   spi::buffer_sequence_t&& send_buf_,
		   spi::buffer_sequence_t&& recv_buf_,
		   std::function<void (void)>&& completion_clb_)
    : pt (std::move (pt_)),
      send_buf (std::move (send_buf_)),
      recv_buf (std::move (recv_buf_)),
      completion_clb (std::move (completion_clb_))
    {
    }

    std::shared_ptr<prepared_transfer_impl> pt;

    spi::buffer_sequence_t send_buf;
    spi::buffer_sequence_t recv_buf;

    std::function<void (void)> completion_clb;
  };


  void txi (void)
  {
  }

  void rxi (void)
  {
  }

  void tei (void)
  {
    // de-assert the CS line for the current transfer.

  }

  void eri (void)
  {
  }


public:
  typedef interrupt::connected_isr<typename sci_dev::txi_interrupt_line,
	interrupt::func<decltype (&rx_sci_spi_master::txi), &rx_sci_spi_master::txi>> txi_isr_t;

  typedef interrupt::connected_isr<typename sci_dev::rxi_interrupt_line,
	interrupt::func<decltype (&rx_sci_spi_master::rxi), &rx_sci_spi_master::rxi>> rxi_isr_t;

  typedef interrupt::connected_isr<typename sci_dev::tei_interrupt_line,
	interrupt::func<decltype (&rx_sci_spi_master::tei), &rx_sci_spi_master::tei>> tei_isr_t;

  typedef interrupt::connected_isr<typename sci_dev::eri_interrupt_line,
	interrupt::func<decltype (&rx_sci_spi_master::eri), &rx_sci_spi_master::eri>> eri_isr_t;

private:
  dma::dispatcher& m_dma_dispatcher;
  txi_isr_t m_txi_isr;
  rxi_isr_t m_rxi_isr;
  tei_isr_t m_tei_isr;
  eri_isr_t m_eri_isr;
};

} // namespace dev
#endif // includeguard_dev_rx_sci_spi_hpp_includeguard


/*

a usart that is implemented using an RX SCIF device.

*/

#ifndef includeguard_dev_rx_scif_usart_hpp_includeguard
#define includeguard_dev_rx_scif_usart_hpp_includeguard

#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <atomic>

#include <utils/langcomp.hpp>

#include <dev/usart.hpp>
#include <dev/interrupt.hpp>
#include <dev/generic_transceiver.hpp>

namespace dev
{

template <typename ScifDev,
	  unsigned int TxFifoSizeBytes, unsigned int RxFifoSizeBytes,
	  typename ExternalTransceiver = default_dummy_transceiver>
class rx_scif_usart final : public usart, public ScifDev, public ExternalTransceiver
{
public:
  typedef ScifDev scif_dev;
  typedef ExternalTransceiver external_transceiver;

  static constexpr unsigned int tx_buffer_size = TxFifoSizeBytes;
  static constexpr unsigned int rx_buffer_size = RxFifoSizeBytes;

  static constexpr unsigned int max_bitrate = ScifDev::max_bitrate;

  [[gnu::cold]] rx_scif_usart (void)
  : m_bri_isr (this), m_eri_isr (this), m_rxi_isr (this), m_txi_isr (this),
    m_tei_isr (this), m_dri_isr (this)
  {
//    m_tx_wr_pos = m_tx_rd_pos = 0;
//    m_rx_wr_pos = m_rx_rd_pos = 0;

//    m_tx_lock = 0;

    m_rx_data_clb = default_rx_data_clb;
    m_rx_data_clb_user_p = nullptr;


    // some compile-time sanity checks
    static_assert (std::is_same<unsigned char, uint8_t>::value, "");

/*
    static_assert (utils::is_pow2 (tx_buffer_size), "");
    static_assert (std::numeric_limits<decltype (m_tx_wr_pos)>::max () >= tx_buffer_size - 1, "");
    static_assert (std::numeric_limits<decltype (m_tx_rd_pos)>::max () >= tx_buffer_size - 1, "");

    static_assert (utils::is_pow2 (rx_buffer_size), "");
    static_assert (std::numeric_limits<decltype (m_rx_wr_pos)>::max () >= rx_buffer_size - 1, "");
    static_assert (std::numeric_limits<decltype (m_rx_rd_pos)>::max () >= rx_buffer_size - 1, "");
*/

    // ScifDev ctor will enable the device (disable module standby) but not the
    // interrupts.
    m_bri_isr.enable (scif_dev::bri_interrupt_type, scif_dev::bri_interrupt_priority);
    m_eri_isr.enable (scif_dev::eri_interrupt_type, scif_dev::eri_interrupt_priority);
    m_rxi_isr.enable (scif_dev::rxi_interrupt_type, scif_dev::rxi_interrupt_priority);
    m_txi_isr.enable (scif_dev::txi_interrupt_type, scif_dev::txi_interrupt_priority);
    m_tei_isr.enable (scif_dev::tei_interrupt_type, scif_dev::tei_interrupt_priority);
    m_dri_isr.enable (scif_dev::dri_interrupt_type, scif_dev::dri_interrupt_priority);

    // FIXME: implement remaining initialization stuff
  }


  virtual void reset_receiver (void) noexcept override
  {
    // FIXME: implement
  }

  virtual bool is_valid (void) const noexcept override
  {
    return true;
  }

  virtual struct config config (void) const noexcept override
  {
    // FIXME: implement
    return { };
  }

  virtual bool set_config (const struct config& val) noexcept override
  {
    // FIXME: implement
    return false;
  }

  virtual void write (const void* data, unsigned int byte_count) noexcept override
  {
    // FIXME: implement
  }

  void write (unsigned char data)
  {
    // FIXME: implement
  }

  virtual unsigned int read (void* data_out, unsigned int max_byte_count) noexcept override
  {
    // FIXME: implement
    return 0;
  }

  virtual void set_recv_clb (void f (void* user_p, unsigned int byte_count), void* user_p) noexcept override
  {
    if (f == nullptr)
      m_rx_data_clb = default_rx_data_clb;
    else
    {
      m_rx_data_clb = f;
      m_rx_data_clb_user_p = user_p;
    }

  }

  virtual void reset_rx_buffer (void) noexcept override
  {
    // FIXME: implement
  }

  using usart::set_recv_clb;

  virtual struct generic_io::buffer_stat buffer_stat (void) const noexcept
  {
    // FIXME: implement
    return { };
  }

  struct buffer_stat
  {
    const uint8_t* ptr0;
    unsigned int count0;

    const uint8_t* ptr1;
    unsigned int count1;

    unsigned int count;
  };

  struct buffer_stat tx_buffer_stat (void) const
  {
    // FIXME: implement

    return { };
  }

  struct buffer_stat rx_buffer_stat (void) const
  {
    // FIXME: implement

    return { };
  }


private:
  void bri (void)
  {
    // FIXME: implement
  }

  void eri (void)
  {
    // FIXME: implement
  }

  void rxi (void)
  {
    // FIXME: implement
  }

  void txi (void)
  {
    // FIXME: implement
  }

  void tei (void)
  {
    // FIXME: implement
  }

  void dri (void)
  {
    // FIXME: implement
  }

  static void default_rx_data_clb (void*, unsigned int)
  {
  }

public:
  typedef interrupt::connected_isr<typename scif_dev::bri_interrupt_line,
	interrupt::func<decltype (&rx_scif_usart::bri), &rx_scif_usart::bri>> bri_isr_t;

  typedef interrupt::connected_isr<typename scif_dev::eri_interrupt_line,
	interrupt::func<decltype (&rx_scif_usart::eri), &rx_scif_usart::eri>> eri_isr_t;

  typedef interrupt::connected_isr<typename scif_dev::rxi_interrupt_line,
	interrupt::func<decltype (&rx_scif_usart::rxi), &rx_scif_usart::rxi>> rxi_isr_t;

  typedef interrupt::connected_isr<typename scif_dev::txi_interrupt_line,
	interrupt::func<decltype (&rx_scif_usart::txi), &rx_scif_usart::txi>> txi_isr_t;

  typedef interrupt::connected_isr<typename scif_dev::tei_interrupt_line,
	interrupt::func<decltype (&rx_scif_usart::tei), &rx_scif_usart::tei>> tei_isr_t;

  typedef interrupt::connected_isr<typename scif_dev::dri_interrupt_line,
	interrupt::func<decltype (&rx_scif_usart::dri), &rx_scif_usart::dri>> dri_isr_t;

  bri_isr_t& bri_isr (void) { return m_bri_isr; }
  eri_isr_t& eri_isr (void) { return m_eri_isr; }
  rxi_isr_t& rxi_isr (void) { return m_rxi_isr; }
  txi_isr_t& txi_isr (void) { return m_txi_isr; }
  tei_isr_t& tei_isr (void) { return m_tei_isr; }
  dri_isr_t& dri_isr (void) { return m_dri_isr; }

private:
  void (*m_rx_data_clb)(void*, unsigned int);
  void* m_rx_data_clb_user_p;

  bri_isr_t m_bri_isr;
  eri_isr_t m_eri_isr;
  rxi_isr_t m_rxi_isr;
  txi_isr_t m_txi_isr;
  tei_isr_t m_tei_isr;
  dri_isr_t m_dri_isr;
};

} // namespace dev
#endif // includeguard_dev_rx_scif_usart_hpp_includeguard

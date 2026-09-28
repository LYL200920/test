
/*

a usart that is implemented using an RX SCI device.

*/

#ifndef includeguard_dev_rx_sci_usart_hpp_includeguard
#define includeguard_dev_rx_sci_usart_hpp_includeguard

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

template <typename SciDev,
	  unsigned int TxFifoSizeBytes, unsigned int RxFifoSizeBytes,
	  typename ExternalTransceiver = default_dummy_transceiver>
class rx_sci_usart : public usart, public SciDev, public ExternalTransceiver
{
public:
  typedef SciDev sci_dev;
  typedef ExternalTransceiver external_transceiver;

  static constexpr unsigned int tx_buffer_size = TxFifoSizeBytes;
  static constexpr unsigned int rx_buffer_size = RxFifoSizeBytes;

  static constexpr unsigned int max_bitrate = SciDev::max_bitrate;

  [[gnu::cold]] rx_sci_usart (void)
  : m_txi_isr (this), m_rxi_isr (this), m_tei_isr (this), m_eri_isr (this)
  {
    m_tx_wr_pos = m_tx_rd_pos = 0;
    m_rx_wr_pos = m_rx_rd_pos = 0;

    m_tx_lock = 0;

    m_rx_data_clb = default_rx_data_clb;
    m_rx_data_clb_user_p = nullptr;


    // some compile-time sanity checks
    static_assert (std::is_same<unsigned char, uint8_t>::value, "");

    static_assert (utils::is_pow2 (tx_buffer_size), "");
    static_assert (std::numeric_limits<decltype (m_tx_wr_pos)>::max () >= tx_buffer_size - 1, "");
    static_assert (std::numeric_limits<decltype (m_tx_rd_pos)>::max () >= tx_buffer_size - 1, "");

    static_assert (utils::is_pow2 (rx_buffer_size), "");
    static_assert (std::numeric_limits<decltype (m_rx_wr_pos)>::max () >= rx_buffer_size - 1, "");
    static_assert (std::numeric_limits<decltype (m_rx_rd_pos)>::max () >= rx_buffer_size - 1, "");


    // SciDev ctor will enable the device (disable module standby) but not the
    // interrupts.
    m_txi_isr.enable (sci_dev::txi_interrupt_type, sci_dev::txi_interrupt_priority);
    m_rxi_isr.enable (sci_dev::rxi_interrupt_type, sci_dev::rxi_interrupt_priority);
    m_tei_isr.enable (sci_dev::tei_interrupt_type, sci_dev::tei_interrupt_priority);
    m_eri_isr.enable (sci_dev::eri_interrupt_type, sci_dev::eri_interrupt_priority);

    struct config cfg;
    set_config (cfg
	.set_baud_rate (sci_dev::max_bitrate)
	.set (parity::none)
	.set_start_stop_bits_4 (1*4)
	.set_data_bits (8)
	.set (flow_ctrl::none));
  }

  virtual void reset_receiver (void) noexcept override
  {
    this->set_status (this->status ()
	.clear_overrun_error ()
	.clear_parity_error ()
	.clear_framing_error ());
  }

  virtual bool is_valid (void) const noexcept override
  {
    return true;
  }

  virtual struct config config (void) const noexcept override
  {
    struct config r;

    auto mode = this->mode ();

    r.baud_rate = m_bitrate_params.bitrate;

    r.parity = parity::none;
    if (mode.parity_enable ())
    {
      if (mode.parity_mode () == sci_dev::parity_even)
	r.parity = parity::even;
      else if (mode.parity_mode () == sci_dev::parity_odd)
	r.parity = parity::odd;
    }

    r.start_stop_bits_4 = 0;
    switch (mode.stop_bits ())
    {
      default: break;
      case sci_dev::stop_bits_1: r.start_stop_bits_4 = 1 * 4; break;
      case sci_dev::stop_bits_2: r.start_stop_bits_4 = 2 * 4; break;
    }

    r.data_bits = 0;
    switch (mode.char_length ())
    {
      default: break;
      case sci_dev::charlen_8: r.data_bits = 8; break;
      case sci_dev::charlen_7: r.data_bits = 7; break;
    // case sci_dev::charlen9: r.data_bits = 9; break;  // not supported by SCIc, only SCIg
    }

    r.flow_ctrl = flow_ctrl::none;
    r.xon_char = r.xoff_char = 0;

    return r;
  }

  virtual bool set_config (const struct config& val) noexcept override
  {
    auto mode = this->mode ();

    switch (val.parity)
    {
      default:
      case parity::none:
	mode.set_parity_enable (false);
	break;

      case parity::odd:
	mode.set_parity_enable (true);
	mode.set_parity_mode (sci_dev::parity_odd);
	break;

      case parity::even:
	mode.set_parity_enable (true);
	mode.set_parity_mode (sci_dev::parity_even);
	break;
    }

    switch (val.start_stop_bits_4)
    {
      default:
      case (1*4):
	mode.set_stop_bits (sci_dev::stop_bits_1);
	break;
      case (2*4):
	mode.set_stop_bits (sci_dev::stop_bits_2);
	break;
    }

    mode.set_char_length_bits (val.data_bits);

    m_bitrate_params = sci_dev::bitrate_to_params (val.baud_rate, sci_dev::base_clock_8);

    auto ctrl_setting = typename sci_dev::control_t ()
	.set_clock_enable (sci_dev::on_chip_generator)
	.set_transmit_end_interrupt_enable (false)
	.set_multi_processor_interrupt_enable (false)
	.set_receive_enable (false)
	.set_transmit_enable (false)
	.set_receive_interrupt_enable (false)
	.set_transmit_interrupt_enable (false);

    this->set_control (ctrl_setting);

    this->set_mode (mode
	.set_clock_select (m_bitrate_params.clk_sel)
	.set_multi_processor_mode (false)
	.set_communication_mode (sci_dev::async_mode)
	.set_interface_mode (sci_dev::interface_mode_serial)
	.set_data_invert (sci_dev::no_invert_rx_tx)
	.set_data_direction (sci_dev::lsb_first)
	.set_async_clock_source_select (sci_dev::ext_clock_input)
	.set_base_clock_select (sci_dev::base_clock_8)
	.set_noise_filter_enable (false));

    this->set_bitrate_params (m_bitrate_params);

    this->set_status (this->status ()
	.clear_overrun_error ()
	.clear_parity_error ()
	.clear_framing_error ());

    // FIXME: disable I2C and SPI mode

    // this->set_i2c_mode (typename sci_dev::i2c_mode_t ()

    // this->set_spi_mode (typename sci_dev::spi_mode_t ()

    // this->set_i2c_disable_state ()

    // enable reception.  the transmission related bits will be set when
    // data needs to be transmitted.
    external_transceiver::enable_external_receiver ();
    external_transceiver::disable_external_transmitter ();

    this->set_control (ctrl_setting
			.set_receive_enable (true)
			.set_receive_interrupt_enable (true));

    return true;
  }

  virtual void write (const void* data, unsigned int byte_count) noexcept override
  {
    for (auto&& ptr = (const uint8_t*)data; byte_count > 0; --byte_count)
      write (*ptr++);
  }

  void write (unsigned char data)
  {
    // wait until we can write a byte.
    // we use the difference of wr_pos and tx_pos to figure
    // out whether the buffer is full.

    // FIXME: if this is being invoked in a higher priority ISR
    // than the TXI ISR, this will deadlock.
    while (true)
    {
      // make sure to re-fetch the wr_pos value.  it is modified
      // by the tx ISR.  the values will wrap around at some point.
      // however, since we use the distance it's OK.
      auto pending_bytes = (buffer_pos_t)(m_tx_wr_pos - *((volatile decltype (m_tx_rd_pos)*)&m_tx_rd_pos));
      if (pending_bytes < (int)m_tx_buffer.size ())
	break;

      // if the transfer end interrupt (TEI) happened shortly before new bytes
      // are written transmission might get stuck.
      auto ctrl = this->control ();
      if (!ctrl.transmit_enable ())
      {
	external_transceiver::enable_external_transmitter ();
	this->set_control (ctrl
				.set_transmit_interrupt_enable (true)
				.set_transmit_enable (true)
				.set_transmit_end_interrupt_enable (false));
      }
    }

    // set the lock to prevent the ISR from accessing the
    // tx buffer.  notice that this code can't interrupt the ISR,
    // but the ISR can interrupt this function.
    if (m_tx_lock.exchange (1) == 0)
    {
      tx_buffer_write_byte (data);

      // if this is the first byte in the transfer, there will
      // be an initial TXI to start pumping the data.
      auto ctrl = this->control ();
      if (!ctrl.transmit_enable ())
      {
	external_transceiver::enable_external_transmitter ();
	this->set_control (ctrl
				.set_transmit_interrupt_enable (true)
				.set_transmit_enable (true));
      }

      while (true)
      {
        auto prev_lockval = m_tx_lock.exchange (0);

	if (prev_lockval == 0)
	{
	  assert_unreachable ();
	}
	else if (prev_lockval == 1)
	{
	  // lock released and there was no TXI.
	  break;
	}
	else if (prev_lockval == 2)
	{
	  // lock released and there was a TXI.
	  txi ();
	  break;
	}
	else
	  assert_unreachable ();
    }
  }
  else
    assert_unreachable ();
  }


  virtual unsigned int read (void* data_out, unsigned int max_byte_count) noexcept override
  {
    // wait until we can read at least one byte.
    // this also latches the last non-zero byte count for the actual read.
    // if more data is received during the read, it will go unnoticed.

    // FIXME: MCB-253
    // this is supposed to block and wait until a byte has been received.
    // but for some weird reason, it ends up in an infinite loop, even through
    // a previous call to "buffer_stat" says that there are bytes available
    // in the rx buffer.
    // for now, let it fail and return after a while to avoid system lock-up.
    unsigned int available_bytes = 0;
    for (unsigned int x = 0; ; ++x)
    {
      available_bytes = (buffer_pos_t)(*((volatile decltype (m_rx_wr_pos)*)&m_rx_wr_pos) - m_rx_rd_pos);
      if (available_bytes > 0)
        break;

      if (x == 256)
	return 0;
    }

    unsigned int count = max_byte_count < available_bytes
			 ? max_byte_count : available_bytes;

    unsigned int count0 = count;
    unsigned int count1 = 0;

    unsigned int p0 = m_rx_rd_pos & (m_rx_buffer.size () - 1);
    unsigned int p1 = p0 + count0;

    if (p1 > m_rx_buffer.size ())
    {
      count1 = p1 - m_rx_buffer.size ();
      p1 = m_rx_buffer.size ();
      count0 = p1 - p0;
    }

    uint8_t* out_ptr = (uint8_t*)data_out;
    const uint8_t* in_ptr = &m_rx_buffer[p0];

    for (; count0 > 0; --count0)
    {
      *out_ptr++ = *in_ptr++;
      ++m_rx_rd_pos;
    }

    in_ptr = &m_rx_buffer[0];

    for (; count1 > 0; --count1)
    {
      *out_ptr++ = *in_ptr++;
      ++m_rx_rd_pos;
    }

    return count;
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
    // modify only m_rx_rd_pos.  this avoids a race condition with the rx isr
    // on the m_rx_wr_pos variable.
    m_rx_rd_pos = m_rx_wr_pos;
    m_rx_buffer_overrun = false;
  }


  using usart::set_recv_clb;

  virtual struct generic_io::buffer_stat buffer_stat (void) const noexcept
  {
    auto rx_st = rx_buffer_stat ();
    auto tx_st = tx_buffer_stat ();
    auto st = this->status ();

    return { st.overrun_error () | st.parity_error () | st.framing_error ()
	     | m_rx_buffer_overrun,
	     false,
	     rx_st.count, rx_buffer_size,
	     tx_st.count, tx_buffer_size };

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
    auto wr_pos = *((volatile decltype (m_tx_wr_pos)*)&m_tx_wr_pos);
    auto rd_pos = *((volatile decltype (m_tx_rd_pos)*)&m_tx_rd_pos);

    // count can't be > m_tx_buffer.size ().
    // if it is, something is terribly broken.
    unsigned int count0 = (buffer_pos_t)(wr_pos - rd_pos);

    unsigned int count = count0;

    unsigned int p0 = rd_pos & (m_tx_buffer.size () - 1);
    unsigned int p1 = p0 + count0;

    unsigned int count1 = 0;

    if (p1 > m_tx_buffer.size ())
    {
      count1 = p1 - m_tx_buffer.size ();
      p1 = m_tx_buffer.size ();
      count0 = p1 - p0;
    }

    return { &m_tx_buffer[p0], count0, &m_tx_buffer[0], count1, count };
  }

  struct buffer_stat rx_buffer_stat (void) const
  {
    auto wr_pos = *((volatile decltype (m_rx_wr_pos)*)&m_rx_wr_pos);
    auto rd_pos = *((volatile decltype (m_rx_rd_pos)*)&m_rx_rd_pos);

    // count can't be > m_tx_buffer.size ().
    // if it is, something is terribly broken.
    unsigned int count0 = (buffer_pos_t)(wr_pos - rd_pos);

    unsigned int count = count0;

    unsigned int p0 = rd_pos & (m_rx_buffer.size () - 1);
    unsigned int p1 = p0 + count0;

    unsigned int count1 = 0;

    if (p1 > m_rx_buffer.size ())
    {
      count1 = p1 - m_rx_buffer.size ();
      p1 = m_rx_buffer.size ();
      count0 = p1 - p0;
    }

    return { &m_rx_buffer[p0], count0, &m_rx_buffer[0], count1, count };
  }


private:
  void tx_buffer_write_byte (uint8_t data)
  {
    m_tx_buffer[m_tx_wr_pos & (m_tx_buffer.size () - 1)] = data;
    ++m_tx_wr_pos;
  }

  uint8_t tx_buffer_read_byte (void)
  {
    uint8_t data = m_tx_buffer[m_tx_rd_pos & (m_tx_buffer.size () - 1)];
    ++m_tx_rd_pos;
    return data;
  }


  bool rx_buffer_write_byte (uint8_t data)
  {
    // if there isn't any space left in the rx buffer, set the buffer
    // overrun flag so the user has a chance to detect the error.
    auto wr_pos = *((volatile decltype (m_rx_wr_pos)*)&m_rx_wr_pos);
    auto rd_pos = *((volatile decltype (m_rx_rd_pos)*)&m_rx_rd_pos);

    unsigned int count0 = (buffer_pos_t)(wr_pos - rd_pos);

    if (count0 >= m_rx_buffer.size () - 1)
    {
      m_rx_buffer_overrun = true;
      return false;
    }

    m_rx_buffer[m_rx_wr_pos & (m_rx_buffer.size () - 1)] = data;
    ++m_rx_wr_pos;
    return true;
  }

  uint8_t rx_buffer_read_byte (void)
  {
    uint8_t data = m_rx_buffer[m_rx_rd_pos & (m_rx_buffer.size () - 1)];
    ++m_rx_rd_pos;
    return data;
  }

  void txi (void)
  {
    if (m_tx_lock.exchange (2) != 0)
    {
      // the TXI interrupted the write_byte function.
      // do nothing.  the write_byte function will handle it.
    }
    else
    {
      // haven't interrupted anything.  see if we can send one more
      // byte from the buffer.
      if (*((volatile decltype (m_tx_wr_pos)*)&m_tx_wr_pos) == m_tx_rd_pos)
      {
	// read pos and write pos are the same, i.e. the buffer is empty.
	this->set_control (this->control ().set_transmit_end_interrupt_enable (true));
      }
      else
      {
	this->set_control (this->control ().set_transmit_end_interrupt_enable (false));
	this->set_transmit_data (tx_buffer_read_byte ());
      }
      m_tx_lock = 0;
    }
  }

  void tei (void)
  {
    this->set_control (this->control ()
			.set_transmit_enable (false)
			.set_transmit_interrupt_enable (false)
			.set_transmit_end_interrupt_enable (false));

    if (m_tx_wr_pos != m_tx_rd_pos)
    {
      // if the transfer end interrupt (TEI) happened shortly before new bytes
      // are written transmission might get stuck.
      external_transceiver::enable_external_transmitter ();
      this->set_control (this->control ()
				.set_transmit_enable (true)
				.set_transmit_interrupt_enable (true)
				.set_transmit_end_interrupt_enable (false));
    }
    else
      external_transceiver::disable_external_transmitter ();
  }

  void rxi (void)
  {
    // copy data from sci to rx buffer
    rx_buffer_write_byte (this->receive_data ());

    // invoke callback
    m_rx_data_clb (m_rx_data_clb_user_p, (buffer_pos_t)(m_rx_wr_pos - m_rx_rd_pos));
  }

  static void default_rx_data_clb (void*, unsigned int)
  {
  }

  void eri (void)
  {

  }

public:
  typedef interrupt::connected_isr<typename sci_dev::txi_interrupt_line,
	interrupt::func<decltype (&rx_sci_usart::txi), &rx_sci_usart::txi>> txi_isr_t;

  typedef interrupt::connected_isr<typename sci_dev::rxi_interrupt_line,
	interrupt::func<decltype (&rx_sci_usart::rxi), &rx_sci_usart::rxi>> rxi_isr_t;

  typedef interrupt::connected_isr<typename sci_dev::tei_interrupt_line,
	interrupt::func<decltype (&rx_sci_usart::tei), &rx_sci_usart::tei>> tei_isr_t;

  typedef interrupt::connected_isr<typename sci_dev::eri_interrupt_line,
	interrupt::func<decltype (&rx_sci_usart::eri), &rx_sci_usart::eri>> eri_isr_t;

  txi_isr_t& txi_isr (void) { return m_txi_isr; }
  rxi_isr_t& rxi_isr (void) { return m_rxi_isr; }
  tei_isr_t& tei_isr (void) { return m_tei_isr; }
  eri_isr_t& eri_isr (void) { return m_eri_isr; }

private:
  // the type of the buffer position is a bit important here.
  // in the calculations we use the defined unsigned int wrap-around.  however
  // when using types smaller than 'unsigned int' the calculations get promoted
  // and the resulting values will be wrong in the higher bits.  to get the
  // desired wrap-around effects, the results need to be truncated back to the
  // actual type.
  // we could also use a type depending on the buffer size...
  typedef uint16_t buffer_pos_t;

  buffer_pos_t m_tx_wr_pos;
  buffer_pos_t m_tx_rd_pos;

  buffer_pos_t m_rx_wr_pos;
  buffer_pos_t m_rx_rd_pos;

  std::atomic<int> m_tx_lock;

  volatile bool m_rx_buffer_overrun = false;

  void (*m_rx_data_clb)(void*, unsigned int);
  void* m_rx_data_clb_user_p;

  std::array<uint8_t, TxFifoSizeBytes> m_tx_buffer;
  std::array<uint8_t, RxFifoSizeBytes> m_rx_buffer;

  txi_isr_t m_txi_isr;
  rxi_isr_t m_rxi_isr;
  tei_isr_t m_tei_isr;
  eri_isr_t m_eri_isr;

  typename sci_dev::bitrate_params m_bitrate_params;
};

} // namespace dev
#endif // includeguard_dev_rx_sci_usart_hpp_includeguard

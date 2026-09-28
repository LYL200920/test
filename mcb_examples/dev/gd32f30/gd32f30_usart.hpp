/*

GD32F30* USART device

*/


#ifndef includeguard_dev_gd32f30_usart_hpp_includeguard
#define includeguard_dev_gd32f30_usart_hpp_includeguard

#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <atomic>

#include <utils/langcomp.hpp>

#include <dev/usart.hpp>
#include <dev/interrupt.hpp>
#include <dev/generic_transceiver.hpp>

#include <utils/value_range.hpp>
#include <utils/bits.hpp>

namespace dev
{
namespace gd32f30_usart
{

template <uint32_t RegBaseAddr, unsigned int PeripheralClock,
	  typename InterruptLine,
	  unsigned int TxFifoSizeBytes, unsigned int RxFifoSizeBytes,
	  unsigned int MaxInitBitRate,
	  typename ExternalTransceiver = default_dummy_transceiver,
	  bool DisableConfigCode = false>
class hw_inst : public ExternalTransceiver
{
private:
  typedef ExternalTransceiver external_transceiver;

  struct regs
  {
    static constexpr hw_reg_rw<uint32_t, const_addr<RegBaseAddr + 0x00>> stat0 = { }; // 0x00
    static constexpr hw_reg_rw<uint32_t, const_addr<RegBaseAddr + 0x04>> data  = { }; // 0x04
    static constexpr hw_reg_rw<uint32_t, const_addr<RegBaseAddr + 0x08>> baud  = { }; // 0x08
    static constexpr hw_reg_rw<uint32_t, const_addr<RegBaseAddr + 0x0C>> ctl0  = { }; // 0x0C
    static constexpr hw_reg_rw<uint32_t, const_addr<RegBaseAddr + 0x10>> ctl1  = { }; // 0x10
    static constexpr hw_reg_rw<uint32_t, const_addr<RegBaseAddr + 0x14>> ctl2  = { }; // 0x14
    static constexpr hw_reg_rw<uint32_t, const_addr<RegBaseAddr + 0x18>> gp    = { }; // 0x18

    static constexpr hw_reg_rw<uint32_t, const_addr<RegBaseAddr + 0x80>> ctl3  = { }; // 0x80
    static constexpr hw_reg_rw<uint32_t, const_addr<RegBaseAddr + 0x84>> rt    = { }; // 0x84
    static constexpr hw_reg_rw<uint32_t, const_addr<RegBaseAddr + 0x88>> stat1 = { }; // 0x88
  };

  void isr (void)
  {
    uint32_t isr_status0 = regs::stat0;
    uint32_t isr_status1 = regs::stat1;

    // test some of the more frequently used bit types first.
    // then test the unlikely cases later.

    // ---------------------------------------
    // regs::stat0
    constexpr unsigned int perr  = 1 << 0;
    constexpr unsigned int ferr  = 1 << 1;
    constexpr unsigned int nerr  = 1 << 2;
    constexpr unsigned int orerr = 1 << 3;
    constexpr unsigned int idlef = 1 << 4;
    constexpr unsigned int rbne  = 1 << 5;
    constexpr unsigned int tc    = 1 << 6;
    constexpr unsigned int tbe   = 1 << 7;
    constexpr unsigned int lbdf  = 1 << 8;
    constexpr unsigned int ctsf  = 1 << 9;

    // ---------------------------------------
    // regs::stat1
    constexpr unsigned int rtf  = 1 << 11;
    constexpr unsigned int ebf  = 1 << 12;
    constexpr unsigned int bsy  = 1 << 16;

    if (isr_status0 & (tc | tbe | ctsf | rbne))
    {
      if (isr_status0 & tc)   { regs::stat0 &= ~tc; isr_tc (); }
      if (isr_status0 & tbe)  { isr_tbe (); }
      if (isr_status0 & ctsf) { regs::stat0 &= ~ctsf; isr_ctsf (); }
      if (isr_status0 & rbne) { isr_rbne (); }
    }

    if (isr_status0 & perr)  { isr_perr (); }
    if (isr_status0 & ferr)  { isr_ferr (); }
    if (isr_status0 & nerr)  { isr_nerr (); }
    if (isr_status0 & orerr) { isr_orerr (); }
    if (isr_status0 & idlef) { isr_idlef (); }
    if (isr_status0 & lbdf)  { regs::stat0 &= ~lbdf; isr_lbdf (); }

    if (isr_status1 & rtf)  { regs::stat1 &= ~rtf; isr_rtf (); }
    if (isr_status1 & ebf)  { regs::stat1 &= ~ebf; isr_ebf (); }
    if (isr_status1 & bsy)  { isr_bsy (); }
  }

  constexpr static uint32_t config_to_ctl0_data_bits (unsigned int val)
  {
    switch (val)
    {
    default:
    case 8: return (0 << 12);
    case 9: return (1 << 12);
    }
  }

  constexpr static unsigned int ctl0_data_bits_to_config (uint32_t val)
  {
    switch ((val >> 12) & 1)
    {
    case 0b0: return 8;
    case 0b1: return 9;
    default: unreachable;
    }
  }

  constexpr static uint32_t config_to_ctl0_parity_bits (usart::parity val)
  {
    switch (val)
    {
    case usart::parity::none: return 0b00 << 9;  // ctl0 bit 10, bit 9
    case usart::parity::odd : return 0b11 << 9;
    case usart::parity::even: return 0b10 << 9;
    default: unreachable;
    }
  }

  constexpr static usart::parity ctl0_parity_bits_to_config (uint32_t val)
  {
    switch ((val >> 9) & 0b11)
    {
    case 0b00: return usart::parity::none;
    case 0b01: return usart::parity::none;
    case 0b10: return usart::parity::even;
    case 0b11: return usart::parity::odd;
    default: unreachable;
    }
  }

  constexpr static uint32_t config_to_ctl1_stop_bits (unsigned int val)
  {
    switch (val)
    {
    case 4: return 0b00 << 12; // 1   stop bit
    case 2: return 0b01 << 12; // 0.5 stop bits
    case 8: return 0b10 << 12; // 2   stop bits
    case 6: return 0b11 << 12; // 1.5 stop bits
    default: return 0;
    }
  }
  constexpr static unsigned int ctl1_stop_bits_to_config (uint32_t val)
  {
    switch ((val >> 12) & 0b11)
    {
    case 0b00: return 4;
    case 0b01: return 2;
    case 0b10: return 8;
    case 0b11: return 6;
    default: unreachable;
    }
  }

  constexpr static uint32_t config_to_ctl2_flow_ctrl (usart::flow_ctrl val)
  {
    switch (val)
    {
    default:
    case usart::flow_ctrl::none:    return 0b00 << 8;
    case usart::flow_ctrl::rts_cts: return 0b11 << 8;
    }
  }

  constexpr static usart::flow_ctrl ctl2_flow_ctrl_to_config (uint32_t val)
  {
    if (((val >> 8) & 0b11) == 0b11)
      return usart::flow_ctrl::rts_cts;

    return usart::flow_ctrl::none;
  }

public:
  static constexpr unsigned int clock_hz = PeripheralClock;

  static constexpr unsigned int tx_buffer_size = TxFifoSizeBytes;
  static constexpr unsigned int rx_buffer_size = RxFifoSizeBytes;

  static constexpr unsigned int max_bitrate = MaxInitBitRate;

  static constexpr unsigned int interrupt_priority = interrupt::priority_7;

  typedef interrupt::connected_isr<InterruptLine,
	interrupt::func<decltype (&hw_inst::isr), &hw_inst::isr>> isr_t;

  static bool transmit_end (void) { return (regs::stat0 & (1 << 6)) != 0; }
  static bool transmit_data_empty (void) { return (regs::stat0 & (1 << 7)) != 0; }
  static void write_transmit_data (char val) { regs::data = val; }

  hw_inst (void)
  {
    isr_t (this);

    m_tx_wr_pos = m_tx_rd_pos = 0;
    m_rx_wr_pos = m_rx_rd_pos = 0;

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

    isr_t::enable (interrupt::low_level, interrupt_priority);

    if constexpr (!DisableConfigCode)
    {
      struct usart::config cfg;
      cfg.baud_rate = max_bitrate;
      cfg.parity = usart::parity::none;
      cfg.start_stop_bits_4 = 4;
      cfg.data_bits = 8;
      cfg.flow_ctrl = usart::flow_ctrl::none;

      set_config (cfg);
    }
    else
    {
      clear_all_interrupt_flag ();
    }
  }

  void reset_receiver (void) noexcept
  {
  }

  bool is_valid (void) const noexcept
  {
    return true;
  }

  struct usart::config config (void) const noexcept
  {
    uint32_t ctl0_val = regs::ctl0;
    uint32_t ctl1_val = regs::ctl1;
    uint32_t ctl2_val = regs::ctl2;
    uint32_t baud_val = regs::baud;

    struct usart::config cfg;
    cfg.baud_rate         = baud_val != 0 ? (clock_hz / baud_val) : 0;
    cfg.parity            = ctl0_parity_bits_to_config (ctl0_val);
    cfg.start_stop_bits_4 = ctl1_stop_bits_to_config (ctl1_val);
    cfg.data_bits         = ctl0_data_bits_to_config (ctl0_val);
    cfg.flow_ctrl         = ctl2_flow_ctrl_to_config (ctl2_val);
    return cfg;
  }

  bool set_config (const struct usart::config& cfg) noexcept
  {
    // disable usart before changing the configuration.
    // dsiable all interrupts

    // disable_usart
    regs::ctl0 = 0;
    regs::ctl1 = 0;
    regs::ctl2 = 0;
    regs::ctl3 = 0;

    clear_all_interrupt_flag ();

    regs::baud = clock_hz / cfg.baud_rate;

    regs::ctl2 = 0
    | config_to_ctl2_flow_ctrl (cfg.flow_ctrl)
    | (0 << 0) // error interrupt disable
    | 0;

    regs::ctl1 = 0
    | config_to_ctl1_stop_bits (cfg.start_stop_bits_4)
    | 0;

    // enable usart
    regs::ctl0 = 0
    | config_to_ctl0_data_bits (cfg.data_bits)
    | config_to_ctl0_parity_bits (cfg.parity)
    | (1 << 13) // usart enable
    | (1 << 7)  // TBE interrupt enable
    | (1 << 6)  // TC interrupt enable
    | (1 << 5)  // RBNE interrupt enable
    | (0 << 4)  // IDLE interrupt disable
    | (0 << 3)  // transmitter disable
    | (1 << 2)  // receiver enable
    | 0;

    return true;
  }

  void write (const void* data, unsigned int byte_count) noexcept
  {
    if (byte_count == 0)
      return;

    // fill up the transmit buffer first and then enable the
    // the transmission once.

    const uint8_t* ptr = (const uint8_t*)data;

    while (true)
    {
      if (!tx_buffer_write_full ())
      {
        tx_buffer_write_byte (*ptr);
        ++ptr;
        if (--byte_count == 0)
          break;
      }
      else
        check_restart_transmission ();
    }

    check_restart_transmission ();
  }

  void write (unsigned char data)
  {
    // wait until we can write a byte.

    // FIXME: if this is being invoked in a higher priority ISR
    // than isr_tbe, this will deadlock.
    for (unsigned int jjjj = 0; tx_buffer_write_full (); ++jjjj)
    {
      assert (jjjj < 100000);
      check_restart_transmission ();
    }

    // it's safe to write one byte to the buffer.
    tx_buffer_write_byte (data);

    check_restart_transmission ();
  }

  unsigned int read (void* data_out, unsigned int max_byte_count) noexcept
  {
    // wait until we can read at least one byte.
    // this also latches the last non-zero byte count for the actual read.
    // if more data is received during the read, it will go unnoticed.
    unsigned int available_bytes = 0;
    while (true)
    {
      available_bytes = (buffer_pos_t)(*((volatile decltype (m_rx_wr_pos)*)&m_rx_wr_pos) - m_rx_rd_pos);
      if (available_bytes > 0)
        break;
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

  void set_recv_clb (void f (void* user_p, unsigned int byte_count), void* user_p) noexcept
  {
    if (f == nullptr)
      m_rx_data_clb = default_rx_data_clb;
    else
    {
      m_rx_data_clb = f;
      m_rx_data_clb_user_p = user_p;
    }

  }

  void reset_rx_buffer (void) noexcept
  {
    // modify only m_rx_rd_pos.  this avoids a race condition with the rx isr
    // on the m_rx_wr_pos variable.
    m_rx_rd_pos = m_rx_wr_pos;
  }


  void reset_transmitter (void) noexcept
  {
    external_transceiver::disable_external_transmitter ();
  }

  struct generic_io::buffer_stat buffer_stat (void) const noexcept
  {
    auto rx_st = rx_buffer_stat ();
    auto tx_st = tx_buffer_stat ();

    // auto st = this->status ();

//    return { st.overrun_error () | st.parity_error () | st.framing_error (),
    return { false,
	     false,
	     rx_st.count, rx_buffer_size,
	     tx_st.count, tx_buffer_size };
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

protected:
  typedef uint16_t buffer_pos_t;

  buffer_pos_t m_tx_wr_pos;
  buffer_pos_t m_tx_rd_pos;

  buffer_pos_t m_rx_wr_pos;
  buffer_pos_t m_rx_rd_pos;

  void (*m_rx_data_clb)(void*, unsigned int);
  void* m_rx_data_clb_user_p;

  std::array<uint8_t, TxFifoSizeBytes> m_tx_buffer;
  std::array<uint8_t, RxFifoSizeBytes> m_rx_buffer;

  unsigned int m_ctl0_tbe_enabled;
  unsigned int m_ctl0_tbe_disabled;

  void isr_perr (void)
  {
    //uint8_t data = regs::data; // read data to clear this isr
  }
  void isr_ferr (void)
  {
    //uint8_t data = regs::data; // read data to clear this isr
  }
  void isr_nerr (void)
  {
    //uint8_t data = regs::data; // read data to clear this isr
  }
  void isr_orerr (void)
  {
    //uint8_t data = regs::data; // read data to clear this isr
  }
  void isr_idlef (void)
  {
    //uint8_t data = regs::data; // read data to clear this isr
  }

  void isr_rbne (void)
  {
    // receive data register not empty -- we can receive one byte of data

    // copy data from usart to rx buffer
    // read data to clear this isr
    rx_buffer_write_byte (regs::data);

    // invoke callback
    m_rx_data_clb (m_rx_data_clb_user_p, (buffer_pos_t)(m_rx_wr_pos - m_rx_rd_pos));
  }

  void isr_tc (void)
  {
    // transmission of the last byte on the wire has been completed.
    // it's safe to disable the external transmitter now.
    external_transceiver::disable_external_transmitter ();

    // clear TC flag
    regs::stat0 &= ~(1 << 6);

    // disable the transmitter after sending the last byte.
    // this will make it always transmit an idle frame before transmitting
    // the first byte.  this is useful to enable the external transmitter
    // before the first bit goes out on the wire.
    regs::ctl0 &= ~(1 << 3);
  }

  void isr_tbe (void)
  {
    // the transmit data register is empty, new data can be written to the
    // data register.
    // when starting the transmission, this isr will be triggered immediately
    // to write the first data byte and at the same time it will transmit
    // an idle frame.  this is allows us to enable the external transmitter
    // before any data goes on the wire, but it introduces 1 byte of latency.
    // the idle byte can be avoided by not clearing the ctrl0:TE bit, in other
    // words, never stop the transmission.

    if (!tx_buffer_read_empty ())
    {
      external_transceiver::enable_external_transmitter ();
      write_transmit_data (tx_buffer_read_byte ());
    }
    else
    {
      // last byte has been written (for now ...).
      // disable TBE interrupt, it will be re-enabled when sending more data.
      // once the last byte has been sent out on the wire, TC interrupt will
      // be triggered, which will disable the external transceiver.
      regs::ctl0 &= ~(1 << 7);
    }
  }

  void check_restart_transmission (void)
  {
    regs::ctl0 |= 0
       | (1 << 7)   // TBEIE enable (isr_tbe will pull the first data byte)
       | (1 << 6)   // TCIE enable
       | (1 << 3)   // TEN enable
       | 0;
  }


  void isr_lbdf (void) { }
  void isr_ctsf (void) { }
  void isr_rtf (void) { }
  void isr_ebf (void) { }
  void isr_bsy (void) { }

  void clear_all_interrupt_flag (void)
  {
    regs::stat0 = 0;
    regs::stat1 = 0;

    // clear flags that cannot be cleared by software. these flags are cleared
    // by reading the USART_STAT0 and USART_DATA registers one by one.
    regs::stat0.read ();
    regs::data.read ();
  }

  bool tx_buffer_write_full (void) const
  {
    auto pending_bytes = (buffer_pos_t)(m_tx_wr_pos - *((volatile decltype (m_tx_rd_pos)*)&m_tx_rd_pos));
    return pending_bytes == (int)m_tx_buffer.size ();
  }

  bool tx_buffer_read_empty (void) const
  {
    return *((volatile decltype (m_tx_wr_pos)*)&m_tx_wr_pos) == m_tx_rd_pos;
  }

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


  void rx_buffer_write_byte (uint8_t data)
  {
    m_rx_buffer[m_rx_wr_pos & (m_rx_buffer.size () - 1)] = data;
    ++m_rx_wr_pos;
  }

  uint8_t rx_buffer_read_byte (void)
  {
    uint8_t data = m_rx_buffer[m_rx_rd_pos & (m_rx_buffer.size () - 1)];
    ++m_rx_rd_pos;
    return data;
  }

  static void default_rx_data_clb (void*, unsigned int)
  {
  }

};

} // namespace gd32f30_usart
} // namespace dev

#endif // includeguard_dev_gd32f30_usart_hpp_includeguard

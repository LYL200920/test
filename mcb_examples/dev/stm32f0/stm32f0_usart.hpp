/*

STM32F0x USART device

*/


#ifndef includeguard_dev_stm32f0_usart_hpp_includeguard
#define includeguard_dev_stm32f0_usart_hpp_includeguard

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
namespace stm32f0_usart
{

template <uintptr_t RegBaseAddr, unsigned int PeripheralClock,
	  typename InterruptLine,
	  unsigned int TxFifoSizeBytes, unsigned int RxFifoSizeBytes,
	  unsigned int MaxInitBitRate,
	  bool DisableConfigCode = false>
class hw_inst
{
private:
  void isr (void)
  {
    auto&& r = regs ();
    uint32_t isr_status = r.isr;

    // test some of the more frequently used bit types first.
    // then test the unlikely cases later.
    constexpr unsigned int pe  = 1 << 0;
    constexpr unsigned int fe  = 1 << 1;
    constexpr unsigned int nf  = 1 << 2;
    constexpr unsigned int ore = 1 << 3;
    constexpr unsigned int idle = 1 << 4;
    constexpr unsigned int rxne = 1 << 5;
    constexpr unsigned int tc = 1 << 6;
    constexpr unsigned int txe = 1 << 7;
    constexpr unsigned int lbdf = 1 << 8;
    constexpr unsigned int ctsif = 1 << 9;
    constexpr unsigned int cts = 1 << 10;
    constexpr unsigned int rtof = 1 << 11;
    constexpr unsigned int eobf = 1 << 12;
    constexpr unsigned int abre = 1 << 14;
    constexpr unsigned int abrf = 1 << 15;
    constexpr unsigned int busy = 1 << 16;
    constexpr unsigned int cmf = 1 << 17;
    constexpr unsigned int sbkf = 1 << 18;
    constexpr unsigned int rwu = 1 << 19;
    constexpr unsigned int wuf = 1 << 20;
    constexpr unsigned int teack = 1 << 21;
    constexpr unsigned int reack = 1 << 22;

    if (isr_status & (tc | txe | ctsif | cts | rxne))
    {
      if (isr_status & tc) { r.icr = tc; isr_tc (); }
      if (isr_status & txe) { isr_txe (); }
      if (isr_status & ctsif) { r.icr = ctsif; isr_ctsif (); }
      if (isr_status & cts) { isr_cts (); }
      if (isr_status & rxne) { isr_rxne (); }
    }

    if (isr_status & pe) { r.icr = pe; isr_pe (); }
    if (isr_status & fe) { r.icr = fe; isr_fe (); }
    if (isr_status & nf) { r.icr = nf; isr_nf (); }
    if (isr_status & ore) { r.icr = ore; isr_ore (); }
    if (isr_status & idle) { r.icr = idle; isr_idle (); }
    if (isr_status & lbdf) { r.icr = lbdf; isr_lbdf (); }
    if (isr_status & rtof) { r.icr = rtof; isr_rtof (); }
    if (isr_status & eobf) { r.icr = eobf; isr_eobf (); }
    if (isr_status & abre) { isr_abre (); }
    if (isr_status & abrf) { isr_abrf (); }
    if (isr_status & busy) { isr_busy (); }
    if (isr_status & cmf) { r.icr = cmf; isr_cmf (); }
    if (isr_status & sbkf) { isr_sbkf (); }
    if (isr_status & rwu) { isr_rwu (); }
    if (isr_status & wuf) { r.icr = wuf; isr_wuf (); }
    if (isr_status & teack) { isr_teack (); }
    if (isr_status & reack) { isr_reack (); }
  }

  constexpr static uint32_t config_to_cr1_data_bits (unsigned int val)
  {
    switch (val)
    {
    default:
    case 8: return (0 << 28) | (0 << 12);
    case 7: return (1 << 28) | (0 << 12);
    case 9: return (0 << 28) | (1 << 12);
    }
  }

  constexpr static unsigned int cr1_data_bits_to_config (uint32_t val)
  {
    switch ((((val >> 28) & 1) << 1) | ((val >> 12) & 1))
    {
    case 0b00: return 8;
    case 0b01: return 9;
    case 0b10: return 7;
    case 0b11: return 0;
    default: unreachable;
    }
  }

  constexpr static uint32_t config_to_cr1_parity_bits (usart::parity val)
  {
    switch (val)
    {
    case usart::parity::none: return 0b00 << 9;  // CR1 bit 10, bit 9
    case usart::parity::odd: return 0b11 << 9;
    case usart::parity::even: return 0b10 << 9;
    default: unreachable;
    }
  }

  constexpr static usart::parity cr1_parity_bits_to_config (uint32_t val)
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

  constexpr static uint32_t config_to_cr2_stop_bits (unsigned int val)
  {
    switch (val)
    {
    case 4: return 0b00 << 12; // 1 stop bit
    case 2: return 0b01 << 12; // 0.5 stop bits
    case 8: return 0b10 << 12; // 2 stop bits
    case 6: return 0b11 << 12; // 1.5 stop bits
    default: return 0;
    }
  }
  constexpr static unsigned int cr2_stop_bits_to_config (uint32_t val)
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

  constexpr static uint32_t config_to_cr3_flow_ctrl (usart::flow_ctrl val)
  {
    switch (val)
    {
    default:
    case usart::flow_ctrl::none: return 0;
    case usart::flow_ctrl::rts_cts: return 0b11 << 8;
    case usart::flow_ctrl::de_high: return 0b01 << 14;
    case usart::flow_ctrl::de_low:  return 0b11 << 14;
    }
  }

  constexpr static usart::flow_ctrl cr3_flow_ctrl_to_config (uint32_t val)
  {
    if (((val >> 8) & 0b11) == 0b11)
      return usart::flow_ctrl::rts_cts;

    if (((val >> 14) & 0b11) == 0b01)
      return usart::flow_ctrl::de_high;

    if (((val >> 14) & 0b11) == 0b11)
      return usart::flow_ctrl::de_low;

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

  struct regs_t
  {
    hw_reg_rw<uint32_t> cr1;	// 0x00
    hw_reg_rw<uint32_t> cr2;	// 0x04
    hw_reg_rw<uint32_t> cr3;	// 0x08
    hw_reg_rw<uint32_t> brr;	// 0x0C
    hw_reg_rw<uint32_t> gtpr;	// 0x10
    hw_reg_rw<uint32_t> rtor;	// 0x14
    hw_reg_w<uint32_t> rqr;	// 0x18
    hw_reg_r<uint32_t> isr;	// 0x1C
    hw_reg_rw<uint32_t> icr;	// 0x20
    hw_reg_r<uint32_t> rdr;	// 0x24
    hw_reg_rw<uint32_t> tdr;	// 0x28
  };

  static_assert (sizeof (regs_t) == 0x2C, "");

  static constexpr regs_t& regs (void) { return *(regs_t*)RegBaseAddr; }

  static bool transmit_end (void) { return (regs ().isr & (1 << 6)) != 0; }
  static bool transmit_data_empty (void) { return (regs ().isr & (1 << 7)) != 0; }
  static void write_transmit_data (char val) { regs ().tdr = val; }

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
      auto&& r = regs ();
      r.icr = 0xFFFFFFFF;
      m_cr1_txe_disabled = r.cr1;
      m_cr1_txe_enabled = m_cr1_txe_disabled | (1 << 7);
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
    auto&& r = regs ();
    uint32_t cr1_val = r.cr1;
    uint32_t cr2_val = r.cr2;
    uint32_t cr3_val = r.cr3;
    uint32_t brr_val = r.brr;

    struct usart::config cfg;
    cfg.baud_rate = brr_val != 0 ? (clock_hz / brr_val) : 0;
    cfg.parity = cr1_parity_bits_to_config (cr1_val);
    cfg.start_stop_bits_4 = cr2_stop_bits_to_config (cr2_val);
    cfg.data_bits = cr1_data_bits_to_config (cr1_val);
    cfg.flow_ctrl = cr3_flow_ctrl_to_config (cr3_val);
    return cfg;
  }

  bool set_config (const struct usart::config& cfg) noexcept
  {
    auto&& r = regs ();

    // disable usart before changing the configuration.
    r.cr1 = 0;

    // clear all interrupts
    r.icr = 0xFFFFFFFF;

    r.cr2 = 0
    | (0 << 23)			// receiver timeout disable
    | (0b00 << 21)		// auto baud rate mode: measure start bit
    | (0 << 20)			// auto baud rate disable
    | (0 << 19)			// LSB first
    | (0 << 18)			// data bit inversion disable
    | (0 << 17)			// TX pin active level inversion disable
    | (0 << 16)			// RX pin active level inversion disable
    | (0 << 15)			// swap RX/TX pins disable
    | (0 << 14)			// LIN mode disable
    | config_to_cr2_stop_bits (cfg.start_stop_bits_4)
    | (0 << 11)			// CLK pin disable
    | (0 << 10)			// clock polarity (ignore)
    | (0 << 9)			// clock phase (ignore)
    | (0 << 8)			// last bit clock pulse (ignore)
    | (0 << 6)			// LIN break detection interrupt disable
    | (0 << 5)			// 10 bit LIN break detection (ignore)
    | (0 << 4)			// 4 bit address detection (ignore)
    | 0;

    r.cr3 = 0
    | config_to_cr3_flow_ctrl (cfg.flow_ctrl)
    | (0 << 22)			// wakeup from stop interrupt disable
    | (0b00 << 20)		// wakeup from stop interrupt type select (ignore)
    | (0b00 << 17)		// smartcard auto-retry count
    | (0 << 13)			// DMA disable on receive error off
    | (1 << 12)			// receive overrun disable
    | (0 << 11)			// 3-sample bit method
    | (0 << 10)			// cts interrupt disable
    | (0 << 7)			// disable transmission DMA
    | (0 << 6)			// disable receive DMA
    | (0 << 5)			// disable smartcard mode
    | (0 << 4)			// smartcard NACK disable
    | (0 << 3)			// full-duplex mode
    | (0 << 2)			// irda low power disable
    | (0 << 1)			// irda mode disable
    | (0 << 0)			// error interrupt disable
    | 0;

    r.brr = clock_hz / cfg.baud_rate;

    // clear errors

    // enable transceiver
    r.cr1 = 0
    | config_to_cr1_data_bits (cfg.data_bits)
    | (0 << 27)			// End of Block interrupt disable
    | (0 << 26)			// Receiver timeout interrupt disable
    | (0 << 21)			// Driver Enable assertion time (disable)
    | (0 << 16)			// Driver Enable de-assertion time (disable)
    | (0 << 15)			// oversampling: x16
    | (0 << 14)			// Character match interrupt disable
    | (0 << 13)			// Mute mode disable
    | (0 << 11)			// Receive wakeup method (ignore, mute mode not used)
    | config_to_cr1_parity_bits (cfg.parity)
    | (0 << 8)			// PE interrupt disable
    | (0 << 7)			// TXE interrupt disable
    | (0 << 6)			// TC interrupt disable
    | (1 << 5)			// RXNE interrupt disable
    | (0 << 4)			// IDLE interrupt disable
    | (1 << 3)			// transmitter enable
    | (1 << 2)			// receiver enable
    | (1 << 0)			// USART enable
    | 0;

    m_cr1_txe_disabled = r.cr1;
    m_cr1_txe_enabled = m_cr1_txe_disabled | (1 << 7);

    return true;
  }

  void write (const void* data, unsigned int byte_count) noexcept
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
    // than isr_txe, this will deadlock.
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


  void reset_transmitter (void) noexcept { }

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

  unsigned int m_cr1_txe_enabled;
  unsigned int m_cr1_txe_disabled;

  void isr_pe (void) { }
  void isr_fe (void) { }
  void isr_nf (void) { }
  void isr_ore (void) { }
  void isr_idle (void) { }

  void isr_rxne (void)
  {
    // receive data register not empty -- we can receive one byte of data

    // copy data from usart to rx buffer
    rx_buffer_write_byte (regs ().rdr);

    // invoke callback
    m_rx_data_clb (m_rx_data_clb_user_p, (buffer_pos_t)(m_rx_wr_pos - m_rx_rd_pos));
  }

  void isr_tc (void)
  {
    // transmission complete.
    // do nothing.  handled by isr_txe
  }

  void disable_txe_interrupt (void) { regs ().cr1 = m_cr1_txe_disabled; }
  void enable_txe_interrupt (void) { regs ().cr1 = m_cr1_txe_enabled; }

  void isr_txe (void)
  {
    // transmit data register is empty
    // this TXE ISR is invoked constantly as long as the TXE interrupt is
    // enabled and the TXE flag is set.
    // it is disabled either in CR1 or by writing data to the data register.

    if (!tx_buffer_read_empty ())
      write_transmit_data (tx_buffer_read_byte ());
    else
      // disable TXE interrupt.
      // it will be re-enabled when sending more data.
      disable_txe_interrupt ();
  }

  void check_restart_transmission (void)
  {
    // just re-enable the TXE interrupt, which will invoke isr_txe at the
    // next best opporutinity.  isr_txe will check if there is any data
    // in the tx buffer and handle it.
    enable_txe_interrupt ();
  }


  void isr_lbdf (void) { }
  void isr_ctsif (void) { }
  void isr_cts (void) { }
  void isr_rtof (void) { }
  void isr_eobf (void) { }
  void isr_abre (void) { }
  void isr_abrf (void) { }
  void isr_busy (void) { }
  void isr_cmf (void) { }
  void isr_sbkf (void) { }
  void isr_rwu (void) { }
  void isr_wuf (void) { }
  void isr_teack (void) { }
  void isr_reack (void) { }



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

} // namespace stm32f0_usart
} // namespace dev

#endif // includeguard_dev_stm32f0_usart_hpp_includeguard

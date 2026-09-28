
/*

a device for using an RX SCI device in simple I2C master mode

in i2c mode external SCI clock is not supported and only the internal
on-chip baudrate generator can be used.

*/

#ifndef includeguard_dev_rx_sci_i2c_hpp_includeguard
#define includeguard_dev_rx_sci_i2c_hpp_includeguard

#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <atomic>

#include <utils/langcomp.hpp>

#include <dev/usart.hpp>
#include <dev/interrupt.hpp>
#include <dev/i2c.hpp>

#include <chrono>

#if __cpp_exceptions
  #define throw_i2c_error(...) throw i2c_error (__VA_ARGS__)
#else
  #define throw_i2c_error(...) __builtin_abort ()
#endif


namespace dev
{

template <typename SciDev>
class rx_sci_i2c_master final : public SciDev, public i2c_master
{
public:
  typedef SciDev sci_dev;

  using i2c_master::send;
  using i2c_master::recv;
  using i2c_master::send_recv;

  static constexpr unsigned int max_bitrate = SciDev::max_bitrate;


  [[gnu::cold]] rx_sci_i2c_master (void)
  : m_txi_isr (this), m_rxi_isr (this), m_tei_isr (this), m_eri_isr (this)
  {
    // FIXME: maybe implement dynamic per-device bit rate switching.
    //        although dynamic bit rate switching will probably not work with
    //        i2c, because slow devices might be confused by faster signals
    //        and wrongly try to respond.  so actually, the max. bitrate
    //        is defined by the connected devices.
    constexpr typename sci_dev::bitrate_params p =
	sci_dev::bitrate_to_params (sci_dev::max_bitrate, sci_dev::base_clock_8);

    m_rxi_flag = 0;

    auto ctrl_setting = typename sci_dev::control_t ()
	.set_clock_enable (sci_dev::on_chip_generator)
	.set_transmit_end_interrupt_enable (false)
	.set_multi_processor_interrupt_enable (false)
	.set_receive_enable (false)
	.set_transmit_enable (false)
	.set_receive_interrupt_enable (false)
	.set_transmit_interrupt_enable (false);

    this->set_control (ctrl_setting);

    this->set_mode (typename sci_dev::mode_t ()
	.set_clock_select (p.clk_sel)
	.set_multi_processor_mode (false)
	.set_stop_bits (sci_dev::stop_bits_1)
	.set_parity_enable (false)
	.set_char_length (sci_dev::charlen_8)
	.set_communication_mode (sci_dev::async_mode)
	.set_interface_mode (sci_dev::interface_mode_serial)
	.set_data_invert (sci_dev::no_invert_rx_tx)
	.set_data_direction (sci_dev::msb_first)
	.set_async_clock_source_select (sci_dev::ext_clock_input)
	.set_base_clock_select (sci_dev::base_clock_8)
	.set_noise_filter_enable (false));

    this->set_bit_rate (p.brr);

    this->set_noise_filter_setting (sci_dev::i2c_mode_div_1);

    this->set_i2c_mode (typename sci_dev::i2c_mode_t ()
		.set_i2c_mode_enable ()
		.set_ssda_output_delay (6)
		.set_interrupt_mode (sci_dev::rx_tx_interrupts)
		.set_clock_sync_enable ()
		.set_ack_transmit_enable ());

    // disable SPI mode
    this->set_spi_mode (typename sci_dev::spi_mode_t ()
		.set_ss_pin_enable (false)
		.set_cts_pin_enable (false)
		.set_master_or_slave_mode (sci_dev::master)
		.set_clock_polarity_invert (false)
		.set_clock_phase_delayed (false));

    this->set_i2c_idle_state ();

    this->set_control (ctrl_setting
//		.set_transmit_end_interrupt_enable ()
		.set_receive_enable ()
		.set_transmit_enable ()
		.set_receive_interrupt_enable ()
//		.set_transmit_interrupt_enable ()
		);

    // SciDev ctor will enable the device (disable module standby) but not the
    // interrupts.
    m_txi_isr.enable (sci_dev::txi_interrupt_type, sci_dev::txi_interrupt_priority);
    m_rxi_isr.enable (sci_dev::rxi_interrupt_type, sci_dev::rxi_interrupt_priority);
    m_tei_isr.enable (sci_dev::tei_interrupt_type, sci_dev::tei_interrupt_priority);
    m_eri_isr.enable (sci_dev::eri_interrupt_type, sci_dev::eri_interrupt_priority);
  }

  virtual i2c_device_id
  query_device_id (uint8_t slave_addr) override
  {
    /* FIXME: implement
	- send START condition
	- send command 0b1111'1000 (write to reserved device id address)
	- send slave address (LSB don't care)
	- wait for ACK by slave
	- send re-start condition
	- send command 0b1111'1001 (read from reserved device id address)
	- read 3 bytes from slave
	- send NACK to abort the read request and reset slave's state machine
	- send STOP command
    */
    throw_i2c_error ();
  }

  virtual bool
  send (uint8_t slave_addr, const void* tx_data, unsigned int count_bytes) override
  {
    return send_1 (slave_addr, (const unsigned char*)tx_data, count_bytes) >= 0;
  }

  using dev::i2c_master::send;

  virtual bool
  recv (uint8_t slave_addr, void* rx_data, unsigned int count_bytes) override
  {
    return recv_1 (slave_addr, (unsigned char*)rx_data, count_bytes) >= 0;
  }

  using dev::i2c_master::recv;

  virtual bool
  send_recv (uint8_t slave_addr,
	     const void* tx_data, unsigned int tx_count_bytes,
	     void* rx_data, unsigned int rx_count_bytes) override
  {
    return send_recv_1 (slave_addr, (const unsigned char*)tx_data, tx_count_bytes,
		        (unsigned char*)rx_data, rx_count_bytes) >= 0;
  }

  using dev::i2c_master::send_recv;

private:
  static auto constexpr TIMEOUT = std::chrono::seconds (1);


  void txi (void)
  {
    //m_txi_flag = 1;
  }

  void rxi (void)
  {
    m_rxi_flag = 1;
  }

  void tei (void)
  {
    //m_tei_flag = 1;
    //this->i2c_start_stop_reset_condition_completed_clear ();
  }

  void eri (void)
  {
    // this interrupt is triggered upon completion of generating a start,
    // restart or stop condition on the i2c bus.
    // this one can't trigger the DTC.
  }

  int rxi_wait (void)
  {
    auto start_time = std::chrono::high_resolution_clock::now ();

    while (true)
    {
      if (m_rxi_flag == 1)
      {
	m_rxi_flag = 0;
	return 0;
      }

      if (std::chrono::high_resolution_clock::now () - start_time > TIMEOUT)
	return -1;
    }
  }

  int sti_wait (void)
  {
    auto start_time = std::chrono::high_resolution_clock::now ();

    while (true)
    {
      if (this->i2c_start_stop_reset_condition_completed ())
      {
	this->i2c_start_stop_reset_condition_completed_clear ();
	return 0;
      }

      if (std::chrono::high_resolution_clock::now () - start_time > TIMEOUT)
	return -1;
    }
  }

  int txi_wait (void)
  {
    auto start_time = std::chrono::high_resolution_clock::now ();
    while (true)
    {
      if (this->status ().transmit_end ())
	return 0;

      if (std::chrono::high_resolution_clock::now () - start_time > TIMEOUT)
	return -1;
    }
  }

  int send_1 (uint8_t slave_addr, const unsigned char* data, unsigned int num)
  {
    unsigned int ucCount = 0;

    this->set_control (this->control ().set_receive_interrupt_enable (false));
    this->set_i2c_start_condition ();

    if (sti_wait () < 0)
      return -101;

    this->set_i2c_begin_serial_rx_tx ();

    // 7 bit address convention: bit 0 indicates read or write request.
    this->set_transmit_data (slave_addr & 0xFE);

    if (txi_wait () < 0)
      return -102;

    if (this->i2c_ack_received ())
    {
      while (ucCount < num)
      {
	this->set_transmit_data (*data++);
	ucCount++;

	if (txi_wait () < 0)
	  return -103;
      }
    }
    else
      return -106;

    this->set_i2c_reset_condition ();

    if (sti_wait () < 0)
      return -104;

    this->set_i2c_idle_state ();

    return ucCount != num ? -105 : 0;
  }

  int recv_1 (uint8_t slave_addr, unsigned char* data, unsigned int num)
  {
    unsigned int ucCount = 0;

    this->set_control (this->control ().set_receive_interrupt_enable (false));
    this->set_i2c_start_condition ();

    if (sti_wait () < 0)
      return -201;

    this->set_i2c_begin_serial_rx_tx ();

    this->set_transmit_data (slave_addr | 0x01);

    if (txi_wait () < 0)
      return -202;

    if (this->i2c_ack_received ())
    {
      this->set_i2c_mode (this->i2c_mode ().set_ack_transmit_enable (false));
      this->set_control (this->control ().set_receive_interrupt_enable (true));

      if (num != 1)
      {
	while (ucCount < (num - 1))
	{
	  this->set_transmit_data (0xFF);
	  if (rxi_wait () < 0)
	    return -203;

	  *data++ = this->receive_data ();
	  ucCount++;

	  if (txi_wait () < 0)
	    return -204;
	}
      }

      this->set_i2c_mode (this->i2c_mode ().set_ack_transmit_enable (true));

      this->set_transmit_data (0xFF);

      if (rxi_wait () < 0)
	return -205;

      *data++ = this->receive_data ();
      ucCount++;

      if (txi_wait () < 0)
	return -206;
    }

    this->set_i2c_reset_condition ();

    if (sti_wait () < 0)
      return -207;

    this->set_i2c_idle_state ();

    return ucCount != num ? -208 : 0;
  }


  int send_recv_1 (uint8_t slave_addr,
		   const unsigned char* tx_data, unsigned int tx_count,
		   unsigned char* rx_data, unsigned int rx_count)
  {
    unsigned int ucCount = 0;

    this->set_control (this->control ().set_receive_interrupt_enable (false));
    this->set_i2c_start_condition ();

    if (sti_wait () < 0)
      return -1;

    this->set_i2c_begin_serial_rx_tx ();

    this->set_transmit_data (slave_addr & 0xFE);

    if (txi_wait () < 0)
      return -2;

    if (this->i2c_ack_received ())
    {
      while (ucCount < tx_count)
      {
	this->set_transmit_data (*tx_data++);
	ucCount++;

	if (txi_wait () < 0)
	  return -3;
      }
    }

    if (ucCount != tx_count)
      return -4;

    ucCount = 0;

    this->set_i2c_stop_condition ();

    if (sti_wait () < 0)
      return -5;

    this->set_i2c_begin_serial_rx_tx ();

    this->set_transmit_data (slave_addr | 0x01);

    if (txi_wait () < 0)
      return -6;

    if (this->i2c_ack_received ())
    {
      this->set_i2c_mode (this->i2c_mode ().set_ack_transmit_enable (false));

      this->set_control (this->control ().set_receive_interrupt_enable (true));

      if (rx_count != 1)
      {
        while (ucCount < (rx_count - 1))
	{
	  this->set_transmit_data (0xFF);

	  if (rxi_wait () < 0)
	    return -7;

	  *rx_data++ = this->receive_data ();
	  ucCount++;

	  if (txi_wait () < 0)
	    return -8;
	}
      }

      this->set_i2c_mode (this->i2c_mode ().set_ack_transmit_enable (true));

      this->set_transmit_data (0xFF);

      if (rxi_wait () < 0)
        return -9;

      *rx_data++ = this->receive_data ();
      ucCount++;

      if (txi_wait () < 0)
        return -10;
    }

    this->set_i2c_reset_condition ();

    if (sti_wait () < 0)
      return -11;

    this->set_i2c_idle_state ();

    return ucCount != rx_count ? -12 : 0;
  }


public:
  typedef interrupt::connected_isr<typename sci_dev::txi_interrupt_line,
	interrupt::func<decltype (&rx_sci_i2c_master::txi), &rx_sci_i2c_master::txi>> txi_isr_t;

  typedef interrupt::connected_isr<typename sci_dev::rxi_interrupt_line,
	interrupt::func<decltype (&rx_sci_i2c_master::rxi), &rx_sci_i2c_master::rxi>> rxi_isr_t;

  typedef interrupt::connected_isr<typename sci_dev::tei_interrupt_line,
	interrupt::func<decltype (&rx_sci_i2c_master::tei), &rx_sci_i2c_master::tei>> tei_isr_t;

  typedef interrupt::connected_isr<typename sci_dev::eri_interrupt_line,
	interrupt::func<decltype (&rx_sci_i2c_master::eri), &rx_sci_i2c_master::eri>> eri_isr_t;

private:
  volatile unsigned char m_rxi_flag;

  txi_isr_t m_txi_isr;
  rxi_isr_t m_rxi_isr;
  tei_isr_t m_tei_isr;
  eri_isr_t m_eri_isr;
};

} // namespace dev
#endif // includeguard_dev_rx_sci_i2c_hpp_includeguard

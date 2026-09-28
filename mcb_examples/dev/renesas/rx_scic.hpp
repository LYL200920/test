
#ifndef includeguard_dev_rx_scic_hpp_includeguard
#define includeguard_dev_rx_scic_hpp_includeguard

#include <cstdint>

#include <dev/interrupt.hpp>

#include <utils/value_range.hpp>
#include <utils/bits.hpp>

namespace dev
{

// FIXME: maybe add external baudrate generator template argument.
// some SCI instances (SCI5, SCI6) can use a timer unit as a baudrate generator.
template <uintptr_t RegAddress, unsigned int SciNum,
	  unsigned int PeripheralClock, unsigned int MaxBitRate,
	  typename TXI_InterruptLine,
	  typename RXI_InterruptLine,
	  typename TEI_InterruptLine,
	  typename ERI_InterruptLine,
	  typename ModuleEnableDisableFunc >
class rx_scic
{
public:
  // transmit data empty
  typedef TXI_InterruptLine txi_interrupt_line;
  static constexpr interrupt::trigger_type txi_interrupt_type = interrupt::low_level;
  static constexpr unsigned int txi_interrupt_priority = interrupt::priority_7;

  // receive data full
  typedef RXI_InterruptLine rxi_interrupt_line;
  static constexpr interrupt::trigger_type rxi_interrupt_type = interrupt::low_level;
  static constexpr unsigned int rxi_interrupt_priority = interrupt::priority_7;

  // transmit end
  typedef TEI_InterruptLine tei_interrupt_line;
  static constexpr interrupt::trigger_type tei_interrupt_type = interrupt::low_level;
  static constexpr unsigned int tei_interrupt_priority = interrupt::priority_7;

  // receive error
  typedef ERI_InterruptLine eri_interrupt_line;
  static constexpr interrupt::trigger_type eri_interrupt_type = interrupt::low_level;
  static constexpr unsigned int eri_interrupt_priority = interrupt::priority_7;


  static constexpr unsigned int pclock_hz = PeripheralClock;
  static constexpr unsigned int max_bitrate = MaxBitRate;


// the SCI device itself doesn't use interrupts.  the interrupts are connected
// by the higher level device like uart, i2c, spi
// this way the code is easier to deal with and also efficient (no function call
// indirections or stored function pointers, which the compiler can't inline).

//  typedef interrupt::connected_isr<TXI_InterruptLine,
//	interrupt::func<decltype (&rx_scic::txi_isr), &rx_scic::txi_isr>> txi_isr_t;

//  typedef interrupt::connected_isr<RXI_InterruptLine,
//	interrupt::func<decltype (&rx_scic::rxi_isr), &rx_scic::rxi_isr>> rxi_isr_t;

//  typedef interrupt::connected_isr<TEI_InterruptLine,
//	interrupt::func<decltype (&rx_scic::tei_isr), &rx_scic::tei_isr>> tei_isr_t;

//  typedef interrupt::connected_isr<ERI_InterruptLine,
//	interrupt::func<decltype (&rx_scic::eri_isr), &rx_scic::eri_isr>> eri_isr_t;


  // ---------------------------------------------------------------------------
  // combined serial mode register (SMR)
  // smart card mode register (SCMR)
  // serial extended mode register (SEMR)
  //
  // it's easier to merge those mode registers.
  //
  // SMIF = 0 (not in smart card mode)
  // FIXME: add SMIF = 1
  enum smr_cks_t
  {
    pclk_1  = 0,  // PCLK/1 clock (BRR n = 0)
    pclk_4  = 1,  // PCLK/4 clock (BRR n = 1)
    pclk_16 = 2,  // PCLK/16 clock (BRR n = 2)
    pclk_64 = 3,  // PCLK/64 clock (BRR n = 3)
  };

  enum smr_stop_bits_t
  {
    stop_bits_1 = 0,
    stop_bits_2 = 1
  };

  enum smr_parity_mode_t
  {
    parity_even = 0,
    parity_odd = 1
  };

  enum smr_char_length_t
  {
    charlen_8 = 0,
    charlen_7 = 1
  };

  enum communication_mode_t
  {
    async_mode = 0,
    clock_sync_mode = 1
  };

  enum scmr_smif_t
  {
    interface_mode_serial = 0,
    interface_mode_smart_card = 1
  };

  enum scmr_sinv_t
  {
    no_invert_rx_tx = 0,
    invert_rx_tx = 1,
  };

  enum scmr_sdir_t
  {
    lsb_first = 0,
    msb_first = 1,
  };

  enum semr_acs0_t
  {
    ext_clock_input = 0,

    // valid only for SCI5, SCI6, SCI12 on RX63
    tmr_clock_input = 1
  };

  enum semr_abcs_t
  {
    // 16 base clock cycles for 1 bit period
    base_clock_16 = 0,

    // 8 base clock cycles for 1 bit period
    base_clock_8 = 1
  };

  class mode_t
  {
  public:
    constexpr mode_t (void) : m_smr_value (0), m_scmr_value (0b11110010), m_semr_value (0) { }

    explicit constexpr mode_t (uint8_t smr_val, uint8_t scmr_val, uint8_t semr_val)
    : m_smr_value (smr_val), m_scmr_value (scmr_val), m_semr_value (semr_val) { }

    constexpr uint8_t smr_value (void) const { return m_smr_value; }
    constexpr uint8_t scmr_value (void) const { return m_scmr_value; }
    constexpr uint8_t semr_value (void) const { return m_semr_value; }

    // clock select
    // can be changed only when transmission and reception are disabled.
    constexpr smr_cks_t clock_select (void) const { return (smr_cks_t)(m_smr_value & 3); }
    mode_t& set_clock_select (smr_cks_t val) { m_smr_value = (m_smr_value & ~3) | val; return *this; }

    // multi-processor mode
    // can be changed only when transmission and reception are disabled.
    constexpr bool multi_processor_mode (void) const { return smr_bit (2); }
    mode_t& set_multi_processor_mode (bool val) { return set_smr_bit (2, val); }

    // stop bit length
    // can be changed only when transmission and reception are disabled.
    // in reception, only the first stop bit is checked regardless of this bit
    // setting.  if the second stop bit is 0, it is treated as the start bit of
    // the next transmit frame.
    constexpr smr_stop_bits_t stop_bits (void) const { return (smr_stop_bits_t)smr_bit (3); }
    mode_t& set_stop_bits (smr_stop_bits_t val) { return set_smr_bit (3, val); }

    // parity mode
    // valid only when parity is enabled in asynchronous mode.
    // invalid when multi-processor mode is enabled.
    // can be changed only when transmission and reception are disabled.
    constexpr smr_parity_mode_t parity_mode (void) const { return (smr_parity_mode_t)smr_bit (4); }
    mode_t& set_parity_mode (smr_parity_mode_t val) { return set_smr_bit (4, val); }

    // parity enable
    // valid only in asynchronous mode.
    // invalid when multi-processor mode is enabled.  parity is not added or
    // checked in multi-processor mode.
    // can be changed only when transmission and reception are disabled.
    constexpr bool parity_enable (void) const { return smr_bit (5); }
    mode_t& set_parity_enable (bool val = true) { return set_smr_bit (5, val); }

    // character length -- selects how many data bits there.
    // (valid only in asynchronous mode)
    // can be changed only when transmission and reception are disabled.
    // in clock synchronous mode, a fixed data length of 8 bits is used.
    constexpr smr_char_length_t char_length (void) const { return (smr_char_length_t)smr_bit (6); }
    mode_t& set_char_length (smr_char_length_t val) { return set_smr_bit (6, val); }

    mode_t& set_char_length_bits (unsigned int val)
    {
      switch (val)
      {
	default: return *this;
	case 7: return set_char_length (charlen_7);
	case 8: return set_char_length (charlen_8);
      }
    }

    // communication mode
    // can be changed only when transmission and reception are disabled.
    communication_mode_t communication_mode (void) const { return (communication_mode_t)smr_bit (7); }
    mode_t& set_communication_mode (communication_mode_t val) { return set_smr_bit (7, val); }

    // after reset the serial mode is the default.
    constexpr scmr_smif_t interface_mode (void) const { return (scmr_smif_t)scmr_bit (0); }
    mode_t& set_interface_mode (scmr_smif_t val) { return set_scmr_bit (0, val); }

    constexpr scmr_sinv_t data_invert (void) const { return (scmr_sinv_t)scmr_bit (2); }
    mode_t& set_data_invert (scmr_sinv_t val) { return set_scmr_bit (2, val); }

    constexpr scmr_sdir_t data_direction (void) const { return (scmr_sdir_t)scmr_bit (3); }
    mode_t& set_data_direction (scmr_sdir_t val) { return set_scmr_bit (3, val); }

    // FIXME: add support for BCP2 and SMR.BCP0, SMR.BCP1

    constexpr semr_acs0_t async_clock_source_select (void) const { return (semr_acs0_t)semr_bit (0); }
    mode_t& set_async_clock_source_select (semr_acs0_t val) { return set_semr_bit (0, val); }

    // the base clock can be changed only when transmission and reception are
    // disabled (see set_control function / SCR register)
    constexpr semr_abcs_t base_clock_select (void) const { return (semr_abcs_t)semr_bit (4); }
    mode_t& set_base_clock_select (semr_abcs_t val) { return set_semr_bit (4, val); }

    // in asynchronous mode, the RXD input signal can be noise filtered.
    // in i2c mode, the SSCL and SSDA input signals can be noise filtered.
    // other modes should disable the noise filter.
    constexpr bool noise_filter_enable (void) const { return semr_bit (5); }
    mode_t& set_noise_filter_enable (bool val = true) { return set_semr_bit (5, val); }


  private:
    constexpr bool smr_bit (unsigned int n) const { return utils::get_bit (m_smr_value, n); }
    mode_t& set_smr_bit (unsigned int n, bool val) { m_smr_value = utils::set_bit (m_smr_value, n, val); return *this; }

    constexpr bool scmr_bit (unsigned int n) const { return utils::get_bit (m_scmr_value, n); }
    mode_t& set_scmr_bit (unsigned int n, bool val) { m_scmr_value = utils::set_bit (m_scmr_value, n, val); return *this; }

    constexpr bool semr_bit (unsigned int n) const { return utils::get_bit (m_semr_value, n); }
    mode_t& set_semr_bit (unsigned int n, bool val) { m_semr_value = utils::set_bit (m_semr_value, n, val); return *this; }

    uint8_t m_smr_value;
    uint8_t m_scmr_value;
    uint8_t m_semr_value;
  };


  // ---------------------------------------------------------------------------
  // FIXME: add SMIF = 1

  enum scr_cke_t
  {
    // to be used in asynchronous mode

    // on-chip baud rate generator is providing the SCI clock.  the SCKn pin
    // functions as I/O port.
    on_chip_generator = 0,

    // on-chip baud rate generator is providing the SCI clock.  the clock
    // signal is also output on the SCKn pin.
    on_chip_generator_sck_out = 1,

    // external clock is input from the SCKn pin or TMR clock input.
    // SEMR.ABCS = 0: SCK clock is 16x bit rate
    // SEMR.ABCS = 1: SCK clock is 8x bit rate
    // SEMR.ACS0 = 0: external clock input
    // SEMR.ACS0 = 1: TMR clock input (SCI5, SCI6, SCI12 only on RX63, RX64)
    sck_tmr_in = 2,

    // to be used in synchronous mode

    // internal clock, the SCKn pin functions as the clock output pin.
    sck_out = 0,

    // external clock, the SCKn pin functions as the clock input pin.
    //sck_in = 2
  };

  class control_t
  {
  public:
    constexpr control_t (void) : m_value (0) { }
    explicit constexpr control_t (uint8_t val) : m_value (val) { }

    constexpr uint8_t value (void) const { return m_value; }

    constexpr scr_cke_t clock_enable (void) const { return (scr_cke_t)(m_value & 3); }
    control_t& set_clock_enable (const scr_cke_t& val) { m_value = (m_value & ~3) | val; return *this; }

    // transmit end interrupt enable (TEIE).
    constexpr bool transmit_end_interrupt_enable (void) const { return bit (2); }
    control_t& set_transmit_end_interrupt_enable (bool val = true) { return set_bit (2, val); }

    constexpr bool multi_processor_interrupt_enable (void) const { return bit (3); }
    control_t& set_multi_processor_interrupt_enable (bool val = true) { return set_bit (3, val); }

    constexpr bool receive_enable (void) const { return bit (4); }
    control_t& set_receive_enable (bool val = true) { return set_bit (4, val); }

    constexpr bool transmit_enable (void) const { return bit (5); }
    control_t& set_transmit_enable (bool val = true) { return set_bit (5, val); }

    constexpr bool receive_interrupt_enable (void) const { return bit (6); }
    control_t& set_receive_interrupt_enable (bool val = true) { return set_bit (6, val); }

    constexpr bool transmit_interrupt_enable (void) const { return bit (7); }
    control_t& set_transmit_interrupt_enable (bool val = true) { return set_bit (7, val); }

  private:
    constexpr bool bit (unsigned int n) const { return utils::get_bit (m_value, n); }
    control_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

    uint8_t m_value;
  };

  // ---------------------------------------------------------------------------
  // SMIF = 0 (not in smart card mode)
  // FIXME: add SMIF = 1
  class status_t
  {
  public:
    constexpr status_t (void) : m_value (0) { }
    explicit constexpr status_t (uint8_t val) : m_value (val) { }

    constexpr uint8_t value (void) const { return m_value; }

    // the multi-processor bit value that should be transmitted.
    constexpr bool multi_processor_tx_bit (void) const { return bit (0); }
    status_t& set_multi_processor_tx_bit (bool val) { return set_bit (0, val); }

    // the multi-processor bit value that has been received.
    constexpr bool multi_processor_rx_bit (void) const { return bit (1); }

    // indicates whether the transfer of a character has been completed or not.
    constexpr bool transmit_end (void) const { return bit (2); }

    // indicates whether a parity error has been detected or not.
    // can only be cleared.
    constexpr bool parity_error (void) const { return bit (3); }
    status_t& clear_parity_error (void) { return set_bit (3, false); }

    // indicates whether a framing error has been detected or not.
    // can only be cleared.
    constexpr bool framing_error (void) const { return bit (4); }
    status_t& clear_framing_error (void) { return set_bit (4, false); }

    // indicates whether an overrun error has been detected or not.
    // can only be cleared.
    constexpr bool overrun_error (void) const { return bit (5); }
    status_t& clear_overrun_error (void) { return set_bit (5, false); }

  private:
    constexpr bool bit (unsigned int n) const { return utils::get_bit (m_value, n); }
    status_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

    uint8_t m_value;
  };

  // ---------------------------------------------------------------------------

  enum snfr_t
  {
    async_mode_div_1 = 0,
    i2c_mode_div_1 = 1,
    i2c_mode_div_2 = 2,
    i2c_mode_div_4 = 3,
    i2c_mode_div_8 = 4
  };


  // ---------------------------------------------------------------------------

  enum simr2_iicintm_t
  {
    ack_nack_interrupts = 0,
    rx_tx_interrupts = 1
  };

  enum simr3_pin_output_mode_t
  {
    output_serial_data = 0,
    output_low = 2,
    output_highz = 3
  };

  class i2c_mode_t
  {
  public:
    constexpr i2c_mode_t (void) : m_simr1 (0), m_simr2 (0) { }
    explicit constexpr i2c_mode_t (uint8_t v1, uint8_t v2)
    : m_simr1 (v1), m_simr2 (v2) { }

    constexpr uint8_t value1 (void) const { return m_simr1; }
    constexpr uint8_t value2 (void) const { return m_simr2; }

    // do not enable SMIF (smart card interface mode) and i2c mode at the same
    // time.  only one mode can be set.  if neither smart card nor i2c mode is
    // set, the SCI will operate in normal async/sync mode or simple SPI mode.
    constexpr bool i2c_mode_enable (void) const { return bit (m_simr1, 0); }
    i2c_mode_t& set_i2c_mode_enable (bool val = true) { return set_bit (m_simr1, 0, val); }

    // specify by how many clock cycles the SSDA pin output is delayed, relative
    // to the falling edge of the SSCL pin output.
    // in I2C mode the minimum value should be 1.
    constexpr unsigned int ssda_output_delay (void) const { return m_simr1 >> 3; }
    i2c_mode_t& set_ssda_output_delay (const utils::clamped_value<unsigned int, 0, 31>& val)
    {
      m_simr1 = (m_simr1 & ~(31u << 3)) | (val << 3);
      return *this;
    }

    constexpr simr2_iicintm_t interrupt_mode (void) const { return (simr2_iicintm_t)bit (m_simr2, 0); }
    i2c_mode_t& set_interrupt_mode (simr2_iicintm_t val) { return set_bit (m_simr2, 0, val); }

    // when clock sync is enabled, the internally generated SSCL clock signal
    // is synchronized when the SSCL bit has been placed at the low level in
    // case of a wait inserted by the other device.
    constexpr bool clock_sync_enable (void) const { return bit (m_simr2, 1); }
    i2c_mode_t& set_clock_sync_enable (bool val = true) { return set_bit (m_simr2, 1, val); }

    // when enabled, transmitted data will contain ACK bits.  enable this when
    // ACK and NACK bits are received.
    constexpr bool ack_transmit_enable (void) const { return bit (m_simr2, 5); }
    i2c_mode_t& set_ack_transmit_enable (bool val = true) { return set_bit (m_simr2, 5, val); }

  private:
    constexpr bool bit (uint8_t val, unsigned int n) const { return utils::get_bit (val, n); }
    i2c_mode_t& set_bit (uint8_t& vval, unsigned int n, bool val) { vval = utils::set_bit (vval, n, val); return *this; }

    uint8_t m_simr1;
    uint8_t m_simr2;
  };


  // ---------------------------------------------------------------------------

  enum spmr_mss_t
  {
    master = 0,
    slave = 1
  };

  class spi_mode_t
  {
  public:
    constexpr spi_mode_t (void) : m_value (0) { }
    explicit constexpr spi_mode_t (uint8_t val) : m_value (val) { }

    constexpr uint8_t value (void) const { return m_value; }

    constexpr bool ss_pin_enable (void) const { return bit (0); }
    spi_mode_t& set_ss_pin_enable (bool val = true) { return set_bit (0, val); }

    // when cts pin is off, rts pin is on
    constexpr bool cts_pin_enable (void) const { return bit (1); }
    spi_mode_t& set_cts_pin_enable (bool val = true) { return set_bit (1, val); }

    constexpr spmr_mss_t master_or_slave_mode (void) const { return (spmr_mss_t)bit (2); }
    spi_mode_t& set_master_or_slave_mode (spmr_mss_t val) { return set_bit (2, val); }

    constexpr bool mode_fault_error (void) const { return bit (4); }
    spi_mode_t& set_mode_fault_error (bool val = true) { return set_bit (4, val); }

    constexpr bool clock_polarity_invert (void) const { return bit (6); }
    spi_mode_t& set_clock_polarity_invert (bool val = true) { return set_bit (6, val); }

    constexpr bool clock_phase_delayed (void) const { return bit (7); }
    spi_mode_t& set_clock_phase_delayed (bool val = true) { return set_bit (7, val); }

  private:
    constexpr bool bit (unsigned int n) const { return utils::get_bit (m_value, n); }
    spi_mode_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

    uint8_t m_value;
  };


  // ---------------------------------------------------------------------------
  // serial mode register (SMR)
  // smart card mode register (SCMR)
  // serial extended mode register (SEMR)

  // SMIF = 0 (not in smart card mode)
  // FIXME: add SMIF = 1

  mode_t mode (void) const { return mode_t (regs ().smr, regs ().scmr, regs ().semr); }
  void set_mode (const mode_t& val)
  {
    regs ().smr = val.smr_value ();

    // some of the bits in SCMR are undefined and should be written as '1'.
    // FIXME: for now always set BCP2 = 1
    //        which is the inital value of 32 clock cycles.

    regs ().scmr = val.scmr_value () | 0b0111'0010 | 0b1000'0000;

    // some of the reserved bits should always be written as 0.
    regs ().semr = val.semr_value () & 0b00110001;
  }

  // ---------------------------------------------------------------------------

  // the bit rate controls the divisor of the clock.  the higher the bit rate
  // value, the slower the speed.  the initial value (after reset) is 0xFF.
  uint8_t bit_rate (void) const { return regs ().bbr; }
  void set_bit_rate (uint8_t val) { regs ().bbr = val; }


  // ---------------------------------------------------------------------------
  // FIXME: add SMIF = 1

  control_t control (void) const { return control_t (regs ().scr); }

  void set_control (const control_t& val)
  {
    // the receive/transmit enable bits will be picked up by the external
    // transceiver and might be altered.
    // this allows controlling an external transceiver device from the SCI.
//    regs ().scr = ExtTransceiver::set_control (val, *this).value ();
    regs ().scr = val.value ();
  }

  // ---------------------------------------------------------------------------
  // SMIF = 0 (not in smart card mode)
  // FIXME: add SMIF = 1

  status_t status (void) const { return status_t (regs ().ssr); }
  void set_status (const status_t& val)
  {
    // bit 7,6 are undefined when read and should be written as '1'.
    regs ().ssr = val.value () | 0b1100'0000;
  }

  // ---------------------------------------------------------------------------

  // a write of the transmit data register will start a new character transfer.
  // the register can be read back at any time.  the read access has no further
  // side effects.
  // when the SCI detects that the TSR (hidden tx shift register) is empty,
  // it transfers the transmit data written to TSR and starts transmission.
  // TSR and TDR form a double buffer and allow continuous transmit operations
  // of multiple bytes.  write data to TDR only once after each transmit data
  // empty interrupt (TXI).  the initial value after reset is 0xFF.
  uint8_t transmit_data (void) const { return regs ().tdr; }
  void set_transmit_data (uint8_t val) { regs ().tdr = val; }

  // when the SCI has received one frame (character) of serial data, it
  // transfers the received serial data from RSR (hidden rx shift register) to
  // RDR where it is stored for CPU access.  after that, RSR can receive the
  // next data.  this means, RSR and RDR function as a double buffer and allow
  // continuous receive operations of multiple bytes.  the receive data should
  // be read only once after a data full interrupt (RXI) has occured.
  // this is a read-only register.  the initial value after reset is 0x00.
  uint8_t receive_data (void) const { return regs ().rdr; }

  // ---------------------------------------------------------------------------

  snfr_t noise_filter_setting (void) const { return snfr_t (regs ().snfr); }
  void set_noise_filter_setting (snfr_t val) { regs ().snfr = val; }

  // ---------------------------------------------------------------------------

  i2c_mode_t i2c_mode (void) const { return i2c_mode_t (regs ().simr1, regs ().simr2); }
  void set_i2c_mode (const i2c_mode_t& val)
  {
    regs ().simr1 = val.value1 () & 0b11111001;
    regs ().simr2 = val.value2 () & 0b00100011;
  }

  // ---------------------------------------------------------------------------

  // SIMR3 is used to control the simple I2C mode start and stop conditions,
  // and to hold the SSDAn and SSCLn pins at fixed levels.

  // only generate a start condition after checking the bus state and
  // confirming that the bus is free.

  void set_i2c_start_condition (void) { regs ().simr3 = 0b01'01'0'001; }
  void set_i2c_stop_condition  (void) { regs ().simr3 = 0b01'01'0'010; }
  void set_i2c_reset_condition (void) { regs ().simr3 = 0b01'01'0'100; }

  // don't generate start or stop sequence, put lines into highz mode.
  void set_i2c_idle_state (void) { regs ().simr3 = 0b11'11'0'000; }

  // disable i2c mode
  void set_i2c_disable_state (void) { regs ().simr3 = 0b00'00'0'000; }

  // after generating a start or stop condition, proceed with sending serial data.
  void set_i2c_begin_serial_rx_tx (void) { regs ().simr3 = 0b00'00'0'000; }

  bool i2c_start_stop_reset_condition_completed (void) const { return regs ().simr3 & 0b00001000; }
  void i2c_start_stop_reset_condition_completed_clear (void) { regs ().simr3 &= 0b11110111; }

  simr3_pin_output_mode_t i2c_ssda_output_mode (void) const { return (simr3_pin_output_mode_t)((regs ().simr3 >> 4) & 3); }
  void set_i2c_ssda_output_mode (simr3_pin_output_mode_t val) { regs ().simr3 = (regs ().simr3 & ~(3 << 4)) | (val << 4); }

  simr3_pin_output_mode_t i2c_sscl_output_mode (void) const { return (simr3_pin_output_mode_t)((regs ().simr3 >> 6) & 3); }
  void set_i2c_sscl_output_mode (simr3_pin_output_mode_t val) { regs ().simr3 = (regs ().simr3 & ~(3 << 6)) | (val << 6); }

  // i2c status register SISR contains only one bit
  bool i2c_ack_received (void) const { return (regs ().sisr & 1) == 0; }
  bool i2c_nack_received (void) const { return (regs ().sisr & 1) == 1; }
  void i2c_clear_ack_ack_received (void) { regs ().sisr = 0; }

  // ---------------------------------------------------------------------------

  spi_mode_t spi_mode (void) const { return spi_mode_t (regs ().spmr); }
  void set_spi_mode (const spi_mode_t& val) { regs ().spmr = val.value () & 0b11010111; }

  // ---------------------------------------------------------------------------

  [[gnu::cold]] rx_scic (void)
  {
    set_device_enable (true);
  }

  [[gnu::cold]] ~rx_scic (void)
  {
    set_device_enable (false);
  }

  void set_device_enable (bool val)
  {
    ModuleEnableDisableFunc () (val);
  }

  static constexpr unsigned int brr_scale (semr_abcs_t val)
  {
    return val == base_clock_8 ? 16 : val == base_clock_16 ? 32
	   : brr_scale_fail ();
  }

  struct bitrate_params
  {
    // actual bitrate and error (deviation from requested value)
    unsigned int bitrate;

    // register values
    unsigned int brr;
    smr_cks_t clk_sel;

    // FIXME: extend this somehow to support TMR as external baudrate source.
  };

  static constexpr bitrate_params
  bitrate_to_params (unsigned int bitrate, semr_abcs_t clk_scale)
  {
    const unsigned int s = brr_scale (clk_scale);

    // start with highest clock value and reduce the clock value if the brr
    // value overflows.
    smr_cks_t clk_div = pclk_1;
    unsigned int brr = (pclock_hz / (bitrate*s)) - 1;
    unsigned int actual_bitrate = pclock_hz / ((brr + 1) * s);

    if (brr > 255)
    {
      clk_div = pclk_4;
      brr = ((pclock_hz/4) / (bitrate*s)) - 1;
      actual_bitrate = (pclock_hz/4) / ((brr + 1) * s);

      if (brr > 255)
      {
	clk_div = pclk_16;
	brr = ((pclock_hz/16) / (bitrate*s)) - 1;
	actual_bitrate = (pclock_hz/16) / ((brr + 1) * s);

	if (brr > 255)
	{
	  clk_div = pclk_64;
	  brr = ((pclock_hz/64) / (bitrate*s)) - 1;
	  actual_bitrate = (pclock_hz/64) / ((brr + 1) * s);
	}
      }
    }

    return { actual_bitrate, brr, clk_div };
  }

  void set_bitrate_params (const bitrate_params& p)
  {
    set_bit_rate (p.brr);
  }

protected:
  struct regs_t
  {
    volatile uint8_t smr;   // serial mode register      sci0 = 0x0008A000
    volatile uint8_t bbr;   // bit rate register         sci0 = 0x0008A001
    volatile uint8_t scr;   // serial control register   sci0 = 0x0008A002
    volatile uint8_t tdr;   // transmit data register    sci0 = 0x0008A003
    volatile uint8_t ssr;   // serial status register    sci0 = 0x0008A004
    volatile uint8_t rdr;   // receive data register     sci0 = 0x0008A005
    volatile uint8_t scmr;  // smart card mode register  sci0 = 0x0008A006
    volatile uint8_t semr;  // serial extended mode reg  sci0 = 0x0008A007
    volatile uint8_t snfr;  // noise filter setting reg  sci0 = 0x0008A008
    volatile uint8_t simr1; // i2c mode regsiter 1       sci0 = 0x0008A009
    volatile uint8_t simr2; // i2c mode register 2       sci0 = 0x0008A00A
    volatile uint8_t simr3; // i2c mode register 3       sci0 = 0x0008A00B
    volatile uint8_t sisr;  // i2c status register       sci0 = 0x0008A00C
    volatile uint8_t spmr;  // spi mode register         sci0 = 0x0008A00D
  };

  static_assert (sizeof (regs_t) == 14, "");

  const regs_t& regs (void) const { return *(const regs_t*)RegAddress; }
  regs_t& regs (void) { return *(regs_t*)RegAddress; }

  static unsigned int brr_scale_fail (void);
};

} // namespace dev
#endif // includeguard_dev_rx_scic_hpp_includeguard

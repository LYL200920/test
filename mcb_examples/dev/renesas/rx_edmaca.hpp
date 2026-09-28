/*

RX EDMACa device driver.

almost the same as EDMAC, with the following differences:

- new registers: PTPEDMAC.EESR, PTPEDMAC.EESIPR
- EDMACn.TRSCER has fewer defined bits (desc_status_copy_bits_t)
- larger receive FIFO selectable in FDR
- RMCR.RNC bit is undefined
- transmit and receive descriptors have some added semantics for PTPEDMAC

*/

#ifndef includeguard_rx_edmaca_hpp_includeguard
#define includeguard_rx_edmaca_hpp_includeguard

#include "rx_edmac.hpp"

namespace dev
{
namespace rx_edmaca
{

using desc_size = rx_edmac::desc_size;
using data_endian = rx_edmac::data_endian;
using mode_t = rx_edmac::mode_t;
using desc_type = rx_edmac::desc_type;


class tx_desc
{
public:
  class status_t
  {
  public:
    constexpr status_t (void) : m_value (0) { }
    constexpr explicit status_t (uint32_t val) : m_value (val) { }

    constexpr uint32_t value (void) const { return m_value; }

    bool retry_over (void) const { return m_value & (1 << 0); }
    bool delayed_collision (void) const { return m_value & (1 << 1); }
    bool carrier_lost (void) const { return m_value & (1 << 2); }
    bool no_carrier (void) const { return m_value & (1 << 3); }
    bool aborted (void) const { return m_value & (1 << 8); }

    // cumulative error bit.  set if one of the error bits is set.
    bool error (void) const { return m_value & (1 << 27); }

    bool active (void) const { return m_value & (1 << 31); }

    // for PTPEDMAC bit 0 is redefined
    bool tx_src_mac_mismatch (void) const { return m_value & (1 << 0); }

  private:
    uint32_t m_value;
  };

  constexpr tx_desc (void) : m_td0 { 0 }, m_td1 { 0 }, m_td2 { nullptr } { };

  // enable interrupt after the hardware device has completed the
  // write-back of this descriptor.
  bool write_back_completion_interrupt_enable (void) const { return m_td0 & (1 << 26); }
  tx_desc& set_write_back_completion_interrupt_enable (bool val = true) { m_td0 = (m_td0 & ~(1<<26)) | (val << 26); return *this; }

  bool ring_end (void) const { return m_td0 & (1 << 30); }
  void set_ring_end (bool val) { m_td0 = (m_td0 & ~(1 << 30)) | (val << 30); }

  // notice that the active bit is written back by the hardware device.
  // thus it's better to access it via the status word, which will always
  // do a memory read.
  bool active (void) const { return m_td0 & (1 << 31); }
  void set_active (bool val) { m_td0 = (m_td0 & ~(1 << 31)) | (val << 31); }

  status_t status (void) const { return status_t (*(volatile const uint32_t*)&m_td0); }

  desc_type type (void) const { return (desc_type)((m_td0 >> 28) & 3); }
  void set_type (desc_type val) { m_td0 = (m_td0 & ~(3 << 28)) | ((unsigned int)val << 28); }

  uint16_t data_length (void) const { return m_td1.tbl; }
  void set_data_length (uint16_t val) { m_td1.tbl = val; }

  const void* data_ptr (void) const { return m_td2; }
  void set_data_ptr (const void* val) { m_td2 = val; }

private:
  uint32_t m_td0;

  union
  {
    struct
    {
      #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	uint16_t res;
	uint16_t tbl;
      #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	uint16_t tbl;
	uint16_t res;
      #endif
    };
    uint32_t val32;
  } m_td1;

  const void* m_td2;
};

static_assert (sizeof (tx_desc) == 12, "");

// a basic receive descriptor.  the first 32 bit word and the length
// field is written back by the hardware device into memory
// to update the descriptor state.  the total descriptor length can be either
// 16, 32 or 64 bytes.  when setting up the descriptor array, make sure to
// use an appropritate rx_desc sub class that meets the descriptor size
// requirements.
class rx_desc
{
public:
  class status_t
  {
  public:
    constexpr status_t (void) : m_value (0) { }
    constexpr explicit status_t (uint32_t val) : m_value (val) { }

    constexpr uint32_t value (void) const { return m_value; }

    bool crc_error (void) const { return m_value & (1 << 0); }
    bool phy_error (void) const { return m_value & (1 << 1); }
    bool truncated (void) const { return m_value & (1 << 2); }
    bool overlong (void) const { return m_value & (1 << 3); }
    bool residual_bit (void) const { return m_value & (1 << 4); } // alignment error
    bool multicast (void) const { return m_value & (1 << 7); }
    bool aborted (void) const { return m_value & (1 << 8); }
    bool fifo_overflow (void) const { return m_value & (1 << 9); }

    // cumulative error bit.  set if one of the error bits is set.
    bool error (void) const { return m_value & (1 << 27); }

    bool active (void) const { return m_value & (1 << 31); }

    // the descriptor type that has been written back by the hardware device
    // after receiving a frame.
    desc_type type (void) const { return (desc_type)((m_value >> 28) & 3); }

    // for PTPEDMAC these are redefined
    bool receive_port (void) const { return m_value & (1 << 7); }
    bool ptpv2 (void) const { return m_value & (1 << 4); }
    unsigned int ptp_message_type (void) const { return m_value & 0b1111; }

  private:
    uint32_t m_value;
  };

  constexpr rx_desc (void) : m_rd0 { 0 }, m_rd1 { 0 }, m_rd2 { nullptr } { }

  bool ring_end (void) const { return m_rd0 & (1 << 30); }
  void set_ring_end (bool val) { m_rd0 = (m_rd0 & ~(1 << 30)) | (val << 30); }

  // notice that the active bit is written back by the hardware device.
  // thus it's better to access it via the status word, which will always
  // do a memory read.
  bool active (void) const { return m_rd0 & (1 << 31); }
  void set_active (bool val) { m_rd0 = (m_rd0 & ~(1 << 31)) | (val << 31); }

  status_t status (void) const { return status_t (*(volatile const uint32_t*)&m_rd0); }

  // notice that the descriptor type is written back by the hardware device
  // after receiving a frame.  e.g. it can change an "desc_scatter" descriptor
  // into an "desc_scatter_begin" or "desc_scatter_end" descriptor.
  desc_type type (void) const { return (desc_type)((m_rd0 >> 28) & 3); }
  void set_type (desc_type val) { m_rd0 = (m_rd0 & ~(3 << 28)) | ((unsigned int)val << 3); }

  // the maximum number of bytes this buffer can hold.
  // the buffer size should be aligned to 32 bytes.
  // this value is only read by the hardware device and not modified.
  uint16_t max_data_length (void) const { return m_rd1.rbl; }
  void set_max_data_length (uint16_t val) { m_rd1.rbl = val; }

  // the number of bytes that have been written into the buffer during
  // reception, excluding any padding bytes.  this field is updated by
  // the hardware device.
  uint16_t data_length (void) const { return *(volatile const uint16_t*)&m_rd1.rfl; }

  // data pointer.  should be 32 byte aligned.
  void* data_ptr (void) const { return m_rd2; }
  void set_data_ptr (void* val) { m_rd2 = val; }

private:
  uint32_t m_rd0;

  union
  {
    struct
    {
      #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	uint16_t rfl;
	uint16_t rbl;
      #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	uint16_t rbl;
	uint16_t rfl;
      #endif
    };
    uint32_t val32;
  } m_rd1;

  void* m_rd2;
};

static_assert (sizeof (rx_desc) == 12, "");

enum struct receive_mode
{
  // after receiving one frame, the receive request bit is cleared
  // automatically by the hardware.  to start reception of the next frame,
  // "edmac_start_receive" must be invoked.  in this mode interrupt control
  // for each frame is possible.  "edmac_stop_receive" has no effect.
  single_frame = 0b00,

  // once "edmac_start_receive" has been called, the hardware will continue
  // receiving frames until it fetches a descriptor that has its active bit
  // set to 0, or until "edmac_stop_receive" is invoked.  "edmac_start_receive"
  // has to be invoked to start reception again.
  multi_frame = 0b01,

  // even if a descriptor is fetched that has the active bit set to 0,
  // do not stop reception.
  // this mode is not supported by all etherc variants, e.g. SH7786.
  // single_frame_cont = 0b10,
  // multi_frame_cont = 0b11
};

using etherc_edmac_status_t = rx_edmac::etherc_edmac_status_t;

// specify which etherc_edmac_status bits should be written back to the
// receive/transmit descriptor status bits.
class desc_status_copy_bits_t
{
public:
  constexpr desc_status_copy_bits_t (void) : m_value (0) { }
  constexpr explicit desc_status_copy_bits_t (uint32_t val) : m_value (val) { }

  constexpr uint32_t value (void) const { return m_value; }

  // notice that the bits are inverted.
  // 0 = the bit is copied into the descriptor (on)
  // 1 = the bit is not copied into the descriptor (off)

  // not supported anymore?
  bool rx_crc_error (void) const { return !get_bit (0); }
  desc_status_copy_bits_t& set_rx_crc_error (bool val = true) { return set_bit (0, !val); }

  // not supported anymore?
  bool rx_phy_error (void) const { return !get_bit (1); }
  desc_status_copy_bits_t& set_rx_phy_error (bool val = true) { return set_bit (1, !val); }

  // not supported anymore?
  bool rx_truncated (void) const { return !get_bit (2); }
  desc_status_copy_bits_t& set_rx_truncated (bool val = true) { return set_bit (2, !val); }

  // not supported anymore?
  bool rx_overlong (void) const { return !get_bit (3); }
  desc_status_copy_bits_t& set_rx_overlong (bool val = true) { return set_bit (3, !val); }

  bool rx_residual_bit (void) const { return !get_bit (4); }
  desc_status_copy_bits_t& set_rx_residual_bit (bool val = true) { return set_bit (4, !val); }

  bool rx_multicast (void) const { return !get_bit (7); }
  desc_status_copy_bits_t& set_rx_multicast (bool val = true) { return set_bit (7, !val); }

  // not supported anymore?
  bool tx_retry_over (void) const { return !get_bit (8); }
  desc_status_copy_bits_t& set_tx_retry_over (bool val = true) { return set_bit (8, !val); }

  // not supported anymore?
  bool tx_delayed_collision (void) const { return !get_bit (9); }
  desc_status_copy_bits_t& set_tx_delayed_collision (bool val = true) { return set_bit (9, !val); }

  // not supported anymore?
  bool tx_carrier_lost (void) const { return !get_bit (10); }
  desc_status_copy_bits_t& set_tx_carrier_lost (bool val = true) { return set_bit (10, !val); }

  // not supported anymore?
  bool tx_no_carrier (void) const { return !get_bit (11); }
  desc_status_copy_bits_t& set_tx_no_carrier (bool val = true) { return set_bit (11, !val); }

private:
  constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
  desc_status_copy_bits_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

  uint32_t m_value;
};

using fifo_sizes_t = rx_edmac::fifo_sizes_t;
using tx_interrupt_mode = rx_edmac::tx_interrupt_mode;

template < uintptr_t RegAddress, unsigned int ModuleClockHz,
	   typename EtherC,
	   typename InterruptLine,
	   typename LinkStatusVirtualInterruptLine,
	   typename ModuleEnableDisableFunc >
class hw_inst
{
public:
  using etherc_t = EtherC;
  using mode_t = rx_edmaca::mode_t;
  using desc_type = rx_edmaca::desc_type;
  using desc_size = rx_edmac::desc_size;
  using data_endian = rx_edmac::data_endian;
  using tx_desc = rx_edmaca::tx_desc;
  using rx_desc = rx_edmaca::rx_desc;
  using etherc_edmac_status_t = rx_edmaca::etherc_edmac_status_t;
  using desc_status_copy_bits_t = rx_edmaca::desc_status_copy_bits_t;
  using receive_mode = rx_edmaca::receive_mode;
  using tx_interrupt_mode = rx_edmac::tx_interrupt_mode;


  static constexpr unsigned int clock_hz = ModuleClockHz;

  hw_inst (etherc_t& etherc) : m_etherc (etherc), m_int (this)
  {
    ModuleEnableDisableFunc () (true);
    m_int.enable (interrupt::low_level, interrupt::priority_5);
  }

  ~hw_inst (void)
  {
    ModuleEnableDisableFunc () (false);
  }

  etherc_t& etherc (void) const { return m_etherc; }

  void reset (void)
  {
    regs ().edmr = regs ().edmr | 1;

    // wait for at least 64 "internal bus" clock cycles.
    // let's assume this is the same as module clock.
    std::chrono::duration<unsigned int, std::ratio<1, clock_hz>> wait_time (64);
    std::this_thread::sleep_for (wait_time);
  }

  mode_t mode (void) const { return mode_t (regs ().edmr); }
  void set_mode (mode_t val) { regs ().edmr = val.value (); }

  // start edmac transmission and query its current state.
  void start_transmit (void) { regs ().edtrr = 1; }
  bool transmitting (void) const { return regs ().edtrr != 0; }

  receive_mode recv_mode (void) const { return (receive_mode)(regs ().rmcr); }
  void set_recv_mode (receive_mode val) { regs ().rmcr = (unsigned int)val; }

  // start/stop edmac reception and query its current state.
  void start_receive (void) { regs ().edrrr = 1; }
  void stop_receive (void) { regs ().edrrr = 0; }
  bool receiving (void) const { return regs ().edrrr != 0; }

  // transmit descriptor array start address.
  // do not modify while transmission is enabled.
  // the address should be aligned to the size of the descriptor
  // (16, 32, 64 byte aligned).
  const void* transmit_desc_array (void) const { return (const void*)(regs ().tdlar); }
  void set_transmit_desc_array (const void* val) { regs ().tdlar = (uint32_t)val; }

  // receive descriptor list start address.
  // do not modify while reception is enabled.
  // the address should be aligned to the size of the descriptor
  // (16, 32, 64 byte aligned).
  const void* receive_desc_array (void) const { return (const void*)(regs ().rdlar); }
  void set_receive_desc_array (const void* val) { regs ().rdlar = (uint32_t)val; }

  // writing the status will clear the corresponding interrupts.
  etherc_edmac_status_t etherc_edmac_status (void) const { return etherc_edmac_status_t (regs ().eesr); }
  void set_etherc_edmac_status (etherc_edmac_status_t val) { regs ().eesr = val.value (); }

  // specify which bits in the status register will trigger an interrupt.
  etherc_edmac_status_t etherc_edmac_status_interrupts (void) const { return etherc_edmac_status_t (regs ().eesipr); }
  void set_etherc_edmac_status_interrupts (etherc_edmac_status_t val) { regs ().eesipr = val.value (); }

  desc_status_copy_bits_t desc_status_copy_bits (void) const { return desc_status_copy_bits_t (regs ().trscer); }
  void set_desc_status_copy_bits (desc_status_copy_bits_t val) { regs ().trscer = val.value (); }

  // number of frames that could not be stored in the receive buffer and
  // were discarded during reception.
  // counter stops counting once it reaches 0xFFFF.
  // it's only possible to reset the counter to zero (any write value).
  uint16_t missed_frame_count (void) const { return regs ().rmfcr; }
  void reset_missed_frame_count (void) { regs ().rmfcr = 0; }

  // specify the threshold at which the first transmission is started in
  // 4-byte units.  the etherc starts transmission when the amount of data
  // in the transmit FIFO exceeds this threshold value.  do not change this
  // value while transmission is enabled.
  // threshold = 0: store and forward mode (wait for full frame to be in the fifo)
  // threshold <= 48: prohibited
  // threshold > 2048: prohibited
  uint16_t transmit_fifo_threshold (void) const { return regs ().tftr << 2; }
  void set_transmit_fifo_threshold (uint16_t val) { regs ().tftr = val >> 2; }

  // number of FIFO underrun that have occurred during transmission.
  // counter stops counting once it reaches 0xFFFF.
  // it's only possible to reset the counter to zero (any write value).
  uint16_t transmit_fifo_underrun_count (void) const { return regs ().tfucr; }
  void reset_transmit_fifo_underrun_count (void) { regs ().tfucr = 0; }

  // number if FIFO overflows that have occured during reception.
  // counter stops counting once it reaches 0xFFFF.
  // it's only possible to reset the counter to zero (any write value).
  uint16_t receive_fifo_overflow_count (void) const { return regs ().rfocr; }
  void reset_receive_fifo_overflow_count (void) { regs ().rfocr = 0; }

  // controls the ET_EXOUT pin output
  bool ext_loopback_pin (void) const { return regs ().iosr; }
  void set_ext_loopback_pin (bool val) { regs ().iosr = val; }


  // controls the condition upon which flow control should start
  // (a PAUSE frame is automatically transmitted)
  // if the following numbers are exceeded, flow control starts.
  // after reset, the initial values are set to the maximum possible values.
  // FIXME: add getter function.
  void set_flow_control_threshold (utils::clamped_value <uint32_t, 256, 2048> rx_fifo_bytes,
				   utils::clamped_value <uint32_t, 2, 16> rx_fifo_frames)
  {
    regs ().fcftr = (((rx_fifo_bytes + 255u - 256u) >> 8) & 3)
		    | ((((rx_fifo_frames + 1u - 2u) >> 1) & 3) << 16);
  }

  // when receiving data and copying it to the receive buffer, the hardware
  // can automatically insert some padding bytes at some specified position
  // in the frame.  this can be used to e.g. align ethernet frame data to
  // 4 bytes, which normally would be 2 byte aligned
  // (6+6+2 = 14 bytes eth header).
  //
  // before changing the padding setting, execute a software reset.
  //
  // FIXME: add getter function.
  void set_receive_data_padding (utils::clamped_value <uint32_t, 0, 63> byte_pos,
				 utils::clamped_value <uint32_t, 0, 3> byte_count)
  {
    regs ().rpadir = byte_pos | (byte_count << 16);
  }

  tx_interrupt_mode tx_int_mode (void) const { return (tx_interrupt_mode)regs ().trimd; }
  void set_tx_int_mode (tx_interrupt_mode val) { regs ().trimd = (unsigned int)val; }

  // the address to which the edmac is currently writing received data.
  // this value is informational only and might deviate.
  uint32_t current_receive_buffer_write_address (void) const { return regs ().rbwar; }

  // the address from which the edmac is currently fetching the receive descriptor.
  // this value is informational only and might deviate.
  uint32_t current_receive_desc_read_address (void) const { return regs ().rdfar; }

  // the address from which the edmac is currently reading transmit data.
  // this value is informational only and might deviate.
  uint32_t current_transmit_buffer_read_address (void) const { return regs ().tbrar; }

  // the address from which the edmac is currently fetching the transmit descriptor.
  // this value is informational only and might deviate.
  uint32_t current_transmit_desc_read_address (void) const { return regs ().tdfar; }

  fifo_sizes_t fifo_sizes (void) const
  {
    uint32_t v = regs ().fdr;
    return { ((v << 8) & 0x1F0) + 256, (v & 0xF0) + 256 };
  }

  void set_fifo_sizes (uint16_t rx_sz, uint16_t tx_sz)
  {
    auto r = ((rx_sz + 255u - 256u) >> 8) & 0x1F;
    auto t = ((tx_sz + 255u - 256u) >> 8) & 0x0F;
    regs ().fdr = (t << 8) | r;
  }


  // FIXME: add support for PTPEDMAC.EESR and PTPEDMAC.EESIPR

protected:
  struct regs_t;

  const regs_t& regs (void) const { return *(const regs_t*)RegAddress; }
  regs_t& regs (void) { return *(regs_t*)RegAddress; }

  struct regs_t
  {
    volatile uint32_t edmr;    // 0x000C0000
    uint32_t res_04;
    volatile uint32_t edtrr;   // 0x000C0008
    uint32_t res_0C;
    volatile uint32_t edrrr;   // 0x000C0010
    uint32_t res_14;
    volatile uint32_t tdlar;   // 0x000C0018
    uint32_t res_1C;
    volatile uint32_t rdlar;   // 0x000C0020
    uint32_t res_24;
    volatile uint32_t eesr;    // 0x000C0028
    uint32_t res_2C;
    volatile uint32_t eesipr;  // 0x000C0030
    uint32_t res_34;
    volatile uint32_t trscer;  // 0x000C0038
    uint32_t res_3C;
    volatile uint32_t rmfcr;   // 0x000C0040
    uint32_t res_44;
    volatile uint32_t tftr;    // 0x000C0048
    uint32_t res_4C;
    volatile uint32_t fdr;     // 0x000C0050
    uint32_t res_54;
    volatile uint32_t rmcr;    // 0x000C0058
    uint32_t res_5C;
    uint32_t res_60;
    volatile uint32_t tfucr;   // 0x000C0064
    volatile uint32_t rfocr;   // 0x000C0068
    volatile uint32_t iosr;    // 0x000C006C
    volatile uint32_t fcftr;   // 0x000C0070
    uint32_t res_74;
    volatile uint32_t rpadir;  // 0x000C0078
    volatile uint32_t trimd;   // 0x000C007C
    uint32_t res_80;
    uint32_t res_84;
    uint32_t res_88;
    uint32_t res_8C;
    uint32_t res_90;
    uint32_t res_94;
    uint32_t res_98;
    uint32_t res_9C;
    uint32_t res_A0;
    uint32_t res_A4;
    uint32_t res_A8;
    uint32_t res_AC;
    uint32_t res_B0;
    uint32_t res_B4;
    uint32_t res_B8;
    uint32_t res_BC;
    uint32_t res_C0;
    uint32_t res_C4;
    volatile uint32_t rbwar;   // 0x000C00C8
    volatile uint32_t rdfar;   // 0x000C00CC
    uint32_t res_D0;
    volatile uint32_t tbrar;   // 0x000C00D4
    volatile uint32_t tdfar;   // 0x000C00D8
    uint32_t res_DC;
    uint32_t res_E0;
    uint32_t res_E4;
    uint32_t res_E8;
    uint32_t res_EC;
    uint32_t res_F0;
    uint32_t res_F4;
    uint32_t res_F8;
    uint32_t res_FC;
    uint32_t res_100_1FC[0x100 / 4];
    uint32_t res_200_2FC[0x100 / 4];
    uint32_t res_300_3FC[0x100 / 4];
    uint32_t res_400;
    uint32_t res_404;
    uint32_t res_408;
    uint32_t res_40C;
    uint32_t res_410;
    uint32_t res_414;
    uint32_t res_418;
    uint32_t res_41C;
    uint32_t res_420;
    uint32_t res_424;
    volatile uint32_t ptp_eesr;
    uint32_t res_42C;
    volatile uint32_t ptp_eesipr;
  };

  static_assert (sizeof (regs_t) == 0x434, "");

  void isr (void)
  {
    auto st = m_etherc.status ();

    if (st.link_signal_changed ())
      link_status_interrupt_line::trigger ();

/*
    // clear pending interrupts, but only those that we have actually
    // enabled in the interrupt mask.
    uint32_t stat_edmac = EDMAC.EESR.LONG & EDMAC.EESIPR.LONG;
    EDMAC.EESR.LONG = stat_edmac;

    if (stat_edmac & EDMAC_EESIPR_INI_EtherC)
    {
      // same for etherc interrupts
      uint32_t stat_EtherC = ETHERC.ECSR.LONG & ETHERC.ECSIPR.LONG;
      ETHERC.ECSR.LONG = stat_EtherC;
      // lan_etherc_handler(stat_EtherC);
    }
*/

    // clear all pending interrupts
    auto s = etherc_edmac_status ();
    set_etherc_edmac_status (s);
  }

public:
  // interrupt line for the interrupt that is triggered by the etherc device
  // and needs to be installed in the interrupt table.
  typedef interrupt::connected_isr<InterruptLine,
	interrupt::func<decltype (&hw_inst::isr), &hw_inst::isr>> isr0_t;

  // virtual interrupt line for the LINKSTA pin.
  typedef LinkStatusVirtualInterruptLine link_status_interrupt_line;

protected:
  etherc_t& m_etherc;
  isr0_t m_int;
};

} // namespace rx_edmaca
} // namespace dev
#endif // includeguard_rx_edmaca_hpp_includeguard



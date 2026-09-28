
/*

LAN8710 / LAN8720 ethernet PHY

uses dev::mdio_sta to perform register access.

the minimum time between edges of the MDC is 160 ns.
the minimum cycle time (time between two consecutive rising or two consecutive
falling edges) is 400 ns.

data is latched on rising clock and must be ready 10 ns before the clock edge.

for faster communication it can also bypass the SMI preamble and accept
SMI packets without the preamble.

SMI addresses 0..7 can be configured using mode pins.  other addresses can
be assigned via registers.  it's also possible to disable address matching.


The LAN8720 is fully register compatible with the LAN8710, however it supports
only RMII operation, as it is a RMII-only chip.  There are a few bits that are
different:

  - PHY SPECIAL CONTROL/STATUS REGISTER (register 31)
    Bits [11:5] should be always written as 0b0000010.
    It's not possible to enable bypass of the 4B5B encoder.

  - SPECIAL MODES REGISTER (regster 18)
    Bit [14] MIIMODE has to be set to 1 (RMII mode).

*/

#ifndef includeguard_dev_lan8710_hpp_includeguard
#define includeguard_dev_lan8710_hpp_includeguard

#include <bitset>

#include <dev/eth_phy.hpp>
#include <dev/interrupt.hpp>

namespace dev
{

template <unsigned int PhyAddress, typename InterruptLine>
class lan8710 final : public eth_phy<PhyAddress, 400>
{
  using eth_phy<PhyAddress, 400>::read_reg;
  using eth_phy<PhyAddress, 400>::write_reg;

  void isr (void)
  {
  }

public:
  typedef interrupt::connected_isr<InterruptLine,
		interrupt::func<decltype (&lan8710::isr), &lan8710::isr>> isr0_t;

  lan8710 (mdio_sta& s)
  : eth_phy<PhyAddress, 400> (s),
    m_int (this)
  {
    m_int.enable (interrupt::low_level, interrupt::priority_5);
  }

  // ----------------------------------------------------------------
  // register 16 -- silicon revision number (vendor specific)
  uint16_t silicon_revision (void) const { return read_reg (16); }

  // ----------------------------------------------------------------
  // register 17 -- mode control/status (vendor specific)

  class mode_ctrl_status_t
  {
  public:
    constexpr mode_ctrl_status_t (void) : m_value (0) { }
    constexpr explicit mode_ctrl_status_t (uint16_t val) : m_value (val) { }

    constexpr uint16_t value (void) const { return m_value; }

    // indicates whether energy is detected on the line.
    constexpr bool energy_on (void) const { return get_bit (1); }

    // force good link status.  this is usually used for testing only.
    constexpr bool force_good_link_status (void) const { return get_bit (2); }
    mode_ctrl_status_t& set_force_good_link_status (bool val = true) { return set_bit (2, val); }

    // if set, ignores the phy address specified in the SMI packet.
    constexpr bool ignore_smi_phy_address (void) const { return get_bit (3); }
    mode_ctrl_status_t& set_ignore_smi_phy_address (bool val = true) const { return set_bit (3, val); }

    constexpr bool alt_int_sys (void) const { return get_bit (6); }
    mode_ctrl_status_t& set_alt_int_sys (bool val = true) { return set_bit (6, val); }

    constexpr bool far_loopback (void) const { return get_bit (9); }
    mode_ctrl_status_t& set_far_loopback (bool val = true) { return set_bit (9, val); }

    constexpr bool smi_preamble_bypass (void) const { return get_bit (10); }
    mode_ctrl_status_t& set_smi_preamble_bypass (bool val = true) { return set_bit (10, val); }

    constexpr bool low_squelch (void) const { return get_bit (11); }
    mode_ctrl_status_t& set_low_squelch (bool val = true) { return set_bit (11, val); }

    constexpr bool powerdown_detect (void) const { return get_bit (13); }
    mode_ctrl_status_t& set_powerdown_detect (bool val = true) { return set_bit (13, val); }

  private:
    constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
    mode_ctrl_status_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

    uint16_t m_value;
  };

  mode_ctrl_status_t mode_ctrl_status (void) const { return mode_ctrl_status_t (read_reg (17)); }
  void set_mode_ctrl_status (mode_ctrl_status_t val) { write_reg (17, val.value ()); }

  // ----------------------------------------------------------------
  // register 18 -- special modes (vendor specific)

  // the MODE[2:0] pins control the 10/100 digital block.  the pin values
  // are latched when the nRST pin is deasserted and the values are loaded
  // into the mode register.  when a soft reset occurs, the values from the
  // mode register are used instead.
  enum mode_t
  {
    // 10Base-T half duplex. auto-negotiation disabled.
    half_10base_t = 0b000,

    // 10Base-T full duplex. auto-negotiation disabled.
    full_10base_t = 0b001,

    // 100Base-TX Half Duplex. Auto-negotiation disabled.  CRS is active
    // during Transmit & Receive.
    half_100base_tx = 0b010,

    // 100Base-TX Full Duplex. Auto-negotiation disabled.  CRS is active
    // during Receive.
    full_100base_tx = 0b011,

    // 100Base-TX Half Duplex is advertised. Auto-negotiation enabled.
    // CRS is active during Transmit & Receive.
    half_100base_tx_auto = 0b100,

    // Repeater mode. Auto-negotiation enabled.  100Base-TX Half Duplex is
    // advertised.  CRS is active during Receive.
    repeater_100base_tx = 0b101,

    // Power Down mode. In this mode the transceiver will wake-up in Power-Down
    // mode. The transceiver cannot be used when set to this mode. To exit this
    // mode, change the mode bits and issue a soft reset.
    powerdown = 0b110,

    // All capable. Auto-negotiation enabled.
    all_capable_auto = 0b111
  };

  enum mii_mode_t
  {
    mii = 0,
    rmii = 1
  };

  class special_modes_t
  {
  public:
    constexpr special_modes_t (void) : m_value (0) { }
    constexpr explicit special_modes_t (uint16_t val) : m_value (val) { }
    constexpr uint16_t value (void) const { return m_value; }

    constexpr unsigned int phy_addr (void) const { return m_value & 0x1F; }
    special_modes_t& set_phy_addr (unsigned int val) { m_value = (m_value & ~0x1F) | (val & 0x1F); return *this; }

    constexpr mode_t mode (void) const { return (mode_t)((m_value >> 5) & 0b111); }
    special_modes_t& set_mode (mode_t val) { m_value = (m_value & ~(0b111 << 5)) | (val << 5); return *this; }

    constexpr mii_mode_t mii_mode (void) const { return (mii_mode_t)get_bit (14); }
    special_modes_t& set_mii_mode (mii_mode_t val) { return set_bit (14, val); }

  private:
    constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
    special_modes_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

    uint16_t m_value;
  };

  special_modes_t special_modes (void) const { return special_modes_t (read_reg (18)); }
  void set_special_modes (special_modes_t val) { write_reg (18, val.value ()); }

  // ----------------------------------------------------------------
  // register 26 -- symbol error counter (vendor specific)

  uint16_t symbol_error_count (void) const { return read_reg (26); }

  // ----------------------------------------------------------------
  // register 27 -- special control/status (vendor specific)

  enum mdi_mode_t
  {
    manual_mdi     = 0b100,
    manual_mdix    = 0b101,

    // auto mode seems to work only when auto negotiation is enabled.
    auto_mdi_mdix = 0b000
  };

  class special_ctrl_status_t
  {
  public:
    constexpr special_ctrl_status_t (void) : m_value (0) { }
    constexpr explicit special_ctrl_status_t (uint16_t val) : m_value (val) { }
    constexpr uint16_t value (void) const { return m_value; }

    // reverse polarity for 10base-T
    constexpr bool rev_10_base_t_polarity (void) const { return get_bit (4); }
    special_ctrl_status_t& set_rev_10_base_t_polarity (bool val = true) { return set_bit (4, val); }

    // disable signal quality error test (heartbeat)
    constexpr bool disable_sqe (void) const { return get_bit (11); }
    special_ctrl_status_t& set_disable_sqe (bool val = true) { return set_bit (11, val); }

    // manual/auto rx-tx crossover select
    constexpr mdi_mode_t mdi_mode (void) const { return (mdi_mode_t)((m_value >> 13) & 0b111); }
    special_ctrl_status_t& set_mdi_mode (mdi_mode_t val) { m_value = (m_value & ~(0b111 << 13)) | (val << 13); return *this; }

  private:
    constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
    special_ctrl_status_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

    uint16_t m_value;
  };

  special_ctrl_status_t special_ctrl_status (void) const { return special_ctrl_status_t (read_reg (27)); }
  void set_special_ctrl_status (special_ctrl_status_t val) { write_reg (27, val.value ()); }

  // ----------------------------------------------------------------
  // register 28 -- internal test register (vendor specific)

  // ----------------------------------------------------------------
  // register 29 -- interrupt source (vendor specific)
  // register 30 -- interrupt mask (vendor specific)

  class interrupts_t
  {
  public:
    constexpr interrupts_t (void) : m_value (0) { }
    constexpr explicit interrupts_t (uint16_t val) : m_value (val) { }
    constexpr uint16_t value (void) const { return m_value; }

    constexpr bool auto_neg_page_received (void) const { return get_bit (1); }
    interrupts_t& set_auto_neg_page_received (bool val = true) { return set_bit (1, val); }

    constexpr bool parallel_detection_fault (void) const { return get_bit (2); }
    interrupts_t& set_parallel_detection_fault (bool val = true) { return set_bit (2, val); }

    constexpr bool auto_neg_lp_ack (void) const { return get_bit (3); }
    interrupts_t& set_auto_neg_lp_ack (bool val = true) { return set_bit (3, val); }

    constexpr bool link_down (void) const { return get_bit (4); }
    interrupts_t& set_link_down (bool val = true) { return set_bit (4, val); }

    constexpr bool remote_fault_detected (void) const { return get_bit (5); }
    interrupts_t& set_remote_fault_detected (bool val = true) { return set_bit (5, val); }

    constexpr bool auto_neg_completed (void) const { return get_bit (6); }
    interrupts_t& set_auto_neg_completed (bool val = true) { return set_bit (6, val); }

    constexpr bool energy_on_generated (void) const { return get_bit (7); }
    interrupts_t& set_energy_on_generated (bool val = true) { return set_bit (7, val); }

  private:
    constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
    interrupts_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

    uint16_t m_value;
  };

  // pending interrupts are automatically cleared when the register is read.
  interrupts_t interrupt_source (void) const { return interrupts_t (read_reg (29)); }

  interrupts_t interrupt_mask (void) const { return interrupts_t (read_reg (30)); }
  void set_interrupt_mask (interrupts_t val) { write_reg (30, val.value ()); }

  // ----------------------------------------------------------------
  // register 31 -- PHY special control/status (vendor specific)

  enum speed_indication_t
  {
    ind_10m_half = 0b001,
    ind_10m_full = 0b101,
    ind_100_half = 0b010,
    ind_100_full = 0b110
  };

  class phy_special_ctrl_status_t
  {
  public:
    constexpr phy_special_ctrl_status_t (void) : m_value (0) { }
    constexpr explicit phy_special_ctrl_status_t (uint16_t val) : m_value (val) { }
    constexpr uint16_t value (void) const { return m_value; }

    constexpr bool disable_scrambling (void) const { return get_bit (0); }
    phy_special_ctrl_status_t& set_disable_scrambling (bool val = true) { return set_bit (0, val); }

    constexpr speed_indication_t speed_indication (void) const { return (speed_indication_t)((m_value >> 2) & 0b111); }

    // enable or disable 4B5B encoding/decoding (default enabled)
    constexpr bool enable_4b5b (void) const { return get_bit (6); }
    phy_special_ctrl_status_t& set_enable_4b5b (bool val = true) { return set_bit (6, val); }

    // general purpose output pins GPO[2:0]
    // FIXME: which pins are these?  not documented.
    constexpr std::bitset<3> gpo (void) const { return { (m_value >> 7) & 0b111 }; }
    phy_special_ctrl_status_t& set_gpo (std::bitset<3> val) { m_value = (m_value & ~(0b111 << 7)) | ((val.to_ulong () & 0b111) << 7); return *this; }

    // same as eth_phy::status_t::auto_neg_completed (register 1, bit 5)
    // except that the bit is not automatically cleared when read.
    constexpr bool auto_neg_completed (void) const { return get_bit (12); }

  private:
    constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
    phy_special_ctrl_status_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

    uint16_t m_value;
  };

  phy_special_ctrl_status_t phy_special_ctrl_status (void) const { return phy_special_ctrl_status_t (read_reg (31)); }
  void set_phy_special_ctrl_status (phy_special_ctrl_status_t val) { write_reg (31, val.value ()); }

private:
  isr0_t m_int;
};

};
#endif // includeguard_dev_lan8710_includeguard

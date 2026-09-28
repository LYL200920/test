
/*

generic ethernet PHY device.

ethernet PHY devices are normally connected to a SMI / MIIM / MDIO STA (master)
device to carry out host and device communication.

*/


#ifndef includeguard_dev_eth_phy_hpp_includeguard
#define includeguard_dev_eth_phy_hpp_includeguard

#include <cstdint>
#include <dev/mdio.hpp>

#include <utils/bits.hpp>


namespace dev
{

template <unsigned int PhyAddress, unsigned int CycleSpeedNanos /*, typename Sta*/>
class eth_phy
{
public:
  eth_phy (mdio_sta& s) : m_sta (s) { }

  constexpr unsigned int mmd_address (void) const { return PhyAddress; }

  // ----------------------------------------------------------------
  // register 0 -- control register (R/W, basic PHY)

  enum speed_t
  {
    speed_10mbps = 0,
    speed_100mbps = 1
  };

  enum duplex_t
  {
    duplex_half = 0,
    duplex_full = 1
  };

  class ctrl_t
  {
  public:
    constexpr ctrl_t (void) : m_value (0) { }
    constexpr explicit ctrl_t (uint16_t val) : m_value (val) { }

    constexpr uint16_t value (void) const { return m_value; }

    // collision test mode, set only when in loopback mode.
    constexpr bool collision_test (void) const { return get_bit (7); }
    ctrl_t& set_collision_test (bool val = true) { return set_bit (7, val); }

    // manual duplex selection (ignored if auto negotiation is used)
    constexpr duplex_t duplex (void) const { return (duplex_t)get_bit (8); }
    ctrl_t& set_duplex (duplex_t val) { return set_bit (8, val); }

    // the restart auto-negotiation bit is self clearing and will always
    // read as false.
    constexpr bool restart_auto_neg (void) const { return get_bit (9); }
    ctrl_t& set_restart_auto_neg (bool val = true) { return set_bit (9, val); }

    constexpr bool isolate (void) const { return get_bit (10); }
    ctrl_t& set_isolate (bool val = true) { return set_bit (10, val); }

    constexpr bool power_down (void) const { return get_bit (11); }
    ctrl_t& set_power_down (bool val = true) { return set_bit (11, val); }

    constexpr bool auto_neg_enable (void) const { return get_bit (12); }
    ctrl_t& set_auto_neg_enable (bool val = true) { return set_bit (12, val); }

    // force speed selection (ignored if auto negotiation is used)
    constexpr speed_t speed (void) const { return (speed_t)get_bit (13); }
    ctrl_t& set_speed (speed_t val) { return set_bit (13, val); }

    constexpr bool loopback (void) const { return get_bit (14); }
    ctrl_t& set_loopback (bool val = true) { return set_bit (14, val); }

    constexpr bool reset (void) const { return get_bit (15); }
    ctrl_t& set_reset (bool val = true) { return set_bit (15, val); }

    constexpr bool operator == (const ctrl_t& rhs) const { return m_value == rhs.m_value; }
    constexpr bool operator != (const ctrl_t& rhs) const { return m_value != rhs.m_value; }

  private:
    constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
    ctrl_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

    uint16_t m_value;
  };

  ctrl_t control (void) const { return ctrl_t (read_reg (0)); }
  void set_control (ctrl_t val) { write_reg (0, val.value ()); }

  // set reset bit in the control register to reset the PHY.
  // the reset bit will be cleared automatically after the reset sequence
  // has completed.  this function polls the control register's reset status
  // until either the reset bit is cleared or until the specified number of
  // iterations is reached.  if the application wants to do something else
  // while the PHY is performing its reset sequence, the application code can
  // use the control register directly.
  bool reset (unsigned int max_wait_count = 10)
  {
    // for some PHYs like the LAN8710 it's recommended to set the
    // reset bit only.
    set_control (ctrl_t ().set_reset (true));
    for (unsigned int i = 0; i < max_wait_count; ++i)
      if (!control ().reset ())
	return true;

    return false;
  }


  // ----------------------------------------------------------------
  // register 1 -- status (RO, basic PHY)

  class status_t
  {
  public:
    constexpr status_t (void) : m_value (0) { }
    constexpr explicit status_t (uint16_t val) : m_value (val) { }

    constexpr uint16_t value (void) const { return m_value; }

    // if true, has extended status register 15.
    constexpr bool extended_stat_reg (void) const { return get_bit (0); }

    // if true, PHY can accept MIIM frames without preamble.
    // this is the only bit that can be written on some devices to enable
    // acceptance of frames without a preamble.
    constexpr bool mf_preamble_suppression (void) const { return get_bit (6); }
    status_t& set_mf_preamble_suppression (bool val = true) { return set_bit (6, val); }

    // capability bits
    constexpr bool can_auto_neg (void) const { return get_bit (3); }
    constexpr bool can_100_base_t4 (void) const { return get_bit (15); }
    constexpr bool can_100_base_tx_full (void) const { return get_bit (14); }
    constexpr bool can_100_base_tx_half (void) const { return get_bit (13); }
    constexpr bool can_10_base_t_full (void) const { return get_bit (12); }
    constexpr bool can_10_base_t_half (void) const { return get_bit (11); }

    // status bits
    constexpr bool jabber_detected (void) const { return get_bit (1); }
    constexpr bool link_status (void) const { return get_bit (2); }
    constexpr bool remote_fault (void) const { return get_bit (4); }
    constexpr bool auto_neg_completed (void) const { return get_bit (5); }

    constexpr bool operator == (const status_t& rhs) const { return m_value == rhs.m_value; }
    constexpr bool operator != (const status_t& rhs) const { return m_value != rhs.m_value; }

  private:
    constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
    status_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

    uint16_t m_value;
  };

  // is setting the status actually possible for the basic PHY?
  status_t status (void) const { return status_t (read_reg (1)); }
  void set_status (status_t val) { write_reg (1, val.value ()); }

  // ----------------------------------------------------------------
  // register 2, 3 -- identifier (RO, extended PHY)

  uint32_t identifier (void) const { return (read_reg (2) << 16) | read_reg (3); }

  // ----------------------------------------------------------------
  // register 4 -- auto negotiation advertisement (R/W, extended PHY)
  // register 5 -- auto negotiation link partner (ca)ability (RO, extended PHY)
  //
  // the register bits are the same for the local and remote sides.

  class auto_neg_caps_t
  {
  public:
    constexpr auto_neg_caps_t (void) : m_value (0) { }
    constexpr explicit auto_neg_caps_t (uint16_t val) : m_value (val) { }

    constexpr uint16_t value (void) const { return m_value; }

    // 0b00001 = IEEE 802.3
    constexpr unsigned int selector_field (void) const { return m_value & 0b11111; }
    auto_neg_caps_t& set_selector_field (unsigned int val) { m_value = (m_value & ~0b11111) | (val & 0b11111); return *this; }

    // which features to advertise (offer) to the link partner.
    // capability bits.  set the bits when starting an auto-negotation in the
    // advertisement register.  get the negotiated remote capabilities from the
    // link partner register.
    constexpr bool cap_10_base_t (void) const { return get_bit (5); }
    auto_neg_caps_t& set_cap_10_base_t (bool val = true) { return set_bit (5, val); }

    constexpr bool cap_10_base_t_full (void) const { return get_bit (6); }
    auto_neg_caps_t& set_cap_10_base_t_full (bool val = true) { return set_bit (6, val); }

    constexpr bool cap_100_base_x (void) const { return get_bit (7); }
    auto_neg_caps_t& set_cap_100_base_x (bool val = true) { return set_bit (7, val); }

    constexpr bool cap_100_base_x_full (void) const { return get_bit (8); }
    auto_neg_caps_t& set_cap_100_base_x_full (bool val = true) { return set_bit (8, val); }

    constexpr bool cap_100_base_t4 (void) const { return get_bit (9); }
    auto_neg_caps_t& set_cap_100_base_t4 (bool val = true) { return set_bit (9, val); }

    constexpr bool cap_pause (void) const { return get_bit (10); }
    auto_neg_caps_t& set_cap_pause (bool val = true) { return set_bit (10, val); }

    constexpr bool cap_asymmetric_pause (void) const { return get_bit (11); }
    auto_neg_caps_t& set_cap_asymmetric_pause (bool val = true) { return set_bit (11, val); }

    // if enabled will sent the remote fault indiciator to the link partner
    // during auto negotation.  to clear write "false" or reset the LSI.
    constexpr bool remote_fault (void) const { return get_bit (13); }
    auto_neg_caps_t& set_remote_fault (bool val = true) { return set_bit (13, val); }

    // register 4 next page register = register 7
    // register 5 next page register = register 8
    constexpr bool next_page (void) const { return get_bit (15); }
    auto_neg_caps_t& set_next_page (bool val = true) { return set_bit (15, val); }

    constexpr bool operator == (const auto_neg_caps_t& rhs) const { return m_value == rhs.m_value; }
    constexpr bool operator != (const auto_neg_caps_t& rhs) const { return m_value != rhs.m_value; }

    // allow some set operations on the capability bits.
    auto_neg_caps_t operator | (const auto_neg_caps_t& rhs) const { return auto_neg_caps_t (m_value | rhs.m_value); }
    auto_neg_caps_t operator & (const auto_neg_caps_t& rhs) const { return auto_neg_caps_t (m_value & rhs.m_value); }
    auto_neg_caps_t operator ^ (const auto_neg_caps_t& rhs) const { return auto_neg_caps_t (m_value ^ rhs.m_value); }
    auto_neg_caps_t operator ~ (void) const { return auto_neg_caps_t (~m_value); }

  private:
    constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
    auto_neg_caps_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

    uint16_t m_value;
  };

  auto_neg_caps_t auto_neg_caps_advertisement (void) const { return auto_neg_caps_t (read_reg (4)); }
  void set_auto_neg_caps_advertisement (auto_neg_caps_t val) { write_reg (4, val.value ()); }

  auto_neg_caps_t auto_neg_caps_link_partner (void) const { return auto_neg_caps_t (read_reg (5)); }


  // ----------------------------------------------------------------
  // register 6 -- auto negotiation expansion (RO, extended PHY)

  class auto_neg_exp_t
  {
  public:
    constexpr auto_neg_exp_t (void) : m_value (0) { }
    constexpr explicit auto_neg_exp_t (uint16_t val) : m_value (val) { }

    constexpr uint16_t value (void) const { return m_value; }

    // if true, link partner has auto negotiation ability.
    constexpr bool link_partner_auto_neg (void) const { return get_bit (0); }

    // if true, a new page has been received from a link partner
    constexpr bool page_received (void) const { return get_bit (1); }

    // if true, local device has next page capability (which register?)
    constexpr bool next_page (void) const { return get_bit (2); }

    // if true, link partner has next page capability (which register?)
    constexpr bool link_partner_next_page (void) const { return get_bit (3); }

    constexpr bool parallel_detection_fault (void) const { return get_bit (4); }

    constexpr bool operator == (const auto_neg_exp_t& rhs) const { return m_value == rhs.m_value; }
    constexpr bool operator != (const auto_neg_exp_t& rhs) const { return m_value != rhs.m_value; }

  private:
    constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }

    uint16_t m_value;
  };

  auto_neg_exp_t auto_neg_exp (void) const { return auto_neg_exp_t (read_reg (6)); }

  // ----------------------------------------------------------------
  // register 7 -- auto negotiation next page transmit (R/W, extended PHY)
  // register 8 -- auto negotation link partner next page (RO, extended PHY)

  class auto_neg_next_page_t
  {
  public:
    constexpr auto_neg_next_page_t (void) : m_value (0) { }
    constexpr explicit auto_neg_next_page_t (uint16_t val) : m_value (val) { }

    constexpr uint16_t value (void) const { return m_value; }

    // next page message code or unformatted data
    constexpr unsigned int msg_data (void) const { return m_value & 0b111'1111'1111; }
    auto_neg_next_page_t& set_msg_data (unsigned int val) { (m_value & ~0b111'1111'1111) | (val & 0b111'1111'1111); }

    // if true: previous value of transmitted link code word was a logic zero
    // if false: previous value of transmitted link code word was a logic one
    constexpr bool toggle (void) const { return get_bit (11); }

    // true: complies with message, false: can't comply with message
    constexpr bool acknowledge2 (void) const { return get_bit (12); }
    auto_neg_next_page_t& set_acknowledge2 (bool val = true) { return set_bit (12, val); }

    // true: formatted page, false: unformatted page
    constexpr bool formatted_page (void) const { return get_bit (13); }
    auto_neg_next_page_t& set_formatted_page (bool val = true) { return set_bit (13, val); }

    // true: additional next pages to follow (where?), false: this is the last page.
    constexpr bool next_page (void) const { return get_bit (15); }
    auto_neg_next_page_t& set_next_page (bool val = true) { return set_bit (15, val); }

    constexpr bool operator == (const auto_neg_next_page_t& rhs) const { return m_value == rhs.m_value; }
    constexpr bool operator != (const auto_neg_next_page_t& rhs) const { return m_value != rhs.m_value; }

  private:
    constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
    auto_neg_next_page_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

    uint16_t m_value;
  };

  auto_neg_next_page_t auto_neg_next_page_transmit (void) const { return auto_neg_next_page_t (read_reg (7)); }
  void set_auto_neg_next_page_transmit (auto_neg_next_page_t val) { write_reg (7, val.value ()); }

  auto_neg_next_page_t auto_neg_next_page_link_partner (void) const { return auto_neg_next_page_t (read_reg (8)); }


  // ----------------------------------------------------------------
  // register 9 -- 1000 base-t control (R/W, extended PHY)

  enum port_type_t
  {
    single_port = 0,  // (prefer slave)
    multi_port = 1,   // (prefer master)
  };

  enum phy_master_slave_t
  {
    phy_auto   = 0b00,
    phy_slave  = 0b10,
    phy_master = 0b11
  };

  enum test_mode_t
  {
    normal = 0b000,
    transmit_waveform_test = 0b001,
    master_transmit_jitter_test = 0b010,
    slave_transmit_jitter_test = 0b011,
    transmit_distortion_test = 0b100
  };

  class ctrl1000_t
  {
  public:
    constexpr ctrl1000_t (void) : m_value (0) { }
    constexpr explicit ctrl1000_t (uint16_t val) : m_value (val) { }

    constexpr uint16_t value (void) const { return m_value; }

    constexpr bool adv_1000_half (void) const { return get_bit (8); }
    ctrl1000_t& set_adv_1000_half (bool val = true) { return set_bit (8, val); }

    constexpr bool adv_1000_full (void) const { return get_bit (9); }
    ctrl1000_t& set_adv_1000_full (bool val = true) { return set_bit (9, val); }

    constexpr port_type_t port_type (void) const { return (port_type_t)get_bit (10); }
    ctrl1000_t& set_port_type (port_type_t val) { return set_bit (10, val); }

    constexpr phy_master_slave_t phy_master_slave (void) const {  return (phy_master_slave_t)((m_value >> 11) & 3); }
    ctrl1000_t& set_phy_master_slave (phy_master_slave_t val) { m_value = (m_value & ~(3 << 11)) | (val << 11); return *this; }

    constexpr test_mode_t test_mode (void) const { return (test_mode_t)((m_value >> 13) & 0b111); }
    ctrl1000_t& set_test_mode (test_mode_t val) { m_value = (m_value & ~(0b111 << 13)) | (val << 13); return *this; }

    constexpr bool operator == (const ctrl1000_t& rhs) const { return m_value == rhs.m_value; }
    constexpr bool operator != (const ctrl1000_t& rhs) const { return m_value != rhs.m_value; }

  private:
    constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }
    ctrl1000_t& set_bit (unsigned int n, bool val) { m_value = utils::set_bit (m_value, n, val); return *this; }

    uint16_t m_value;
  };

  ctrl1000_t control1000 (void) const { return ctrl1000_t (read_reg (9)); }
  void set_control10000 (ctrl1000_t val) { write_reg (9, val.value ()); }

  // ----------------------------------------------------------------
  // register 10 -- 1000 base-t status (RO, extended PHY)

  class status1000_t
  {
  public:
    constexpr status1000_t (void) : m_value (0) { }
    constexpr explicit status1000_t (uint16_t val) : m_value (val) { }

    constexpr uint16_t value (void) const { return m_value; }

    constexpr unsigned int idle_error_count (void) const { return m_value & 0xFF; }
    constexpr bool link_partner_can_half_duplex (void) const { return get_bit (10); }
    constexpr bool link_partner_can_full_duplex (void) const { return get_bit (11); }
    constexpr bool remote_receiver_correct (void) const { return get_bit (12); }
    constexpr bool local_receiver_correct (void) const { return get_bit (13); }
    constexpr phy_master_slave_t phy_master_slave (void) const { return get_bit (14) ? phy_master : phy_slave; }
    constexpr bool master_slave_config_error (void) const { return get_bit (15); }

    constexpr bool operator == (const status1000_t& rhs) const { return m_value == rhs.m_value; }
    constexpr bool operator != (const status1000_t& rhs) const { return m_value != rhs.m_value; }

  private:
    constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }

    uint16_t m_value;
  };

  status1000_t status1000 (void) const { return status1000_t (read_reg (10)); }

  // ----------------------------------------------------------------
  // register 11, 12, 13, 14 -- reserved

  // ----------------------------------------------------------------
  // register 15 -- extended status (RO, extended PHY)

  class ext_status_t
  {
  public:
    constexpr ext_status_t (void) : m_value (0) { }
    constexpr explicit ext_status_t (uint16_t val) : m_value (val) { }

    constexpr uint16_t val (void) const { return m_value; }

    constexpr bool can_1000_base_t_half (void) const { return get_bit (12); }
    constexpr bool can_1000_base_t_full (void) const { return get_bit (13); }
    constexpr bool can_1000_base_x_half (void) const { return get_bit (14); }
    constexpr bool can_1000_base_x_full (void) const { return get_bit (15); }

    constexpr bool operator == (const ext_status_t& rhs) const { return m_value == rhs.m_value; }
    constexpr bool operator != (const ext_status_t& rhs) const { return m_value != rhs.m_value; }

  private:
    constexpr bool get_bit (unsigned int n) const { return utils::get_bit (m_value, n); }

    uint16_t m_value;
  };

  ext_status_t ext_status (void) const { return ext_status_t (read_reg (15)); }

  // ----------------------------------------------------------------
  // register 16 - 31 -- vendor specific



protected:
  mdio_sta& m_sta;

  uint16_t read_reg (unsigned int r) const
  {
    return m_sta.mdio_read_reg (PhyAddress, CycleSpeedNanos, r);
  }

  void write_reg (unsigned int r, uint16_t val)
  {
    m_sta.mdio_write_reg (PhyAddress, CycleSpeedNanos, r, val);
  }
};

};

#endif // includeguard_dev_eth_phy_hpp_includeguard


/*

LAN9250 driver


wake up from power-down mode:
an HBI write (CS and WR or CS, RD_WR and ENB) is performed to the device. Although all writes are ignored
until the device has been woken and a read performed, the host should direct the write to the Byte Order Test
Register (BYTE_TEST). Writes to any other addresses should not be attempted until the device is awake

Following device initialization, writes from the Host Bus are ignored until after a read cycle is performed.

*/

#ifndef includeguard_dev_lan9250_hpp_includeguard
#define includeguard_dev_lan9250_hpp_includeguard

#include <cstring>

#include <dev/interrupt.hpp>
#include <dev/eth_phy.hpp>
#include <dev/hwreg.hpp>
#include <utils/byte_order.hpp>

namespace dev
{
namespace lan9250
{

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template < unsigned int PhyAddress >
class phy_hw_inst final : public eth_phy < PhyAddress, 400 >
{
public:
  phy_hw_inst (mdio_sta& s) : eth_phy < PhyAddress, 400 > (s) { }

private:

};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// 16 bit parallel indexed mode bus with dedicated fifo area
template < uintptr_t RegBaseAddr,
	   uintptr_t FifoAddressBegin, uintptr_t FifoAddressEnd,
	   utils::byte_order_t HostByteOrder>
class if_parallel_indexed_16_fifo_area
{
private:
  struct regs_t
  {
    // there are 3 index/data sets which can be used for concurrent register
    // access, like with multiple threads or interrupt handlers.
    // it's also possible to select the endian setting for each register set
    // and the fifo area, but we don't do that now.
    struct
    {
      // the index register occupies 32 bits but only 16 bits are actually used.
      // this allows accessing it in 1 bus cycle.
      volatile uint16_t idx;		// 0x00 - 0x03, 0x08 - 0x0B, 0x10 - 0x13
      uint16_t idx_;

      // the data register is a 32 bit register.
      volatile uint32_t data;		// 0x04 - 0x07, 0x0C - 0x0F, 0x14 - 0x17
    } hbi_idx_data[3];

    volatile uint32_t data_fifo;	// 0x18 - 0x1B

    // the HBI CFG register is a 16 bit register
    volatile uint16_t hbi_cfg;		// 0x1C - 0x1F
    uint16_t hbi_cfg_;
  };

  static_assert (sizeof (regs_t) == 0x20, "");

  constexpr regs_t& regs (void) const { return *(regs_t*)RegBaseAddr; }

public:
  if_parallel_indexed_16_fifo_area (void)
  {
    // wait until the HBI comes up after reset release.
    // it takes a few cycles.
    for (unsigned int i = 0; ; ++i)
      if (regs ().hbi_idx_data[0].idx == 0x1234)
	break;

    // for now support only little endian interface.
    // if we enable big endian in lan9250 HBI, it will swap bytes, not only
    // 16 bit words in 16 bit mode.  in this case, we would need to pre swap
    // the data in software.
    static_assert (HostByteOrder == utils::little_endian, "");

    // select endian setting for all register sets at once.
    // regs ().hbi_cfg = HostByteOrder == utils::little_endian ? 0b0000 : 0b1111;
  }

  void write_reg (unsigned int reg_set_i, unsigned int reg, uint32_t val)
  {
    auto& r = regs ().hbi_idx_data[reg_set_i];
    r.idx = reg;
    r.data = val;
  }

  uint32_t read_reg (unsigned int reg_set_i, unsigned int reg) const
  {
    auto& r = regs ().hbi_idx_data[reg_set_i];
    r.idx = reg;
    return r.data;
  }

  void write_fifo (uint32_t val)
  {
    *(volatile uint32_t*)FifoAddressBegin = val;
  }

  uint32_t read_fifo (void) const
  {
    return *(volatile uint32_t*)FifoAddressBegin;
  }

  // FIXME: for background DMA transfers, this should be async.
  // but need MCB-156 first.
  // for SPI bus interface this will be a must-have because of the slow SPI
  // link speed.

  void write_fifo (const void* data_in, unsigned int size_bytes)
  {
    std::memcpy ((void*)FifoAddressBegin, data_in, size_bytes);
  }

  void read_fifo (void* data_out, unsigned int size_bytes)
  {
    std::memcpy (data_out, (const void*)FifoAddressBegin, size_bytes);
  }
};


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// the LAN9250 has an internal MDIO bus for the internal PHY.
// we treat it like any other external MDIO bus.
template < typename BusInterface, typename InterruptLine >
class hw_inst final : public mdio_sta
{
public:
  // the default phy MMD address in the LAN9250 is 1.
  using phy_t = phy_hw_inst < 1 >;

  typedef BusInterface bus_interface;

  hw_inst (bus_interface&& bif = { }) : m_int (this), m_bif (bif)
  {
    // assume that the bus interface is up and running here.
    // the bus interface constructor should have taken care of that.

    // wait for the LSI to come up.

    // check the byte_test register
    for (unsigned int i = 0; ; ++i)
      if (byte_test () == 0x87654321)
	break;

    // wait for the device ready bit to become on and set the
    // "must be on" (MBO) bit.
    for (unsigned int i = 0; ; ++i)
    {
      auto r = hw_cfg ();
      if (r & (1 << 27))
      {
	set_hw_cfg (r | (1 << 20));
	break;
      }
    }
  }

  virtual uint16_t mdio_read_reg (unsigned int mmd_addr,
				  [[gnu::unused]] unsigned int cycle_speed_ns,
				  unsigned int reg) override
  {
    // wait for the MII interface to become ready
    while (hmac_mii_acc () & 1) { }

    set_hmac_mii_acc (  ((mmd_addr & 0b11111) << 11)
		      | ((reg & 0b11111) << 6)
		      | (0 << 1));

    // wait for the read to complete
    while (hmac_mii_acc () & 1) { }

    return hmac_mii_data ();
  }

  virtual void mdio_write_reg (unsigned int mmd_addr,
			       [[gnu::unused]] unsigned int cycle_speed_ns,
			       unsigned int reg, uint16_t data) override
  {
    // wait for the MII interface to become ready
    while (hmac_mii_acc () & 1) { }

    set_hmac_mii_data (data);
    set_hmac_mii_acc (  ((mmd_addr & 0b11111) << 11)
		      | ((reg & 0b11111) << 6)
		      | (1 << 1));

    // don't wait after the write to the register.  the next access will
    // wait before the access.
  }


  // system registers
  uint32_t id_rev (void) const { return m_bif.read_reg (0, 0x50); }

  uint32_t irq_cfg (void) const { return m_bif.read_reg (0, 0x54); }
  void set_irq_cfg (uint32_t val) { m_bif.write_reg (0, 0x54, val); }

  uint32_t int_sts (void) const { return m_bif.read_reg (0, 0x58); }
  void set_int_sts (uint32_t val) { m_bif.write_reg (0, 0x58, val); }

  uint32_t int_en (void) const { return m_bif.read_reg (0, 0x5C); }
  void set_int_en (uint32_t val) { m_bif.write_reg (0, 0x5C, val); }

  uint32_t byte_test (void) const { return m_bif.read_reg (0, 0x64); }
  void set_byte_test (uint32_t val) { m_bif.write_reg (0, 0x64, val); }

  uint32_t fifo_int (void) const { return m_bif.read_reg (0, 0x68); }
  void set_fifo_int (uint32_t val) { m_bif.write_reg (0, 0x68, val); }

  uint32_t rx_cfg (void) const { return m_bif.read_reg (0, 0x6C); }
  void set_rx_cfg (uint32_t val) { m_bif.write_reg (0, 0x6C, val); }

  uint32_t tx_cfg (void) const { return m_bif.read_reg (0, 0x70); }
  void set_tx_cfg (uint32_t val) { m_bif.write_reg (0, 0x70, val); }

  uint32_t hw_cfg (void) const { return m_bif.read_reg (0, 0x74); }
  void set_hw_cfg (uint32_t val) { m_bif.write_reg (0, 0x74, val); }

  uint32_t rx_dp_ctrl (void) const { return m_bif.read_reg (0, 0x78); }
  void set_rx_dp_ctrl (uint32_t val) { m_bif.write_reg (0, 0x78, val); }

  uint32_t rx_fifo_inf (void) const { return m_bif.read_reg (0, 0x7C); }
  void set_rx_fifo_inf (uint32_t val) { m_bif.write_reg (0, 0x7C, val); }

  uint32_t tx_fifo_inf (void) const { return m_bif.read_reg (0, 0x80); }
  void set_tx_fifo_inf (uint32_t val) { m_bif.write_reg (0, 0x80, val); }

  uint32_t pmt_ctrl (void) const { return m_bif.read_reg (0, 0x84); }
  void set_pmt_ctrl (uint32_t val) { m_bif.write_reg (0, 0x84, val); }

  uint32_t gpt_cfg (void) const { return m_bif.read_reg (0, 0x8C); }
  void set_gpt_cfg (uint32_t val) { m_bif.write_reg (0, 0x8C, val); }

  uint32_t gpt_cnt (void) const { return m_bif.read_reg (0, 0x90); }
  void set_gpt_cnt (uint32_t val) { m_bif.write_reg (0, 0x90, val); }

  uint32_t free_run (void) const { return m_bif.read_reg (0, 0x9C); }
  void set_free_run (uint32_t val) { m_bif.write_reg (0, 0x9C, val); }

  uint32_t rx_drop (void) const { return m_bif.read_reg (0, 0xA0); }
  void set_rx_drop (uint32_t val) { m_bif.write_reg (0, 0xA0, val); }

  uint32_t afc_cfg (void) const { return m_bif.read_reg (0, 0xAC); }
  void set_afc_cfg (uint32_t val) { m_bif.write_reg (0, 0xAC, val); }

  uint32_t hmac_rx_lpi_transition (void) const { return m_bif.read_reg (0, 0xB0); }
  void set_hmac_rx_lpi_transition (uint32_t val) { m_bif.write_reg (0, 0xB0, val); }

  uint32_t hmac_rx_lpi_time (void) const { return m_bif.read_reg (0, 0xB4); }
  void set_hmac_rx_lpi_time (uint32_t val) { m_bif.write_reg (0, 0xB4, val); }

  uint32_t hmac_tx_lpi_transition (void) const { return m_bif.read_reg (0, 0xB8); }
  void set_hmac_tx_lpi_transition (uint32_t val) { m_bif.write_reg (0, 0xB8, val); }

  uint32_t hmac_tx_lpi_time (void) const { return m_bif.read_reg (0, 0xBC); }
  void set_hmac_tx_lpi_time (uint32_t val) { m_bif.write_reg (0, 0xBC, val); }

  // access to "host mac control and status register" access is done via
  // another indirection through the MAC CSR command/data registers.
  uint32_t mac_csr_cmd (void) const { return m_bif.read_reg (0, 0xA4); }
  void set_mac_csr_cmd (uint32_t val) const { m_bif.write_reg (0, 0xA4, val); }

  uint32_t mac_csr_data (void) const { return m_bif.read_reg (0, 0xA8); }
  void set_mac_csr_data (uint32_t val) { m_bif.write_reg (0, 0xA8, val); }

  uint32_t mac_csr_read (unsigned int reg) const
  {
    // wait until the port is available
    while (mac_csr_cmd () & (1 << 31)) { }

    // write command
    set_mac_csr_cmd (((1 << 31) | (1 << 30)) | reg);

    // wait until we can read the data
    while (mac_csr_cmd () & (1 << 31)) { }

    return mac_csr_data ();
  }

  void mac_csr_write (unsigned int reg, uint32_t val)
  {
    // wait until the port is available
    while (mac_csr_cmd () & (1 << 31)) { }

    // write command
    set_mac_csr_data (val);
    set_mac_csr_cmd (((1 << 31) | (0 << 30)) | reg);

    // do not wait until the write has completed to get a little bit better
    // performance.  anyway a check and wait is needed before issuing a command.
  }

  uint32_t hmac_cr (void) const { return mac_csr_read (0x01); }
  void set_hmac_cr (uint32_t val) { mac_csr_write (0x01, val); }

  uint32_t hmac_addrh (void) const { return mac_csr_read (0x02); }
  void set_hmac_addrh (uint32_t val) { mac_csr_write (0x02, val); }

  uint32_t hmac_addrl (void) const { return mac_csr_read (0x03); }
  void set_hmac_addrl (uint32_t val) { mac_csr_write (0x03, val); }

  uint32_t hmac_hashh (void) const { return mac_csr_read (0x04); }
  void set_hmac_hashh (uint32_t val) { mac_csr_write (0x04, val); }

  uint32_t hmac_hashl (void) const { return mac_csr_read (0x05); }
  void set_hmac_hashl (uint32_t val) { mac_csr_write (0x05, val); }

  uint32_t hmac_mii_acc (void) const { return mac_csr_read (0x06); }
  void set_hmac_mii_acc (uint32_t val) { mac_csr_write (0x06, val); }

  uint32_t hmac_mii_data (void) { return mac_csr_read (0x07); }
  void set_hmac_mii_data (uint32_t val) { mac_csr_write (0x07, val); }

  uint32_t hmac_flow (void) const { return mac_csr_read (0x08); }
  void set_hmac_flow (uint32_t val) { mac_csr_write (0x08, val); }

  uint32_t hmac_vlan1 (void) const { return mac_csr_read (0x09); }
  void set_hmac_vlan1 (uint32_t val) { mac_csr_write (0x09, val); }

  uint32_t hmac_vlan2 (void) const { return mac_csr_read (0x0A); }
  void set_hmac_vlan2 (uint32_t val) { mac_csr_write (0x0A, val); }

  uint32_t hmac_wuff (void) const { return mac_csr_read (0x0B); }
  void set_hmac_wuff (uint32_t val) { mac_csr_write (0x0B, val); }

  uint32_t hmac_wucsr (void) const { return mac_csr_read (0x0C); }
  void set_hmac_wucsr (uint32_t val) { mac_csr_write (0x0C, val); }

  uint32_t hmac_coe_cr (void) const { return mac_csr_read (0x0D); }
  void set_hmac_coe_cr (uint32_t val) { mac_csr_write (0x0D, val); }

  uint32_t hmac_eee_tw_tx_sys (void) const { return mac_csr_read (0x0E); }
  void set_hmac_eee_tw_tx_sys (uint32_t val) { mac_csr_write (0x0E, val); }

  uint32_t hmac_eee_tx_lpi_req_delay (void) const { return mac_csr_read (0x0F); }
  void set_hmac_eee_tx_lpi_req_delay (uint32_t val) { mac_csr_write (0x0F, val); }


  // 1588 registers
  uint32_t _1558_cmd_ctl (void) const { return m_bif.read_reg (0, 0x100); }
  void set_1558_cmd_ctl (uint32_t val) { m_bif.write_reg (0, 0x100, val); }

  uint32_t _1558_general_config (void) const { return m_bif.read_reg (0, 0x104); }
  void set_1558_general_config (uint32_t val) { m_bif.write_reg (0, 0x104, val); }

  uint32_t _1588_int_sts (void) const { return m_bif.read_reg (0, 0x108); }
  void set_1588_int_sts (uint32_t val) { m_bif.write_reg (0, 0x108, val); }

  uint32_t _1558_int_en (void) const { return m_bif.read_reg (0, 0x10C); }
  void set_1558_int_en (uint32_t val) { m_bif.write_reg (0, 0x10C, val); }

  uint32_t _1558_clock_sec (void) const { return m_bif.read_reg (0, 0x110); }
  void set_1558_clock_sec (uint32_t val) { m_bif.write_reg (0, 0x110, val); }

  uint32_t _1588_clock_ns (void) const { return m_bif.read_reg (0, 0x114); }
  void set_1588_clock_ns (uint32_t val) { m_bif.write_reg (0, 0x114, val); }

  uint32_t _1588_clock_subns (void) const { return m_bif.read_reg (0, 0x118); }
  void set_1588_clock_subns (uint32_t val) { m_bif.write_reg (0, 0x118, val); }

  uint32_t _1588_clock_rate_adj (void) const { return m_bif.read_reg (0, 0x11C); }
  void set_1588_clock_rate_adj (uint32_t val) { m_bif.write_reg (0, 0x11C, val); }

  uint32_t _1588_clock_temp_rate_adj (void) const { return m_bif.read_reg (0, 0x120); }
  void set_1588_clock_temp_rate_adj (uint32_t val) { m_bif.write_reg (0, 0x120, val); }

  uint32_t _1588_clock_temp_rate_duration (void) const { return m_bif.read_reg (0, 0x124); }
  void set_1588_clock_temp_rate_duration (uint32_t val) { m_bif.write_reg (0, 0x124, val); }

  uint32_t _1588_clock_step_adj (void) const { return m_bif.read_reg (0, 0x128); }
  void set_1588_clock_step_adj (uint32_t val) { m_bif.write_reg (0, 0x128, val); }

  uint32_t _1588_clock_target_sec_a (void) const { return m_bif.read_reg (0, 0x12C); }
  void set_1588_clock_target_sec_a (uint32_t val) { m_bif.write_reg (0, 0x12C, val); }

  uint32_t _1588_clock_target_ns_a (void) const { return m_bif.read_reg (0, 0x130); }
  void set_1588_clock_target_ns_a (uint32_t val) { m_bif.write_reg (0, 0x130, val); }

  uint32_t _1588_clock_target_reload_sec_a (void) const { return m_bif.read_reg (0, 0x134); }
  void set_1588_clock_target_reload_sec_a (uint32_t val) { m_bif.write_reg (0, 0x134, val); }

  uint32_t _1588_clock_target_reload_ns_a (void) const { return m_bif.read_reg (0, 0x138); }
  void set_1588_clock_target_reload_ns_a (uint32_t val) { m_bif.write_reg (0, 0x138, val); }

  uint32_t _1588_clock_target_sec_b (void) const { return m_bif.read_reg (0, 0x13C); }
  void set_1588_clock_target_sec_b (uint32_t val) { m_bif.write_reg (0, 0x13C, val); }

  uint32_t _1588_clock_target_ns_b (void) const { return m_bif.read_reg (0, 0x140); }
  void set_1588_clock_target_ns_b (uint32_t val) { m_bif.write_reg (0, 0x140, val); }

  uint32_t _1588_clock_target_reload_sec_b (void) const { return m_bif.read_reg (0, 0x144); }
  void set_1588_clock_target_reload_sec_b (uint32_t val) { m_bif.write_reg (0, 0x144, val); }

  uint32_t _1588_clock_target_reload_ns_b (void) const { return m_bif.read_reg (0, 0x148); }
  void set_1588_clock_target_reload_ns_b (uint32_t val) { m_bif.write_reg (0, 0x148, val); }

  uint32_t _1588_user_mac_hi (void) const { return m_bif.read_reg (0, 0x14C); }
  void set_1588_user_mac_hi (uint32_t val) { m_bif.write_reg (0, 0x14C, val); }

  uint32_t _1588_user_mac_lo (void) const { return m_bif.read_reg (0, 0x150); }
  void set_1588_user_mac_lo (uint32_t val) { m_bif.write_reg (0, 0x150, val); }

  // FIXME: cache the gpio and 1588 register bank selection register
  //        and cache it only as needed.  this can be implemented transparently
  //        in the set_1588_bank_port_gpio_sel function.
  uint32_t _1588_bank_port_gpio_sel (void) const { return m_bif.read_reg (0, 0x154); }
  void set_1588_bank_port_gpio_sel (uint32_t val) { m_bif.write_reg (0, 0x154, val); }

  uint32_t _1588_latency (void) const { set_1588_bank_port_gpio_sel (0); return m_bif.read_reg (0, 0x158); }
  void set_1588_latency (uint32_t val) { set_1588_bank_port_gpio_sel (0); m_bif.write_reg (0, 0x158, val); }

  uint32_t _1588_asym_peerdly (void) const { set_1588_bank_port_gpio_sel (0); return m_bif.read_reg (0, 0x15C); }
  void set_1588_asym_peerdly (uint32_t val) { set_1588_bank_port_gpio_sel (0); m_bif.write_reg (0, 0x15C, val); }

  uint32_t _1588_cap_info (void) const { set_1588_bank_port_gpio_sel (0); return m_bif.read_reg (0, 0x160); }


  uint32_t _1588_rx_parse_config (void) const { set_1588_bank_port_gpio_sel (1); return m_bif.read_reg (0, 0x158); }
  void set_1588_rx_parse_config (uint32_t val) { set_1588_bank_port_gpio_sel (1); m_bif.write_reg (0, 0x158, val); }

  uint32_t _1588_rx_timestamp_config (void) const { set_1588_bank_port_gpio_sel (1); return m_bif.read_reg (0, 0x15C); }
  void set_1588_rx_timestamp_config (uint32_t val) { set_1588_bank_port_gpio_sel (1); m_bif.write_reg (0, 0x15C, val); }

  uint32_t _1588_rx_ts_insert_config (void) const { set_1588_bank_port_gpio_sel (1); return m_bif.read_reg (0, 0x160); }
  void set_1588_rx_ts_insert_config (uint32_t val) { set_1588_bank_port_gpio_sel (1); m_bif.write_reg (0, 0x160, val); }

  uint32_t _1588_port_rx_filter_config (void) const { set_1588_bank_port_gpio_sel (1); return m_bif.read_reg (0, 0x168); }
  void set_1588_port_rx_filter_config (uint32_t val) { set_1588_bank_port_gpio_sel (1); m_bif.write_reg (0, 0x168, val); }

  uint32_t _1588_rx_ingress_sec (void) const { return set_1588_bank_port_gpio_sel (1); return m_bif.read_reg (0, 0x16C); }
  void set_1588_rx_ingress_sec (uint32_t val) { set_1588_bank_port_gpio_sel (1); m_bif.write_reg (0, 0x16C, val); }

  uint32_t _1588_rx_ingress_ns (void) const { return set_1588_bank_port_gpio_sel (1); return m_bif.read_reg (0, 0x170); }
  void set_1588_rx_ingress_ns (uint32_t val) { set_1588_bank_port_gpio_sel (1); m_bif.write_reg (0, 0x170, val); }

  uint32_t _1588_rx_msg_header (void) const { return set_1588_bank_port_gpio_sel (1); return m_bif.read_reg (0, 0x174); }

  uint32_t _1588_rx_pdreq_sec (void) const { return set_1588_bank_port_gpio_sel (1); return m_bif.read_reg (0, 0x178); }
  void set_1588_rx_pdreq_sec (uint32_t val) { set_1588_bank_port_gpio_sel (1); m_bif.write_reg (0, 0x178, val); }

  uint32_t _1588_rx_pdreq_ns (void) const { return set_1588_bank_port_gpio_sel (1); return m_bif.read_reg (0, 0x17C); }
  void set_1588_rx_pdreq_ns (uint32_t val) { set_1588_bank_port_gpio_sel (1); m_bif.write_reg (0, 0x17C, val); }

  uint32_t _1588_rx_pdreq_cf_hi (void) const { return set_1588_bank_port_gpio_sel (1); return m_bif.read_reg (0, 0x180); }
  void set_1588_rx_pdreq_cf_hi (uint32_t val) { set_1588_bank_port_gpio_sel (1); m_bif.write_reg (0, 0x180, val); }

  uint32_t _1588_rx_pdreq_cf_low (void) const { return set_1588_bank_port_gpio_sel (1); return m_bif.read_reg (0, 0x184); }
  void set_1588_rx_pdreq_cf_low (uint32_t val) { set_1588_bank_port_gpio_sel (1); m_bif.write_reg (0, 0x184, val); }

  uint32_t _1588_rx_chksum_dropped_cnt (void) const { return set_1588_bank_port_gpio_sel (1); return m_bif.read_reg (0, 0x188); }
  void set_1588_rx_chksum_dropped_cnt (uint32_t val) { set_1588_bank_port_gpio_sel (1); m_bif.write_reg (0, 0x188, val); }

  uint32_t _1588_rx_filtered_cnt (void) const { return set_1588_bank_port_gpio_sel (1); return m_bif.read_reg (0, 0x18C); }
  void set_1588_rx_filtered_cnt (uint32_t val) { set_1588_bank_port_gpio_sel (1); m_bif.write_reg (0, 0x18C, val); }


  uint32_t _1588_tx_parse_config (void) const { set_1588_bank_port_gpio_sel (2); return m_bif.read_reg (0, 0x158); }
  void set_1588_tx_parse_config (uint32_t val) { set_1588_bank_port_gpio_sel (2); m_bif.write_reg (0, 0x158, val); }

  uint32_t _1588_tx_timestamp_config (void) const { set_1588_bank_port_gpio_sel (2); return m_bif.read_reg (0, 0x15C); }
  void set_1588_tx_timestamp_config (uint32_t val) { set_1588_bank_port_gpio_sel (2); m_bif.write_reg (0, 0x15C, val); }

  uint32_t _1588_tx_mod (void) const { set_1588_bank_port_gpio_sel (2); return m_bif.read_reg (0, 0x164); }
  void set_1588_tx_mod (uint32_t val) { set_1588_bank_port_gpio_sel (2); m_bif.write_reg (0, 0x164, val); }

  uint32_t _1588_tx_mod2 (void) const { set_1588_bank_port_gpio_sel (2); return m_bif.read_reg (0, 0x168); }
  void set_1588_tx_mod2 (uint32_t val) { set_1588_bank_port_gpio_sel (2); m_bif.write_reg (0, 0x168, val); }

  uint32_t _1588_tx_egress_sec (void) const { return set_1588_bank_port_gpio_sel (2); return m_bif.read_reg (0, 0x16C); }
  void set_1588_tx_egress_sec (uint32_t val) { set_1588_bank_port_gpio_sel (2); m_bif.write_reg (0, 0x16C, val); }

  uint32_t _1588_tx_egress_ns (void) const { return set_1588_bank_port_gpio_sel (2); return m_bif.read_reg (0, 0x170); }
  void set_1588_tx_egress_ns (uint32_t val) { set_1588_bank_port_gpio_sel (2); m_bif.write_reg (0, 0x170, val); }

  uint32_t _1588_tx_msg_header (void) const { set_1588_bank_port_gpio_sel (2); return m_bif.read_reg (0, 0x174); }

  uint32_t _1588_tx_dreq_sec (void) const { set_1588_bank_port_gpio_sel (2); return m_bif.read_reg (0, 0x178); }

  uint32_t _1588_tx_dreq_ns (void) const { set_1588_bank_port_gpio_sel (2); return m_bif.read_reg (0, 0x17C); }

  uint32_t _1588_tx_one_step_sync_sec (void) const { return set_1588_bank_port_gpio_sel (2); return m_bif.read_reg (0, 0x180); }
  void set_1588_tx_one_step_sync_sec (uint32_t val) { set_1588_bank_port_gpio_sel (2); m_bif.write_reg (0, 0x180, val); }


  uint32_t _1588_gpio_cap_config (void) const { set_1588_bank_port_gpio_sel (3); return m_bif.read_reg (0, 0x15C); }
  void set_1588_gpio_cap_config (uint32_t val) { set_1588_bank_port_gpio_sel (3); m_bif.write_reg (0, 0x15C, val); }

  uint32_t _1588_gpio_re_clock_sec_cap_x (void) const { set_1588_bank_port_gpio_sel (3); return m_bif.read_reg (0, 0x16C); }
  uint32_t _1588_gpio_re_clock_ns_cap_x (void) const { set_1588_bank_port_gpio_sel (3); return m_bif.read_reg (0, 0x170); }

  uint32_t _1588_gpio_fe_clock_sec_cap_x (void) const { set_1588_bank_port_gpio_sel (3); return m_bif.read_reg (0, 0x178); }
  uint32_t _1588_gpio_fe_clock_ns_cap_x (void) const { set_1588_bank_port_gpio_sel (3); return m_bif.read_reg (0, 0x17C); }


  // EEPROM/LED registers
  uint32_t e2p_cmd (void) const { return m_bif.read_reg (0, 0x1B4); }
  void set_e2p_cmd (uint32_t val) { m_bif.write_reg (0, 0x1B4, val); }

  uint32_t ep2_data (void) const { return m_bif.read_reg (0, 0x1B8); }
  void set_ep2_data (uint32_t val) { m_bif.write_reg (0, 0x1B8, val); }

  uint32_t led_cfg (void) const { return m_bif.read_reg (0, 0x1BC); }
  void set_led_cfg (uint32_t val) { m_bif.write_reg (0, 0x1BC, val); }
 
  // GPIO registers
  uint32_t gpio_cfg (void) const { return m_bif.read_reg (0, 0x1E0); }
  void set_gpio_cfg (uint32_t val) { m_bif.write_reg (0, 0x1E0, val); }

  uint32_t gpio_data_dir (void) const { return m_bif.read_reg (0, 0x1E4); }
  void set_gpio_data_dir (uint32_t val) { m_bif.write_reg (0, 0x1E4, val); }

  uint32_t gpio_int_sts_en (void) const { return m_bif.read_reg (0, 0x1E8); }
  void set_gpio_int_sts_en (uint32_t val) { m_bif.write_reg (0, 0x1E8, val); }

  // reset register
  uint32_t reset_ctl (void) const { return m_bif.read_reg (0, 0x1F8); }
  void set_reset_ctl (uint32_t val) { m_bif.write_reg (0, 0x1F8, val); }


private:
  void isr (void)
  {

  }

public:
  typedef interrupt::connected_isr<InterruptLine,
	interrupt::func<decltype (&hw_inst::isr), &hw_inst::isr>> isr0_t;

private:
  isr0_t m_int;
  mutable bus_interface m_bif;
};

} // namespace lan9250
} // namespace dev

#endif // includeguard_dev_lan9250_hpp_includeguard

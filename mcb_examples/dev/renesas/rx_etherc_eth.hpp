
/*
  an "eth" driver for the renesas etherc device and some phy.
  usually the phy LSI will be connected to the etherc MDIO port.
  this driver allows using a phy that is connected in some other way.

*/

#ifndef includeguard_rx_etherc_eth_hpp_includeguard
#define includeguard_rx_etherc_eth_hpp_includeguard

#include <chrono>
#include <cstdint>
#include <array>
#include <algorithm>
#include <cstring>

#include <dev/eth_phy.hpp>
#include <dev/eth.hpp>

#include <utils/byte_order.hpp>

namespace dev
{

template < typename EDMAC, typename Phy, typename DefaultMacAddrFunc >
class rx_etherc_eth final : public eth
{
public:
  rx_etherc_eth (void) = delete;
  rx_etherc_eth (const rx_etherc_eth&) = delete;
  rx_etherc_eth (rx_etherc_eth&&) = delete;

  [[gnu::cold]] rx_etherc_eth (EDMAC& edmac, Phy& phy)
  : m_edmac (edmac), m_phy (phy)
  {
    m_cur_rx_desc = &(m_rx_desc.front ());
    m_cur_tx_desc = &(m_tx_desc.front ());
  }

  [[gnu::cold]] ~rx_etherc_eth (void)
  {
  }

  virtual const std::array<uint8_t, 6>
  default_mac_address (void) const noexcept override
  {
    return DefaultMacAddrFunc () ();
  }

  // FIXME: the mac address or other settings like promiscuous
  // mode can't be changed on-the-fly while the etherc is operating.
  // it needs a re-init sequence.
  // thus, maybe it's better to make a "struct params" and set all the
  // parameters at once (and do the re-init).
  [[gnu::cold]] virtual void set_mac_address (const std::array<uint8_t, 6>& val) noexcept override
  {
    m_macaddr = val;
    init ();
  }

  virtual std::array<uint8_t, 6> mac_address (void) const noexcept override
  {
    return m_macaddr;
  }

  // FIXME: maybe rename to "re-init" or something.
  [[gnu::cold]] virtual void reset (void) noexcept override
  {
    // this will reset the PHY and the link.  after that, wait until the link
    // comes up again.  if the link doesn't come up, then there's nothing
    // to initialize further.
    m_linkinfo_valid = false;
    m_phy.reset ();

    m_edmac.reset ();

    // if the PHY is already up and the link is already up, there will be no
    // link change down -> up.
    update_link_status ();
    if (m_linkinfo.status == link_status::up)
      init ();
  }

  [[gnu::cold]] virtual struct link_info link_info (void) const noexcept override
  {
    check_link_status ();
    return m_linkinfo;
  }

  [[gnu::cold]] virtual void
  link_autonegotiate (link_speed_set allowed_speeds,
		      link_duplex_set allowed_duplex) noexcept override
  {
    m_linkinfo.status = link_status::negotiating;
    m_linkinfo.speed = link_speed::invalid;
    m_linkinfo.duplex = link_duplex::invalid;

    m_phy.set_auto_neg_caps_advertisement (typename Phy::auto_neg_caps_t ()
	.set_selector_field (1)
	.set_cap_10_base_t       (allowed_speeds[link_speed::m10]  && allowed_duplex[link_duplex::half])
	.set_cap_10_base_t_full  (allowed_speeds[link_speed::m10]  && allowed_duplex[link_duplex::full])
	.set_cap_100_base_x      (allowed_speeds[link_speed::m100] && allowed_duplex[link_duplex::half])
	.set_cap_100_base_x_full (allowed_speeds[link_speed::m100] && allowed_duplex[link_duplex::full])
	.set_cap_100_base_t4     (allowed_speeds[link_speed::m100] && allowed_duplex[link_duplex::half])
	.set_cap_pause ());

    m_phy.set_control (m_phy.control ().set_restart_auto_neg ()
				       .set_auto_neg_enable ());
  }

  // electrically isolate cable pins.
  [[gnu::cold]] virtual void link_isolate (bool val) noexcept override
  {
    m_phy.set_control (m_phy.control ().set_isolate (val));
  }

  // explicitly set the link type, overriding the current link setting,
  // even if it has been auto negotiated.
  [[gnu::cold]] virtual void link_force_type (link_speed spd, link_duplex dpl) noexcept override
  {
    // FIXME: implement
  }

  // best effort packet read and write.
  // if a packet can't be written because the transmit queue is full, drop it.
  // if a packet can't be read because the queue is empty, return 0.
  virtual void write (const void* data, unsigned int byte_count) noexcept override
  {
    if (m_linkinfo.status != link_status::up)
      return;

    // drop the packet if the descriptor is still active.
    if (m_cur_tx_desc->status ().active ())
      return;

    unsigned int send_len = std::min (byte_count, buffer_size);
    std::memcpy ((void*)m_cur_tx_desc->data_ptr (), data, send_len);
    if (send_len < 60)
    {
      std::memset ((char*)m_cur_tx_desc->data_ptr () + send_len, 0, 60 - send_len);
      send_len = 60;
    }

    m_cur_tx_desc->set_data_length (send_len);
    m_cur_tx_desc->set_active (true);

    m_edmac.start_transmit ();

    m_cur_tx_desc = m_cur_tx_desc->next;
  }

  virtual unsigned int read (void* data, unsigned int max_byte_count) noexcept override
  {
    if (m_linkinfo.status != link_status::up)
      return 0;

    unsigned int result = 0;

    auto cur_rx_st = m_cur_rx_desc->status ();

    if (!cur_rx_st.active ())
    {
      // if the receive descriptor is not active, it means either data has
      // been received or there was an error.
      if (!cur_rx_st.error ())
      {
	result = std::min (max_byte_count, (unsigned int)m_cur_rx_desc->data_length ());
	std::memcpy (data, m_cur_rx_desc->data_ptr (), result);
      }
      else
      {
        // maybe check link status etc here?
      }

      m_cur_rx_desc->set_active (true);
      m_cur_rx_desc = m_cur_rx_desc->next;
    }

    // if it can't keep up with the inbound traffic, it will set the
    // "frame counter overflow" flag in the status register (EDMACn.EESR)
    // while the flag is set it will discard any further packets.
    m_edmac.set_etherc_edmac_status (
      m_edmac.etherc_edmac_status ().set_receive_frame_counter_overflow (false));

    m_edmac.start_receive ();
    return result;
  }

  [[gnu::cold]] virtual void check_link_status (void) const noexcept override
  {
    /* because we don't have a link status interrupt, we need to check
       the link status periodically.
       FIXME: this probably should go into the PHY driver.  the PHY's public
       interface would offer a link-status-change callback.  if set, the PHY
       driver can implement it either via a hardware interrupt or via
       software polling.

       we can omit the polling if there is some continuous data transfer
       going on.  if packets can sent and received the link must be up.  */

    if (!m_linkinfo_valid)
      update_link_status ();

    else if ((std::chrono::system_clock::now () - m_last_linkinfo_check_time)
		> std::chrono::milliseconds (250))
    {
      auto prev_st = m_linkinfo;
      update_link_status ();

      if (prev_st.status != link_status::up && m_linkinfo.status == link_status::up)
      {
	// link status went down -> up
	// try to re-init the etherc and start operation.
	init ();
      }

      if (prev_st.status == link_status::up && m_linkinfo.status != link_status::up)
      {
	// link status went up -> down
	// disable rx and tx operations by resetting the etherc.
	m_edmac.reset ();
      }
    }
  }

private:
  struct tx_desc : public EDMAC::tx_desc
  {
    tx_desc* next = nullptr;
  };

  struct rx_desc : public EDMAC::rx_desc
  {
    rx_desc* next = nullptr;
  };


  EDMAC& m_edmac;
  Phy& m_phy;

  std::array<uint8_t, 6> m_macaddr;

  mutable struct link_info m_linkinfo;
  mutable bool m_linkinfo_valid = false;
  mutable std::chrono::system_clock::time_point m_last_linkinfo_check_time;

  mutable tx_desc* m_cur_tx_desc;
  mutable rx_desc* m_cur_rx_desc;

  static constexpr unsigned int tx_buffer_count = 4;
  static constexpr unsigned int rx_buffer_count = 4;
  static constexpr unsigned int buffer_size = 2048;

  alignas (sizeof (tx_desc)) std::array<tx_desc, tx_buffer_count> mutable m_tx_desc;
  alignas (sizeof (rx_desc)) std::array<rx_desc, rx_buffer_count> mutable m_rx_desc;

  typedef std::array<uint8_t, buffer_size> buffer;

  alignas (32) std::array<buffer, tx_buffer_count> mutable m_tx_buffers;
  alignas (32) std::array<buffer, rx_buffer_count> mutable m_rx_buffers;


  [[gnu::cold]] void init (void) const
  {
    m_edmac.reset ();

    if (m_linkinfo.status != link_status::up)
      return;

    // setup descriptors.
    for (auto& d : m_tx_desc)
      d = { };

    for (auto& d : m_rx_desc)
      d = { };

    for (unsigned int i = 0; i < tx_buffer_count; ++i)
    {
      auto& d = m_tx_desc[i];
      d.set_data_ptr (m_tx_buffers[i].data ());
      d.set_type (EDMAC::desc_type::single_frame);
      d.set_data_length (0);
      d.set_active (false);

      d.next = &m_tx_desc[(i+1) % tx_buffer_count];
    }

    for (unsigned int i = 0; i < rx_buffer_count; ++i)
    {
      auto& d = m_rx_desc[i];
      d.set_data_ptr (m_rx_buffers[i].data ());
      d.set_type (EDMAC::desc_type::single_frame);
      d.set_max_data_length (buffer_size);
      d.set_active (true);

      d.next = &m_rx_desc[(i+1) % rx_buffer_count];
    }

    m_tx_desc.back ().set_ring_end (true);
    m_rx_desc.back ().set_ring_end (true);

    m_cur_rx_desc = &(m_rx_desc.front ());
    m_cur_tx_desc = &(m_tx_desc.front ());


    // setup registers

    m_edmac.set_mode (typename EDMAC::mode_t ()
	.set_descriptor_size (EDMAC::desc_size::sz_16_bytes)
	.set_data_endian (utils::native_byte_order () == utils::little_endian
			  ? EDMAC::data_endian::little
			  : EDMAC::data_endian::big));
    static_assert (sizeof (tx_desc) == 16 && sizeof (rx_desc) == 16, "");

    m_edmac.set_transmit_desc_array (m_tx_desc.data ());
    m_edmac.set_receive_desc_array (m_rx_desc.data ());

    m_edmac.set_desc_status_copy_bits (typename EDMAC::desc_status_copy_bits_t ()
	.set_rx_crc_error ()
	.set_rx_phy_error ()
	.set_rx_truncated ()
	.set_rx_overlong ()
	.set_rx_residual_bit ()
	.set_rx_multicast ()
	.set_tx_retry_over ()
	.set_tx_delayed_collision ()
	.set_tx_carrier_lost ()
	.set_tx_no_carrier ());

    m_edmac.set_transmit_fifo_threshold (0); // store and forward mode
    m_edmac.set_fifo_sizes (2048, 2048);
    m_edmac.set_recv_mode (EDMAC::receive_mode::multi_frame);
    m_edmac.set_flow_control_threshold (0, 16);
    m_edmac.set_tx_int_mode (EDMAC::tx_interrupt_mode::no_tx_interrupt);

    m_edmac.etherc ().set_mac_address (m_macaddr);
    m_edmac.etherc ().set_receive_max_frame_length (2048);

    m_edmac.etherc ().set_inter_packet_gap (m_linkinfo.speed == link_speed::m100 ? 960 : 9600);

    // clear all pending status interrupts by writing 1's
    m_edmac.set_etherc_edmac_status (typename EDMAC::etherc_edmac_status_t (0x47FF0F9F));
    m_edmac.etherc ().set_status (typename EDMAC::etherc_t::status_t (0x00000037));

    // enable interrupts for certain things
    // m_edmac.set_etherc_edmac_status_interrupts (...);
    // m_edmac.set_etherc_status_interrupts (...);

    typename EDMAC::etherc_t::mode_t mode;
    mode.set_duplex (m_linkinfo.duplex == link_duplex::full
		     ? EDMAC::etherc_t::mode_t::full_duplex
		     : EDMAC::etherc_t::mode_t::half_duplex);

    mode.set_speed (m_linkinfo.speed == link_speed::m100
		    ? EDMAC::etherc_t::mode_t::speed_100m
		    : EDMAC::etherc_t::mode_t::speed_10m);

    mode.set_rx_enable ();
    mode.set_tx_enable ();

    m_edmac.etherc ().set_mode (mode);
    m_edmac.start_receive ();
    m_edmac.start_transmit ();
  }


  void update_link_status (void) const
  {
    auto sta = m_phy.status ();
    auto ctl = m_phy.control ();

    m_linkinfo.isolated = ctl.isolate ();

    if (!sta.link_status ())
    {
      m_linkinfo.status = ctl.auto_neg_enable () && !sta.auto_neg_completed ()
			  ? link_status::negotiating
			  : link_status::down;

      m_linkinfo.speed = link_speed::invalid;
      m_linkinfo.duplex = link_duplex::invalid;
    }
    else
    {
      m_linkinfo.status = link_status::up;

      if (ctl.auto_neg_enable ())
      {
	// take the union of the caps that have been advertised by this phy
	// and the caps reported by the link partner.
	auto caps = m_phy.auto_neg_caps_advertisement ()
		    & m_phy.auto_neg_caps_link_partner ();

	// start with the best link type
	if (caps.cap_100_base_x_full ())
	{
	  m_linkinfo.speed = link_speed::m100;
	  m_linkinfo.duplex = link_duplex::full;
	}
	else if (caps.cap_100_base_x ())
	{
	  m_linkinfo.speed = link_speed::m100;
	  m_linkinfo.duplex = link_duplex::half;
	}
	else if (caps.cap_100_base_t4 ())
	{
	  m_linkinfo.speed = link_speed::m100;
	  m_linkinfo.duplex = link_duplex::half;
	}
	else if (caps.cap_10_base_t_full ())
	{
	  m_linkinfo.speed = link_speed::m10;
	  m_linkinfo.duplex = link_duplex::full;
	}
	else if (caps.cap_10_base_t ())
	{
	  m_linkinfo.speed = link_speed::m10;
	  m_linkinfo.duplex = link_duplex::half;
	}
	else
	{
	  m_linkinfo.speed = link_speed::invalid;
	  m_linkinfo.duplex = link_duplex::invalid;
	}
      }
      else
      {
	m_linkinfo.duplex = ctl.duplex () ? link_duplex::full : link_duplex::half;
	m_linkinfo.speed = ctl.speed () ? link_speed::m100 : link_speed::m10;
      }
    }

    m_linkinfo_valid = true;
    m_last_linkinfo_check_time = std::chrono::system_clock::now ();
  }

};

} // namespace dev
#endif // includeguard_rx_etherc_eth_hpp_includeguard


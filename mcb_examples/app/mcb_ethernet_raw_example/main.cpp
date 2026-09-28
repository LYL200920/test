/*
  MCB example application to access and control the ethernet controller in
  raw mode without an TCP/IP stack or the standard RX ethernet driver.

*/

#include <cstdio>
#include <chrono>

#include <board/board.hpp>
#include <dev/eth.hpp>
#include <hash/crc.hpp>

auto& debug_io = this_board::inst ().debug_usart;
auto& phy = this_board::inst ().eth0_phy;
auto& eth = this_board::inst ().eth0_mac;

using phy_t = std::remove_reference<decltype (phy)>::type;
using eth_t = std::remove_reference<decltype (eth)>::type;

void eth_init (void);
void eth_send_packet (unsigned int size);
void eth_receive_packet (void);

int main (void)
{
  const auto dipswitch = this_board::inst ().dipswitch_inputs.read ();

  // put the PHY into fixed 100 mbps full-duplex mode
  phy.set_control (phy_t::ctrl_t ()
		.set_auto_neg_enable (false)
		.set_speed (phy_t::speed_100mbps)
		.set_duplex (phy_t::duplex_full));

  phy.set_special_ctrl_status (phy_t::special_ctrl_status_t ()
		.set_mdi_mode (dipswitch[1] ? phy_t::manual_mdi : phy_t::manual_mdix));

  phy_t::status_t prev_phy_status;

  auto prev_time = std::chrono::system_clock::now ();

  auto phy_event_time = prev_time;
  unsigned int phy_event_count = 0;

  auto send_packet_event_time = prev_time;
  unsigned int send_packet_event_count = 0;

  for (unsigned int main_loop_count = 0; ; ++main_loop_count)
  {
    auto cur_time = std::chrono::system_clock::now ();
    this_board::inst ().exec ();

    if (cur_time - phy_event_time >= std::chrono::milliseconds (100))
    {
      phy_event_time = cur_time;
      ++phy_event_count;

      auto phy_status = phy.status ();
      if (phy_status != prev_phy_status)
      {
	auto link = phy.auto_neg_caps_link_partner ();
	auto ct = phy.control ();

	std::printf ("phy status change 0x%04x -> 0x%04x: \n"
		     "   link status: %d\n"
		     "   auto neg compl: %d\n"
		     "   lp caps: 0x%04x\n"
		     "   lp caps 10baseT half:  %d\n"
		     "   lp caps 10baseT full:  %d\n"
		     "   lp caps 100baseX half: %d\n"
		     "   lp caps 100baseX full: %d\n"
		     "   lp caps 100baseT4:     %d\n"
		     "   lp caps pause:         %d\n"
		     "   lp caps asym pause:    %d\n"
		     "   lp remote fault:       %d\n"
		     "   ctrl = 0x%04x  duplex: %d  speed: %d\n"
		     "   speed indication = %d\n",
			prev_phy_status.value (), phy_status.value (),
			phy_status.link_status (),
			phy_status.auto_neg_completed (),
			link.value (),
			link.cap_10_base_t (),
			link.cap_10_base_t_full (),
			link.cap_100_base_x (),
			link.cap_100_base_x_full (),
			link.cap_100_base_t4 (),
			link.cap_pause (),
			link.cap_asymmetric_pause (),
			link.remote_fault (),
			ct.value (), ct.duplex (), ct.speed (),
			(int)phy.phy_special_ctrl_status ().speed_indication ());

	bool link_down = prev_phy_status.link_status () & !phy_status.link_status ();
	bool link_up = !prev_phy_status.link_status () && phy_status.link_status ();

	if (link_down)
	  std::printf ("link down\n");

	if (link_up)
	{
	  std::printf ("link up\n");
	  eth_init ();
	}

	prev_phy_status = phy_status;
      }
    }

    if (dipswitch[0])
    {
      if (prev_phy_status.link_status ()
	  && (cur_time - send_packet_event_time >= std::chrono::milliseconds (500)))
      {
	const unsigned int send_sizes[] = { 4, 8, 12, 16, 20, 24, 32, 40, 42, 52, 60 };

	eth_send_packet (send_sizes[send_packet_event_count
				    % std::extent<decltype (send_sizes)>::value]);

	send_packet_event_time = cur_time;
	++send_packet_event_count;
      }
      else if (!prev_phy_status.link_status ())
      {
	send_packet_event_time = cur_time;
	send_packet_event_count = 0;
      }
    }

    if (prev_phy_status.link_status ())
      eth_receive_packet ();


    prev_time = cur_time;
  }


  return 0;
}

struct tx_desc : public eth_t::tx_desc
{
  tx_desc* next;
};

struct rx_desc : public eth_t::rx_desc
{
  rx_desc* next;
};

tx_desc* cur_tx_desc;
rx_desc* cur_rx_desc;

static constexpr unsigned int tx_buffer_count = 4;
static constexpr unsigned int rx_buffer_count = 4;
static constexpr unsigned int buffer_size = 2048;

alignas (sizeof (tx_desc)) std::array<tx_desc, tx_buffer_count> tx_descs;
alignas (sizeof (rx_desc)) std::array<rx_desc, rx_buffer_count> rx_descs;

typedef std::array<uint8_t, buffer_size> buffer;

alignas (32) std::array<buffer, tx_buffer_count> tx_buffers;
alignas (32) std::array<buffer, rx_buffer_count> rx_buffers;



void eth_init (void)
{
  eth.reset ();

  // setup descriptors.
  std::memset (tx_descs.data (), 0, tx_descs.size () * sizeof (tx_desc));
  std::memset (rx_descs.data (), 0, rx_descs.size () * sizeof (rx_desc));

  for (unsigned int i = 0; i < tx_buffer_count; ++i)
  {
    auto& d = tx_descs[i];
    d.set_data_ptr (tx_buffers[i].data ());
    d.set_type (eth_t::desc_single_frame);
    d.set_data_length (0);
    d.set_active (false);

    d.next = &tx_descs[(i+1) % tx_buffer_count];
  }

  for (unsigned int i = 0; i < rx_buffer_count; ++i)
  {
    auto& d = rx_descs[i];
    d.set_data_ptr (rx_buffers[i].data ());
    d.set_type (eth_t::desc_single_frame);
    d.set_max_data_length (buffer_size);
    d.set_active (true);

    d.next = &rx_descs[(i+1) % rx_buffer_count];
  }

  tx_descs.back ().set_ring_end (true);
  rx_descs.back ().set_ring_end (true);

  cur_rx_desc = &(rx_descs.front ());
  cur_tx_desc = &(tx_descs.front ());


  // setup registers

  eth.set_edmac_mode (eth_t::edmac_mode_t ()
	.set_descriptor_size (eth_t::desc_16_bytes)
	.set_data_endian (utils::native_byte_order () == utils::little_endian
			  ? eth_t::little_endian
			  : eth_t::big_endian));

  static_assert (sizeof (tx_desc) == 16, "");
  static_assert (sizeof (rx_desc) == 16, "");

  eth.set_edmac_transmit_desc_array (tx_descs.data ());
  eth.set_edmac_receive_desc_array (rx_descs.data ());

  eth.set_desc_status_copy_bits (eth_t::desc_status_copy_bits_t ()
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

//  eth.set_transmit_fifo_threadhold (0); // store and forward mode
  eth.set_transmit_fifo_threadhold (1);
  eth.set_fifo_sizes (2048, 2048);
  eth.set_receive_mode (eth_t::multi_frame);
  eth.set_flow_control_threshold (0, 16);
  eth.set_tx_interrupt_mode (eth_t::no_tx_interrupt);

//  eth.set_mac_address (m_macaddr);
  eth.set_receive_max_frame_length (2048);

  // 100 mbps ethernet standard inter-packet-gap
//  eth.set_inter_packet_gap (960);
  eth.set_inter_packet_gap (160);

  // clear all pending status interrupts by writing 1's
  eth.set_etherc_edmac_status (eth_t::etherc_edmac_status_t (0x47FF0F9F));
  eth.set_etherc_status (eth_t::etherc_status_t (0x00000037));

  // enable interrupts for certain things
  // eth.set_etherc_edmac_status_interrupts (...);
  // eth.set_etherc_status_interrupts (...);

  eth.set_mode (eth_t::mode_t ()
	.set_promiscuous (true)
	.set_duplex (eth_t::full_duplex)
	.set_speed (eth_t::speed_100m)
	.set_loop_back (false)
	.set_tx_enable ()
	.set_rx_enable ()
	.set_magic_packet_detection_enable (false)
	.set_crc_error_frame_rx_enable (true)
	.set_pause_frame_tx_enable (false)
	.set_pause_frame_rx_enable (false)
	.set_pause_frame_edmac_enable (false));

  eth.edmac_start_receive ();
  eth.edmac_start_transmit ();
}

void eth_receive_packet (void)
{
  auto cur_rx_st = cur_rx_desc->status ();

  if (!cur_rx_st.active ())
  {
    printf ("received packet len = %u bytes  err = %d\n",
	    cur_rx_desc->data_length (), cur_rx_st.error ());
    if (cur_rx_st.error ())
      printf ("  crc error:     %d\n"
	      "  phy error:     %d\n"
	      "  truncated:     %d\n"
	      "  overlong:      %d\n"
	      "  residual bit:  %d\n"
	      "  multicast:     %d\n"
	      "  aborted:       %d\n"
	      "  fifo overflow: %d\n",
		cur_rx_st.crc_error (), cur_rx_st.phy_error (),
		cur_rx_st.truncated (), cur_rx_st.overlong (),
		cur_rx_st.residual_bit (), cur_rx_st.multicast (),
		cur_rx_st.aborted (), cur_rx_st.fifo_overflow ());

    cur_rx_desc->set_active (true);
    cur_rx_desc = cur_rx_desc->next;
  }

  eth.edmac_start_receive ();
}

void eth_send_packet (unsigned int size)
{
  // drop the packet if the descriptor is still active.
  if (cur_tx_desc->status ().active ())
    return;

  printf ("sending packet with size = %u\n", size);

  size = std::min (size, (unsigned int)cur_tx_desc->data_length () - 4+60);

  for (unsigned int i = 0; i < size; ++i)
    ((uint8_t*)cur_tx_desc->data_ptr ())[i] = (uint8_t)i;

  hash::crc_32 checksum;
//  hash::crc < 32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true > checksum;

  checksum (cur_tx_desc->data_ptr (), size);

  uint32_t crc_val = checksum ();

  uint8_t* crc_in_packet = (uint8_t*)cur_tx_desc->data_ptr () + size;
  crc_in_packet[0] = (crc_val >> 24) & 0xFF;
  crc_in_packet[1] = (crc_val >> 16) & 0xFF;
  crc_in_packet[2] = (crc_val >>  8) & 0xFF;
  crc_in_packet[3] = (crc_val >>  0) & 0xFF;

  //uint32_t tx_buf_end_ptr = (uint32_t)&crc_in_packet[4];
  uint32_t tx_buf_end_ptr = (uint32_t)cur_tx_desc->data_ptr () + 4;

  for (unsigned int i = 0; i < 60; ++i)
    crc_in_packet[4+i] = 0x55;

 // cur_tx_desc->set_data_length (size + 4 + 60);
  cur_tx_desc->set_data_length (size + 4);
  cur_tx_desc->set_active (true);

  eth.edmac_start_transmit ();
//  eth.set_mode (eth.mode ().set_tx_enable ());
//  eth.set_mode (eth.mode ().set_tx_enable (false));

  unsigned int wait_count = 0;

  while (cur_tx_desc->status ().active ()
	 && eth.current_transmit_buffer_read_address () < tx_buf_end_ptr)
  {
    ++wait_count;
  }

  printf ("wait count = %u\n", wait_count);

/*
//  eth.set_mode (eth.mode ().set_tx_enable (false));
  phy.set_phy_special_ctrl_status (phy_t::phy_special_ctrl_status_t ()
		.set_disable_scrambling (true)
		.set_enable_4b5b (true));
*/

/*
  std::this_thread::sleep_for (std::chrono::microseconds (10));

  phy.set_phy_special_ctrl_status (phy_t::phy_special_ctrl_status_t ()
		.set_disable_scrambling (false)
		.set_enable_4b5b (true));
*/

  cur_tx_desc = cur_tx_desc->next;
}



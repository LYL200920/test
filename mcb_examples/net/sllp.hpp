/*
  simple link layer protocol (SLLP)

  this is a simple protocol for realizing packetized communication over
  arbitrary links such as RS232, RS485, ...
  it's basically like a smaller version of the ethernet MAC layer.  it has
  the same frame structure but the fields are smaller.

  in addition to simple packet switched data transfer, it also has some features
  to build small device networks.


frame structure:
{
  uint8_t		0x55 ( 0b0101'0101 ) frame start

  uint8_t		[7:6] 2 bit frame format
  			[5:0] 6 bit protocol id

  frame format 0b00:
  {
    uint8_t		src address
    uint8_t		dst address (0xFF = broadcast)

    uint16_t (BE)	payload data length

    n bytes		payload data (2 byte aligned)

    uint16_t (BE)	CRC-16 over the whole packet,
  			excluding the frame start and crc fields
  }

  frame format 0b01:    invalid to avoid conflict with frame start marker bits

  frame format 0b10:    reserved

  frame format 0b11:
  {
    uint8_t		src address
    uint8_t		dst address (0xFF = broadcast)
    uint8_t		payload data length

    n bytes		payload data (1 byte aligned)

    uint8_t		CRC-8-CCITT
  }
}

multi-byte fields denoted with (BE) are transmitted in big endian order.

frame format (2 bits)
 0b00 - 8 bit addressing, 16 bit payload length field, 16 bit crc
 0b01 - invalid
 0b10 - reserved
 0b11 - 8 bit adressing, 8 bit bit payload length, 8 bit crc

protocol id / packet type
 0x00 - device identification
 0x01 - device configuration
 0x02 - device ping

 0x10 - RPC
 0x11 - bridged ethernet


src and dst addresses can be byte swapped as a 16 bit value when
creating response packets.

FIXME: define inter packet gap (IPG)

---------------------------------------------------

device identification can be performed in 2 ways.

1) one way is similar to LLDP, where each station periodically broadcasts its
ID / hardware address.  after the master station picks up the newly found
device ID / hardware address, it assigns a shorter logical address to it.

to avoid collisions on the network, the ID transmission is triggered by a
broacast packet by the master station.  this also allows using a lower baudrate
for this communication.  when the stations receive the device inquiry packet,
they send a response packet.  if there are multiple stations it will require
some sort of collision handling.

2) another way is to use active address search, which is completely initiated
by the master station.  for that, 2 different packets are used
   - query address range
     sends min. and max. hardware address range.  each device compares its own
     hardware address.  if it is in range, it sends a positive response.
     if it is not in range, it doesn't send anything.

   - hardware address compare/ping
     sends one hardware address.  each device compares the address to its own
     hardware address.  if it is equal it sends a positive response.  if it is
     not equal it doesn't send anything.

the host then uses a binary search like algorithm to find all devices.  this
has limitations on the practically useable length of the hardware address.
for 6-byte MAC addresses about 200 bytes of RAM are required and 20
addresses can be discovered in 0.6 seconds at a baudrate of 250000.

for 16-byte addresses about 900 bytes of RAM are required and 8 addresses
can be discovered in 0.5 seconds at a baudrate of 250000.

the scanning time depends on the distribution of the address numbers.
the above are just examples.


general notes on device identification

device identification packets are used to enumerate and identify devices
on the network.  since slower devices might only support lower communication
speeds, device identification is done at a slow speed of 250000 bps.  when a
device is powered up (or reset) it starts communication at 250000 bps.

after a device has joined the network (the master has assigned a short local
address to it) the communication speed might be changed by the master station
based on the device capabilities.

after a while, the master station or some device might go offline (reset)
and re-join the network again.  for example, if the host stays online, but a
station is reset, they will initially not be able to communicate
if the communication baudrate that is used by the host is different from the
default baudrate of the station.  in this case, the host has to periodically
send out ping packets to each station to make sure it's still there.  in order
to re-discover a station that has been reset, the host needs to perform a bus
scan at the default baudrate.

if active bus scanning is used, it must be implemented in such a way that
it can run in parallel with regular communication.  the master station has to
switch the baudrate before it does one step of bus scanning (send one or more
packets, then switch to regular communication).  the bus scanning itself can be
triggered either manually or run continuously in the background.


in the other case, where a station remains online but the host is reset,
the host will start bus scanning at the default baudrate.  if the station has
been set to use a different baudrate before, it will not be able to receive
the bus scanning packets correctly and will also not receive any ping packets.
to solve this problem, there are two options

- the host does bus scanning at different baudrates, which it supports.
  the stations keep their previous baudrate setting and will be eventually
  re-discovered by the host

- the stations reset their baudrate to 250000 bps after they haven't received
  a ping packet from the host for a while (a few seconds).  they will be
  rediscovered by the host at the default baudrate again, like on initial
  power-on.


each device is identified either by a unique global hardware ID that is burned
into the hardware components of the board or a physical address switch or
a combination thereof.

for active bus scanning, the hardware ID is limited to 16 bytes.
for LLDP-like bus scanning, the hardware ID can be of any size, but should not
exceed 16 bytes.

---------------------------------------------------

device configuration packet format

sent by the host to a device to set communication speed and other parameters.
to support devices that don't have non-volatile configuration memory, the
logical device bus address is assigned dynamically, similar to DHCP on IP
networks.  each device has a unique device ID, which can be used by the master
station device to remember the device if it needs to and maybe also store some
parameters for that device.
for communication purposes, transferring the unique device ID would be
a significant overhead, thus a shorter 8 bit address is assigned dynamically
when the network is built up.

---------------------------------------------------

device ping packet format

uint8_t		ping request (0x11)
		ping response (0xEE)

*/

#ifndef includeguard_net_sllp_includeguard
#define includeguard_net_sllp_includeguard

#include <cstdint>
#include <array>
#include <atomic>
#include <cstring>
#include <vector>
#include <algorithm>
#include <chrono>
#include <optional>
#include <string_view>

#include <hash/crc.hpp>
#include <utils/byte_order.hpp>
#include <utils/ptm.hpp>
#include <utils/rodata.hpp>
#include <utils/text.hpp>

// enable for some logging for development/debugging
// make sure that the debug uart (or whatever device) has large enough transmit
// buffer size. otherwise the debug logging will block the processing code
// and it might malfunction.
// e.g. on MCB2, set the debug uart tx buffer size to 512 or 1024 bytes.
//#define NET_SLLP_DEVEL_DEBUG_TX
//#define NET_SLLP_DEVEL_DEBUG_TX_BYTES
//#define NET_SLLP_DEVEL_DEBUG_RX
//#define NET_SLLP_DEVEL_DEBUG_RX_BYTES

namespace net
{
namespace sllp
{

// protocol identifier
enum struct pid : uint8_t
{
  devid = 0x00,
  devcfg = 0x01,
  devstat = 0x02,

  rpc = 0x10,
  ethernet = 0x11

};

// ---------------------------------------------------------------------------

#pragma pack (1)
class frame_header_00
{
public:
  using address_type = uint8_t;
  using length_type = uint16_t;
  using crc_type = hash::crc_16_ibm;

  static constexpr address_type broadcast_addr = (address_type)0xFFFFFFFFu;
  static constexpr unsigned int max_data_length = std::numeric_limits<length_type>::max ();

  frame_header_00 (void) { }

  constexpr frame_header_00 (enum pid pid,
			     address_type src_addr, address_type dst_addr,
			     length_type len)
  : m_pid ((uint8_t)pid & 0b00111111),
    m_src_addr (src_addr), m_dst_addr (dst_addr), m_length (utils::native_to (utils::big_endian, len))
  {
  }

  constexpr enum pid pid (void) const { return (enum pid)m_pid; }

  constexpr address_type src_addr (void) const { return m_src_addr; }
  void set_src_addr (address_type val) { m_src_addr = val; }

  constexpr address_type dst_addr (void) const { return m_dst_addr; }
  void set_dst_addr (address_type val) { m_dst_addr = val; }

  constexpr length_type length (void) const { return utils::to_native (utils::big_endian, m_length); }
  void set_length (length_type val) { m_length = utils::native_to (utils::big_endian, val); }

  // this can be used in an if statement that also declares the header pointer
  // variable.
  static frame_header_00* check_frame_id_pid (void* data, enum pid p)
  {
    frame_header_00* h = (frame_header_00*)data;
    return h->m_pid == (((uint8_t)p & 0b00'111111) | 0b00'000000) ? h : nullptr;
  }
  static const frame_header_00* check_frame_id_pid (const void* data, enum pid p)
  {
    const frame_header_00* h = (const frame_header_00*)data;
    return h->m_pid == (((uint8_t)p & 0b00'111111) | 0b00'000000) ? h : nullptr;
  }


private:
  uint8_t m_pid;
  address_type m_src_addr;
  address_type m_dst_addr;
  length_type m_length;
};
#pragma pack ()

static_assert (sizeof (frame_header_00) == 5, "");

// ---------------------------------------------------------------------------

#pragma pack (1)
class frame_header_11
{
public:
  using address_type = uint8_t;
  using length_type = uint8_t;
  using crc_type = hash::crc_8_ccitt;

  static constexpr address_type broadcast_addr = (address_type)0xFFFFFFFFu;
  static constexpr unsigned int max_data_length = std::numeric_limits<length_type>::max ();

  frame_header_11 (void) { }

  constexpr frame_header_11 (enum pid pid,
			     address_type src_addr, address_type dst_addr,
			     length_type len)
  : m_pid (((uint8_t)pid & 0b00111111) | 0b11000000),
    m_src_addr (src_addr), m_dst_addr (dst_addr), m_length (len)
  {
  }

  constexpr enum pid pid (void) const { return (enum pid)m_pid; }

  constexpr address_type src_addr (void) const { return m_src_addr; }
  void set_src_addr (address_type val) { m_src_addr = val; }

  constexpr address_type dst_addr (void) const { return m_dst_addr; }
  void set_dst_addr (address_type val) { m_dst_addr = val; }

  constexpr length_type length (void) const { return m_length; }
  void set_length (length_type val) { m_length = val; }

  static frame_header_11* check_frame_id_pid (void* data, enum pid p)
  {
    frame_header_11* h = (frame_header_11*)data;
    return h->m_pid == (((uint8_t)p & 0b00'111111) | 0b11'000000) ? h : nullptr;
  }
  static const frame_header_11* check_frame_id_pid (const void* data, enum pid p)
  {
    const frame_header_11* h = (const frame_header_11*)data;
    return h->m_pid == (((uint8_t)p & 0b00'111111) | 0b11'000000) ? h : nullptr;
  }

private:
  uint8_t m_pid;
  address_type m_src_addr;
  address_type m_dst_addr;
  length_type m_length;
};
#pragma pack ()


static_assert (sizeof (frame_header_11) == 4, "");

// ---------------------------------------------------------------------------

struct data_receive_listener
{
  virtual ~data_receive_listener (void) { }
  virtual void sllp_data_received (const void* data, unsigned int data_size) = 0;

  virtual void sllp_attached (void) { }
  virtual void sllp_detached (void) { }
};


class data_receive_listener_wrapper : public data_receive_listener
{
public:
  template <typename F> data_receive_listener_wrapper (F&& f) : m_func (f) { }

  virtual void sllp_data_received (const void* data, unsigned int len) override
  {
  if (m_func != nullptr)
    m_func (data, len);
  }

  virtual void sllp_detached (void) override
  {
    delete this;
  }

private:
  std::function<void (const void*, unsigned int)> m_func;
};


// ---------------------------------------------------------------------------
// an endpoint that is built on top of a device that can send and receive
// raw data packets and/or bytes.

// if interrupts are not used for receiving bytes, polling frequency must be...
// 250 kbit/sec transmission rate = 31250 bytes / sec = 32 kHz for an
// unbuffered device.

template <typename Device,
	  unsigned int TxBufferCount, unsigned int RxBufferCount,
	  unsigned int UserBufferSize,
	  typename listener_container = std::vector<data_receive_listener*>,
	  bool AssumeListenerContainerAlwaysHasSpace = false,

          // to enable rx-to-tx delay specify a std::chrono::duration.
          // e.g.
          //  std::chrono::duration<unsigned int, std::ratio<1, 10'000 >>
          // will set a duration of 100 usec.
	  typename Rx_to_Tx_Delay_Duration = void >
class endpoint
{
  static constexpr unsigned int ActualBufferSize = UserBufferSize + 8;

  static_assert (UserBufferSize >= 16, "user buffer size is too small");


public:
  endpoint (Device& dev, uint8_t addr) : m_dev (dev), m_addr (addr)
  {
/*
    dev.set_recv_clb (this, utils::ptm_clb<decltype (&endpoint::on_data_recv),
					   &endpoint::on_data_recv> ());
*/
    init_rx ();
    init_tx ();
  }

  ~endpoint (void)
  {
    for (auto& l : m_data_receive_listeners)
      l->sllp_detached ();
  }

  uint8_t address (void) const { return m_addr; }
  void set_address (uint8_t val) { m_addr = val; }

  Device& dev (void) const { return m_dev; }

  void exec (void)
  {
    if constexpr (std::is_same_v<Rx_to_Tx_Delay_Duration, void>)
      exec_1 ();
    else
      exec_1_rx_to_tx_delay ();
  }

  // copy the raw packet to the transmit queue.
  // if the transmit queue is already full, drop it.
  // the data is expected to start with the PID field, not including the crc
  // field or the frame start marker.
  // CAUTION -- the RX CRC unit might be used here to calculate the packet
  //            checksum.  it's not safe to use it in an interrupt context.
  void write (const void* data, unsigned int byte_count)
  {
    // drop the packet if the buffer is still active.
    if (m_tx_buf_wr->active)
      return;

    // refuse to send overlong packets which would not fit into the tx buffer
    // or if it's too short.
    if (byte_count > UserBufferSize || byte_count < 3)
      return;

    uint8_t frame_type = (*(const uint8_t*)data) & 0b11'000000;
    uint8_t* buf_out = m_tx_buf_wr->data.data ();

    // write frame start marker
    *buf_out++ = 0x55;

    std::memcpy (buf_out, data, byte_count);
    buf_out += byte_count;

  #ifndef NET_SLLP_DISABLE_FRAME_11
    if (frame_type == 0b11'000000)
    {
      // send as short frame
      frame_header_11::crc_type crc;
      crc (data, byte_count);

      static_assert (std::is_same<decltype (crc)::value_type, uint8_t>::value, "");
      *buf_out++ = crc ();

      #ifdef NET_SLLP_DEVEL_DEBUG_TX
      {
        unsigned int tx_byte_count = buf_out - m_tx_buf_wr->data.data ();

        printf ("sllp send short frame %u bytes: ", buf_out - m_tx_buf_wr->data.data ());

        #ifdef NET_SLLP_DEVEL_DEBUG_TX_BYTES
          print_bytes (nullptr, m_tx_buf_wr->data.data (), tx_byte_count);
        #endif
      }
      #endif
    }
    else
  #endif // NET_SLLP_DISABLE_FRAME_11
    if (frame_type == 0b00'000000)
    {
      // send as long frame
      frame_header_00::crc_type crc;
      crc (data, byte_count);

      static_assert (std::is_same<decltype (crc)::value_type, uint16_t>::value, "");

      uint16_t crc_val = crc ();
      *buf_out++ = (uint8_t)(crc_val >> 8);
      *buf_out++ = (uint8_t)(crc_val >> 0);

      #ifdef NET_SLLP_DEVEL_DEBUG_TX
      {
        unsigned int tx_byte_count = buf_out - m_tx_buf_wr->data.data ();
        printf ("sllp send long frame %u bytes: ", tx_byte_count);
        print_bytes (nullptr, m_tx_buf_wr->data.data (), tx_byte_count);
      }
      #endif
    }
    else
    {
      // drop invalid / reserved frame type
      m_statistics.tx_early_dropped_frames += 1;
      return;
    }

    // when transmitting over rs485 we need to append a trailing byte
    // because of timing issues when switching the transceiver on/off.
    *buf_out++ = 0x00;

    m_tx_buf_wr->data_len = buf_out - m_tx_buf_wr->data.data ();
    m_tx_buf_wr->active = true;
    m_tx_buf_wr = m_tx_buf_wr->next;

    // if it's not already transmitting, start it.
    //transmit ();
  }

  // read a packet from the receive queue.
  // instead of copying the data from the packet buffer into some user buffer,
  // let the user work directly on the buffer.
  //
  // there can be multiple clients running over the same SLLP endpoint, for
  // example multiple RPC channels over the same link (which will use different
  // RPC ports).  whenever data is received on the endpoint, it has to notify
  // its listeners, which are supposed to implement packet filtering.
  template <typename F> void read (F&& callback_func)
  {
    if (!m_rx_buf_rd->active)
    {
      // because the received frame includes the frame headers it is impossible
      // to receive a zero length frame for the user.  even if the payload is
      // zero bytes, the header fields make the whole frame non-zero long. 
      if (m_rx_buf_rd->data_len > 0)
      {
	callback_func (m_rx_buf_rd->data.data (), m_rx_buf_rd->data_len);

	// FIXME: not re-entrant
	for (auto& l : m_data_receive_listeners)
	  l->sllp_data_received (m_rx_buf_rd->data.data (), m_rx_buf_rd->data_len);
      }

      // re-enable the buffer for reception and proceed to the next.
      m_rx_buf_rd->active = true;
      m_rx_buf_rd = m_rx_buf_rd->next;
    }
  }

  // check if the listener container has a member function 'push_back_unchecked'
  // if so, use it.  if not, use the generic 'push_back'
  template <typename C> static bool test_push_back_unchecked (
	decltype (static_cast<void(C::*)(data_receive_listener* const&)>(&C::push_back_unchecked)));

  template <typename C> static int test_push_back_unchecked (...);

  template < typename SS = listener_container >
  uintptr_t attach_data_receive_listener(data_receive_listener* l)
  {
	  if constexpr (sizeof(test_push_back_unchecked<SS>(0)) == sizeof(bool) && AssumeListenerContainerAlwaysHasSpace)
		  m_data_receive_listeners.push_back_unchecked(l);
	  else
		  m_data_receive_listeners.push_back(l);

	  l->sllp_attached();
	  return (uintptr_t)l;
  }

  uintptr_t attach_data_receive_listener (std::function<void (const void* data, unsigned int len)> f)
  {
    // construct a listener object which will self-destruct when it is detached
    return attach_data_receive_listener (new data_receive_listener_wrapper (f));
  }

  void detach_data_receive_listener (data_receive_listener* l)
  {
    // FIXME: not re-entrant.
    auto i = std::find (m_data_receive_listeners.begin (),
			m_data_receive_listeners.end (), l);

    if (i != m_data_receive_listeners.end ())
    {
      auto last_i = m_data_receive_listeners.end () - 1;
      std::iter_swap (i, last_i);
      (*last_i)->sllp_detached ();  // might potentially self-destruct
      m_data_receive_listeners.pop_back ();
    }
  }

  void detach_data_receive_listener (uintptr_t n)
  {
    detach_data_receive_listener ((data_receive_listener*)n);
  }

  struct statistics
  {
    // total number if input bytes processed from the device.
    unsigned int rx_bytes = 0;

    // number of ok frames received.
    unsigned int rx_ok_frames = 0;

    // number of ng frames received.
    unsigned int rx_ng_frames = 0;

    // number of frames received that had a crc error (and were dropped).
    unsigned int rx_crc_errors = 0;

    // number of frames received that were too long.
    unsigned int rx_overruns = 0;

   // number of frames received that had a mismatching target address.
   unsigned int rx_frames_address_mismatch = 0;


    // total number of output bytes sent to the device.
    unsigned int tx_bytes = 0;

    // number of ok frames transmitted.
    unsigned int tx_ok_frames = 0;

    // number of frames dropped early and not even tried to transmit.
    unsigned int tx_early_dropped_frames = 0;

    // number of transmitted frames which had some error.
    unsigned int tx_errors = 0;
  };

  const statistics& stats (void) const { return m_statistics; }

  void reset_rx (void)
  {
    init_rx ();
  }

  void reset_tx (void)
  {
    init_tx ();
  }

private:
  struct packet_buffer
  {
    // for tx buffers, an active buffer means it contains valid data to be
    // transmitted.  the flag will be cleared after the buffer has been
    // transmitted.
    //
    // for rx buffers, an active buffer means it does not contain data yet
    // but has been queued for reception.  the flag will be cleared after
    // data has been received.
    std::atomic<int> active;

    // the number of valid bytes (frame size) in the buffer.
    unsigned int data_len;

    // next buffer in the chain.
    packet_buffer* next;

    alignas (16) std::array<uint8_t, ActualBufferSize> data;
  };

  Device& m_dev;

  // address of this endpoint.
  uint8_t m_addr;


  // the buffer to where the next data will be written by the user.
  packet_buffer* m_tx_buf_wr;

  // the buffer that is currently being transmitted.
  packet_buffer* m_tx_buf_rd;

  // the buffer that is currently being used for writing received data.
  packet_buffer* m_rx_buf_wr;

  // the buffer from where the next data will be read by the user.
  packet_buffer* m_rx_buf_rd;

  enum struct rx_state
  {
    frame_start_0,
    frame_start_1,

    frame_00_src_dst_len_fields,
    frame_00_data,
    frame_00_crc,

  #ifndef NET_SLLP_DISABLE_FRAME_11
    frame_11_src_dst_len_fields,
    frame_11_data,
    frame_11_crc
  #endif
  };

  enum struct tx_state
  {
    idle,
    transmitting
  };

  std::atomic_flag m_on_data_recv_lock = ATOMIC_FLAG_INIT;
  std::atomic_flag m_transmit_lock = ATOMIC_FLAG_INIT;

  rx_state m_rx_state;
  tx_state m_tx_state;

  // the current buffer pointer from where it's transmitting.
  uint8_t* m_tx_cur_rd_ptr;

  // the number of bytes that are still to be transmitted.
  unsigned int m_tx_remaining_count;


  // list of listeners
  // FIXME: not re-entrant
  listener_container m_data_receive_listeners;


  statistics m_statistics;

  alignas (32) std::array<packet_buffer, TxBufferCount> m_tx_buffers;
  alignas (32) std::array<packet_buffer, RxBufferCount> m_rx_buffers;

  std::optional<std::chrono::high_resolution_clock::time_point> m_last_rx_time;

  [[gnu::cold]] void init_rx (void)
  {
    // setup circular linked lists for the buffers.
    for (unsigned int i = 0; i < m_rx_buffers.size (); ++i)
    {
      auto& b = m_rx_buffers[i];
      b.active = true;
      b.data_len = 0;
      b.next = &m_rx_buffers[(i + 1) % m_rx_buffers.size ()];
    }

    m_rx_buf_wr = m_rx_buf_rd = &m_rx_buffers.front ();
    m_rx_state = rx_state::frame_start_0;

    m_statistics.rx_bytes = 0;
    m_statistics.rx_ok_frames = 0;
    m_statistics.rx_ng_frames = 0;
    m_statistics.rx_crc_errors = 0;
    m_statistics.rx_overruns = 0;
    m_statistics.rx_frames_address_mismatch = 0;

    m_dev.reset_receiver ();
    m_dev.reset_rx_buffer ();
  }

  [[gnu::cold]] void init_tx (void)
  {
    // setup circular linked lists for the buffers.
    for (unsigned int i = 0; i < m_tx_buffers.size (); ++i)
    {
      auto& b = m_tx_buffers[i];
      b.active = false;
      b.data_len = 0;
      b.next = &m_tx_buffers[(i + 1) % m_tx_buffers.size ()];
    }

    m_tx_buf_wr = m_tx_buf_rd = &m_tx_buffers.front ();
    m_tx_state = tx_state::idle;

    m_statistics.tx_bytes = 0;
    m_statistics.tx_ok_frames = 0;
    m_statistics.tx_early_dropped_frames = 0;
    m_statistics.tx_errors = 0;

    m_dev.reset_transmitter ();
  }

  // main function that takes care of data transmission.  this function can
  // either be called periodically from some thread or form the device's
  // "transmit low watermark" interrupt callback.
  void transmit (unsigned int send_max_bytes)
  {
    // do not allow concurrent re-entrancy from multiple execution contexts
    // (normal thread and interrupt context).
    if (m_transmit_lock.test_and_set ())
      return;

    transmit_1 (send_max_bytes);
    m_transmit_lock.clear ();
  }

  void exec_1 (void)
  {
    auto buffer_st = m_dev.buffer_stat ();

    if (buffer_st.rx_error)
    {
      m_dev.reset_receiver ();
      m_dev.reset_rx_buffer ();
    }
    else if (buffer_st.rx_fifo_size > 0)
      on_data_recv (buffer_st.rx_fifo_size);

    transmit (buffer_st.tx_fifo_capacity - buffer_st.tx_fifo_size);
  }

  void exec_1_rx_to_tx_delay (void)
  {
    auto buffer_st = m_dev.buffer_stat ();

    if (buffer_st.rx_error)
    {
      m_dev.reset_receiver ();
      m_dev.reset_rx_buffer ();
    }
    else if (buffer_st.rx_fifo_size > 0)
    {
      on_data_recv (buffer_st.rx_fifo_size);
      m_last_rx_time = std::chrono::high_resolution_clock::now ();
    }
    else
    {
      // for half-duplex link: don't transmit anything for a while after
      // receiving something
      if (!m_last_rx_time.has_value ()
          || (std::chrono::high_resolution_clock::now () - m_last_rx_time.value ()
               > Rx_to_Tx_Delay_Duration (1) ))
      {
      	m_last_rx_time = { };
        //transmit (tx_count);
        transmit (buffer_st.tx_fifo_capacity - buffer_st.tx_fifo_size);
      }
    }
  }

  void transmit_1 (unsigned int send_max_bytes)
  {
    do
    {
      if (m_tx_state == tx_state::transmitting)
      {
        #ifdef NET_SLLP_DEVEL_DEBUG_TX
          printf ("sllp tx max: %u\n", send_max_bytes);
        #endif

	unsigned int send_byte_count = std::min (send_max_bytes, m_tx_remaining_count);
	if (send_byte_count > 0)
	{
	  m_dev.write (m_tx_cur_rd_ptr, send_byte_count);
	  m_tx_cur_rd_ptr += send_byte_count;
	  m_tx_remaining_count -= send_byte_count;
	  send_max_bytes -= send_byte_count;
	  m_statistics.tx_bytes += send_byte_count;

          #ifdef NET_SLLP_DEVEL_DEBUG_TX
            printf ("sllp tx to dev: %u\n", send_byte_count);
          #endif
	}

	if (m_tx_remaining_count == 0)
	{
          #ifdef NET_SLLP_DEVEL_DEBUG_TX
            printf ("sllp tx next buf\n");
          #endif

	  // finish this buffer, try next.
	  m_tx_buf_rd->active = false;
	  m_tx_buf_rd = m_tx_buf_rd->next;
	  m_tx_state = tx_state::idle;
	  m_statistics.tx_ok_frames += 1;
	}
      }

      if (m_tx_state == tx_state::idle && m_tx_buf_rd->active)
      {
	m_tx_cur_rd_ptr = m_tx_buf_rd->data.data ();
	m_tx_remaining_count = m_tx_buf_rd->data_len;
	m_tx_state = tx_state::transmitting;
      }
    } while (m_tx_state == tx_state::transmitting && send_max_bytes > 0);
  }



  // main function that takes care of data reception.  this function can either
  // be called periodically from some thread or from the device's receive
  // interrupt callback.
  void on_data_recv (unsigned int rx_buffer_count)
  {
    // do not allow concurrent re-entrancy from multiple execution contexts
    // (normal thread and interrupt context).
    if (m_on_data_recv_lock.test_and_set ())
      return;

    on_data_recv_1 (rx_buffer_count);
    m_on_data_recv_lock.clear ();
  }

  // debug printing utilities which bypass printf to get a little more speed.
  [[gnu::hot]] static void print_hexchar (uint8_t b)
  {
    static const char tostr[] = "0123456789abcdef";
    putchar (tostr[(b >> 4) & 0x0F]);
    putchar (tostr[(b >> 0) & 0x0F]);
  };

  static void print_str (const char* s)
  {
    if (s == nullptr)
      return;

    while (*s != 0)
      putchar (*s++);
  }

  static void print_bytes (const char* prefix, const void* data, unsigned int len)
  {
    print_str (prefix);
    const uint8_t* data8 = (const uint8_t*)data;
    for (unsigned int i = 0; i < len; ++i)
    {
      print_hexchar (*data8++);
      putchar (' ');
    }
    putchar ('\n');
  }

  void on_data_recv_1 (unsigned int rx_buffer_count)
  {
    #ifdef NET_SLLP_DEVEL_DEBUG_RX
      printf ("sllp on_data_recv_1 %u\n", rx_buffer_count);
    #endif
    // the device has some new data in the receive buffer which we should
    // read out.  notice that this callback function might be invoked in an
    // interrupt context, so the things we can do here are somewhat limited.
    std::array<uint8_t, 16> tmpbuf;

    m_statistics.rx_bytes += rx_buffer_count;

    while (rx_buffer_count > 0)
    {
      if (!m_rx_buf_wr->active)
      {
	// we don't have a free buffer.  drop the received data.
	m_dev.reset_rx_buffer ();
	m_rx_state = rx_state::frame_start_0;
	return;
      }

      switch (m_rx_state)
      {
	// look for the 0x55 marker followed by the PID byte. the PID byte
	// should have the top bits set to 0b00 or 0b11.
	// FIXME: reading byte-by-byte is slow and here we're slowing down
	// the interrupt handler.  optimize this later.
	case rx_state::frame_start_0:
	  m_dev.read (tmpbuf.data (), 1);
	  --rx_buffer_count;
	  m_rx_buf_wr->data_len = 0;

          #ifdef NET_SLLP_DEVEL_DEBUG_RX_BYTES
	    print_bytes ("sllp rx: ", tmpbuf.data (), 1);
          #endif

	  if (tmpbuf.front () == 0x55)
	    m_rx_state = rx_state::frame_start_1;

	  break;

	case rx_state::frame_start_1:
	{
	  m_dev.read (tmpbuf.data (), 1);
	  --rx_buffer_count;

          #ifdef NET_SLLP_DEVEL_DEBUG_RX_BYTES
	    print_bytes ("sllp rx: ", tmpbuf.data (), 1);
          #endif


	  if (tmpbuf.front () == 0x55)
	  {
	    // it seems there are multiple frame start bytes... just skip them.
	  }
	  else if ((tmpbuf.front () & 0b11'000000) == 0b00'000000)
	  {
	    m_rx_buf_wr->data.front () = tmpbuf.front ();
	    m_rx_buf_wr->data_len = 1;
	    m_rx_state = rx_state::frame_00_src_dst_len_fields;
	  }

	#ifndef NET_SLLP_DISABLE_FRAME_11
	  else if ((tmpbuf.front () & 0b11'000000) == 0b11'000000)
	  {
	    m_rx_buf_wr->data.front () = tmpbuf.front ();
	    m_rx_buf_wr->data_len = 1;
	    m_rx_state = rx_state::frame_11_src_dst_len_fields;
	  }
	#endif
	  else
	  {
	    // something else .. maybe we have started receiving in the
	    // middle of a frame.
	    m_rx_state = rx_state::frame_start_0;
	    m_statistics.rx_ng_frames += 1;
	    m_rx_buf_wr->data_len = 0;
	  }

	  break;
	}

	// read 4 bytes src address, dst address and length field.
	// try to read the data at once.  if there's not enough data, wait
	// for the next invocation of this receive function.
	case rx_state::frame_00_src_dst_len_fields:
	{
	  if (rx_buffer_count < 4)
	    return;

	  m_dev.read (m_rx_buf_wr->data.data () + m_rx_buf_wr->data_len, 4);

          #ifdef NET_SLLP_DEVEL_DEBUG_RX_BYTES
	    print_bytes ("sllp rx: ", m_rx_buf_wr->data.data () + m_rx_buf_wr->data_len, 4);
          #endif

	  m_rx_buf_wr->data_len += 4;
	  rx_buffer_count -= 4;
	  m_rx_state = rx_state::frame_00_data;
	  break;
	}

	case rx_state::frame_00_data:
	{
	  // 2 bytes extra for the crc field.
	  unsigned int expected_data_len =
		sizeof (frame_header_00::crc_type::value_type)
		+ ((m_rx_buf_wr->data.data ()[3] << 8) | m_rx_buf_wr->data.data ()[4]);

	  unsigned int cur_data_len =
		m_rx_buf_wr->data_len - sizeof (frame_header_00);

	  #ifdef NET_SLLP_DEVEL_DEBUG_RX
	  printf ("rx_state::frame_00_data expected_data_len = %u  cur_data_len = %u\n",
		  expected_data_len, cur_data_len);
	  #endif

	  if (expected_data_len == cur_data_len)
	    m_rx_state = rx_state::frame_00_crc;
	  else
	  {
	    uint8_t* wr_ptr = m_rx_buf_wr->data.data () + m_rx_buf_wr->data_len;
	    uint8_t* const wr_ptr_max = (m_rx_buf_wr->data.data() + m_rx_buf_wr->data.size());

	    unsigned int const max_read_count =
		std::min ((unsigned int)(wr_ptr_max - wr_ptr), rx_buffer_count);

	    unsigned int bytes_to_read =
		std::min (max_read_count, expected_data_len - cur_data_len);

	    if (bytes_to_read > 0)
	    {
	      m_dev.read (wr_ptr, bytes_to_read);

              #ifdef NET_SLLP_DEVEL_DEBUG_RX_BYTES
                printf ("sllp rx (a) %u: ", bytes_to_read);
		print_bytes (nullptr, wr_ptr, bytes_to_read);
              #endif

	      m_rx_buf_wr->data_len += bytes_to_read;
	      rx_buffer_count -= bytes_to_read;
	      cur_data_len += bytes_to_read;
	    }

	    if (expected_data_len == cur_data_len)
	    {
	      m_rx_state = rx_state::frame_00_crc;
	      goto rx_state_frame_00_crc;
	    }
	    else if (m_rx_buf_wr->data_len >= m_rx_buf_wr->data.size())
	    {
	      m_statistics.rx_overruns += 1;
	      m_rx_buf_wr->active = true;
	      m_rx_buf_wr->data_len = 0;
	      m_rx_state = rx_state::frame_start_0;
	      m_dev.reset_rx_buffer ();
	    }
	  }

	  break;
	}

	// calculate and validate the crc of the packet.  we can't safely
	// use the crc library from within an interrupt context, because
	// it might use some hardware device (on RX MCUs it uses the hardware
	// crc calculator for some crc types).  thus, postpone crc calculation
	// until this function is invoked from another thread, outside the
	// interrupt context.
	case rx_state::frame_00_crc:
	{
          rx_state_frame_00_crc:
	  auto target_address = ((const frame_header_00*)m_rx_buf_wr->data.data ())->dst_addr ();

	  if (target_address != m_addr && target_address != frame_header_00::broadcast_addr)
	  {
	    // drop frames that have a mismatching destination address early.
	    m_statistics.rx_frames_address_mismatch += 1;
	    m_rx_buf_wr->active = true;
	    m_rx_buf_wr->data_len = 0;
	    m_rx_state = rx_state::frame_start_0;
	    break;
	  }

	  #ifdef __cpp_this_thread_is_interrupt_context
	  if (std::this_thread::is_interrupt_context ())
	    return;
	  #endif

	  unsigned int const crc_len = sizeof (frame_header_00::crc_type::value_type);
	  uint8_t* wr_ptr = m_rx_buf_wr->data.data () + m_rx_buf_wr->data_len;

	  frame_header_00::crc_type crc;
	  crc (m_rx_buf_wr->data.data (), m_rx_buf_wr->data_len - crc_len);

	  unsigned int expected_crc = crc ();
	  unsigned int received_crc = (*(wr_ptr - crc_len) << 8) | *(wr_ptr - crc_len + 1);

	  #ifdef NET_SLLP_DEVEL_DEBUG_RX
	  printf ("\n\nrx data %u bytes: ", m_rx_buf_wr->data_len);
	  for (unsigned int i = 0; i < m_rx_buf_wr->data_len; ++i)
	    printf ("%02x ", ((const unsigned char*)m_rx_buf_wr->data.data ())[i]);
	  printf ("\n expected crc = %04x  received crc = %04x\n", expected_crc, received_crc);
	  #endif

	  if (expected_crc != received_crc)
	  {
	    m_statistics.rx_crc_errors += 1;

	    #ifdef NET_SLLP_DEVEL_DEBUG_RX
	    printf ("crc error %u\n", m_statistics.rx_crc_errors);
	    #endif

	    // drop the bad frame.  keep the current rx packet descriptor
	    // and start receiving the next frame in the same place.
	    m_rx_buf_wr->active = true;
	    m_rx_buf_wr->data_len = 0;
	  }
	  else
	  {
	    // mark the current rx buffer descriptor as in-active, i.e. indicate
	    // that it has finished receiving some data.
	    // take the next descriptor and use it for receiving the next frame.

	    m_statistics.rx_ok_frames += 1;

	    #ifdef NET_SLLP_DEVEL_DEBUG_RX
	    printf ("rx ok frame %u\n", m_statistics.rx_ok_frames);
	    #endif

	    m_rx_buf_wr->data_len -= crc_len;
	    m_rx_buf_wr->active = false;
	    m_rx_buf_wr = m_rx_buf_wr->next;
	  }

	  m_rx_state = rx_state::frame_start_0;
	  break;
	}

      #ifndef NET_SLLP_DISABLE_FRAME_11
	// read 3 bytes src address, dst address and length field.
	case rx_state::frame_11_src_dst_len_fields:
	{
	  if (rx_buffer_count < 3)
	    return;

	  m_dev.read (m_rx_buf_wr->data.data () + m_rx_buf_wr->data_len, 3);

          #ifdef NET_SLLP_DEVEL_DEBUG_RX_BYTES
	    print_bytes ("sllp rx: ", m_rx_buf_wr->data.data () + m_rx_buf_wr->data_len, 3);
          #endif

	  m_rx_buf_wr->data_len += 3;
	  rx_buffer_count -= 3;
	  m_rx_state = rx_state::frame_11_data;
	  break;
	}

	case rx_state::frame_11_data:
	{
	  unsigned int expected_data_len =
		sizeof (frame_header_11::crc_type::value_type)
		+ m_rx_buf_wr->data.data ()[3];

	  unsigned int cur_data_len =
		m_rx_buf_wr->data_len - sizeof (frame_header_11);

	  if (expected_data_len == cur_data_len)
	    m_rx_state = rx_state::frame_11_crc;
	  else
	  {
	    uint8_t* wr_ptr = m_rx_buf_wr->data.data () + m_rx_buf_wr->data_len;
	    uint8_t* const wr_ptr_max = (m_rx_buf_wr->data.data() + m_rx_buf_wr->data.size());

	    unsigned int const max_read_count =
		std::min ((unsigned int)(wr_ptr_max - wr_ptr), rx_buffer_count);

	    unsigned int bytes_to_read =
		std::min (max_read_count, expected_data_len - cur_data_len);

	    if (bytes_to_read > 0)
	    {
	      m_dev.read (wr_ptr, bytes_to_read);

              #ifdef NET_SLLP_DEVEL_DEBUG_RX_BYTES
                printf ("sllp rx (b) %u: ", bytes_to_read);
                print_bytes (nullptr, wr_ptr, bytes_to_read);
              #endif

	      m_rx_buf_wr->data_len += bytes_to_read;
	      rx_buffer_count -= bytes_to_read;
	      cur_data_len += bytes_to_read;
	    }

	    if (expected_data_len == cur_data_len)
	    {
	      m_rx_state = rx_state::frame_11_crc;
	      goto rx_state_frame_11_crc;
	    }
	    else if (m_rx_buf_wr->data_len >= m_rx_buf_wr->data.size())
	    {
	      m_statistics.rx_overruns += 1;
	      m_rx_buf_wr->active = true;
	      m_rx_buf_wr->data_len = 0;
	      m_rx_state = rx_state::frame_start_0;
	      m_dev.reset_rx_buffer ();
	    }
	  }

	  break;
	}

	case rx_state::frame_11_crc:
	{
	  rx_state_frame_11_crc:
	  auto target_address = ((const frame_header_11*)m_rx_buf_wr->data.data ())->dst_addr ();

	  if (target_address != m_addr && target_address != frame_header_11::broadcast_addr)
	  {
	    // drop frames that have a mismatching destination address early.
	    m_statistics.rx_frames_address_mismatch += 1;
	    m_rx_buf_wr->active = true;
	    m_rx_buf_wr->data_len = 0;
	    m_rx_state = rx_state::frame_start_0;
	    break;
	  }

	  #ifdef __cpp_this_thread_is_interrupt_context
	  if (std::this_thread::is_interrupt_context ())
	    return;
	  #endif

	  unsigned int const crc_len = sizeof (frame_header_11::crc_type::value_type);
	  uint8_t* wr_ptr = m_rx_buf_wr->data.data () + m_rx_buf_wr->data_len;

	  frame_header_11::crc_type crc;
	  crc (m_rx_buf_wr->data.data (), m_rx_buf_wr->data_len - crc_len);

	  unsigned int expected_crc = crc ();
	  unsigned int received_crc = *(wr_ptr - crc_len);

	  if (expected_crc != received_crc)
	  {
	    m_statistics.rx_crc_errors += 1;

	    // drop the bad frame.  keep the current rx packet descriptor
	    // and start receiving the next frame in the same place.
	    m_rx_buf_wr->active = true;
	    m_rx_buf_wr->data_len = 0;
	  }
	  else
	  {
	    // mark the current rx buffer descriptor as inactive, i.e. indicate
	    // that it has finished receiving some data.
	    // take the next descriptor and use it for receiving the next frame.

	   m_statistics.rx_ok_frames += 1;

	    m_rx_buf_wr->data_len -= crc_len;
	    m_rx_buf_wr->active = false;
	    m_rx_buf_wr = m_rx_buf_wr->next;
	  }

	  m_rx_state = rx_state::frame_start_0;
	  break;
	}
      #endif // NET_SLLP_DISABLE_FRAME_11

      }  // switch (m_rx_state)
    }
  }

};


// ---------------------------------------------------------------------------
// device identification
//
// used to implement bus scanning and assigning logical addresses to
// devices with a unique device ID.

template <unsigned int LengthBytes >
struct devid_addr_n
{
  static constexpr unsigned int size_bytes = LengthBytes;

  std::array<uint8_t, size_bytes> bytes;

  using value_type = uint8_t;

  template <typename... Args>
  constexpr devid_addr_n (Args&&... args) : bytes { std::forward<Args> (args)... } { }

  constexpr devid_addr_n (const devid_addr_n& other) : bytes { other.bytes } { }
  constexpr devid_addr_n (devid_addr_n& other) : bytes { other.bytes } { }
  constexpr devid_addr_n (devid_addr_n&& other) : bytes { other.bytes } { }

  constexpr devid_addr_n (uint32_t x) : bytes { zero_value ().bytes }
  {
    bytes[bytes.size () - 1 - 0] = (x >> 0) & 0xFF;
    if constexpr (size_bytes > 1)
      bytes[bytes.size () - 1 - 1] = (x >> 8) & 0xFF;
    if constexpr (size_bytes > 2)
      bytes[bytes.size () - 1 - 2] = (x >> 16) & 0xFF;
    if constexpr (size_bytes > 3)
      bytes[bytes.size () - 1 - 3] = (x >> 24) & 0xFF;
  }

  constexpr devid_addr_n& operator = (const devid_addr_n& other)
  {
    bytes = other.bytes;
    return *this;
  }

  constexpr bool operator < (const devid_addr_n& other) const
  {
    // check if number 'a' is less than number 'b'
    // check high digits first.  if high digit a < b, then
    // whole number is a < b.
    // if high digit a == b, check lower digits.

    // b   = 11 00 23 08
    // a   = 00 80 00 00

    for (unsigned int i = 0; i < bytes.size (); ++i)
    {
      if (this->bytes[i] < other.bytes[i])
	return true;
      else if (this->bytes[i] > other.bytes[i])
	return false;
    }

    // when here we have checked all digits and they are all equal.
    return false;
  }

  constexpr bool operator == (const devid_addr_n& other) const
  {
    for (unsigned int i = 0; i < bytes.size (); ++i)
      if (this->bytes[i] != other.bytes[i])
	return false;

     return true;
  }

  constexpr bool is_zero (void) const
  {
    for (auto&& i : bytes)
      if (i != 0)
	return false;

    return true;
  }

  constexpr devid_addr_n shr_1 (void) const
  {
    // shift right by one bit
    auto r = bytes;

    for (unsigned int i = r.size () - 1; i != 0; --i)
      r[i] = ((r[i-1] & 1) << 7) | (r[i] >> 1);

    r[0] = r[0] >> 1;
    return { r };
  }

  constexpr devid_addr_n operator >> (unsigned int n) const
  {
    auto r = *this;
    for (unsigned int i = 0; i < n; ++i)
      r = r.shr_1 ();

    return r;
  }

  constexpr devid_addr_n operator - (const devid_addr_n& other) const
  {
    auto r = zero_value ().bytes;
    unsigned int borrow = 0;

    for (unsigned int i = 0; i < this->bytes.size (); ++i)
    {
      unsigned int ii = this->bytes.size () - i - 1;

      unsigned int tmp = (this->bytes[ii] - other.bytes[ii]) - borrow;
      r[ii] = tmp;
      borrow = (tmp >> 8) & 1;
    }

    return { r };
  }

  constexpr devid_addr_n operator + (const devid_addr_n& other) const
  {
    auto r = zero_value ().bytes;
    unsigned int carry = 0;

    for (unsigned int i = 0; i < this->bytes.size (); ++i)
    {
      unsigned int ii = this->bytes.size () - i - 1;
      unsigned int tmp = this->bytes[ii] + other.bytes[ii] + carry;
      r[ii] = tmp;
      carry = (tmp >> 8) & 1;
    }

    return { r };
  }

  constexpr devid_addr_n& operator ++ (void)
  {
    unsigned int carry = 1;

    for (unsigned int i = 0; i < this->bytes.size (); ++i)
    {
      unsigned int ii = this->bytes.size () - i - 1;
      unsigned int tmp = this->bytes[ii] + carry;
      this->bytes[ii] = tmp;
      carry = (tmp >> 8) & 1;
    }

    return *this;
  }

  constexpr devid_addr_n& operator -- (void)
  {
    unsigned int borrow = 1;

    for (unsigned int i = 0; i < this->bytes.size (); ++i)
    {
      unsigned int ii = this->bytes.size () - i - 1;
      unsigned int tmp = this->bytes[ii] - borrow;
      this->bytes[ii] = tmp;
      borrow = (tmp >> 8) & 1;
    }

    return *this;
  }

  struct array_init_value_max_addr { static constexpr uint8_t func (size_t) { return 0xFF; } };
  struct array_init_value_min_addr { static constexpr uint8_t func (size_t) { return 0x00; } };

  static constexpr devid_addr_n max_value (void)
  {
    return { utils::make_array<uint8_t, size_bytes, array_init_value_max_addr::func> () };
  }

  static constexpr devid_addr_n zero_value (void)
  {
    return { utils::make_array<uint8_t, size_bytes, array_init_value_min_addr::func> () };
  }

  constexpr const uint8_t& operator[] (unsigned int i) const { return bytes[i]; }
  constexpr uint8_t& operator[] (unsigned int i) { return bytes[i]; }
  constexpr unsigned int size (void) const { return bytes.size (); }
  auto begin (void) const { return bytes.begin (); }
  auto end (void) const { return bytes.end (); }
  void fill (uint8_t val) { bytes.fill (val); }
};

// default address length is 128 bit = 16 bytes
using devid_addr = devid_addr_n<16>;

enum bitrate_t
{
  bitrate_57600,
  bitrate_115200,
  bitrate_172800,
  bitrate_230400,
  bitrate_250000,
  bitrate_500000,
  bitrate_750000,
  bitrate_1000000,
  bitrate_1500000,
  bitrate_3000000,
  bitrate_6000000
};

constexpr inline unsigned int bitrate_value (bitrate_t x)
{
  constexpr unsigned int table[] =
  {
    57600, 115200, 172800, 230400, 250000, 500000, 750000,
    1000000, 1500000, 3000000, 6000000
  };

  return table[x];
};

enum struct devid_message_type : uint8_t
{
  reset_link_bitrate = 1, // broadcast message, has no ack

  request_addr_range_check,
  request_addr_eq_check,
  response_ok,

  // the following devid commands are used only after assigning a local address
  request_devinfo,
  response_devinfo,
  request_flash_id_led, // uses response_ok
};

#pragma pack (1)
struct devid_reset_link_bitrate
{
  devid_message_type msg_type;
  uint32_t bitrate;
};
#pragma pack ()

#pragma pack (1)
struct devid_request_addr_range_check
{
  devid_message_type msg_type;
  devid_addr min_addr;
  devid_addr max_addr;
};
#pragma pack ()

#pragma pack (1)
struct devid_request_addr_eq_check
{
  devid_message_type msg_type;
  devid_addr addr;
};
#pragma pack ()

#pragma pack (1)
struct devid_response_ok
{
  devid_message_type msg_type;
};
#pragma pack ()

#pragma pack (1)
struct devid_request_devinfo
{
  devid_message_type msg_type;
};
#pragma pack ()


// CAUTION!
// originally the 'devid_response_devinfo' contained only a 16 byte string
// for the device name.  there are still old devices out there in the field
// that send out the old packet format.
// for that reason, the receiving side needs to actually parse the packet
// dynamically, and determine the dev_name array size based on the size of
// the total packet.
#pragma pack (1)
struct devid_response_devinfo
{
  devid_message_type msg_type;
  std::array<uint8_t, 32> dev_name; // zero terminated ascii string
  uint32_t protocol_version;
  uint32_t supported_bitrates;
};
#pragma pack ()

#pragma pack (1)
struct devid_request_flash_id_led
{
  devid_message_type msg_type;
  uint8_t enable_value;
};
#pragma pack ()

// ---------------------------------------------------------------------------

enum struct devcfg_message_type : uint8_t
{
  request_assign_addr,
  response_ok
};

#pragma pack (1)
struct devcfg_assign_addr
{
  devcfg_message_type msg_type;
  devid_addr dev_id_addr;
  uint8_t local_addr;
};
#pragma pack ()

#pragma pack (1)
struct devcfg_response_ok
{
  devcfg_message_type msg_type;
};
#pragma pack ()

// ---------------------------------------------------------------------------
// device status

enum struct devstat_message_type : uint8_t
{
  ping,
  pong
};

// no other message data


// ---------------------------------------------------------------------------
//
// devid handler delegate is expected to implement the following functions
//
// struct delegate
// {
//   std::array<uint8_t, 16> board_id (void);
//   std::string_view device_name (void);  // max. 63 chars + zero-terminator
//                                         // or std::array<uint8_t, 64>
//   uint32_t protocol_version (void);
//   void flash_id_led (bool en);
//   void reset_link_bitrate (uint32_t new_bitrate);
//   std::bitset<32> supported_bitrates (void);
// };

template <typename SLLPEndPoint, typename Delegate>
class dev_id_handler : public data_receive_listener
{
public:
  dev_id_handler (SLLPEndPoint& sllp_ep) : m_sllp_ep (sllp_ep)
  {
    m_sllp_ep.attach_data_receive_listener (this);
  }

  virtual ~dev_id_handler (void) override
  {
    m_sllp_ep.detach_data_receive_listener (this);
  }

  virtual void sllp_data_received (const void* data, unsigned int data_size) override
  {
    if (const auto* h = frame_header_00::check_frame_id_pid (data, pid::devid))
    {
      data = h + 1;
      data_size -= sizeof (*h);

      #pragma pack (1)
      struct response_msg
      {
	frame_header_00 hdr;
	devid_response_ok msg;
      };
      #pragma pack ()

      response_msg resp;
      resp.hdr = frame_header_00 (h->pid (), m_sllp_ep.address (), h->src_addr (), sizeof (resp.msg));
      resp.msg.msg_type = devid_message_type::response_ok;

      if (sllp_local_address_assigned ())
	m_last_sllp_dev_addr_update_time = std::chrono::high_resolution_clock::now ();

      switch (*(const devid_message_type*)data)
      {
	case devid_message_type::reset_link_bitrate:
	{
	  auto& msg = *(const devid_reset_link_bitrate*)((uintptr_t)data);
	  Delegate ().reset_link_bitrate (msg.bitrate & 0x00FFFFFF);

	  // if this is a bitrate change during bus address scan start, reset
	  // the local address
	  if (msg.bitrate & (1 << 31))
	  {
	    m_sllp_ep.set_address (0xFF);
	    m_last_sllp_dev_addr_update_time = { };
	  }
	  break;
	}

	case devid_message_type::request_addr_range_check:
	{
	  // if a local address has been already assigned to this device,
	  // do not respond to devid address search requests, but bump
	  // the timer to avoid address timeout.
	  if (sllp_local_address_assigned ())
	    break;

	  auto& msg = *(const devid_request_addr_range_check*)((uintptr_t)data);

	  auto my_id = Delegate ().board_id ();
	  if (my_id.size () == msg.min_addr.size ())
	  {
	    devid_addr my_id_addr (my_id);
	    if (my_id_addr < msg.max_addr && !(my_id_addr < msg.min_addr))
	      m_sllp_ep.write (&resp, sizeof (resp));
	  }
	  break;
	}
	case devid_message_type::request_addr_eq_check:
	{
	  // if a local address has been already assigned to this device,
	  // do not respond to devid address search requests, but bump
	  // the timer to avoid address timeout.
	  if (sllp_local_address_assigned ())
	    break;

	  auto& msg = *(const devid_request_addr_eq_check*)((uintptr_t)data);

	  auto my_id = Delegate ().board_id ();
	  if (my_id.size () == msg.addr.size ())
	  {
	    devid_addr my_id_addr (my_id);
	    if (my_id_addr == msg.addr)
	      m_sllp_ep.write (&resp, sizeof (resp));
	  }
	  break;
	}
	case devid_message_type::request_devinfo:
	{
	  #pragma pack (1)
	  struct
	  {
	    frame_header_00 hdr;
	    devid_response_devinfo msg;
	  } r;
	  #pragma pack ()

	  r.hdr = frame_header_00 (h->pid (), m_sllp_ep.address (), h->src_addr (), sizeof (r.msg));
	  r.msg.msg_type = devid_message_type::response_devinfo;

	  auto dev_name = Delegate ().device_name ();

	  if constexpr (std::is_same_v<std::string_view, decltype (dev_name)>)
	  {
	    r.msg.dev_name = utils::str_to_array<uint8_t, decltype (r.msg.dev_name)().size ()> (dev_name);
	  }
	  else
	  {
	    // assume that the delegate returned the correct array
	    r.msg.dev_name = dev_name;
	  }

	  r.msg.protocol_version = Delegate ().protocol_version ();
	  r.msg.supported_bitrates = Delegate ().supported_bitrates ().to_ulong ();
	  m_sllp_ep.write (&r, sizeof r);
	  break;
	}
	case devid_message_type::request_flash_id_led:
	{
	  auto& msg = *(const devid_request_flash_id_led*)((uintptr_t)data);
	  Delegate ().flash_id_led (msg.enable_value != 0);
	  m_sllp_ep.write (&resp, sizeof (resp));
	  break;
	}

	default:
	  break;
      }
    }

    else if (const auto* h = frame_header_00::check_frame_id_pid (data, pid::devcfg))
    {
      data = h + 1;
      data_size -= sizeof (*h);

      switch (*(const devcfg_message_type*)data)
      {
	case devcfg_message_type::request_assign_addr:
	{
	  auto& msg = *(const devcfg_assign_addr*)((uintptr_t)data);

	  auto my_id = Delegate ().board_id ();
	  if (my_id.size () == msg.dev_id_addr.size ()
	      && devid_addr (my_id) == msg.dev_id_addr)
	  {
	    m_sllp_ep.set_address (msg.local_addr);

	    m_last_sllp_dev_addr_update_time = std::chrono::high_resolution_clock::now ();

	    #pragma pack (1)
	    struct response_msg
	    {
	      frame_header_00 hdr;
	      devcfg_response_ok msg;
	    };
	    #pragma pack ()

	    response_msg resp;
	    resp.hdr = frame_header_00 (h->pid (), m_sllp_ep.address (), h->src_addr (), sizeof (resp.msg));
	    resp.msg.msg_type = devcfg_message_type::response_ok;
	    m_sllp_ep.write (&resp, sizeof (resp));
	  }
	  break;
	}
	default:
	  break;
      }
    }
#if 0
    else if (const auto* h = frame_header_00::check_frame_id_pid (data, pid::devstat))
    {
      data = h + 1;
      data_size -= sizeof (*h);

      // device presence status check
      // reset address timeout timer, send OK packet
      if (*(const devstat_message_type*)data == devstat_message_type::ping)
      {
	#pragma pack (1)
	struct response_msg
	{
	  frame_header_00 hdr;
	  devstat_message_type msg_type;
	};
	#pragma pack ()

	response_msg resp;
	resp.hdr = frame_header_00 (h->pid (), m_sllp_ep.address (), h->src_addr (), sizeof (resp.msg_type));
	resp.msg_type = devstat_message_type::pong;
	m_sllp_ep.write (&resp, sizeof (resp));

	m_last_sllp_dev_addr_update_time = std::chrono::high_resolution_clock::now ();
      }
    }
#endif
  };

  template <typename T>
  void exec (const T& time_now)
  {
#if 0
    if (m_last_sllp_dev_addr_update_time.has_value ()
	&& time_now - m_last_sllp_dev_addr_update_time.value () > std::chrono::seconds (5))
    {
      // sllp communication seems to have been dead for a while.
      // reset the assigned address
      m_sllp_ep.set_address (0xFF);
      m_last_sllp_dev_addr_update_time = { };
    }
#endif
  }

  bool sllp_local_address_assigned (void) const
  {
    return m_last_sllp_dev_addr_update_time.has_value ();
  }

protected:
  SLLPEndPoint& m_sllp_ep;
  std::optional<std::chrono::high_resolution_clock::time_point> m_last_sllp_dev_addr_update_time;
};


} // namespace sllp
} // namespace net
#endif // includeguard_net_sllp_includeguard

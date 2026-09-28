
#define uip_cpp_impl
#define net_impl

#include "uip_callback.hpp"
#include <net/net.hpp>

#include <memory>
#include <chrono>

extern "C"
{
#include "uip.h"
#include "uip_arp.h"
}

#include <cstdio>
#include <cstring>
#include <cassert>

//#define log_all
#include <logging/logging.hpp>

namespace
{

static std::chrono::high_resolution_clock::time_point g_last_uip_periodic_time;
static std::chrono::high_resolution_clock::time_point g_last_uip_arp_time;
static std::chrono::high_resolution_clock::time_point g_last_exec_time;

static dev::eth* g_eth_dev = nullptr;


// uip is not re-entrant.  thus we have to keep track of the current
// connection that is being processed when invoking callbacks into the
// application.
static struct uip_conn* g_cur_uip_conn = nullptr;

struct uip_appcall_scope
{
  uip_appcall_scope (struct uip_conn* c) { g_cur_uip_conn = c; }
  ~uip_appcall_scope (void) { g_cur_uip_conn = nullptr; }
};


// --------------------------------------------------------------------------

#ifndef NET_NO_TCP

// the listening port impl that is stored in the net::tcp_listening_port
// subclass data area.
struct tcp_listening_port_impl
{
  net::tcp_listening_port* prev = nullptr;
  net::tcp_listening_port* next = nullptr;
};

// FIXME: use intrusive boost / stl container somehow.
static net::tcp_listening_port* g_listening_ports_head = nullptr;

#endif // NET_NO_TCP

} // anonymous namespace


#ifndef NET_NO_TCP

net::tcp_listening_port::tcp_listening_port (void)
{
  static_assert (sizeof (tcp_listening_port_impl) <= sizeof (m_subclass_data), "");
  new (m_subclass_data) tcp_listening_port_impl ();
}

[[gnu::cold]]
net::tcp_listening_port::tcp_listening_port (uint16_t p, const new_conn_func& clb)
: m_port (p), m_new_conn_clb (clb)
{
  static_assert (sizeof (tcp_listening_port_impl) <= sizeof (m_subclass_data), "");

  new (m_subclass_data) tcp_listening_port_impl ();

  if (m_port == 0)
    return;

  for (auto* pp = g_listening_ports_head; pp != nullptr;
       pp = impl<tcp_listening_port_impl> ().next)
  {
    // there is already a listening port for that port number..
    // assert ("tcp_listening_port already exists" && 0);
    if (pp->local_port () == p)
    {
      m_port = 0;
      return;
    }
  }


  // linked list push_front
  impl<tcp_listening_port_impl> ().prev = nullptr;
  impl<tcp_listening_port_impl> ().next = g_listening_ports_head;

  if (g_listening_ports_head != nullptr)
    g_listening_ports_head->impl<tcp_listening_port_impl> ().prev = this;

  g_listening_ports_head = this;

  uip_listen (htons (p));
}

[[gnu::cold]]
net::tcp_listening_port::tcp_listening_port (tcp_listening_port&& rhs)
: m_port (std::move (rhs.m_port)), m_new_conn_clb (std::move (rhs.m_new_conn_clb))
{
  new (m_subclass_data) tcp_listening_port_impl (
			std::move (rhs.impl<tcp_listening_port_impl> ()));

  rhs.m_port = 0;
  rhs.m_new_conn_clb = nullptr;

  auto& rhs_impl = rhs.impl<tcp_listening_port_impl> ();
  auto& this_impl = impl<tcp_listening_port_impl> ();

  if (m_port == 0)
  {
    this_impl.next = nullptr;
    this_impl.prev = nullptr;
    return;
  }

  // replace the 'rhs' element in the list with 'this'.
  if (g_listening_ports_head == &rhs)
    g_listening_ports_head = this;

  this_impl.prev = rhs_impl.prev;
  this_impl.next = rhs_impl.next;

  if (this_impl.prev != nullptr)
    rhs_impl.prev->impl<tcp_listening_port_impl> ().next = this;

  if (this_impl.next != nullptr)
    rhs_impl.next->impl<tcp_listening_port_impl> ().prev = this;

  rhs_impl.next = nullptr;
  rhs_impl.prev = nullptr;
}


[[gnu::cold]]
net::tcp_listening_port& net::tcp_listening_port::operator = (tcp_listening_port&& rhs)
{
  this->~tcp_listening_port ();
  new (this) tcp_listening_port (std::move (rhs));
  return *this;
}


[[gnu::cold]]
net::tcp_listening_port::~tcp_listening_port (void)
{
  if (m_port == 0)
    return;

  uip_unlisten (htons ((uint16_t)m_port));

  auto& this_impl = impl<tcp_listening_port_impl> ();

  // remove this from the list
  if (g_listening_ports_head == this)
    g_listening_ports_head = this_impl.next;

  if (this_impl.next != nullptr)
    this_impl.next->impl<tcp_listening_port_impl> ().prev = this_impl.prev;

  if (this_impl.prev != nullptr)
    this_impl.prev->impl<tcp_listening_port_impl> ().next = this_impl.next;
}

#endif // #ifndef NET_NO_TCP

// --------------------------------------------------------------------------

#ifndef NET_NO_TCP

namespace
{

struct tcp_socket_buffer
{
  static constexpr unsigned int send_buffer_size = 1024*2 + 512;

  // number of pending bytes in the send buffer.
  // current send buffer read & write position.
  uint16_t m_txbuf_pending = 0;
  uint16_t m_txbuf_rd_pos = 0;
  uint16_t m_txbuf_wr_pos = 0;

  // the number of bytes that was transmitted the last time using uip_send.
  // we always transmit from the m_txbuf_rd_pos in the buffer.
  uint16_t m_last_tx_count = 0;

  std::array<uint8_t, send_buffer_size> m_txbuf;
};

struct tcp_socket_impl
{
  struct uip_conn* m_conn = nullptr;
  std::unique_ptr<tcp_socket_buffer> m_buffer;
};

} // anonymous namespace

[[gnu::cold]]
net::tcp_socket::tcp_socket (void)
{
  static_assert (sizeof (tcp_socket_impl) <= sizeof (m_subclass_data), "");
  new (m_subclass_data) tcp_socket_impl ();
}

[[gnu::cold]]
net::tcp_socket::~tcp_socket (void)
{
  close ();

  auto& this_impl = impl<tcp_socket_impl> ();

  if (this_impl.m_conn != nullptr)
  {
    this_impl.m_conn->appstate.socket = nullptr;
    this_impl.m_conn = nullptr;
  }

  this_impl.~tcp_socket_impl ();
}

uint16_t net::tcp_socket::local_port (void) const
{
  auto& this_impl = impl<tcp_socket_impl> ();
  return this_impl.m_conn != nullptr ? ntohs (this_impl.m_conn->lport) : 0;
}

uint16_t net::tcp_socket::remote_port (void) const
{
  auto& this_impl = impl<tcp_socket_impl> ();
  return this_impl.m_conn != nullptr ? ntohs (this_impl.m_conn->rport) : 0;
}

[[gnu::cold]] std::array<uint8_t, 4>
net::tcp_socket::local_addr (void) const
{
  uip_ipaddr_t addr = uip_gethostaddr ();

  std::array<uint8_t, 4> r;
  std::memcpy (r.data (), &addr, r.size ());
  return r;
}

[[gnu::cold]] std::array<uint8_t, 4>
net::tcp_socket::remote_addr (void) const
{
  std::array<uint8_t, 4> r;

  auto& this_impl = impl<tcp_socket_impl> ();
  if (this_impl.m_conn != nullptr)
    std::memcpy (r.data (), &(this_impl.m_conn->ripaddr), r.size ());
  else
    std::memset (r.data (), 0, r.size ());

  return r;
}

[[gnu::cold]] bool
net::tcp_socket::keep_alive_enabled (void) const
{
  auto& this_impl = impl<tcp_socket_impl> ();
  return this_impl.m_conn->keep_alive_enable != 0;
}

[[gnu::cold]] void
net::tcp_socket::set_keep_alive_enabled (bool val)
{
  auto& this_impl = impl<tcp_socket_impl> ();
  this_impl.m_conn->keep_alive_enable = val;
}

[[gnu::cold]] void
net::tcp_socket::close (void)
{
  if (m_state == closed || m_state == closing)
    return;

  auto& this_impl = impl<tcp_socket_impl> ();
  assert (this_impl.m_conn != nullptr);

  m_state = closing;
  uip_close (this_impl.m_conn);
}

[[gnu::cold]] void
net::tcp_socket::connect (const uint8_t addr[4], uint16_t port)
{
  auto& this_impl = impl<tcp_socket_impl> ();

  if (m_state != closed)
  {
    log_error ("tcp_socket::connect state != closed.  ignoring.\n");
    return;
  }

  assert (this_impl.m_conn == nullptr);

  m_state = connecting;

  auto ipaddr = uip_ipaddr (addr[0], addr[1], addr[2], addr[3]);

  this_impl.m_conn = uip_connect (ipaddr, htons (port));
  struct uip_connection_state* s = (uip_connection_state*)&(this_impl.m_conn->appstate);
  s->socket = this;

  if (this_impl.m_buffer != nullptr)
  {
    // reset tx buffer
    this_impl.m_buffer->m_txbuf_pending = 0;
    this_impl.m_buffer->m_txbuf_rd_pos = 0;
    this_impl.m_buffer->m_txbuf_wr_pos = 0;
    this_impl.m_buffer->m_last_tx_count = 0;
  }
}


unsigned int net::tcp_socket::send (const void* data, unsigned int len,
				    send_flags flags)
{
  auto& this_impl = impl<tcp_socket_impl> ();

  if (m_state != connected)
    return 0;

  if (len == 0)
    return 0;

  // the number of bytes we can copy into our ring buffer.
  const unsigned int send_count = std::min (len, send_max_count ());

  if (this_impl.m_buffer == nullptr)
  {
    this_impl.m_buffer = std::unique_ptr<tcp_socket_buffer> (new (std::nothrow) tcp_socket_buffer ());

    // can't allocate new tx buffer (out of memory?)...
    if (this_impl.m_buffer == nullptr)
      return 0;
  }

  {
    // copy bytes into the ring buffer.
    const uint8_t* dptr = (const uint8_t*)data;
    
    unsigned int c = std::min ((unsigned int)this_impl.m_buffer->m_txbuf.size () - this_impl.m_buffer->m_txbuf_wr_pos, send_count);
    std::memcpy (this_impl.m_buffer->m_txbuf.data () + this_impl.m_buffer->m_txbuf_wr_pos, dptr, c);

    this_impl.m_buffer->m_txbuf_wr_pos += c;
    dptr += c;

    if (this_impl.m_buffer->m_txbuf_wr_pos == this_impl.m_buffer->m_txbuf.size ())
      this_impl.m_buffer->m_txbuf_wr_pos = 0;

    c = send_count - c;

    if (c > 0)
    {
      std::memcpy (this_impl.m_buffer->m_txbuf.data () + this_impl.m_buffer->m_txbuf_wr_pos, dptr, c);
      this_impl.m_buffer->m_txbuf_wr_pos += c;
    }
  }

  this_impl.m_buffer->m_txbuf_pending += send_count;

  // if we are inside of the uip "appcall" thing, we can't change the global
  // variable uip_conn.  uip_poll_conn changes that variable, so don't call
  // it in this case.
  if (((unsigned int)flags & (unsigned int)write_immediately) != 0
      || send_buffer_pending_count () >= (uint16_t)uip_mss (this_impl.m_conn))
    flush_send_buffer ();

  return send_count;
}

void net::tcp_socket::flush_send_buffer (void)
{
  auto& this_impl = impl<tcp_socket_impl> ();

  if (g_cur_uip_conn == nullptr && this_impl.m_buffer != nullptr)
  {
    if (auto send_len = uip_poll_conn (this_impl.m_conn))
    {
      send_len = uip_arp_out (send_len);

      if (g_eth_dev != nullptr)
	g_eth_dev->write (uip_buf, send_len);
    }
  }
}

void net::tcp_socket::suspend_recv (void)
{
  auto& this_impl = impl<tcp_socket_impl> ();

  if (this_impl.m_conn == nullptr)
    return;

  this_impl.m_conn->tcpstateflags |= UIP_STOPPED;
}

void net::tcp_socket::resume_recv (void)
{
  auto& this_impl = impl<tcp_socket_impl> ();

  if (this_impl.m_conn == nullptr)
    return;

  this_impl.m_conn->tcpstateflags &= ~UIP_STOPPED;
}

bool net::tcp_socket::recv_suspended (void) const
{
  auto& this_impl = impl<tcp_socket_impl> ();

  if (this_impl.m_conn == nullptr)
    return true;

  return (this_impl.m_conn->tcpstateflags & UIP_STOPPED) != 0;
}


// returns the total number of bytes the send buffer can hold.
unsigned int net::tcp_socket::send_buffer_capacity (void) const
{
  return tcp_socket_buffer::send_buffer_size;
}

unsigned int net::tcp_socket::send_buffer_pending_count (void) const
{
  auto& this_impl = impl<tcp_socket_impl> ();

  return this_impl.m_buffer != nullptr ? this_impl.m_buffer->m_txbuf_pending : 0;
}



namespace
{

static unsigned int send_data (net::tcp_socket& s, bool rexmit, void* uip_sappdata)
{
  auto& this_impl = s.impl<tcp_socket_impl> ();

  if (this_impl.m_buffer == nullptr)
    return 0;

  if (this_impl.m_buffer->m_txbuf_pending == 0)
    return 0;

  if (this_impl.m_buffer->m_last_tx_count > 0 && !rexmit)
    return 0;

  log_trace ("send data %u\n", this_impl.m_buffer->m_txbuf_pending);

  // the total number of bytes we can send from the ring buffer.
  const uint16_t mss = uip_mss (this_impl.m_conn);

  unsigned int count0 = std::min (mss,
				  rexmit ? this_impl.m_buffer->m_last_tx_count
					 : this_impl.m_buffer->m_txbuf_pending);

  // the number of bytes we can send before the buffer wraps around.
  unsigned int count1 = std::min (count0,
				  (unsigned int)this_impl.m_buffer->m_txbuf.size ()
				  - this_impl.m_buffer->m_txbuf_rd_pos);

  // the number of bytes we can send after the buffer wraps around.
  unsigned int count2 = count0 - count1;

//  log_info ("send_data socket 0x%08x count0 %u count1 %u count2 %u\n", this, count0, count1, count2);

  uint8_t* outptr = (uint8_t*)uip_sappdata;

  assert (count1 <= mss);
  assert (count2 <= mss);

  if (count1 > 0)
    std::memcpy (outptr, this_impl.m_buffer->m_txbuf.data ()
			 + this_impl.m_buffer->m_txbuf_rd_pos, count1);

  if (count2 > 0)
    std::memcpy (outptr + count1, this_impl.m_buffer->m_txbuf.data (), count2);

  this_impl.m_buffer->m_last_tx_count = count0;
  return count0;
}


static void ack_sent_data (net::tcp_socket& s)
{
//  log_info ("ack_sent_data %08p m_last_tx_count = %u  m_txbuf_pending = %u\n",
//	   m_last_tx_count, m_txbuf_pending, this);

  auto& this_impl = s.impl<tcp_socket_impl> ();

  if (this_impl.m_buffer == nullptr)
    return;

  this_impl.m_buffer->m_txbuf_rd_pos += this_impl.m_buffer->m_last_tx_count;
  if (this_impl.m_buffer->m_txbuf_rd_pos >= this_impl.m_buffer->m_txbuf.size ())
    this_impl.m_buffer->m_txbuf_rd_pos -= this_impl.m_buffer->m_txbuf.size ();

  this_impl.m_buffer->m_txbuf_pending -= this_impl.m_buffer->m_last_tx_count;
  this_impl.m_buffer->m_last_tx_count = 0;

  // assert ((int)m_txbuf_pending >= 0);
}

static void recv_data (net::tcp_socket& s, const char* data, unsigned int len)
{
  s.invoke_recv_clb (data, len);
}

// --------------------------------------------------------------------------

[[gnu::cold]] static void
on_connected (uip_connection_state* s, int port, struct uip_conn* uip_conn)
{
  auto* socket = (net::tcp_socket*)s->socket;

  if (socket != nullptr && socket->state () == net::tcp_socket::connecting)
  {
    log_info ("new outgoing connection %p\n", s->socket);
    socket->m_state = net::tcp_socket::connected;
    socket->invoke_connected_clb ();
    return;
  }

  net::tcp_listening_port* lp = g_listening_ports_head;
  for (; lp != nullptr; lp = lp->impl<tcp_listening_port_impl> ().next)
    if (lp->local_port () == port)
      break;

  if (lp != nullptr)
  {
    socket = new (std::nothrow) net::tcp_socket ();
    if (socket != nullptr)
    {
      log_info ("new incoming connection %p\n", socket);

      socket->m_state = net::tcp_socket::connected;
      socket->impl<tcp_socket_impl> ().m_conn = uip_conn;
      s->socket = socket;

      // if the application does not move the unique_ptr somewhere else
      // or anything like that, the new socket will be deleted after this
      // function call.
      lp->invoke_new_connection_clb (std::unique_ptr<net::tcp_socket> (socket));
    }
    else
    {
      s->socket = nullptr;

      log_warn ("new incoming connection - could not create new tcp_socket\n");
      uip_close (uip_conn);
    }
  }
}

} // anonymous namespace

extern "C" unsigned int uip_tcp_retransmit (struct uip_conn* uip_conn,
					    void* uip_appdata)
{
  struct uip_connection_state* s = &(uip_conn->appstate);
  net::tcp_socket* socket = (net::tcp_socket*)s->socket;

  // when here the connection is always valid.
  // log_warn ("uip_rexmit %p\n", socket);

  return send_data (*socket, true, uip_appdata);
}

static void close_tcp_connection (struct uip_connection_state* s,
				  net::tcp_socket* socket, struct uip_conn* c)
{
  uip_appcall_scope scope (c);

  s->socket = nullptr;
  if (socket != nullptr)
  {
    auto& si = socket->impl<tcp_socket_impl> ();
    si.m_conn = nullptr;
    socket->m_state = net::tcp_socket::closed;
    socket->invoke_close_clb ();
  }
}

extern "C" void uip_tcp_abort (struct uip_conn* c)
{
  struct uip_connection_state* s = &(c->appstate);
  net::tcp_socket* socket = (net::tcp_socket*)s->socket;

  log_info ("uip aborted %p\n", socket);
  uip_close (c);
  close_tcp_connection (s, socket, c);
}

extern "C" void uip_tcp_close (struct uip_conn* c)
{
  struct uip_connection_state* s = &(c->appstate);
  net::tcp_socket* socket = (net::tcp_socket*)s->socket;

  log_info ("uip closed %p  %p\n", s, socket);
  close_tcp_connection (s, socket, c);
}

extern "C" void uip_tcp_timed_out (struct uip_conn* c)
{
  struct uip_connection_state* s = &(c->appstate);
  net::tcp_socket* socket = (net::tcp_socket*)s->socket;

  log_info ("uip timeout %p  %p\n", s, socket);
  uip_close (c);
  close_tcp_connection (s, socket, c);
}

extern "C" void uip_tcp_connected (struct uip_conn* c)
{
  uip_appcall_scope scope (c);

  struct uip_connection_state* s = &(c->appstate);
  on_connected (s, ntohs (c->lport), c);
}

extern "C" void uip_tcp_ackdata (struct uip_conn* c)
{
  struct uip_connection_state* s = &(c->appstate);
  if (net::tcp_socket* socket = (net::tcp_socket*)s->socket)
    ack_sent_data (*socket);
}

extern "C" void
uip_tcp_newdata (struct uip_conn* c, void* uip_appdata, unsigned int uip_len)
{
  uip_appcall_scope scope (c);

  struct uip_connection_state* s = &(c->appstate);
  if (net::tcp_socket* socket = (net::tcp_socket*)s->socket)
    recv_data (*socket, (const char*)uip_appdata, uip_len);
}

extern "C" unsigned int
uip_tcp_senddata (struct uip_conn* c, void* uip_appdata)
{
  uip_appcall_scope scope (c);

  struct uip_connection_state* s = &(c->appstate);
  if (net::tcp_socket* socket = (net::tcp_socket*)s->socket)
    return send_data (*socket, false, uip_appdata);
  else
    return 0;
}

extern "C" void uip_tcp_no_more_free_connections (void)
{
  for (uip_conn& c : uip_conns)
    if (c.tcpstateflags == UIP_ESTABLISHED)
    {
      if (net::tcp_socket* socket = (net::tcp_socket*)c.appstate.socket)
      {
	// close the socket (on the next exec iteration).
	// after the socket is closed, the application will receive a
	// notification via a callback and normally should delete the socket.
	if (socket->auto_close_enabled ())
	  socket->close ();
      }
    }
}

#endif // NET_NO_TCP

// --------------------------------------------------------------------------

namespace
{

struct udp_socket_impl
{
  struct uip_udp_conn* conn = nullptr;
  const void* pending_send_data = nullptr;
  unsigned int pending_send_data_len = 0;
};

} // anonymous namespace


[[gnu::cold]]
net::udp_socket::udp_socket (void)
{
  static_assert (sizeof (udp_socket_impl) <= sizeof (m_subclass_data), "");
  new (m_subclass_data) udp_socket_impl ();
}

[[gnu::cold]]
net::udp_socket::udp_socket (udp_socket&& rhs)
: m_recv_clb (std::move (rhs.m_recv_clb))
{
  auto& rhs_impl = rhs.impl<udp_socket_impl> ();

  new (m_subclass_data) udp_socket_impl (std::move (rhs_impl));
  rhs_impl.conn = nullptr;

  auto& this_impl = impl<udp_socket_impl> ();
  if (this_impl.conn != nullptr)
    this_impl.conn->appstate.socket = this;
}

[[gnu::cold]] net::udp_socket&
net::udp_socket::operator = (udp_socket&& rhs)
{
  this->~udp_socket ();
  new (this) udp_socket (std::move (rhs));
  return *this;
}

[[gnu::cold]]
net::udp_socket::~udp_socket (void)
{
  auto& this_impl = impl<udp_socket_impl> ();

  if (this_impl.conn != nullptr)
  {
    uip_udp_remove (this_impl.conn);
    this_impl.conn->appstate.socket = nullptr;
  }
}

[[gnu::cold]]
net::udp_socket::udp_socket (uint16_t local_port)
{
  new (m_subclass_data) udp_socket_impl ();
  bind (local_port);
}

[[gnu::cold]] uint16_t
net::udp_socket::local_port (void) const
{
  auto& this_impl = impl<udp_socket_impl> ();
  return this_impl.conn != nullptr ? ntohs (this_impl.conn->lport) : 0;
}

[[gnu::cold]] uint16_t
net::udp_socket::remote_port (void) const
{
  auto& this_impl = impl<udp_socket_impl> ();
  return this_impl.conn != nullptr ? ntohs (this_impl.conn->rport) : 0;
}

[[gnu::cold]] std::array<uint8_t, 4>
net::udp_socket::local_addr (void) const
{
  auto addr = uip_gethostaddr ();

  std::array<uint8_t, 4> r;
  std::memcpy (r.data (), &addr, r.size ());
  return r;
}

[[gnu::cold]] std::array<uint8_t, 4>
net::udp_socket::remote_addr (void) const
{
  std::array<uint8_t, 4> r;

  auto& this_impl = impl<udp_socket_impl> ();
  if (this_impl.conn != nullptr)
    std::memcpy (r.data (), &this_impl.conn->ripaddr, r.size ());
  else
    std::memset (r.data (), 0, r.size ());

  return r;
}

[[gnu::cold]] void
net::udp_socket::close (void)
{
  auto& this_impl = impl<udp_socket_impl> ();

  if (this_impl.conn != nullptr)
  {
    uip_udp_remove (this_impl.conn);
    this_impl.conn = nullptr;
  }
}

[[gnu::cold]] bool
net::udp_socket::bind (uint16_t local_port)
{
  auto& this_impl = impl<udp_socket_impl> ();

  if (this_impl.conn == nullptr)
    this_impl.conn = uip_udp_new (0, 0);

  if (this_impl.conn != nullptr)
  {
    uip_udp_bind (this_impl.conn, htons (local_port));
    this_impl.conn->appstate.socket = this;
  }

  log_info ("udp_socket::bind conn = %p\n", this_impl.conn);

  return this_impl.conn != nullptr;
}

[[gnu::cold]] void
net::udp_socket::connect (const uint8_t remote_addr[4], uint16_t remote_port)
{
  auto& this_impl = impl<udp_socket_impl> ();

  // if we already have a connection, preserve the local port binding.
  uint16_t prev_local_port = local_port ();

  close ();

  auto addr = uip_ipaddr (remote_addr[0], remote_addr[1], remote_addr[2], remote_addr[3]);
  this_impl.conn = uip_udp_new (addr, htons (remote_port));

  if (this_impl.conn != nullptr)
  {
    this_impl.conn->appstate.socket = this;

    if (prev_local_port != 0)
      uip_udp_bind (this_impl.conn, htons (prev_local_port));
  }
}

unsigned int
net::udp_socket::send (const void* data, unsigned int len)
{
  auto& this_impl = impl<udp_socket_impl> ();

  if (this_impl.conn == nullptr)
    return 0;

  len = std::min (len, (unsigned int)UIP_APPDATA_SIZE_UDP);

  assert (len <= UIP_APPDATA_SIZE_UDP);
  std::memcpy (&uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN], data, len);

  if (auto send_len = uip_process (UIP_UDP_SEND_CONN, nullptr, this_impl.conn, len))
  {
    send_len = uip_arp_out (send_len);
    if (g_eth_dev != nullptr)
      g_eth_dev->write (uip_buf, send_len);
  }

  return len;
}


extern "C" void uip_appcall_udp (int flags, struct uip_udp_conn* uip_udp_conn,
				 struct uip_udpip_hdr* hdr,
				 unsigned int uip_len)
{
  log_trace ("uip_appcall_udp\n");

  struct uip_connection_state_udp* s = &(uip_udp_conn->appstate);
 // const int port = ntohs (uip_udp_conn->lport);

  auto* socket = (net::udp_socket*)s->socket;

  if (socket == nullptr)
    return;

  //auto& socket_impl = socket->impl<udp_socket_impl> ();

  if (flags & UIP_NEWDATA)
  {
    uint8_t remote_addr[4];

    std::memcpy (remote_addr, &(hdr->srcipaddr), sizeof (remote_addr));
    uint16_t remote_port = ntohs (hdr->srcport);

    socket->invoke_recv_clb ((const char*)hdr + sizeof (uip_udpip_hdr),
			     uip_len, remote_addr, remote_port);
  }
}


// --------------------------------------------------------------------------

[[gnu::cold]] void
net::init (dev::eth& eth_dev,
	   const unsigned char device_ipaddr[4],
	   const unsigned char gateway_ipaddr[4],
	   const unsigned char subnetmask[4])
{
  g_eth_dev = &eth_dev;

  // initialize & configure uIP
  // assume that the ethernet device has been initialized and has its mac
  // address set.
  auto mac_addr = eth_dev.mac_address ();

  log_info ("uip_init\n");
  log_info ("    MAC %02x:%02x:%02x:%02x:%02x:%02x\n", mac_addr[0],
	    mac_addr[1], mac_addr[2], mac_addr[3],
	    mac_addr[4], mac_addr[5]);
  log_info ("    device addr  %d.%d.%d.%d\n", device_ipaddr[0], device_ipaddr[1],
	    device_ipaddr[2], device_ipaddr[3]);
  log_info ("    gateway addr %d.%d.%d.%d\n", gateway_ipaddr[0], gateway_ipaddr[1],
	    gateway_ipaddr[2], gateway_ipaddr[3]);
  log_info ("    subnet mask  %d.%d.%d.%d\n", subnetmask[0], subnetmask[1],
	    subnetmask[2], subnetmask[3]);

  uip_init ();
  uip_arp_init ();

  uip_setethaddr ( (*(const uip_eth_addr*)&mac_addr) );

  uip_sethostaddr (uip_ipaddr (device_ipaddr[0], device_ipaddr[1],
			       device_ipaddr[2], device_ipaddr[3]));

  uip_setdraddr (uip_ipaddr (gateway_ipaddr[0], gateway_ipaddr[1],
			     gateway_ipaddr[2], gateway_ipaddr[3]));

  uip_setnetmask (uip_ipaddr (subnetmask[0], subnetmask[1],
			      subnetmask[2], subnetmask[3]));

  g_last_uip_periodic_time = g_last_uip_arp_time = g_last_exec_time =
				std::chrono::high_resolution_clock::now ();
}

void net::exec (void)
{
  if (g_eth_dev == nullptr)
    return;

  g_eth_dev->check_link_status ();

  auto cur_time = std::chrono::high_resolution_clock::now ();

  // try to drain the receive queue, but only up until some limited number
  // of packets.  otherwise the whole thing might be stuck in the receive
  // loop forever if there is a continuous stream of incoming packets.

  auto* buf = (struct uip_eth_hdr*)&uip_buf[0];

  for (int i = 0; i < 4; ++i)
  {
    unsigned int uip_len = g_eth_dev->read (buf, UIP_BUFSIZE);

    if (uip_len == 0)
      break;

    if (buf->type == htons (UIP_ETHTYPE_IP))
    {
      uip_len = uip_arp_ipin (uip_len);

      if(auto send_len = uip_input (uip_len))
      {
	send_len = uip_arp_out (send_len);
	g_eth_dev->write (buf, send_len);
      }
    }
    else if (buf->type == htons (UIP_ETHTYPE_ARP))
    {
      if (auto send_len = uip_arp_arpin (uip_len))
	g_eth_dev->write (buf, send_len);
    }
  }

  // 100 ms interval periodic timer.  this influences stuff like TCP timeouts.
  // notice that we don't poll udp connections, because they send their
  // data directly.
  #ifndef NET_NO_TCP
  if (cur_time - g_last_uip_periodic_time > std::chrono::milliseconds (100))
  {
    g_last_uip_periodic_time = cur_time;

    for (unsigned int i = 0; i < UIP_CONNS; ++i)
    {
      if (auto send_len = uip_periodic_conn (&uip_conns[i]))
      {
	send_len = uip_arp_out (send_len);
	g_eth_dev->write (buf, send_len);
      }
    }
  }
  #endif // NET_NO_TCP

  // 10 sec ARP timer.
  if (cur_time - g_last_uip_arp_time > std::chrono::seconds (10))
  {
    g_last_uip_arp_time = cur_time;
    uip_arp_timer();
  }

  g_last_exec_time = cur_time;
}

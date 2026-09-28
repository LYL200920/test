#define net_impl

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/param.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#include <cassert>
#include <vector>
#include <array>
#include <algorithm>
#include <cstring>
#include <net/net.hpp>

#include <iostream>

struct socket_impl
{
  int socket_handle = -1;
};

template <typename T> struct socket_set
{
  static constexpr size_t max_count = FD_SETSIZE;

  mutable std::vector<T> obj;
  mutable bool obj_dirty = false;
  fd_set fdset;

  static int socket_handle (T from_obj)
  {
    return from_obj->template impl<socket_impl> ().socket_handle;
  }

  socket_set (void)
  {
    FD_ZERO (&fdset);
  }

  void add (T o)
  {
    assert (obj.size () < max_count);
    obj.push_back (o);
    obj_dirty = true;

    int h = socket_handle (o);
    if (h >= 0)
      FD_SET (h, &fdset);
    // std::cout << "add " << o << " " << h << std::endl;
  }

  void remove (T o)
  {
    int h = socket_handle (o);
    if (h >= 0)
      FD_CLR (h, &fdset);

    // std::cout << "remove " << o << " " << h << std::endl;

    auto ii = std::find (std::begin (obj), std::end (obj), o);
    if (ii != std::end (obj))
    {
      std::iter_swap (ii, std::end (obj) - 1);
      obj.pop_back ();
      obj_dirty = true;
    }
  }

  void update_handle (T o, int old_handle)
  {
    int new_handle = socket_handle (o);
    assert (old_handle < FD_SETSIZE);
    assert (new_handle < FD_SETSIZE);

    if (old_handle >= 0)
      FD_CLR (old_handle, &fdset);
    if (new_handle >= 0)
      FD_SET (new_handle, &fdset);

    // std::cout << "update " << o << " " << old_handle << " -> " << new_handle << std::endl;

    obj_dirty = true;
  }

  T obj_for_handle (int h) const
  {
    if (obj_dirty)
    {
      std::sort (std::begin (obj), std::end (obj),
		[] (T a, T b)
		{
		  return socket_handle (a) < socket_handle (b);
		});
      obj_dirty = false;
    }

    auto i = std::lower_bound (std::begin (obj), std::end (obj), h,
				[] (T a, int b)
				{
				  return socket_handle (a) < b;
				});

    if (i != std::end (obj) && socket_handle (*i) == h)
      return *i;

    assert (false && "obj_for_handle NG");
    return nullptr;
  }
};

// --------------------------------------------------------------------------

static std::unique_ptr<socket_set<net::tcp_listening_port*>> g_listening_ports;

net::tcp_listening_port::tcp_listening_port (void)
{
  static_assert (sizeof (socket_impl) <= sizeof (m_subclass_data), "");
  new (m_subclass_data) socket_impl ();

  if (g_listening_ports == nullptr)
    g_listening_ports = std::make_unique<socket_set<net::tcp_listening_port*>> ();

  g_listening_ports->add (this);
}

net::tcp_listening_port::tcp_listening_port (uint16_t port, const new_conn_func& clb)
: m_port (port), m_new_conn_clb (clb)
{
  static_assert (sizeof (socket_impl) <= sizeof (m_subclass_data), "");
  auto& impl = *new (m_subclass_data) socket_impl ();

  if (m_port == 0)
    return;

  // create socket
  int sock = impl.socket_handle = socket (PF_INET, SOCK_STREAM, 0);
  if (sock < 0)
    assert (false && "socket NG");

  // if the program is existed and restarted within a short time interval,
  // it might not be able to bind.
  int reuse = 1;
  if (setsockopt (sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof (reuse)) < 0)
    assert (false && "setsockopt (SO_REUSEADDR) NG");

  #ifdef SO_REUSEPORT
  if (setsockopt (sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof (reuse)) < 0)
    assert (false && "setsockopt (SO_REUSEPORT) NG");
  #endif

  // bind socket to an address and port.
  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons (port);
  addr.sin_addr.s_addr = htonl (INADDR_ANY);

  if (bind (sock, (struct sockaddr*)&addr, sizeof (addr)) < 0)
    assert (false && "bind NG");

  // start listening on the socket
  if (listen (sock, 1) < 0)
    assert (false && "listen NG");

  // remember the listening port object in the global list for periodic polling.
  if (g_listening_ports == nullptr)
    g_listening_ports = std::make_unique<socket_set<net::tcp_listening_port*>> ();

  g_listening_ports->add (this);
}

net::tcp_listening_port::~tcp_listening_port (void)
{
  auto& i = impl<socket_impl> ();

  if (i.socket_handle >= 0)
    close (i.socket_handle);

  if (g_listening_ports != nullptr)
    g_listening_ports->remove (this);
}

net::tcp_listening_port::tcp_listening_port (tcp_listening_port&& rhs)
: m_port (std::move (rhs.m_port)), m_new_conn_clb (std::move (rhs.m_new_conn_clb))
{
  auto& this_impl = *new (m_subclass_data) socket_impl ();
  auto& rhs_impl = rhs.impl<socket_impl> ();

  this_impl.socket_handle = rhs_impl.socket_handle;

  rhs.m_port = 0;
  rhs.m_new_conn_clb = nullptr;
  rhs_impl.socket_handle = -1;

  if (g_listening_ports == nullptr)
    g_listening_ports = std::make_unique<socket_set<net::tcp_listening_port*>> ();

  g_listening_ports->add (this);
}

net::tcp_listening_port& net::tcp_listening_port::operator = (tcp_listening_port&& rhs)
{
  if (this == &rhs)
    return *this;

  auto& i = impl<socket_impl> ();

  const auto old_handle = i.socket_handle;

  if (old_handle >= 0)
    close (old_handle);

  auto& rhs_impl = rhs.impl<socket_impl> ();
  i.socket_handle = std::move (rhs_impl.socket_handle);
  rhs_impl.socket_handle = -1;

  m_port = std::move (rhs.m_port);
  m_new_conn_clb = std::move (rhs.m_new_conn_clb);

  assert (g_listening_ports != nullptr);
  g_listening_ports->update_handle (this, old_handle);
  return *this;
}

// --------------------------------------------------------------------------

static std::unique_ptr<socket_set<net::tcp_socket*>> g_tcp_sockets = nullptr;

net::tcp_socket::tcp_socket (void)
{
  static_assert (sizeof (socket_impl) <= sizeof (m_subclass_data), "");
  auto& impl = *new (m_subclass_data) socket_impl ();

  // create socket, even if not yet connecting.
  // this is to allow getting/setting socket options before connecting.
  int sock = impl.socket_handle = socket (PF_INET, SOCK_STREAM, 0);
  if (sock < 0)
    assert (false && "socket NG");

  // remember the socket object for periodic polling
  if (g_tcp_sockets == nullptr)
    g_tcp_sockets = std::make_unique<socket_set<net::tcp_socket*>> ();

  g_tcp_sockets->add (this);

  // state remains "closed" until connect is invoked.
}

net::tcp_socket::~tcp_socket (void)
{
  // if it has been connected, close that first and invoke the
  // close callback.
  close ();

  // if it was only created but never connected close () won't do anything.
  // need to delete the socket object via ::close
  auto& i = impl<socket_impl> ();
  if (i.socket_handle >= 0)
    ::close (i.socket_handle);

  if (g_tcp_sockets != nullptr)
    g_tcp_sockets->remove (this);
}


uint16_t net::tcp_socket::local_port (void) const
{
  struct sockaddr_in addr;
  socklen_t addr_size = sizeof (addr);

  getsockname (impl<socket_impl> ().socket_handle, (struct sockaddr*)&addr, &addr_size);
  return ntohs (addr.sin_port);
}

std::array<uint8_t, 4> net::tcp_socket::local_addr (void) const
{
  struct sockaddr_in addr;
  socklen_t addr_size = sizeof (addr);

  getsockname (impl<socket_impl> ().socket_handle, (struct sockaddr*)&addr, &addr_size);

  std::array<uint8_t, 4> r;
  std::memcpy ((char*)r.data (), &addr.sin_addr.s_addr, 4);
  return r;
}

uint16_t net::tcp_socket::remote_port (void) const
{
  struct sockaddr_in addr;
  socklen_t addr_size = sizeof (addr);

  getpeername (impl<socket_impl> ().socket_handle, (struct sockaddr*)&addr, &addr_size);
  return ntohs (addr.sin_port);
}

std::array<uint8_t, 4> net::tcp_socket::remote_addr (void) const
{
  struct sockaddr_in addr;
  socklen_t addr_size = sizeof (addr);

  getpeername (impl<socket_impl> ().socket_handle, (struct sockaddr*)&addr, &addr_size);

  std::array<uint8_t, 4> r;
  std::memcpy ((char*)r.data (), &addr.sin_addr.s_addr, 4);
  return r;
}

bool net::tcp_socket::keep_alive_enabled (void) const
{
  int optval;
  socklen_t optlen = sizeof (optval);
  getsockopt (impl<socket_impl> ().socket_handle, SOL_SOCKET, SO_KEEPALIVE, &optval, &optlen);
  return optval != 0;
}

void net::tcp_socket::set_keep_alive_enabled (bool val)
{
  int optval = 1;
  socklen_t optlen = sizeof (optval);
  setsockopt (impl<socket_impl> ().socket_handle, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen);
}

void net::tcp_socket::connect (const uint8_t addr_[4], uint16_t p)
{
  if (m_state != closed)
  {
    std::cout << "socket not closed.  ignoring call to connect." << std::endl;
    return;
  }

  struct sockaddr_in addr;
  std::memset (&addr, 0, sizeof (addr));

  addr.sin_family = AF_INET;
  std::memcpy (&addr.sin_addr.s_addr, addr_, 4);
  addr.sin_port = htons (p);

  if (::connect (impl<socket_impl> ().socket_handle, (const sockaddr*)&addr, sizeof (addr)) != 0)
  {
    std::cout << "connect NG" << std::endl;
  }
  else
  {
    m_state = connected;
    invoke_connected_clb ();
  }
}

void net::tcp_socket::close (void)
{
  if (m_state != closed && m_state != closing)
  {
    std::cout << "tcp_socket::close " << this << std::endl;

    ::close (impl<socket_impl> ().socket_handle);

    if (g_tcp_sockets != nullptr)
      g_tcp_sockets->remove (this);

    impl<socket_impl> ().socket_handle = -1;
    m_state = closed;

    invoke_close_clb ();
  }
}


unsigned int net::tcp_socket::send (const void* data, unsigned int len,
				    send_flags flags)
{
  ssize_t r = ::send (impl<socket_impl> ().socket_handle, data, len, 0);
  return r;
}

void net::tcp_socket::flush_send_buffer (void)
{
  int prev_val;
  socklen_t prev_val_len = sizeof (prev_val);

  getsockopt (impl<socket_impl>().socket_handle, IPPROTO_TCP, TCP_NODELAY, &prev_val, &prev_val_len);

  if (prev_val == 0)
  {
    int val = 1;
    setsockopt (impl<socket_impl>().socket_handle, IPPROTO_TCP, TCP_NODELAY, &val, sizeof (val));
    setsockopt (impl<socket_impl>().socket_handle, IPPROTO_TCP, TCP_NODELAY, &prev_val, sizeof (prev_val));
  }
}

unsigned int net::tcp_socket::send_buffer_capacity (void) const
{
  int val = 0;
  socklen_t val_len = sizeof (val);
  getsockopt (impl<socket_impl> ().socket_handle, SOL_SOCKET, SO_SNDBUF, &val, &val_len);
  return val;
}

unsigned int net::tcp_socket::send_buffer_pending_count (void) const
{
  // ioctl( socket_descriptor, TIOCOUTQ, &size );  // alternative 1
  return 0;
}


// --------------------------------------------------------------------------



static void exec_listening_ports (void)
{
  if (g_listening_ports == nullptr)
    return;

  fd_set read_fds = g_listening_ports->fdset;
  timeval t = { 0, 0, };

  if (select (FD_SETSIZE, &read_fds, nullptr, nullptr, &t) < 0)
    assert (false && "select NG");

  for (int i = 0; i < FD_SETSIZE; ++i)
    if (FD_ISSET (i, &read_fds))
    {
      auto* o = g_listening_ports->obj_for_handle (i);

      // it must be a connection request, because we're checking only
      // listening ports here.
      struct sockaddr_in client_addr;
      socklen_t client_addr_size = sizeof (client_addr);

      int new_socket_handle = accept (i, (struct sockaddr*)&client_addr, &client_addr_size);
      if (new_socket_handle < 0)
	assert (false && "accept NG");

      std::cout << "new connection from "
		<< inet_ntoa (client_addr.sin_addr)
		<< ":" << ntohs (client_addr.sin_port) << std::endl;

      auto new_socket = std::make_unique<net::tcp_socket> (net::tcp_socket::no_init);
      new_socket->impl<socket_impl> ().socket_handle = new_socket_handle;
      new_socket->m_state = net::tcp_socket::connected;

      if (g_tcp_sockets == nullptr)
	g_tcp_sockets = std::make_unique<socket_set<net::tcp_socket*>> ();

      g_tcp_sockets->add (new_socket.get ());

      o->invoke_new_connection_clb (std::move (new_socket));
    }
}

static void exec_tcp_sockets (void)
{
  if (g_tcp_sockets == nullptr)
    return;

  fd_set read_fds = g_tcp_sockets->fdset;
  timeval t = { 0, 0, };

  if (select (FD_SETSIZE, &read_fds, nullptr, nullptr, &t) < 0)
    assert (false && "select NG");

  uint32_t recv_buffer[1024];

  for (int i = 0; i < FD_SETSIZE; ++i)
    if (FD_ISSET (i, &read_fds))
    {
      auto* o = g_tcp_sockets->obj_for_handle (i);

      ssize_t r = recv (i, recv_buffer, sizeof (recv_buffer), 0);

      if (r == 0)
        o->close ();

      else if (r > 0)
        o->invoke_recv_clb (recv_buffer, r);

      else // must be < 0, i.e. error
      {
	std::cout << "recv error, closing socket" << std::endl;
	o->close ();
      }
    }
}

namespace net
{

void init (dev::eth&,
	const unsigned char device_ipaddr[4],
	const unsigned char gateway_ipaddr[4],
	const unsigned char subnetmask[4])
{
}

void exec (void)
{
  exec_listening_ports ();
  exec_tcp_sockets ();
}

} // namespace net

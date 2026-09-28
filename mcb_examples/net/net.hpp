
#ifndef inlcudeguard_net_net_hpp_includeguard
#define inlcudeguard_net_net_hpp_includeguard

#include <cstdlib>
#include <cstdint>
#include <array>
#include <functional>
#include <type_traits>
#include <memory>
#include <string_view>
#include <optional>

#include <utils/byte_order.hpp>

#include <dev/eth.hpp>

namespace net
{

void init (dev::eth& eth_dev,
	   const unsigned char device_ipaddr[4],
	   const unsigned char gateway_ipaddr[4],
	   const unsigned char subnetmask[4]);

void exec (void);

template <typename T> inline constexpr
typename std::enable_if < std::is_integral<T>::value && sizeof (T) == 1, T>::type
hton (T val)
{
  return val;
}

template <typename T> inline constexpr
typename std::enable_if < std::is_integral<T>::value && sizeof (T) == 2, T>::type
hton (T val)
{
  return utils::native_to (utils::big_endian, val);
}

template <typename T> inline constexpr
typename std::enable_if < std::is_integral<T>::value && sizeof (T) == 4, T>::type
hton (T val)
{
  return utils::native_to (utils::big_endian, val);
}

template <typename T> inline constexpr T ntoh (T val)
{
  return hton (val);
}

// -----------------------------------------------------------------------

std::optional<std::array<uint8_t, 4>>
parse_ipv4_addr (std::string_view str);

// -----------------------------------------------------------------------

#ifndef NET_NO_TCP

class tcp_socket
{
public:
  enum state_t
  {
    closed,
    closing,
    connected,
    connecting,
  };

  enum send_flags
  {
    write_immediately = 1 << 0,
    queue_write = 0 << 0
  };

  tcp_socket (void);

  // if the connection is open, close it.
  ~tcp_socket (void);

  state_t state (void) const { return m_state; }

  uint16_t local_port (void) const;
  uint16_t remote_port (void) const;

  std::array<uint8_t, 4> local_addr (void) const;
  std::array<uint8_t, 4> remote_addr (void) const;

  // enable transmission of periodic keep-alive packets if the connection
  // is inactive.  the default is disabled.
  bool keep_alive_enabled (void) const;
  void set_keep_alive_enabled (bool val = true);

  // try to connect to the specified IPv4 address and port.
  void connect (const uint8_t addr[4], uint16_t p);

  // close the connection if not already closed.
  void close (void);

  // allow the system to close the connection automatically if resources
  // are low.  the default is disabled.
  bool auto_close_enabled (void) const { return m_auto_close_en; }
  void set_auto_close_enabled (bool val = true) { m_auto_close_en = val; }

  // write the specified data into the buffer.  if write_immediately is
  // specified, try to send the data from the buffer.  otherwise buffer the
  // data.  if the data buffer becomes too full a send operation might be
  // triggered automatically.
  // returns the number of bytes sent (copied into the buffer).
  unsigned int send (const void* data, unsigned int len,
		     send_flags flags = write_immediately);

  template <typename T, unsigned int N> unsigned int
  send (const std::array<T, N>& a, send_flags flags = write_immediately)
  {
    return send (a.data (), N * sizeof (T), flags);
  }

  template <typename T, unsigned int N> unsigned int
  send (T(&a)[N], send_flags flags = write_immediately)
  {
    return N > 0 ? send (a, N * sizeof (T), flags) : 0;
  }

  template <unsigned int N> unsigned int
  send (const char(&a)[N], send_flags flags = write_immediately)
  {
    // don't send the zero terminating character.
    return N > 0 ? send (a, a[N - 1] == '\0' ? N - 1 : N, flags) : 0;
  }

  template <typename T> unsigned int
  send (const T& d, send_flags flags = write_immediately)
  {
    return send (&d, sizeof (T), flags);
  }

  // flush the send buffer.
  void flush_send_buffer (void);

  // returns the total number of bytes the send buffer can hold.
  unsigned int send_buffer_capacity (void) const;

  // returns the number of bytes currently in the send buffer.
  unsigned int send_buffer_pending_count (void) const;

  // returns the number of bytes that can be written at most.
  unsigned int send_max_count (void) const
  {
    return send_buffer_capacity () - send_buffer_pending_count ();
  }

  // suspend receiving data on this socket.
  void suspend_recv (void);

  // resume receiving data on this socket.
  void resume_recv (void);

  // returns true if data reception has been suspended.
  bool recv_suspended (void) const;

  // when data is received the callback function is invoked.
  // the application must copy or process the data or else it will get lost.
  // if the application can't or doesn't want to receive any more data, it
  // should suspend data reception using 'suspend_recv'.
  typedef std::function<void (tcp_socket* s, const void* data, unsigned int sz)> receive_func;
  void on_receive (receive_func&& f) { m_recv_clb = std::move (f); }
  void on_receive (const receive_func& f) { m_recv_clb = f; }

  // callback function that is invoked when the connection is closed.
  typedef std::function<void (tcp_socket* s)> close_func;
  void on_close (const close_func& f) { m_close_clb = f; }
  void on_close (close_func&& f) { m_close_clb = std::move (f); }

  // callback function that is invoked when the socket gets connected.
  typedef std::function<void (tcp_socket*)> conn_func;
  void on_connect (const conn_func& f) { m_conn_clb = f; }
  void on_connect (conn_func&& f) { m_conn_clb = std::move (f); }


#ifndef net_impl
protected:
#endif //net_impl
  enum no_init_tag { no_init };
  tcp_socket (no_init_tag) { }

  state_t m_state = closed;
  bool m_auto_close_en = false;

  receive_func m_recv_clb;
  close_func m_close_clb;
  conn_func m_conn_clb;

  void invoke_recv_clb (const void* data, unsigned int len)
  {
    if (m_recv_clb != nullptr)
      m_recv_clb (this, data, len);
  }

  void invoke_close_clb (void)
  {
    if (m_close_clb != nullptr)
      m_close_clb (this);
  }

  void invoke_connected_clb (void)
  {
    if (m_conn_clb != nullptr)
      m_conn_clb (this);
  }

  std::aligned_storage_t<16, 4> m_subclass_data[1];

  template <typename T> T& impl (void) { return *(T*)m_subclass_data; }
  template <typename T> const T& impl (void) const { return *(const T*)m_subclass_data; }
};

// -----------------------------------------------------------------------

class tcp_listening_port
{
public:
  // the callback function will be invoked for each new incoming connection.
  // the application should keep a reference to the new socket, for example
  // by moving the unique_ptr into some other place.  if it doesn't, the
  // socket will be deleted automatically.
  typedef std::function<void (std::unique_ptr<tcp_socket>)> new_conn_func;

  // create a listening port.
  // if a listening port for the specific port number already exists,
  // the new listening port will have no function.
  tcp_listening_port (uint16_t p, const new_conn_func& clb);

  tcp_listening_port (void);
  tcp_listening_port (const tcp_listening_port&) = delete;
  
  tcp_listening_port (tcp_listening_port&& rhs);

  tcp_listening_port& operator = (const tcp_listening_port&) = delete;
  tcp_listening_port& operator = (tcp_listening_port&& rhs);

  ~tcp_listening_port (void);

  uint16_t local_port (void) const { return m_port; }

#ifndef net_impl
protected:
#endif //net_impl

  uint16_t m_port = 0;

  new_conn_func m_new_conn_clb;

  void invoke_new_connection_clb (std::unique_ptr<tcp_socket> s)
  {
    if (m_new_conn_clb != nullptr)
      m_new_conn_clb (std::move (s));
  }

  std::aligned_storage_t<16, 4> m_subclass_data[1];

  template <typename T> T& impl (void) { return *(T*)m_subclass_data; }
  template <typename T> const T& impl (void) const { return *(const T*)m_subclass_data; }
};


#endif // NET_NO_TCP

// -----------------------------------------------------------------------

class udp_socket
{
public:
  // create an unbound and unconnected socket.  it will not receive any packets
  // and/ sending will also fail.
  udp_socket (void);

  udp_socket (udp_socket&& rhs);
  udp_socket& operator = (udp_socket&& rhs);

  // create a socket bound to the local port.  this will allow receiving
  // incoming packets on that port but not sending anything.
  udp_socket (uint16_t local_port);

  // close and destroy a socket.
  ~udp_socket (void);

  uint16_t local_port (void) const;
  uint16_t remote_port (void) const;

  std::array<uint8_t, 4> local_addr (void) const;
  std::array<uint8_t, 4> remote_addr (void) const;

  // connect to the specified ipv4 address and the remote port.  the local
  // port will be assigned randomly.
  // since udp is connectionless this is a meta-operation, which just records
  // the remote address and port for future socket operations and enables
  // receiving packets from the remote host.
  void connect (const uint8_t remote_addr[4], uint16_t remote_port);

  // close the connection if not already closed.
  // because udp is connectionless this is a meta-operation, which just
  // disables sending and receiving packets from/on this socket.
  void close (void);

  // set the local port to be used for this socket.  this can also be used
  // to change the local port of an already open socket.  returns true on
  // success and false on failure.
  bool bind (uint16_t new_port);

  // send the specified data as a data packet.  returns the number of
  // bytes actually sent.
  unsigned int send (const void* data, unsigned int len);

  template <typename T, unsigned int N> unsigned int
  send (const std::array<T, N>& a)
  {
    return send (a.data (), N * sizeof (T));
  }

  template <typename T, unsigned int N> unsigned int
  send (T(&a)[N])
  {
    return N > 0 ? send (a, N * sizeof (T)) : 0;
  }

  template <unsigned int N> unsigned int
  send (const char(&a)[N])
  {
    // don't send the zero terminating character.
    return N > 0 ? send (a, a[N - 1] == '\0' ? N - 1 : N) : 0;
  }

  template <typename T> unsigned int
  send (const T& d)
  {
    return send (&d, sizeof (T));
  }


  // when data is received the callback function is invoked.
  // the application must copy or process the data or else it will get lost.
  typedef std::function<void (udp_socket* s, const void* data, unsigned int sz,
			      uint8_t remote_addr[4], uint16_t remote_port)> receive_func;
  void on_receive (receive_func&& f) { m_recv_clb = std::move (f); }
  void on_receive (const receive_func& f) { m_recv_clb = f; }

#ifndef net_impl
protected:
#endif //net_impl

  enum no_init_tag { no_init };
  udp_socket (no_init_tag) { }

  receive_func m_recv_clb;

  void invoke_recv_clb (const void* data, unsigned int len,
			uint8_t remote_addr[4], uint16_t remote_port)
  {
    if (m_recv_clb != nullptr)
      m_recv_clb (this, data, len, remote_addr, remote_port);
  }

  std::aligned_storage_t<16, 4> m_subclass_data[1];

  template <typename T> T& impl (void) { return *(T*)m_subclass_data; }
  template <typename T> const T& impl (void) const { return *(const T*)m_subclass_data; }
};

} // namespace net
#endif // inlcudeguard_net_net_hpp_includeguard

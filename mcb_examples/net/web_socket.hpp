
/*
  websocket
*/

#ifndef includeguard_net_web_socket_includeguard
#define includeguard_net_web_socket_includeguard

#include <memory>
#include <net/net.hpp>

namespace net
{

class web_socket
{
public:
  // same as tcp_socket ...
  enum state_t
  {
    closed,
    closing,
    connected,
    connecting,
  };

  // same as tcp_socket ...
  enum send_flags
  {
    write_immediately = 1 << 0,
    queue_write = 0 << 0
  };


  web_socket (std::unique_ptr<net::tcp_socket> s);
  ~web_socket (void);

  uint16_t local_port (void) const { return m_socket->local_port (); }
  uint16_t remote_port (void) const { return m_socket->remote_port (); }




private:
  state_t m_state = closed;
  std::unique_ptr<net::tcp_socket> m_socket;
};

} // namespace net
#endif // includeguard_net_web_socket_includeguard

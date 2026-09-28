#ifndef includeguard_mcb_binary_server_hpp
#define includeguard_mcb_binary_server_hpp

#include "mcb_binary_service.hpp"

#include <memory>
#include <net/net.hpp>

namespace mcb_binary
{

class server
{
public:
  using clock_func = uint64_t (*) (void);
  explicit server (const read_only_backend& backend, uint16_t port = 55000,
                   clock_func clock = &now_ms);
  // The server has firmware lifetime. Drain closing sockets via exec before
  // destroying it while the network stack is still running.
  ~server ();
  server (const server&) = delete;
  server& operator = (const server&) = delete;

  void exec (void);

private:
  void accept (std::unique_ptr<net::tcp_socket> socket);
  void release_socket (void);
  static uint64_t now_ms (void);

  const read_only_backend& m_backend;
  clock_func m_clock;
  session m_session;
  std::unique_ptr<net::tcp_socket> m_socket;
  bool m_dead = false;
  unsigned int m_pending_count = 0;
  uint64_t m_pending_since = 0;
  net::tcp_listening_port m_listener;
};

} // namespace mcb_binary

#endif

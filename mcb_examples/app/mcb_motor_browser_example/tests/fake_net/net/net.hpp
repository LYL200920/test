#ifndef includeguard_mcb_binary_fake_net_hpp
#define includeguard_mcb_binary_fake_net_hpp

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace net
{
class tcp_socket
{
public:
  enum state_t { closed, closing, connected, connecting };
  using receive_func = std::function<void (tcp_socket*, const void*, unsigned int)>;
  using close_func = std::function<void (tcp_socket*)>;

  ~tcp_socket () { ++destroyed; }
  state_t state () const { return m_state; }
  void close () { if (m_state != closed) m_state = closing; }
  void complete_close ()
  {
    m_state = closed;
    if (m_close)
      m_close (this);
  }
  void on_receive (receive_func callback) { m_receive = std::move (callback); }
  void on_close (close_func callback) { m_close = std::move (callback); }
  void deliver (const void* data, unsigned int size)
  {
    // May be called while stopped to reproduce the existing uIP ACK path.
    if (m_receive)
      m_receive (this, data, size);
  }
  unsigned int send (const void* data, unsigned int size)
  {
    const unsigned int accepted = std::min (size, accept_limit);
    const auto* bytes = static_cast<const uint8_t*> (data);
    output.insert (output.end (), bytes, bytes + accepted);
    pending += accepted;
    return accepted;
  }
  void flush_send_buffer () { }
  unsigned int send_buffer_pending_count () const { return pending; }
  void suspend_recv () { suspended = true; }
  void resume_recv () { suspended = false; }
  bool recv_suspended () const { return suspended; }

  inline static unsigned int destroyed = 0;
  unsigned int accept_limit = 128;
  unsigned int pending = 0;
  bool suspended = false;
  std::vector<uint8_t> output;

private:
  state_t m_state = connected;
  receive_func m_receive;
  close_func m_close;
};

class tcp_listening_port
{
public:
  using new_conn_func = std::function<void (std::unique_ptr<tcp_socket>)>;
  tcp_listening_port (uint16_t port, new_conn_func callback) : m_callback (std::move (callback))
  {
    last = this;
    last_port = port;
  }
  ~tcp_listening_port () { last = nullptr; }
  tcp_socket* connect ()
  {
    auto socket = std::make_unique<tcp_socket> ();
    auto* pointer = socket.get ();
    m_callback (std::move (socket));
    return pointer;
  }
  inline static tcp_listening_port* last = nullptr;
  inline static uint16_t last_port = 0;
private:
  new_conn_func m_callback;
};
}
#endif

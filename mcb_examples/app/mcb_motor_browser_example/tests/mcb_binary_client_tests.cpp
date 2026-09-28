#include "mcb_binary_service.hpp"
#include <mcb_cmd/li5000_mcb_cmd_client.hpp>

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{
void require (bool condition, const char* message)
{
  if (!condition)
    throw std::runtime_error (message);
}

class socket_handle
{
public:
  explicit socket_handle (int fd) : m_fd (fd) { require (fd >= 0, "Socket creation failed"); }
  ~socket_handle () { ::close (m_fd); }
  socket_handle (const socket_handle&) = delete;
  socket_handle& operator = (const socket_handle&) = delete;
  int get () const { return m_fd; }
private:
  int m_fd;
};

struct backend : mcb_binary::read_only_backend
{
  std::array<mcb_binary::axis_snapshot, 4> axes { };
  mutable std::atomic<unsigned int> calls { 0 };

  backend ()
  {
    for (unsigned int index = 0; index < 4; ++index)
    {
      axes[index].logical_position = index == 3 ? std::numeric_limits<int32_t>::min () : -12345 - int (index);
      axes[index].encoder_position = std::numeric_limits<int32_t>::max () - int (index);
      axes[index].speed_pps = 0x12345678 + index;
    }
    axes[0].moving = true;
    axes[1].homing = true;
    axes[2].error = true;
    axes[2].positive_limit = true;
    axes[3].negative_limit = true;
  }

  bool read_axis (unsigned int index, mcb_binary::axis_snapshot& result) const override
  {
    ++calls;
    if (index >= axes.size ())
      return false;
    result = axes[index];
    return true;
  }
};

class loopback_server
{
public:
  backend source;

  loopback_server () : m_listener (::socket (AF_INET, SOCK_STREAM, 0))
  {
    sockaddr_in address { };
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl (INADDR_LOOPBACK);
    address.sin_port = 0;
    require (::bind (m_listener.get (), reinterpret_cast<sockaddr*> (&address), sizeof (address)) == 0, "Loopback bind failed");
    require (::listen (m_listener.get (), 1) == 0, "Loopback listen failed");
    socklen_t size = sizeof (address);
    require (::getsockname (m_listener.get (), reinterpret_cast<sockaddr*> (&address), &size) == 0, "Port lookup failed");
    m_port = ntohs (address.sin_port);
    m_thread = std::thread ([this]
    {
      try { run (); }
      catch (...) { m_failure = std::current_exception (); }
    });
  }

  ~loopback_server () { stop (); }
  unsigned int port () const { return m_port; }
  void finish ()
  {
    stop ();
    if (m_failure)
      std::rethrow_exception (m_failure);
  }

private:
  static uint64_t now_ms ()
  {
    return std::chrono::duration_cast<std::chrono::milliseconds> (
        std::chrono::steady_clock::now ().time_since_epoch ()).count ();
  }

  void stop ()
  {
    m_stop = true;
    if (m_thread.joinable ())
      m_thread.join ();
  }

  void run ()
  {
    while (!m_stop)
    {
      pollfd listening { m_listener.get (), POLLIN, 0 };
      const int ready = ::poll (&listening, 1, 20);
      if (ready < 0 && errno == EINTR)
        continue;
      require (ready >= 0, "Listener poll failed");
      if (ready == 0)
        continue;
      socket_handle socket (::accept (m_listener.get (), nullptr, nullptr));
      const timeval timeout { 2, 0 };
      require (::setsockopt (socket.get (), SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof (timeout)) == 0, "Send timeout setup failed");
      mcb_binary::session session;
      session.reset (now_ms ());
      while (!m_stop && !session.should_close ())
      {
        pollfd peer { socket.get (), POLLIN, 0 };
        const int available = ::poll (&peer, 1, 5);
        if (available < 0 && errno == EINTR)
          continue;
        require (available >= 0, "Peer poll failed");
        if (available > 0)
        {
          // Deliberately split every native client request across receives.
          std::array<uint8_t, 3> input;
          const auto received = ::recv (socket.get (), input.data (), input.size (), 0);
          if (received == 0)
            break;
          if (received < 0 && errno == EINTR)
            continue;
          require (received > 0, "Peer receive failed");
          require (session.receive (input.data (), std::size_t (received), now_ms ()), "Core receive failed");
        }
        session.exec (source, now_ms ());
        while (session.output_size () != 0 && !m_stop)
        {
          // Exercise client recv_wait with headers and bodies in small writes.
          const auto size = std::min (session.output_size (), std::size_t (3));
          const auto sent = ::send (socket.get (), session.output_data (), size, MSG_NOSIGNAL);
          if (sent < 0 && errno == EINTR)
            continue;
          require (sent > 0, "Peer send failed");
          session.consume_output (std::size_t (sent), now_ms ());
        }
      }
    }
  }

  socket_handle m_listener;
  unsigned int m_port = 0;
  std::atomic<bool> m_stop { false };
  std::thread m_thread;
  std::exception_ptr m_failure;
};

template<typename Function>
void invalid_request (Function action)
{
  try { action (); }
  catch (const li5000_mcb_cmd::error_response& error)
  {
    require (error == li5000_mcb_cmd::invalid_request (), "Wrong typed protocol error");
    return;
  }
  throw std::runtime_error ("Unsupported request succeeded");
}

void test_client ()
{
  using namespace li5000_mcb_cmd;
  loopback_server server;
  client client;
  client.connect (std::string ("127.0.0.1"), server.port ());
  require (client.invoke<gantry_check_st> (0, 0).value ().value () == 0, "Gantry discovery failed");
  invalid_request ([&] { client.invoke<gantry_check_st> (1, 0); });
  invalid_request ([&] { client.invoke<conv_check_st> (0); });
  require (server.source.calls == 0, "Discovery/error probes read backend");

  const std::array<uint8_t, 4> expected_status { 0x20, 0x40, 0x1A, 0x11 };
  for (unsigned int index = 0; index < 4; ++index)
  {
    const unsigned int mask = 1U << index;
    const auto position = client.invoke<gantry_get_pos> (0, mask);
    require (position.axes () == mask && position.gantry_num () == 0, "Response identity");
    require (position.value () == uint32_t (server.source.axes[index].logical_position), "Signed logical position bits");
    require (client.invoke<gantry_get_enc_pos> (0, mask).value () == uint32_t (server.source.axes[index].encoder_position), "Encoder position bits");
    require (client.invoke<gantry_get_cur_speed> (0, mask).value () == server.source.axes[index].speed_pps, "Speed response");
    require (client.invoke<gantry_check_st> (0, mask).value ().value () == expected_status[index], "Status response");
  }
  const auto status = client.invoke<gantry_check_st> (0, 15).value ();
  require (status.value () == 0x7B && !status.home () && !status.homing_ok (), "Combined/unknown home status");

  const auto before = server.source.calls.load ();
  invalid_request ([&] { client.invoke<set_servo_on> (); });
  invalid_request ([&] { client.invoke<set_servo_off> (); });
  for (auto function : { gantry_func::go_home, gantry_func::go_move, gantry_func::stop })
    invalid_request ([&] { client.invoke<gantry_func> (0, gantry_axis::u, function, 123); });
  invalid_request ([&] { client.invoke<gantry_set_param> (0, gantry_axis::x, gantry_set_param::normal_speed, 8000); });
  invalid_request ([&] { client.invoke<gantry_wait_for_stop> (0, 15); });
  invalid_request ([&] { client.invoke<gantry_get_pos> (0, 3); });
  require (server.source.calls == before, "Rejected native requests read backend");

  client.invoke_n (gantry_get_pos::request (0, 1), gantry_get_enc_pos::request (0, 2),
                   gantry_get_cur_speed::request (0, 8), gantry_check_st::request (0, 15));
  std::vector<std::unique_ptr<basic_request>> batch;
  for (unsigned int index = 0; index < 20; ++index)
    batch.emplace_back (new gantry_get_pos::request (0, 1U << (index % 4)));
  client.invoke_v (batch);
  require (client.invoke<gantry_get_pos> (0, 8).value () == uint32_t (server.source.axes[3].logical_position), "Batch left responses queued");
  client.disconnect ();
  client.connect (std::string ("127.0.0.1"), server.port ());
  require (client.invoke<gantry_check_st> (0, 0).value ().value () == 0, "Reconnection failed");
  client.disconnect ();
  server.finish ();
}
}

int main ()
{
  try
  {
    test_client ();
    std::cout << "PASS: native client discovery, four-axis queries, mutation rejection, batches, reconnect\n";
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << "FAIL: " << error.what () << '\n';
    return 1;
  }
}

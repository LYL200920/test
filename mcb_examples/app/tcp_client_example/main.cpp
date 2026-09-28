
#include <cstdio>
#include <memory>
#include <list>
#include <chrono>
#include <experimental/string_view>

#include <board/board.hpp>
#include <net/net.hpp>

#define log_all
#include <logging/logging.hpp>


int main (void)
{
  log_level::enable (log_level::warn);
  log_level::enable (log_level::error);
  log_level::enable (log_level::info);
//  log_level::enable (log_level::debug);
//  log_level::enable (log_level::trace);

  uint8_t device_ipaddr[4] = { 192, 168, 0, 81 };
  uint8_t gateway_ipaddr[4] = { 192, 168, 0, 1 };
  uint8_t subnetmask[4] = { 255, 255, 255, 0 };

  net::init (this_board::inst ().eth0, device_ipaddr, gateway_ipaddr, subnetmask);

  this_board::inst ().led_outputs.write (0b0000);


  net::tcp_socket socket;

  auto prev_socket_state = socket.state ();
  auto last_socket_send_time = std::chrono::system_clock::now ();

  for (unsigned int main_loop_count = 0; ; ++main_loop_count)
  {
    auto time_now = std::chrono::system_clock::now ();
    this_board::inst ().exec ();

    if (main_loop_count == 0 || prev_socket_state != socket.state ())
    {
      switch (socket.state ())
      {
	case net::tcp_socket::closed:  log_info ("socket state = closed\n"); break;
	case net::tcp_socket::closing: log_info ("socket state = closing\n"); break;
	case net::tcp_socket::connected: log_info ("socket state = connected\n"); break;
	case net::tcp_socket::connecting: log_info ("socket state = connecting\n"); break;
	default: log_info ("socket state = ???\n"); break;
      }

      prev_socket_state = socket.state ();
    }

    // send a message every 500 milliseconds if the socket is connected.
    if (socket.state () == net::tcp_socket::connected
	&& (time_now - last_socket_send_time) >= std::chrono::milliseconds (500))
    {
      log_info ("sending message\n");
      std::experimental::string_view msg = "test moon please ignore - ";
      socket.send (msg.data (), msg.size ());
    }

    // if the socket is in a closed state, try connecting
    if (socket.state () == net::tcp_socket::closed)
    {
      uint8_t server_addr[4] = { 192, 168, 0, 40 };
      const unsigned int server_port = 5555;

      socket.connect (server_addr, server_port);
    }

    net::exec ();
  }

  return 0;
}

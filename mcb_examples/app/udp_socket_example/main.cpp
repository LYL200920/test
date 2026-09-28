
#include <cstdio>
#include <memory>
#include <list>
#include <algorithm>
#include <chrono>

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

  uint8_t device_ipaddr[4] = { 192, 168, 0, 80 };
  uint8_t gateway_ipaddr[4] = { 192, 168, 0, 1 };
  uint8_t subnetmask[4] = { 255, 255, 255, 0 };

  net::init (this_board::inst ().eth0, device_ipaddr, gateway_ipaddr, subnetmask);

  this_board::inst ().led_outputs.write (0b0000);

  net::udp_socket udp_socket (69);
  udp_socket.on_receive ([] (net::udp_socket* socket, const void* data, unsigned int sz,
			     uint8_t remote_addr[4], uint16_t remote_port)
  {
    log_info ("udp receive len %u remote %u.%u.%u.%u:%u\n", sz,
	      remote_addr[0], remote_addr[1], remote_addr[2], remote_addr[3],
	      remote_port);

    socket->connect (remote_addr, remote_port);
    socket->send ("hello");

    socket->bind (12345);
    socket->send ("hello");
  });

  for (unsigned int main_loop_count = 0; ; ++main_loop_count)
  {
    this_board::inst ().exec ();
    net::exec ();
  }

  return 0;
}

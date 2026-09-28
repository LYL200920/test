
#include <cstdio>
#include <memory>
#include <list>
#include <chrono>

#include <board/board.hpp>
#include <net/net.hpp>


#define log_all
#include <logging/logging.hpp>

class tcp_server
{
public:
  tcp_server (void) = delete;
  tcp_server (const tcp_server&) = delete;
  tcp_server (tcp_server&&) = delete;
  tcp_server& operator = (const tcp_server&) = delete;
  tcp_server& operator = (tcp_server&&) = delete;

  tcp_server (uint16_t port)
  {
    m_listening_port = net::tcp_listening_port (port,
	[this] (net::tcp_socket* new_socket)
	{
	  log_info ("new connection\n");
	  m_connections.emplace_back (std::make_shared<connection> (new_socket));

	  auto inserted_iter = std::prev (m_connections.end ());

	  new_socket->on_close ([this, inserted_iter] (net::tcp_socket* s)
	  {
	    log_info ("connection closed %p socket %p\n", this, s);
	    m_connections.erase (inserted_iter);
	  });
	});
  }

  void exec (void)
  {
    auto cur_time = std::chrono::high_resolution_clock::now ();

    for (auto&& c : m_connections)
      c->exec (cur_time);
  }

private:
  struct connection : std::enable_shared_from_this<connection>
  {
    std::unique_ptr<net::tcp_socket> m_socket;

    std::chrono::high_resolution_clock::time_point m_last_update_time =
				std::chrono::high_resolution_clock::now ();
    unsigned int m_count = 0;
    unsigned int m_drop_count = 0;

    connection (net::tcp_socket* socket)
    : m_socket (socket)
    {
      using namespace std::placeholders;
      socket->on_receive (std::bind (&connection::recv, this, _2, _3));

      char buf[64];
      m_socket->send (buf, std::sprintf (buf, "hello %p\n", this));
    }

    void recv (const void* data, unsigned int len)
    {
    }

    void exec (std::chrono::high_resolution_clock::time_point cur_time)
    {
      using namespace std::chrono_literals;

      if (cur_time - m_last_update_time > 500ms || true)
      {
	m_last_update_time = cur_time;
	++m_count;

	char buf[128];
	int send_sz = std::sprintf (buf,
		"hello %p %08x %08x %u %u\n", m_socket.get (),
		m_count & 1 ? 0x55555555 : 0x33333333,
		1 << ((m_count * 4) & 31), m_socket->send_max_count (),
		m_drop_count);

	if (send_sz <= (int)m_socket->send_max_count ())
	  m_socket->send (buf, send_sz);
	else
	  ++m_drop_count;

      }
    }
  };

  net::tcp_listening_port m_listening_port;
  std::list<std::shared_ptr<connection>> m_connections;
};


int main (void)
{
  log_level::enable (log_level::warn);
  log_level::enable (log_level::error);
//  log_level::enable (log_level::info);
//  log_level::enable (log_level::debug);
//  log_level::enable (log_level::trace);

  uint8_t device_ipaddr[4] = { 192, 168, 0, 80 };
  uint8_t gateway_ipaddr[4] = { 192, 168, 0, 1 };
  uint8_t subnetmask[4] = { 255, 255, 255, 0 };

  net::init (this_board::inst ().eth0, device_ipaddr, gateway_ipaddr, subnetmask);

  this_board::inst ().led_outputs.write (0b0000);

  tcp_server srv (5000);

  for (unsigned int main_loop_count = 0; ; ++main_loop_count)
  {
    this_board::inst ().exec ();
    srv.exec ();
    net::exec ();
  }

  return 0;
}

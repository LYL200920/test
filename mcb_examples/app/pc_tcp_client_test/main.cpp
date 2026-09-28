# include <sys/types.h>
# include <sys/socket.h>
# include <sys/param.h>
# include <netinet/in.h>
# include <arpa/inet.h>
# include <netinet/tcp.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <string>
#include <cassert>
#include <cstring>
#include <thread>
#include <atomic>
#include <array>
#include <chrono>
#include <vector>


/*

TCP/IP stress test

1) send data to multiple sockets
2) receive data from multiple sockets
3) send to/receive from multiple sockets
4) rapid connection open/close
5) send/receive + connection open/close

*/

class tcp_connection
{
public:
  tcp_connection (void) = delete;
  tcp_connection (const tcp_connection&) = delete;
  tcp_connection& operator = (const tcp_connection&) = delete;

  tcp_connection (const std::string& ipv4_addr, int port)
  {
    m_sockfd = socket (AF_INET, SOCK_STREAM, 0);
    if (m_sockfd == -1)
    {
      std::cout << "create socket NG" << std::endl;
      return;
    }

    if (0)
    {
      int val = true;
      if (setsockopt (m_sockfd,
		    IPPROTO_TCP, TCP_NODELAY, (char*)&val, sizeof (val)) < 0)
        std::cout << "setsockopt TCP_NODELAY NG" << std::endl;
    }

    sockaddr_in servaddr;
    std::memset (&servaddr, 0, sizeof (servaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr (ipv4_addr.c_str ());
    servaddr.sin_port = htons (port);

    if (connect (m_sockfd, (const sockaddr*)&servaddr, sizeof (servaddr)) != 0)
    {
      close (m_sockfd);
      m_sockfd = -1;
      std::cout << "socket connect NG" << std::endl;
    }

    m_recv_thread = std::thread (std::bind (&tcp_connection::recv_thread_func, this));
  }

  tcp_connection (tcp_connection&& rhs)
  : m_sockfd ((int)rhs.m_sockfd),
    m_recv_thread (std::move (rhs.m_recv_thread)),
    m_send_thread (std::move (rhs.m_send_thread))
  {
    rhs.m_sockfd = -1;
  }

  tcp_connection& operator = (tcp_connection&& rhs)
  {
    if (m_sockfd != -1)
    {
      close (m_sockfd);
      m_sockfd = -1;
    }

    m_sockfd = (int)rhs.m_sockfd;
    rhs.m_sockfd = -1;

    m_recv_thread = std::move (rhs.m_recv_thread);
    m_send_thread = std::move (rhs.m_send_thread);
    return *this;
  }

  ~tcp_connection (void)
  {
    if (m_sockfd != -1)
      close (m_sockfd);

    m_sockfd = -1;

    if (m_recv_thread.joinable ())
      m_recv_thread.join ();

    if (m_send_thread.joinable ())
      m_send_thread.join ();
  }

  void start_send (void)
  {
    if (m_send_thread.joinable ())
      return;

    m_send_thread = std::thread (std::bind (&tcp_connection::send_thread_func, this));
  }


private:
  std::atomic<int> m_sockfd;
  std::array<char, 1024*32> m_recv_buf;
  std::array<char, 1024*2> m_send_buf;

  std::thread m_recv_thread;
  std::thread m_send_thread;

  void recv_thread_func (void)
  {
    std::cout << "recv func start" << std::endl;
    while (m_sockfd != -1)
      recv (m_sockfd, m_recv_buf.data (), m_recv_buf.size (), 0);

    std::cout << "recv func end" << std::endl;
  }

  void send_thread_func (void)
  {
    std::cout << "send func start" << std::endl;

    while (m_sockfd != -1)
      send (m_sockfd, m_send_buf.data (), m_send_buf.size (), 0);

    std::cout << "send func end" << std::endl;
  }
};


// ---------------------------------------------------------------------------

void test0 (void)
{
  std::cout << std::endl << "test0" << std::endl;

  for (unsigned int j = 0; j < 3; ++j)
  {
    std::vector<tcp_connection> conns;
    conns.reserve (1024);

    for (unsigned int i = 0; i < 16; ++i)
    {
      std::cout << "connecting ..." << std::endl;
      conns.emplace_back ("192.168.0.80", 5000);
    }

    std::this_thread::sleep_for (std::chrono::seconds (10));
  }
}

// ---------------------------------------------------------------------------

void test1 (void)
{
  std::cout << std::endl << "test1" << std::endl;

  for (unsigned int j = 0; j < 3; ++j)
  {
    std::vector<tcp_connection> conns;
    conns.reserve (1024);

    for (unsigned int i = 0; i < 8; ++i)
    {
      std::cout << "connecting ..." << std::endl;
      conns.emplace_back ("192.168.0.80", 5000);
      conns.back ().start_send ();
    }

    std::this_thread::sleep_for (std::chrono::seconds (10));
  }
}


// ---------------------------------------------------------------------------


int main (int argc, const char* argv[])
{
  std::cout << "hello" << std::endl;

//  test0 ();
  test1 ();

  return 0;
}

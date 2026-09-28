extern "C"
{
#include <uip.h>
}

#include <cstring>
#include <iostream>
#include <stdexcept>

namespace
{
unsigned int acknowledgements = 0;
unsigned int received = 0;
unsigned int timed_out = 0;
void require (bool condition, const char* message)
{
  if (!condition)
    throw std::runtime_error (message);
}

void packet (const uip_conn& connection, uint32_t acknowledgement, unsigned int payload)
{
  std::memset (uip_buf, 0, UIP_BUFSIZE);
  auto* header = reinterpret_cast<uip_tcpip_hdr*> (uip_buf + UIP_LLH_LEN);
  header->vhl = 0x45;
  header->len = htons (UIP_IPTCPH_LEN + payload);
  header->ttl = 64;
  header->proto = UIP_PROTO_TCP;
  header->srcipaddr = connection.ripaddr;
  header->destipaddr = uip_hostaddr;
  header->srcport = connection.rport;
  header->destport = connection.lport;
  header->seqno = hton (connection.rcv_nxt);
  header->ackno = hton (acknowledgement);
  header->tcpoffset = 5 << 4;
  header->flags = 0x10;
  header->wnd = htons (1024);
  header->tcpchksum = ~uip_tcpchksum ();
  header->ipchksum = ~uip_ipchksum ();
  uip_input (UIP_LLH_LEN + UIP_IPTCPH_LEN + payload);
}
}

extern "C"
{
unsigned int uip_tcp_retransmit (uip_conn*, void*) { return 0; }
void uip_tcp_no_more_free_connections () { }
void uip_tcp_abort (uip_conn*) { }
void uip_tcp_close (uip_conn*) { }
void uip_tcp_timed_out (uip_conn*) { ++timed_out; }
void uip_tcp_connected (uip_conn*) { }
void uip_tcp_ackdata (uip_conn*) { ++acknowledgements; }
void uip_tcp_newdata (uip_conn*, void*, unsigned int length) { received += length; }
unsigned int uip_tcp_senddata (uip_conn*, void*) { return 0; }
void uip_appcall_udp (int, uip_udp_conn*, uip_udpip_hdr*, unsigned int) { }
}

int main ()
{
  try
  {
    uip_init ();
    uip_hostaddr = uip_ipaddr (127, 0, 0, 1);
    auto& connection = uip_conns[0];
    std::memset (&connection, 0, sizeof (connection));
    connection.tcpstateflags = UIP_ESTABLISHED;
    connection.ripaddr = uip_ipaddr (127, 0, 0, 2);
    connection.lport = htons (55000);
    connection.rport = htons (30000);
    connection.rcv_nxt = 100;
    connection.snd_nxt = 200;
    connection.len = 8;
    connection.mss = connection.initialmss = 1024;
    connection.rto = 3;
    connection.timer = 3;
    packet (connection, 200, 8);
    require (received == 8, "New request was not delivered");
    require (acknowledgements == 0, "Old ACK discarded outstanding response");
    require (connection.len == 8, "Old ACK cleared outstanding TCP bytes");
    packet (connection, 208, 0);
    require (acknowledgements == 1, "Valid ACK did not release output");
    require (connection.len == 0, "Valid ACK retained outstanding TCP bytes");
    connection.tcpstateflags = UIP_FIN_WAIT_2 | UIP_STOPPED;
    connection.timer = UIP_TIME_WAIT_TIMEOUT - 1;
    uip_periodic_conn (&connection);
    require (connection.tcpstateflags == UIP_CLOSED, "Stopped FIN_WAIT_2 never expired");
    require (timed_out == 1, "FIN_WAIT_2 did not notify socket owner");
    connection.tcpstateflags = UIP_TIME_WAIT | UIP_STOPPED;
    connection.timer = UIP_TIME_WAIT_TIMEOUT - 1;
    uip_periodic_conn (&connection);
    require (connection.tcpstateflags == UIP_CLOSED && timed_out == 2, "TIME_WAIT cleanup failed");
    std::cout << "PASS: real uIP ACK bookkeeping and closing-state timeouts\n";
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << "FAIL: " << error.what () << '\n';
    return 1;
  }
}

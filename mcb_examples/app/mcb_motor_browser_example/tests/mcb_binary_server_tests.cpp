#include "mcb_binary_server.hpp"

#include <array>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
uint64_t current_time = 0;
uint64_t clock_ms () { return current_time; }
void require (bool condition, const char* message)
{
  if (!condition)
    throw std::runtime_error (message);
}
struct backend : mcb_binary::read_only_backend
{
  mutable unsigned int calls = 0;
  bool read_axis (unsigned int, mcb_binary::axis_snapshot& result) const override
  {
    ++calls;
    result.logical_position = -1;
    return true;
  }
};

void test_adapter ()
{
  backend source;
  current_time = 0;
  mcb_binary::server server (source, 55000, clock_ms);
  auto* listener = net::tcp_listening_port::last;
  require (net::tcp_listening_port::last_port == 55000, "Wrong listening port");
  auto* peer = listener->connect ();
  const auto destroyed = net::tcp_socket::destroyed;
  listener->connect ();
  require (net::tcp_socket::destroyed == destroyed + 1, "Extra client retained");

  const std::array<uint8_t, 8> query { 0, 0x83, 6, 0, 0, 0, 0, 0 };
  const std::array<uint8_t, 8> expected { 0, 0x83, 6, 0, 0xFF, 0xFF, 0xFF, 0xFF };
  peer->accept_limit = 0;
  peer->deliver (query.data (), query.size ());
  server.exec ();
  require (source.calls == 1 && peer->output.empty (), "Zero send lost output");
  peer->accept_limit = 3;
  for (unsigned int count = 0; count < 3; ++count)
    server.exec ();
  require (peer->output == std::vector<uint8_t> (expected.begin (), expected.end ()), "Partial send duplicated/lost bytes");
  require (source.calls == 1, "Partial send repeated dispatch");
  peer->pending = 0;
  peer->output.clear ();
  peer->accept_limit = 128;

  // Previously a 512-byte prefix suspended receive and could never complete.
  std::vector<uint8_t> variable (1028, 0);
  variable[0] = 0xE0;
  variable[2] = 4;
  peer->deliver (variable.data (), 512);
  server.exec ();
  require (!peer->recv_suspended (), "Incomplete variable frame deadlocked receive");
  peer->deliver (variable.data () + 512, 515);
  server.exec ();
  require (!peer->recv_suspended (), "Maximum partial variable frame blocked");
  peer->deliver (variable.data () + 1027, 1);
  server.exec ();
  require (peer->output.size () == 8 && peer->output[0] == 0xFF, "Variable response missing");
  peer->pending = 0;
  peer->output.clear ();

  // Fill input while output cannot progress; then simulate a stopped ACK
  // carrying unaccepted data. It must be ignored until TCP retransmits it.
  peer->accept_limit = 0;
  std::vector<uint8_t> batch;
  for (unsigned int count = 0; count < 200; ++count)
    batch.insert (batch.end (), query.begin (), query.end ());
  peer->deliver (batch.data (), batch.size ());
  require (peer->recv_suspended (), "Full RX did not suspend");
  const auto before = source.calls;
  peer->deliver (query.data (), query.size ());
  peer->accept_limit = 128;
  for (unsigned int count = 0; count < 50; ++count)
  {
    peer->pending = 0;
    server.exec ();
  }
  require (!peer->recv_suspended (), "Drained input did not resume");
  require (source.calls == before + 200, "Stopped ACK payload dispatched");
  require (peer->output.size () == 1600, "Batch response count");
  peer->deliver (query.data (), query.size ());
  peer->pending = 0;
  server.exec ();
  require (source.calls == before + 201, "Retransmitted input lost");

  // No ACK progress for ten seconds requests close, but the object must
  // remain valid until the stack has finished retransmission/close handling.
  const auto before_close = net::tcp_socket::destroyed;
  current_time = 10000;
  server.exec ();
  require (peer->state () == net::tcp_socket::closing, "No-progress timeout did not close");
  require (net::tcp_socket::destroyed == before_close, "Async close destroyed live socket");
  peer->deliver (query.data (), query.size ());
  server.exec ();
  require (source.calls == before + 201, "Closing socket dispatched input");
  peer->complete_close ();
  require (net::tcp_socket::destroyed == before_close, "Close callback deleted its socket");
  server.exec ();
  require (net::tcp_socket::destroyed == before_close + 1, "Closed socket not reclaimed");

  peer = listener->connect ();
  peer->deliver (query.data (), query.size ());
  server.exec ();
  require (peer->output == std::vector<uint8_t> (expected.begin (), expected.end ()), "Reconnect retained old data");
  peer->pending = 0;
  current_time = 19999;
  server.exec ();
  peer->deliver (query.data (), query.size ());
  server.exec ();
  current_time = 20000;
  peer->pending -= 1;
  server.exec ();
  current_time = 29999;
  server.exec ();
  require (peer->state () == net::tcp_socket::connected, "ACK did not refresh progress deadline");
  current_time = 30000;
  server.exec ();
  require (peer->state () == net::tcp_socket::closing, "ACK deadline did not expire");
  peer->complete_close ();
  server.exec ();
}
}

int main ()
{
  try
  {
    test_adapter ();
    std::cout << "PASS: adapter partial sends, receive backpressure, async close, reconnect, timeouts\n";
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << "FAIL: " << error.what () << '\n';
    return 1;
  }
}

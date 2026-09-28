#include "mcb_binary_server.hpp"

#include <chrono>

namespace mcb_binary
{

uint64_t server::now_ms (void)
{
  return static_cast<uint64_t> (std::chrono::duration_cast<std::chrono::milliseconds> (
      std::chrono::high_resolution_clock::now ().time_since_epoch ()).count ());
}

server::server (const read_only_backend& backend, uint16_t port, clock_func clock)
: m_backend (backend), m_clock (clock), m_listener (port, [this] (std::unique_ptr<net::tcp_socket> socket)
  {
    accept (std::move (socket));
  })
{
}

server::~server ()
{
  release_socket ();
}

void server::release_socket (void)
{
  if (m_socket && m_socket->state () != net::tcp_socket::closed)
  {
    // close() is asynchronous in uIP. Keep the object alive for outstanding
    // retransmissions until the stack detaches it and reports closed.
    m_dead = true;
    m_socket->suspend_recv ();
    m_socket->close ();
    return;
  }
  if (m_socket)
  {
    m_socket->on_receive (net::tcp_socket::receive_func { });
    m_socket->on_close (net::tcp_socket::close_func { });
    m_socket->close ();
    m_socket.reset ();
  }
  m_dead = false;
  m_pending_count = 0;
  m_session.reset (m_clock ());
}

void server::accept (std::unique_ptr<net::tcp_socket> socket)
{
  if (m_socket)
  {
    socket->close ();
    return;
  }

  m_session.reset (m_clock ());
  m_dead = false;
  m_pending_count = 0;
  m_pending_since = m_clock ();
  m_socket = std::move (socket);
  m_socket->on_receive ([this] (net::tcp_socket*, const void* data, unsigned int size)
  {
    // uIP may deliver an ACK packet's unaccepted payload while UIP_STOPPED.
    // Those bytes will be retransmitted after resume, so do not enqueue them.
    if (m_dead || m_socket->recv_suspended ())
      return;
    if (!m_session.receive (data, size, m_clock ()))
      m_dead = true;
    if (m_dead || m_session.receive_space () < 1540)
      m_socket->suspend_recv ();
  });
  m_socket->on_close ([this] (net::tcp_socket*)
  {
    m_dead = true;
  });
}

void server::exec (void)
{
  if (!m_socket)
    return;
  if (m_dead)
  {
    release_socket ();
    return;
  }

  const uint64_t now = m_clock ();
  const unsigned int pending_before = m_socket->send_buffer_pending_count ();
  if (pending_before < m_pending_count)
    m_pending_since = now;
  if (pending_before != 0 && now - m_pending_since >= session::send_timeout_ms)
  {
    release_socket ();
    return;
  }

  m_session.exec (m_backend, now);
  if (m_session.should_close ())
  {
    release_socket ();
    return;
  }

  const unsigned int offered = static_cast<unsigned int> (m_session.output_size ());
  if (offered != 0)
  {
    const unsigned int accepted = m_socket->send (m_session.output_data (), offered);
    m_session.consume_output (accepted, now);
    if (pending_before == 0 && accepted != 0)
      m_pending_since = now;
  }
  m_socket->flush_send_buffer ();
  m_pending_count = m_socket->send_buffer_pending_count ();

  if (m_dead || m_session.should_close ())
  {
    release_socket ();
    return;
  }

  // uIP callback data is transient; reserve an entire incoming packet before
  // resuming, rather than assuming suspend_recv preserves a callback suffix.
  if (m_session.receive_space () >= 1540)
    m_socket->resume_recv ();
  else
    m_socket->suspend_recv ();
}

} // namespace mcb_binary

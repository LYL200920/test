#include "tcp_client.h"

#include <utility>

tcp_client::tcp_client()
    : m_socket(std::make_shared<tcp::socket>(m_ctx)), m_resolver(m_ctx)
{
}

tcp_client::~tcp_client()
{
  disconnect();
}

void tcp_client::set_status_callback(status_cb cb)
{
  std::lock_guard<std::mutex> lock(m_cb_mtx);
  m_status_cb = std::move(cb);
}

void tcp_client::set_recv_callback(recv_cb cb)
{
  std::lock_guard<std::mutex> lock(m_cb_mtx);
  m_recv_cb = std::move(cb);
}

void tcp_client::connect(const std::string &host, unsigned short port)
{
  if (m_connected)
    return;

  const auto generation = ++m_generation;
  m_stop = false;
  start_io_thread();

  asio::post(m_ctx, [this, self = shared_from_this(), generation, host, port]
             {
               if (!is_current(generation))
                 return;

               m_resolver.cancel();
               close_socket();
               clear_write_queue();
               // Composed operations retain the old socket until completion.
               m_socket = std::make_shared<tcp::socket>(m_ctx);
               m_host = host;
               m_port = port;
               do_resolve(generation);
             });
}

void tcp_client::disconnect()
{
  m_stop = true;
  ++m_generation;
  if (m_io_thread.joinable())
  {
    asio::post(m_ctx,
               [this]
               {
                 m_resolver.cancel();
                 close_socket();
                 clear_write_queue();
               });
  }
  else
  {
    close_socket();
    clear_write_queue();
  }

  m_connected = false;
  stop_io_thread();
}

void tcp_client::send(const std::string &msg)
{
  const auto generation = m_generation.load();
  if (!m_connected || msg.empty())
    return;

  asio::post(m_ctx,
             [this, self = shared_from_this(), generation, msg]
             {
               if (!is_current(generation) || !m_connected)
                 return;

               std::lock_guard<std::recursive_mutex> lock(m_write_mtx);
               m_write_queue.push(msg);
               if (!m_writing)
                 do_write(generation);
             });
}

bool tcp_client::is_connected() const
{
  return m_connected;
}

bool tcp_client::is_current(std::uint64_t generation) const
{
  return !m_stop && generation == m_generation.load();
}

void tcp_client::fail_connection(std::uint64_t generation, const std::string &info)
{
  if (!is_current(generation))
    return;

  ++m_generation;
  m_connected = false;
  m_resolver.cancel();
  close_socket();
  clear_write_queue();
  // Finish cleanup before callbacks are allowed to request a new connection.
  notify_status(false, info);
}

void tcp_client::do_resolve(std::uint64_t generation)
{
  if (!is_current(generation))
    return;

  auto self = shared_from_this();
  m_resolver.async_resolve(m_host, std::to_string(m_port),
                           [this, self, generation](std::error_code ec, tcp::resolver::results_type results)
                           {
                             if (!is_current(generation))
                               return;
                             if (ec)
                             {
                               fail_connection(generation, "Resolve failed: " + ec.message());
                               return;
                             }
                             do_connect(generation, results);
                           });
}

void tcp_client::do_connect(std::uint64_t generation, tcp::resolver::results_type endpoints)
{
  if (!is_current(generation))
    return;

  auto self = shared_from_this();
  auto socket = m_socket;
  asio::async_connect(*socket, endpoints,
                      [this, self, socket, generation](std::error_code ec, const tcp::endpoint &)
                      {
                        if (!is_current(generation))
                          return;
                        if (ec)
                        {
                          fail_connection(generation, "Connect failed: " + ec.message());
                          return;
                        }

                        // Robot-control messages are short and latency-sensitive.
                        // Disable Nagle aggregation once the connection is ready.
                        std::error_code option_ec;
                        socket->set_option(tcp::no_delay(true), option_ec);
                        if (option_ec)
                        {
                          fail_connection(generation, "TCP_NODELAY failed: " +
                                                          option_ec.message());
                          return;
                        }

                        m_connected = true;
                        notify_status(true, "Connected to " + m_host + ":" + std::to_string(m_port));
                        do_read(generation);
                      });
}

void tcp_client::do_read(std::uint64_t generation)
{
  if (!is_current(generation) || !m_connected)
    return;

  auto self = shared_from_this();
  auto socket = m_socket;
  auto buffer = std::make_shared<std::array<char, 4096>>();
  socket->async_read_some(asio::buffer(*buffer),
                           [this, self, socket, buffer, generation](std::error_code ec, std::size_t bytes)
                           {
                             if (!is_current(generation))
                               return;
                             if (ec)
                             {
                               fail_connection(generation,
                                               ec == asio::error::eof
                                                   ? "Disconnected (EOF)"
                                                   : "Read error: " + ec.message());
                               return;
                             }

                             if (bytes > 0)
                             {
                               std::string msg(buffer->data(), bytes);
                               notify_recv(msg);
                             }

                             do_read(generation);
                           });
}

void tcp_client::do_write(std::uint64_t generation)
{
  if (!is_current(generation) || !m_connected)
    return;

  std::lock_guard<std::recursive_mutex> lock(m_write_mtx);
  if (m_write_queue.empty())
  {
    m_writing = false;
    return;
  }

  m_writing = true;
  // Queue cleanup must not invalidate buffers still owned by async_write.
  auto buffer = std::make_shared<std::string>(std::move(m_write_queue.front()));
  m_write_queue.pop();

  auto self = shared_from_this();
  auto socket = m_socket;
  asio::async_write(*socket, asio::buffer(*buffer),
                    [this, self, socket, buffer, generation](std::error_code ec, std::size_t)
                    {
                      if (!is_current(generation))
                        return;
                      if (ec)
                      {
                        fail_connection(generation, "Write error: " + ec.message());
                        return;
                      }

                      do_write(generation);
                    });
}

void tcp_client::close_socket()
{
  std::error_code ec;
  m_socket->shutdown(tcp::socket::shutdown_both, ec);
  m_socket->close(ec);
}

void tcp_client::clear_write_queue()
{
  std::lock_guard<std::recursive_mutex> lock(m_write_mtx);
  std::queue<std::string> empty;
  m_write_queue.swap(empty);
  m_writing = false;
}

void tcp_client::notify_status(bool ok, const std::string &info)
{
  status_cb cb;
  {
    std::lock_guard<std::mutex> lock(m_cb_mtx);
    cb = m_status_cb;
  }
  if (cb)
    cb(ok, info);
}

void tcp_client::notify_recv(const std::string &msg)
{
  recv_cb cb;
  {
    std::lock_guard<std::mutex> lock(m_cb_mtx);
    cb = m_recv_cb;
  }
  if (cb)
    cb(msg);
}

void tcp_client::start_io_thread()
{
  if (m_io_thread.joinable())
    return;

  m_work_guard.emplace(asio::make_work_guard(m_ctx));
  m_io_thread = std::thread(
      [this]
      {
        try
        {
          m_ctx.run();
        }
        catch (const std::exception &)
        {
          // ignore
        }
      });
}

void tcp_client::stop_io_thread()
{
  m_work_guard.reset();

  if (m_io_thread.joinable())
    m_io_thread.join();

  m_ctx.stop();
  m_ctx.restart();
}

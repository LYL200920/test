#include "tcp_client.h"

#include <utility>

tcp_client::tcp_client()
    : m_socket(m_ctx), m_resolver(m_ctx)
{
  m_read_buf.fill(0);
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

  m_host = host;
  m_port = port;
  m_stop = false;

  start_io_thread();

  asio::post(m_ctx, [this, self = shared_from_this()]
             { do_resolve(); });
}

void tcp_client::disconnect()
{
  m_stop = true;
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
  if (!m_connected || msg.empty())
    return;

  asio::post(m_ctx,
             [this, self = shared_from_this(), msg]
             {
               std::lock_guard<std::recursive_mutex> lock(m_write_mtx);
               m_write_queue.push(msg);
               if (!m_writing)
                 do_write();
             });
}

bool tcp_client::is_connected() const
{
  return m_connected;
}

void tcp_client::do_resolve()
{
  if (m_stop)
    return;

  auto self = shared_from_this();
  m_resolver.async_resolve(m_host, std::to_string(m_port),
                           [this, self](std::error_code ec, tcp::resolver::results_type results)
                           {
                             if (ec)
                             {
                               if (!m_stop)
                                 notify_status(false, "Resolve failed: " + ec.message());
                               return;
                             }
                             do_connect(results);
                           });
}

void tcp_client::do_connect(tcp::resolver::results_type endpoints)
{
  if (m_stop)
    return;

  auto self = shared_from_this();
  asio::async_connect(m_socket, endpoints,
                      [this, self](std::error_code ec, const tcp::endpoint &)
                      {
                        if (ec)
                        {
                          if (!m_stop)
                            notify_status(false, "Connect failed: " + ec.message());
                          return;
                        }

                        if (m_stop)
                        {
                          close_socket();
                          return;
                        }

                        m_connected = true;
                        notify_status(true, "Connected to " + m_host + ":" + std::to_string(m_port));
                        do_read();
                      });
}

void tcp_client::do_read()
{
  if (!m_connected || m_stop)
    return;

  auto self = shared_from_this();
  m_socket.async_read_some(asio::buffer(m_read_buf),
                           [this, self](std::error_code ec, std::size_t bytes)
                           {
                             if (ec)
                             {
                               m_connected = false;
                               if (!m_stop)
                               {
                                 if (ec == asio::error::eof)
                                   notify_status(false, "Disconnected (EOF)");
                                 else
                                   notify_status(false, "Read error: " + ec.message());
                               }
                               close_socket();
                               return;
                             }

                             if (bytes > 0)
                             {
                               std::string msg(m_read_buf.data(), bytes);
                               notify_recv(msg);
                             }

                             do_read();
                           });
}

void tcp_client::do_write()
{
  if (!m_connected || m_stop)
    return;

  std::lock_guard<std::recursive_mutex> lock(m_write_mtx);
  if (m_write_queue.empty())
  {
    m_writing = false;
    return;
  }

  m_writing = true;
  m_current_write = std::move(m_write_queue.front());
  m_write_queue.pop();

  auto self = shared_from_this();
  asio::async_write(m_socket, asio::buffer(m_current_write),
                    [this, self](std::error_code ec, std::size_t)
                    {
                      m_current_write.clear();
                      if (ec)
                      {
                        m_connected = false;
                        if (!m_stop)
                          notify_status(false, "Write error: " + ec.message());
                        close_socket();
                        return;
                      }

                      do_write();
                    });
}

void tcp_client::close_socket()
{
  std::error_code ec;
  m_socket.shutdown(tcp::socket::shutdown_both, ec);
  m_socket.close(ec);
}

void tcp_client::clear_write_queue()
{
  std::lock_guard<std::recursive_mutex> lock(m_write_mtx);
  std::queue<std::string> empty;
  m_write_queue.swap(empty);
  m_current_write.clear();
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

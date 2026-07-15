#ifndef includeguard_tcp_client_h_includeguard
#define includeguard_tcp_client_h_includeguard

#include <asio/asio.hpp>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>

using asio::ip::tcp;

class tcp_client : public std::enable_shared_from_this<tcp_client>
{
public:
  using status_cb = std::function<void(bool connected, const std::string &info)>;
  using recv_cb = std::function<void(const std::string &msg)>;

  tcp_client();
  ~tcp_client();

  void set_status_callback(status_cb cb);
  void set_recv_callback(recv_cb cb);

  void connect(const std::string &host, unsigned short port);
  void disconnect();

  void send(const std::string &msg);

  bool is_connected() const;

private:
  void do_resolve();
  void do_connect(tcp::resolver::results_type endpoints);
  void do_read();
  void do_write();
  void close_socket();
  void clear_write_queue();
  void notify_status(bool ok, const std::string &info);
  void notify_recv(const std::string &msg);
  void start_io_thread();
  void stop_io_thread();

private:
  asio::io_context m_ctx;
  std::optional<asio::executor_work_guard<asio::io_context::executor_type>> m_work_guard;
  std::thread m_io_thread;

  tcp::socket m_socket;
  tcp::resolver m_resolver;

  std::string m_host;
  unsigned short m_port = 0;

  std::atomic<bool> m_connected{false};
  std::atomic<bool> m_stop{false};

  status_cb m_status_cb;
  recv_cb m_recv_cb;
  std::mutex m_cb_mtx;

  std::queue<std::string> m_write_queue;
  std::recursive_mutex m_write_mtx;
  bool m_writing = false;
  std::string m_current_write;

  std::array<char, 4096> m_read_buf;
};

#endif

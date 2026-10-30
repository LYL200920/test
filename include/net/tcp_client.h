#ifndef includeguard_tcp_client_h_includeguard
#define includeguard_tcp_client_h_includeguard

#include <asio/asio.hpp>
#include <array>
#include <atomic>
#include <cstdint>
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
  friend struct tcp_client_test_access;

  void do_resolve(std::uint64_t generation);
  void do_connect(std::uint64_t generation, tcp::resolver::results_type endpoints);
  void do_read(std::uint64_t generation);
  void do_write(std::uint64_t generation);
  bool is_current(std::uint64_t generation) const;
  void fail_connection(std::uint64_t generation, const std::string &info);
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

  std::shared_ptr<tcp::socket> m_socket;
  tcp::resolver m_resolver;

  std::string m_host;
  unsigned short m_port = 0;

  std::atomic<bool> m_connected{false};
  std::atomic<bool> m_stop{false};
  std::atomic<std::uint64_t> m_generation{0};

  status_cb m_status_cb;
  recv_cb m_recv_cb;
  std::mutex m_cb_mtx;

  std::queue<std::string> m_write_queue;
  std::recursive_mutex m_write_mtx;
  bool m_writing = false;
};

#endif

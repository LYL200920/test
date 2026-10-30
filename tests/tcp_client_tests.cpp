#include "tcp_client.h"

#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>

// Drive the write-error path without a concurrent read racing to report it first.
struct tcp_client_test_access
{
  static bool fail_write(const std::shared_ptr<tcp_client> &client)
  {
    client->m_connected = true;
    client->m_write_queue.push(std::string(65536, 'x'));
    client->m_write_queue.push("discard this queued message");
    client->do_write(client->m_generation.load());
    client->m_ctx.run();
    client->m_ctx.restart();
    return !client->m_writing && client->m_write_queue.empty();
  }

  static std::future<bool> writing(const std::shared_ptr<tcp_client> &client)
  {
    auto result = std::make_shared<std::promise<bool>>();
    auto future = result->get_future();
    asio::post(client->m_ctx, [client, result]
               { result->set_value(client->m_writing); });
    return future;
  }
};

namespace
{
using namespace std::chrono_literals;

void Require(bool condition, const char *message)
{
  if (!condition)
    throw std::runtime_error(message);
}

template <typename Predicate>
void Wait(Predicate predicate, const char *message)
{
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!predicate())
  {
    Require(std::chrono::steady_clock::now() < deadline, message);
    std::this_thread::sleep_for(1ms);
  }
}

struct Client
{
  std::shared_ptr<tcp_client> value = std::make_shared<tcp_client>();
  ~Client() { value->disconnect(); }
};

struct Server
{
  asio::io_context context;
  tcp::acceptor acceptor{context, tcp::endpoint(asio::ip::address_v4::loopback(), 0)};

  Server()
  {
    acceptor.set_option(asio::socket_base::receive_buffer_size(4096));
    acceptor.non_blocking(true);
  }

  unsigned short port() const { return acceptor.local_endpoint().port(); }

  std::shared_ptr<tcp::socket> accept()
  {
    auto socket = std::make_shared<tcp::socket>(context);
    Wait([&]
         {
           std::error_code ec;
           acceptor.accept(*socket, ec);
           if (ec == asio::error::would_block || ec == asio::error::try_again)
             return false;
           Require(!ec, "Loopback accept failed");
           return true;
         }, "Timed out accepting client");
    socket->non_blocking(true);
    return socket;
  }

  std::string read(tcp::socket &socket, std::size_t size)
  {
    std::string result;
    Wait([&]
         {
           std::array<char, 4096> buffer;
           std::error_code ec;
           const auto count = socket.read_some(asio::buffer(buffer), ec);
           if (ec == asio::error::would_block || ec == asio::error::try_again)
             return false;
           Require(!ec, "Loopback read failed");
           result.append(buffer.data(), count);
           return result.size() >= size;
         }, "Timed out reading client data");
    return result;
  }
};

void TestWriteFailureThenReconnect()
{
  std::atomic<int> errors{0};
  std::atomic<bool> write_error{false};
  Client client;
  client.value->set_status_callback([&](bool ok, const std::string &info)
                                   {
                                     if (!ok)
                                     {
                                       write_error = info.find("Write error:") == 0;
                                       ++errors;
                                     }
                                   });
  Require(tcp_client_test_access::fail_write(client.value),
          "Write failure did not reset writing state and queue");
  Require(write_error && errors == 1 && !client.value->is_connected(),
          "Write failure did not report one disconnect");

  Server server;
  client.value->connect("127.0.0.1", server.port());
  auto socket = server.accept();
  Wait([&] { return client.value->is_connected(); }, "Reconnect failed");
  client.value->send("after write failure");
  Require(server.read(*socket, 19) == "after write failure",
          "Reconnect retained stale data or could not write");
}

void TestResetDuringWriteThenReconnect()
{
  Server server;
  std::atomic<int> connected{0};
  std::atomic<int> disconnected{0};
  std::atomic<bool> reply_received{false};
  std::string reply;
  Client client;
  client.value->set_status_callback([&](bool ok, const std::string &)
                                   {
                                     if (ok)
                                       ++connected;
                                     else
                                     {
                                       if (++disconnected == 1)
                                         client.value->connect("127.0.0.1", server.port());
                                     }
                                   });
  client.value->set_recv_callback([&](const std::string &msg)
                                 {
                                   reply += msg;
                                   reply_received = reply == "new connection reply";
                                 });
  client.value->connect("127.0.0.1", server.port());
  auto first = server.accept();
  Wait([&] { return connected == 1; }, "Initial connection failed");

  client.value->send(std::string(16 * 1024 * 1024, 'x'));
  client.value->send("old queued message");
  auto writing = tcp_client_test_access::writing(client.value);
  Require(writing.wait_for(5s) == std::future_status::ready && writing.get(),
          "Test did not establish an outstanding write");
  first->set_option(asio::socket_base::linger(true, 0));
  first->close();

  auto second = server.accept();
  Wait([&] { return connected == 2; }, "Reconnect from failure callback failed");
  client.value->send("new connection data");
  Require(server.read(*second, 19) == "new connection data",
          "Old write callback corrupted the new connection or replayed data");
  asio::write(*second, asio::buffer("new connection reply", 20));
  Wait([&] { return reply_received.load(); }, "New connection read failed");
  Require(client.value->is_connected() && disconnected == 1,
          "Canceled old operations disconnected the new connection");
  client.value->disconnect();
}

void TestDisconnectDuringWriteThenReconnect()
{
  Server server;
  std::atomic<int> errors{0};
  Client client;
  client.value->set_status_callback([&](bool ok, const std::string &)
                                   { if (!ok) ++errors; });
  client.value->connect("127.0.0.1", server.port());
  auto first = server.accept();
  Wait([&] { return client.value->is_connected(); }, "Initial connection failed");
  client.value->send(std::string(16 * 1024 * 1024, 'x'));
  auto writing = tcp_client_test_access::writing(client.value);
  Require(writing.wait_for(5s) == std::future_status::ready && writing.get(),
          "Test did not establish an outstanding write");
  client.value->disconnect();
  Require(!client.value->is_connected() && errors == 0,
          "Explicit disconnect reported a canceled operation as an error");

  client.value->connect("127.0.0.1", server.port());
  auto second = server.accept();
  Wait([&] { return client.value->is_connected(); }, "Explicit reconnect failed");
  client.value->send("after disconnect");
  Require(server.read(*second, 16) == "after disconnect",
          "Explicit reconnect could not write");
}

void TestEofThenReconnect()
{
  Server server;
  std::atomic<int> errors{0};
  Client client;
  client.value->set_status_callback([&](bool ok, const std::string &)
                                   { if (!ok) ++errors; });
  client.value->connect("127.0.0.1", server.port());
  auto first = server.accept();
  Wait([&] { return client.value->is_connected(); }, "Initial connection failed");
  first->shutdown(tcp::socket::shutdown_send);
  Wait([&] { return errors == 1; }, "EOF was not reported");
  Require(!client.value->is_connected(), "EOF left client connected");
  client.value->connect("127.0.0.1", server.port());
  auto second = server.accept();
  Wait([&] { return client.value->is_connected(); }, "Reconnect after EOF failed");
  client.value->send("after eof");
  Require(server.read(*second, 9) == "after eof", "Write after EOF failed");
}
} // namespace

int main()
{
  try
  {
    std::cout << "write failure" << std::endl;
    TestWriteFailureThenReconnect();
    std::cout << "reset during write" << std::endl;
    TestResetDuringWriteThenReconnect();
    std::cout << "disconnect during write" << std::endl;
    TestDisconnectDuringWriteThenReconnect();
    std::cout << "EOF" << std::endl;
    TestEofThenReconnect();
    std::cout << "tcp_client_tests passed\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

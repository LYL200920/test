#define ASIO_STANDALONE
#include <asio/asio.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

class send_telent
{
public:
  static send_telent& getInstance ()
  {
    static send_telent instance;
    return instance;
  }

  void send_reset ()
  {
    sendCommand (socket, "kuka reset");
    receiveResponse (socket, 2000);
  }

  void send_auto ()
  {
    sendCommand (socket, "kuka auto");
    receiveResponse (socket, 2000);
  }

  void send_start ()
  {
    sendCommand (socket, "kuka start");
    receiveResponse (socket, 2000);
  }

  void send_stop ()
  {
    sendCommand (socket, "kuka stop");
    receiveResponse (socket, 2000);
  }

  void re_connect ()
  {
    try
    {
      if (connect_with_timeout (io, socket, "10.2.1.80", "23", 1))
      {
        std::cout << "connect 10.2.1.80 success\n";
      }
      else
      {
        std::cout << "connect 10.2.1.80 fail\n";
      }
    }
    catch (const std::exception& e)
    {
      std::cerr << "Connection error: " << e.what () << "\n";
    }
  }

  ~send_telent ()
  {
    std::lock_guard<std::mutex> lock (mutex);
    if (socket.is_open ())
    {
      socket.close ();
    }
  }
private:
  std::vector<std::string> commands;
  asio::io_context io;
  asio::ip::tcp::socket socket;
  std::mutex mutex;

  send_telent ():socket (io)
  {
    re_connect ();
  }
  send_telent (const send_telent&) = delete;
  send_telent& operator=(const send_telent&) = delete;

  bool sendCommand (asio::ip::tcp::socket& socket, const std::string& command)
  {
    std::string cmd = command + "\r\n";
    asio::error_code ec;
    asio::write (socket, asio::buffer (cmd), ec);
    if (ec)
    {
      std::cerr << "send fail: " << ec.message () << "\n";
      return false;
    }
    std::cout << "send success: " << command << "\n";
    return true;
  }

  std::string receiveResponse (asio::ip::tcp::socket& socket, int timeout_ms = 1000)
  {
    std::string response;
    char buf[1024];
    asio::error_code ec;

    socket.non_blocking (true);
    auto start = std::chrono::steady_clock::now ();

    while (true)
    {
      std::size_t n = socket.read_some (asio::buffer (buf), ec);
      if (!ec)
      {
        response.append (buf, buf + n);
      }

      if ((std::chrono::steady_clock::now () - start) > std::chrono::milliseconds (timeout_ms))
      {
        break;
      }

      std::this_thread::sleep_for (std::chrono::milliseconds (50));
    }

    socket.non_blocking (false);
    std::cout << "response: \n" << response << "\n";
    return response;
  }

  bool connect_with_timeout (asio::io_context& io,
                             asio::ip::tcp::socket& socket,
                             const std::string& host,
                             const std::string& port,
                             int timeout_seconds)
  {
    asio::steady_timer timer (io);
    asio::error_code ec;
    bool connected = false;

    asio::ip::tcp::resolver resolver (io);
    auto endpoints = resolver.resolve (host, port, ec);
    if (ec)
    {
      std::cerr << "connect ip check fail: " << ec.message () << "\n";
      return false;
    }

    timer.expires_after (std::chrono::seconds (timeout_seconds));
    timer.async_wait ([&](const asio::error_code& error)
                      {
                        if (!error)
                        {
                          socket.close ();
                        }
                      });

    asio::async_connect (socket, endpoints, [&](const asio::error_code& error, const  asio::ip::tcp::endpoint&)
                         {
                           ec = error;
                           if (!error)
                           {
                             connected = true;
                           }
                           timer.cancel ();
                         });

    io.run ();
    io.restart (); 

    return connected;
  }

};

int main ()
{
  send_telent::getInstance ().send_reset ();
  send_telent::getInstance ().send_auto ();
  send_telent::getInstance ().send_start ();
  std::this_thread::sleep_for (std::chrono::seconds (2));
  send_telent::getInstance ().send_stop ();
  std::this_thread::sleep_for (std::chrono::seconds (2));
  send_telent::getInstance ().send_start ();
  std::this_thread::sleep_for (std::chrono::seconds (2));
  send_telent::getInstance ().send_stop ();
  std::this_thread::sleep_for (std::chrono::seconds (2));

  return 0;
}
#include "kuka_robot_service.h"

#include <asio/asio.hpp>

#include <chrono>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
void Require(bool condition, const char *message)
{
  if (!condition)
    throw std::runtime_error(message);
}

std::string Read_Frame(asio::ip::tcp::socket &socket,
                       asio::streambuf &buffer)
{
  asio::read_until(socket, buffer, ';');
  std::istream input(&buffer);
  std::string frame;
  std::getline(input, frame, ';');
  return frame + ';';
}

void Write(asio::ip::tcp::socket &socket, const std::string &frame)
{
  asio::write(socket, asio::buffer(frame));
}

bool Wait_For(const std::function<bool()> &condition,
              std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (condition())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return condition();
}
} // namespace

int main()
{
  std::thread server_thread;
  try
  {
    asio::io_context context;
    asio::ip::tcp::acceptor acceptor(
        context,
        asio::ip::tcp::endpoint(asio::ip::address_v4::loopback(), 0));
    const unsigned short port = acceptor.local_endpoint().port();

    std::string server_error;
    server_thread = std::thread(
        [&]
        {
          try
          {
            asio::ip::tcp::socket socket(context);
            acceptor.accept(socket);
            asio::streambuf buffer;

            Write(socket, "V1,A,0,DONE,0,CONNECTED;");
            const auto synchronize = Read_Frame(socket, buffer);
            Require(synchronize == "V1,C,1,GET_STATE;",
                    "Service did not automatically request state");
            Write(socket, "V1,A,1,DONE,0,OK;");
            Write(socket,
                  "V1,S,1,IDLE,0,"
                  "1,2,3,4,5,6,"
                  "100,200,300,10,20,30,"
                  "0,0,10;");

            const auto move = Read_Frame(socket, buffer);
            Require(move == "V1,C,2,MOVEJ,20,50,0,2,2,3,4,5,6;",
                    "Service MOVEJ frame mismatch");
            Write(socket, "V1,A,2,ACCEPTED,0,OK;");
            Write(socket, "V1,A,2,RUNNING,0,OK;");
            Write(socket, "V1,A,2,DONE,0,OK;");
            Write(socket,
                  "V1,S,2,IDLE,0,"
                  "2,2,3,4,5,6,"
                  "101,200,300,10,20,30,"
                  "0,0,10;");

            // The real controller keeps the TCP session open after DONE.
            // Wait for the test client to disconnect instead of turning the
            // valid Ready state immediately into Disconnected.
            std::array<char, 1> byte{};
            std::error_code disconnect_error;
            socket.read_some(asio::buffer(byte), disconnect_error);
          }
          catch (const std::exception &error)
          {
            server_error = error.what();
          }
        });

    kuka::Robot_Service service;
    std::mutex states_mutex;
    std::vector<kuka::Control_State> states;
    kuka::Service_Observer observer;
    observer.service_status =
        [&](const kuka::Service_Status &status)
        {
          std::lock_guard<std::mutex> lock(states_mutex);
          states.push_back(status.state);
        };
    const auto token = service.Subscribe(std::move(observer));
    service.Connect("127.0.0.1", port);

    Require(
        Wait_For([&] { return service.Can_Move(); },
                 std::chrono::milliseconds(2000)),
        "Service did not become ready after synchronization");

    kuka::Robot_State state;
    std::uint64_t revision = 0;
    Require(service.Latest_State(&state, &revision),
            "Synchronized state is unavailable");
    Require(revision == 1 && state.axis[0] == 1.0,
            "Initial synchronized state mismatch");

    kuka::Axis target = state.axis;
    target[0] = 2.0;
    const auto sequence = service.Move_Joint(target);
    Require(sequence == 2, "Unexpected MOVEJ sequence");
    Require(
        Wait_For(
            [&]
            {
              kuka::Robot_State updated;
              std::uint64_t updated_revision = 0;
              return service.Can_Move() &&
                     service.Latest_State(&updated, &updated_revision) &&
                     updated_revision == 2 && updated.axis[0] == 2.0;
            },
            std::chrono::milliseconds(2000)),
        "Service did not complete the MOVEJ state cycle");

    bool saw_running = false;
    {
      std::lock_guard<std::mutex> lock(states_mutex);
      for (const auto observed : states)
        saw_running = saw_running ||
                      observed == kuka::Control_State::Running;
    }
    Require(saw_running, "Service never reported Running");

    service.Unsubscribe(token);
    service.Disconnect();
    server_thread.join();
    Require(server_error.empty(), server_error.c_str());
  }
  catch (const std::exception &error)
  {
    if (server_thread.joinable())
      server_thread.join();
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }

  std::cout << "KUKA robot service tests passed.\n";
  return 0;
}

#include "kuka_robot_service.h"

#include <asio/asio.hpp>

#include <atomic>
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
    std::atomic<bool> move_received{false};
    std::atomic<bool> sent_before_notification{false};
    std::atomic<bool> reentrant_move_rejected{false};
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
            move_received = true;
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

            const auto plc_rotation = Read_Frame(socket, buffer);
            Require(plc_rotation == "V1,C,9001,PLC_ROTATION;",
                    "Service PLC_ROTATION frame mismatch");
            Write(socket, "V1,A,9001,ACCEPTED,0,OK;");
            Write(socket, "V1,A,9001,RUNNING,0,OK;");
            Write(socket, "V1,A,9001,DONE,0,OK;");

            Require(Read_Frame(socket, buffer) == "V1,C,4,STOP,CONTROLLED;",
                    "First STOP frame mismatch");
            Write(socket, "V1,A,4,DONE,0,STOPPED;");
            Require(Read_Frame(socket, buffer) == "V1,C,5,STOP,CONTROLLED;",
                    "Second STOP frame mismatch");
            Write(socket, "V1,A,5,FAILED,1,STOP_FAILED;");
            Require(Read_Frame(socket, buffer) == "V1,C,6,STOP,CONTROLLED;",
                    "Delayed STOP frame mismatch");
            Require(Read_Frame(socket, buffer) ==
                        "V1,C,7,MOVEJ,20,50,0,2,2,3,4,5,6;",
                    "New motion after delayed STOP mismatch");
            Write(socket, "V1,A,7,RUNNING,0,OK;");
            Write(socket, "V1,A,6,DONE,0,STOPPED;");
            Write(socket, "V1,A,6,FAILED,1,DUPLICATE;");
            Require(Read_Frame(socket, buffer) == "V1,C,8,STOP,CONTROLLED;",
                    "Overlapping STOP frame mismatch");
            Require(Read_Frame(socket, buffer) == "V1,C,9,STOP,CONTROLLED;",
                    "Latest STOP frame mismatch");
            Write(socket, "V1,A,8,FAILED,1,OLD_STOP_FAILED;");
            Write(socket, "V1,A,9,DONE,0,STOPPED;");
            // Wait until the client checks that STOP DONE did not turn Running
            // into Ready. PING is only a test barrier, not a stop confirmation.
            Require(Read_Frame(socket, buffer) == "V1,C,9002,PING;",
                    "STOP verification barrier mismatch");
            Write(socket, "V1,A,7,DONE,0,OK;");
            Write(socket,
                  "V1,S,7,IDLE,0,2,2,3,4,5,6,101,200,300,10,20,30,0,0,10;");

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
    std::vector<kuka::Service_Status> stop_statuses;
    std::vector<kuka::Acknowledgement> acknowledgements;
    kuka::Service_Observer observer;
    observer.service_status =
        [&](const kuka::Service_Status &status)
        {
          {
            std::lock_guard<std::mutex> lock(states_mutex);
            states.push_back(status.state);
            if (status.stop_result != kuka::Stop_Result::None)
              stop_statuses.push_back(status);
          }
          if (status.state == kuka::Control_State::Command_Sent &&
              status.active_sequence == 2)
          {
            // Keep the submitter here long enough for the loopback server to
            // reply if the command was incorrectly sent before publication.
            sent_before_notification = Wait_For(
                [&] { return move_received.load(); },
                std::chrono::milliseconds(100));
            try
            {
              service.Move_Joint(kuka::Axis{});
            }
            catch (const std::runtime_error &)
            {
              reentrant_move_rejected = true;
            }
          }
        };
    observer.acknowledgement =
        [&](const kuka::Acknowledgement &acknowledgement)
        {
          std::lock_guard<std::mutex> lock(states_mutex);
          acknowledgements.push_back(acknowledgement);
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
    Require(!sent_before_notification,
            "Command reached the wire before Command_Sent notification");
    Require(reentrant_move_rejected,
            "Command_Sent observer was allowed to submit another motion");
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

    service.Send_Raw("V1,C,9001,PLC_ROTATION");
    Require(
        Wait_For(
            [&]
            {
              std::lock_guard<std::mutex> lock(states_mutex);
              return !acknowledgements.empty() &&
                     acknowledgements.back().sequence == 9001 &&
                     acknowledgements.back().state ==
                         kuka::Acknowledgement_State::Done;
            },
            std::chrono::milliseconds(2000)),
        "Service did not complete the PLC_ROTATION acknowledgement cycle");

    bool saw_sent = false;
    bool saw_running = false;
    bool saw_completed = false;
    {
      std::lock_guard<std::mutex> lock(states_mutex);
      for (const auto observed : states)
      {
        if (observed == kuka::Control_State::Command_Sent)
        {
          Require(!saw_running && !saw_completed,
                  "Command_Sent overwrote a later motion state");
          saw_sent = true;
        }
        if (observed == kuka::Control_State::Running)
        {
          Require(saw_sent, "Running preceded Command_Sent");
          saw_running = true;
        }
        if (observed == kuka::Control_State::Completed)
        {
          Require(saw_running, "Completed preceded Running");
          saw_completed = true;
        }
      }
    }
    Require(saw_completed, "Service never reported Completed");

    const auto wait_for_stop = [&](std::uint32_t expected,
                                   kuka::Stop_Result result)
    {
      return Wait_For([&]
      {
        const auto status = service.Status();
        return status.stop_sequence == expected && status.stop_result == result;
      }, std::chrono::milliseconds(2000));
    };
    Require(service.Request_Stop() == 4 &&
                wait_for_stop(4, kuka::Stop_Result::Done),
            "Fast STOP DONE was not correlated/published");
    Require(service.Can_Move(), "STOP DONE replaced the existing Ready state");
    Require(service.Request_Stop() == 5 &&
                wait_for_stop(5, kuka::Stop_Result::Failed),
            "STOP FAILED was not correlated/published");
    Require(service.Can_Move(), "STOP FAILED overwrote unrelated motion state");
    Require(service.Request_Stop() == 6, "Delayed STOP sequence mismatch");
    Require(service.Move_Joint(target) == 7, "New motion sequence mismatch");
    Require(wait_for_stop(6, kuka::Stop_Result::Done), "Late STOP DONE not observed");
    Require(service.Status().state == kuka::Control_State::Running &&
                service.Status().active_sequence == 7 && !service.Can_Move(),
            "Old STOP ACK overwrote a newer running motion");
    Require(service.Request_Stop() == 8 && service.Request_Stop() == 9,
            "Overlapping STOP sequences mismatch");
    Require(wait_for_stop(9, kuka::Stop_Result::Done), "Latest STOP result missing");
    Require(service.Status().state == kuka::Control_State::Running &&
                service.Status().active_sequence == 7 && !service.Can_Move(),
            "STOP DONE incorrectly confirmed physical stop / Ready");
    {
      std::lock_guard<std::mutex> lock(states_mutex);
      for (const auto &status : stop_statuses)
        Require(status.stop_sequence != 8 &&
                    !(status.stop_sequence == 6 &&
                      status.stop_result == kuka::Stop_Result::Failed),
                "Superseded or duplicate STOP terminal ACK changed its result");
    }
    service.Send_Raw("V1,C,9002,PING;");
    Require(Wait_For([&] { return service.Can_Move(); },
                     std::chrono::milliseconds(2000)),
            "Motion did not finish after STOP test barrier");

    kuka::Joint_Motion_Options invalid_options;
    invalid_options.velocity_percent = 0.0;
    bool invalid_rejected = false;
    try
    {
      service.Move_Joint(target, invalid_options);
    }
    catch (const std::invalid_argument &)
    {
      invalid_rejected = true;
    }
    Require(invalid_rejected && service.Can_Move(),
            "Invalid motion changed the ready state");

    service.Unsubscribe(token);

    kuka::Service_Observer failing_observer;
    failing_observer.service_status = [](const kuka::Service_Status &status)
    {
      if (status.state == kuka::Control_State::Command_Sent)
        throw std::runtime_error("Submission observer failed");
    };
    const auto failing_token = service.Subscribe(std::move(failing_observer));
    bool submission_failed = false;
    try
    {
      service.Move_Joint(target);
    }
    catch (const std::runtime_error &)
    {
      submission_failed = true;
    }
    Require(submission_failed &&
                service.Status().state == kuka::Control_State::Fault &&
                service.Status().active_sequence == 0,
            "Failed submission left the service waiting for a reply");
    service.Unsubscribe(failing_token);
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

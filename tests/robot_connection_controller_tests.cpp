#include "robot_connection_controller.h"

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
using namespace application;

void Require(bool condition, const char *message)
{
  if( !condition ) throw std::runtime_error(message);
}

class Fake_Robot_Backend final : public Robot_Connection_Backend
{
public:
  void Set_Observer(Robot_Backend_Observer value) override
  {
    observer = std::move(value);
  }
  void Connect(const std::string &value, unsigned short port_value) override
  {
    host = value;
    port = port_value;
    connect_count++;
  }
  void Disconnect() override
  {
    connected = false;
    disconnect_count++;
  }
  bool Is_Connected() const override { return connected; }
  bool Can_Move() const override
  {
    return status.state == Robot_Control_State::Ready;
  }
  Robot_Connection_Status Status() const override { return status; }
  std::uint32_t Synchronize() override { return 10; }
  std::uint32_t Move_Joint(
    const Robot_Axis &value,
    const Robot_Joint_Motion_Options &options) override
  {
    last_axis = value;
    last_joint_options = options;
    return 11;
  }
  std::uint32_t Move_Pose_Ptp(
    const Robot_Pose &value,
    const Robot_Cartesian_Motion_Options &options) override
  {
    last_pose = value;
    last_cartesian_options = options;
    return 12;
  }
  std::uint32_t Move_Pose_Ptp(
    const Robot_Pose &value,
    const Robot_Axis &joint_solution,
    const Robot_Cartesian_Motion_Options &options) override
  {
    last_pose = value;
    last_axis = joint_solution;
    last_cartesian_options = options;
    return 13;
  }
  std::uint32_t Move_Linear(
    const Robot_Pose &value,
    const Robot_Cartesian_Motion_Options &options) override
  {
    last_pose = value;
    last_cartesian_options = options;
    return 14;
  }
  std::uint32_t Request_Stop() override { return 15; }
  void Send_Raw(std::string value) override { last_frame = std::move(value); }

  void Emit_Connection(bool value, const std::string &detail)
  {
    connected = value;
    if( observer.connection ) observer.connection(value, detail);
  }
  void Emit_Status(Robot_Connection_Status value)
  {
    status = std::move(value);
    if( observer.status ) observer.status(status);
  }
  void Emit_State(const Robot_Actual_State &value, std::uint64_t revision)
  {
    if( observer.robot_state ) observer.robot_state(value, revision);
  }
  void Emit_Log(const std::string &value)
  {
    if( observer.event_log ) observer.event_log(value);
  }

  Robot_Backend_Observer observer;
  Robot_Connection_Status status;
  bool connected = false;
  int connect_count = 0;
  int disconnect_count = 0;
  std::string host;
  unsigned short port = 0;
  Robot_Axis last_axis{};
  Robot_Pose last_pose{};
  Robot_Joint_Motion_Options last_joint_options;
  Robot_Cartesian_Motion_Options last_cartesian_options;
  std::string last_frame;
};

void Test_Connection_And_Observer_State()
{
  auto backend = std::make_shared<Fake_Robot_Backend>();
  Robot_Connection_Controller controller(backend);
  bool connection_notified = false;
  int status_notifications = 0;
  std::string log;
  Robot_Connection_Observer observer;
  observer.connection = [&](bool connected, const std::string &)
  {
    connection_notified = connected;
  };
  observer.status = [&](const Robot_Connection_Status &)
  {
    ++status_notifications;
  };
  observer.event_log = [&](const std::string &value) { log = value; };
  const auto token = controller.Subscribe(std::move(observer));

  controller.Connect("10.0.0.5", 1234);
  Require(controller.Is_Connecting(), "connecting state");
  Require(backend->host == "10.0.0.5" && backend->port == 1234,
          "backend connection arguments");
  backend->Emit_Connection(true, "connected");
  Require(connection_notified, "connection notification");
  Require(!controller.Is_Connecting(), "connection completed");

  backend->Emit_Status({Robot_Control_State::Ready, 7, "ready"});
  Require(controller.Can_Move(), "ready can move");
  Require(controller.Status().active_sequence == 7, "cached status");
  Require(status_notifications == 1, "status notification");
  backend->Emit_Log("ACK seq=7");
  Require(log == "ACK seq=7", "event log forwarding");

  controller.Unsubscribe(token);
  backend->Emit_Log("ignored");
  Require(log == "ACK seq=7", "observer unsubscribe");
}

void Test_State_Cache_And_Command_Forwarding()
{
  auto backend = std::make_shared<Fake_Robot_Backend>();
  Robot_Connection_Controller controller(backend);
  Robot_Actual_State state;
  state.axis = {1, 2, 3, 4, 5, 6};
  state.pose = {10, 20, 30, 40, 50, 60};
  backend->Emit_State(state, 2);

  Robot_Actual_State cached;
  std::uint64_t revision = 0;
  Require(controller.Latest_State(&cached, &revision), "latest state exists");
  Require(revision == 2 && cached.pose[5] == 60, "latest state contents");
  Robot_Actual_State stale = state;
  stale.pose[5] = 99;
  backend->Emit_State(stale, 1);
  controller.Latest_State(&cached, &revision);
  Require(cached.pose[5] == 60, "stale state ignored");

  Robot_Joint_Motion_Options joint_options;
  joint_options.velocity_percent = 35;
  Require(controller.Move_Joint(state.axis, joint_options) == 11,
          "joint sequence");
  Require(backend->last_axis[0] == 1 &&
          backend->last_joint_options.velocity_percent == 35,
          "joint forwarding");
  Robot_Cartesian_Motion_Options cartesian_options;
  cartesian_options.velocity = 450;
  Require(controller.Move_Linear(state.pose, cartesian_options) == 14,
          "linear sequence");
  Require(backend->last_pose[0] == 10 &&
          backend->last_cartesian_options.velocity == 450,
          "linear forwarding");
  Require(controller.Request_Stop() == 15, "stop forwarding");
  Robot_Connection_Status stop_status{Robot_Control_State::Running, 14, "moving"};
  stop_status.stop_sequence = 15;
  stop_status.stop_result = application::Robot_Stop_Result::Done;
  stop_status.stop_detail = "STOPPED";
  backend->Emit_Status(stop_status);
  Require(controller.Status().stop_sequence == 15 &&
            controller.Status().stop_result == application::Robot_Stop_Result::Done &&
            controller.Status().stop_detail == "STOPPED" &&
            controller.Status().state == Robot_Control_State::Running,
          "STOP result must propagate independently of motion state");
  controller.Send_Raw("PING");
  Require(backend->last_frame == "PING", "raw forwarding");
}
void Test_Unsubscribe_Invalidates_Snapshot_And_All_Callbacks()
{
  auto backend = std::make_shared<Fake_Robot_Backend>();
  Robot_Connection_Controller controller(backend);
  Robot_Connection_Controller::Observer_Token first = 0;
  Robot_Connection_Controller::Observer_Token second = 0;
  int calls = 0;
  auto cancel = [&]
  {
    ++calls;
    controller.Unsubscribe(first);
    controller.Unsubscribe(second);
  };
  Robot_Connection_Observer observer;
  observer.event_log = [&](const std::string &) { cancel(); };
  observer.connection = [&](bool, const std::string &) { cancel(); };
  observer.status = [&](const Robot_Connection_Status &) { cancel(); };
  observer.robot_state = [&](const Robot_Actual_State &, std::uint64_t)
  {
    cancel();
  };
  // Whichever unordered-map entry runs first removes both entries. The other
  // token is already in the notification snapshot, but must not be invoked.
  first = controller.Subscribe(observer);
  second = controller.Subscribe(observer);
  backend->Emit_Log("cancel both");
  backend->Emit_Connection(true, "ignored");
  backend->Emit_Status({Robot_Control_State::Ready, 0, "ignored"});
  backend->Emit_State({}, 1);
  Require(calls == 1, "unsubscribe invalidates copied notification entries");
}

void Test_Self_Unsubscribe_And_Reentrant_Notification()
{
  auto backend = std::make_shared<Fake_Robot_Backend>();
  Robot_Connection_Controller controller(backend);
  Robot_Connection_Controller::Observer_Token token = 0;
  int calls = 0;
  Robot_Connection_Observer observer;
  observer.event_log = [&](const std::string &)
  {
    ++calls;
    controller.Unsubscribe(token);
    backend->Emit_Log("nested after cancellation");
    // The callback object must still exist after erasing its registration.
    ++calls;
  };
  token = controller.Subscribe(std::move(observer));
  backend->Emit_Log("self cancel");
  Require(calls == 2, "self unsubscribe preserves current callable only");
}

void Test_Unsubscribe_Waits_For_In_Flight_Callback()
{
  using namespace std::chrono_literals;
  auto backend = std::make_shared<Fake_Robot_Backend>();
  Robot_Connection_Controller controller(backend);
  std::promise<void> entered;
  std::promise<void> release;
  const auto released = release.get_future().share();
  std::atomic<int> calls{0};
  Robot_Connection_Observer observer;
  observer.event_log = [&](const std::string &)
  {
    ++calls;
    entered.set_value();
    released.wait();
  };
  const auto token = controller.Subscribe(std::move(observer));
  const auto copied_backend = backend->observer;
  auto notification = std::async(std::launch::async, [&]
  {
    copied_backend.event_log("in flight");
  });
  entered.get_future().wait();
  std::promise<void> cancelling;
  auto cancellation = std::async(std::launch::async, [&]
  {
    cancelling.set_value();
    controller.Unsubscribe(token);
  });
  cancelling.get_future().wait();
  const bool waited = cancellation.wait_for(50ms) == std::future_status::timeout;
  release.set_value();
  notification.get();
  cancellation.get();
  copied_backend.event_log("after cancellation");
  Require(waited, "unsubscribe must wait for an in-flight callback");
  Require(calls == 1, "cancelled callback cannot restart");
}

void Test_Destruction_Invalidates_Copied_Backend_Callbacks()
{
  auto backend = std::make_shared<Fake_Robot_Backend>();
  int calls = 0;
  Robot_Backend_Observer copied;
  {
    Robot_Connection_Controller controller(backend);
    Robot_Connection_Observer observer;
    observer.event_log = [&](const std::string &) { ++calls; };
    controller.Subscribe(std::move(observer));
    copied = backend->observer;
  }
  copied.connection(true, "late connection");
  copied.status({Robot_Control_State::Ready, 0, "late status"});
  copied.robot_state({}, 1);
  copied.event_log("late log");
  Require(calls == 0, "backend snapshots cannot access destroyed controller");
  Require(backend->disconnect_count == 1, "destruction disconnects backend");
}

void Test_Destruction_Waits_For_In_Flight_Callback()
{
  using namespace std::chrono_literals;
  auto backend = std::make_shared<Fake_Robot_Backend>();
  auto controller = std::make_unique<Robot_Connection_Controller>(backend);
  std::promise<void> entered;
  std::promise<void> release;
  const auto released = release.get_future().share();
  Robot_Connection_Observer observer;
  observer.event_log = [&](const std::string &)
  {
    entered.set_value();
    released.wait();
  };
  controller->Subscribe(std::move(observer));
  const auto copied = backend->observer;
  auto notification = std::async(std::launch::async, [&]
  {
    copied.event_log("in flight during destruction");
  });
  entered.get_future().wait();
  std::promise<void> destroying;
  auto destruction = std::async(std::launch::async, [&]
  {
    destroying.set_value();
    controller.reset();
  });
  destroying.get_future().wait();
  const bool waited = destruction.wait_for(50ms) == std::future_status::timeout;
  release.set_value();
  notification.get();
  destruction.get();
  copied.event_log("after destruction");
  Require(waited, "controller destruction must drain active notifications");
}

void Test_Callback_Exception_Releases_Dispatch_Gate()
{
  auto backend = std::make_shared<Fake_Robot_Backend>();
  Robot_Connection_Controller controller(backend);
  Robot_Connection_Observer observer;
  observer.event_log = [](const std::string &)
  {
    throw std::runtime_error("observer failure");
  };
  const auto token = controller.Subscribe(std::move(observer));
  bool threw = false;
  try { backend->Emit_Log("throw"); }
  catch( const std::runtime_error & ) { threw = true; }
  auto cancellation = std::async(std::launch::async, [&]
  {
    controller.Unsubscribe(token);
  });
  cancellation.get();
  backend->Emit_Log("ignored");
  Require(threw, "callback exceptions propagate without retaining the gate");
}
} // namespace

int main()
{
  try
  {
    Test_Connection_And_Observer_State();
    Test_State_Cache_And_Command_Forwarding();
    Test_Unsubscribe_Invalidates_Snapshot_And_All_Callbacks();
    Test_Self_Unsubscribe_And_Reentrant_Notification();
    Test_Unsubscribe_Waits_For_In_Flight_Callback();
    Test_Destruction_Invalidates_Copied_Backend_Callbacks();
    Test_Destruction_Waits_For_In_Flight_Callback();
    Test_Callback_Exception_Releases_Dispatch_Gate();
  }
  catch( const std::exception &error )
  {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }
  std::cout << "Robot connection controller tests passed.\n";
  return 0;
}

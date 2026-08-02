#include "robot_connection_controller.h"

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
  controller.Send_Raw("PING");
  Require(backend->last_frame == "PING", "raw forwarding");
}
} // namespace

int main()
{
  try
  {
    Test_Connection_And_Observer_State();
    Test_State_Cache_And_Command_Forwarding();
  }
  catch( const std::exception &error )
  {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }
  std::cout << "Robot connection controller tests passed.\n";
  return 0;
}

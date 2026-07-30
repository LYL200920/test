#include "kuka_robot_service.h"

#include <stdexcept>
#include <utility>
#include <vector>

namespace kuka
{
Robot_Service::Robot_Service()
    : m_client(std::make_shared<Robot_Client>())
{
  m_client->Set_Connection_Callback(
      [this](bool connected, const std::string &detail)
      { On_Connection(connected, detail); });
  m_client->Set_Acknowledgement_Callback(
      [this](const Acknowledgement &acknowledgement)
      { On_Acknowledgement(acknowledgement); });
  m_client->Set_State_Callback(
      [this](const Robot_State &state) { On_Robot_State(state); });
  m_client->Set_Protocol_Error_Callback(
      [this](const std::string &error, const std::string &frame)
      { On_Protocol_Error(error, frame); });
}

Robot_Service::~Robot_Service()
{
  m_client->Set_Connection_Callback(nullptr);
  m_client->Set_Acknowledgement_Callback(nullptr);
  m_client->Set_State_Callback(nullptr);
  m_client->Set_Protocol_Error_Callback(nullptr);
  m_client->Disconnect();
}

void Robot_Service::Connect(const std::string &host, unsigned short port)
{
  m_client->Connect(host, port);
}

void Robot_Service::Disconnect()
{
  m_client->Disconnect();
  Set_Status(Control_State::Disconnected, 0, "Disconnected");
}

bool Robot_Service::Is_Connected() const
{
  return m_client->Is_Connected();
}

std::uint32_t Robot_Service::Synchronize()
{
  if (!Is_Connected())
    throw std::runtime_error("KUKA robot is not connected.");

  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_status.state == Control_State::Running ||
        m_status.state == Control_State::Command_Sent)
      throw std::runtime_error("KUKA robot is executing a motion command.");
  }

  const auto sequence = m_client->Get_State();
  Track_Command(sequence, Pending_Kind::Synchronize);
  Set_Status(Control_State::Connected_Unsynced,
             sequence,
             "Waiting for robot state");
  return sequence;
}

void Robot_Service::Require_Ready() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_status.state != Control_State::Ready)
    throw std::runtime_error(
        std::string("KUKA robot is not ready (") +
        To_String(m_status.state) + ").");
}

std::uint32_t Robot_Service::Move_Joint(
    const Axis &target,
    const Joint_Motion_Options &options)
{
  Require_Ready();
  const auto sequence = m_client->Move_Joint(target, options);
  Track_Command(sequence, Pending_Kind::Motion);
  Set_Status(Control_State::Command_Sent, sequence, "MOVEJ sent");
  return sequence;
}

std::uint32_t Robot_Service::Move_Pose_Ptp(
    const Pose &target,
    const Cartesian_Motion_Options &options)
{
  Require_Ready();
  const auto sequence = m_client->Move_Pose_Ptp(target, options);
  Track_Command(sequence, Pending_Kind::Motion);
  Set_Status(Control_State::Command_Sent, sequence, "MOVEPTP sent");
  return sequence;
}

std::uint32_t Robot_Service::Move_Pose_Ptp(
    const Pose &target,
    const Axis &joint_solution,
    const Cartesian_Motion_Options &options)
{
  Require_Ready();
  const auto sequence =
      m_client->Move_Pose_Ptp(target, joint_solution, options);
  Track_Command(sequence, Pending_Kind::Motion);
  Set_Status(
      Control_State::Command_Sent, sequence, "MOVEPTP axis-guided sent");
  return sequence;
}

std::uint32_t Robot_Service::Move_Linear(
    const Pose &target,
    const Cartesian_Motion_Options &options)
{
  Require_Ready();
  const auto sequence = m_client->Move_Linear(target, options);
  Track_Command(sequence, Pending_Kind::Motion);
  Set_Status(Control_State::Command_Sent, sequence, "MOVEL sent");
  return sequence;
}

std::uint32_t Robot_Service::Request_Stop()
{
  if (!Is_Connected())
    throw std::runtime_error("KUKA robot is not connected.");
  const auto sequence = m_client->Stop(true);
  Track_Command(sequence, Pending_Kind::Stop);
  return sequence;
}

void Robot_Service::Send_Raw(std::string frame)
{
  Protocol_Message message;
  std::string error;
  if (!Decode_Message(frame, &message, &error) ||
      message.type != Message_Type::Command)
    throw std::invalid_argument(
        error.empty() ? "Only KUKA command frames are accepted." : error);

  if (message.name == "PING")
  {
    m_client->Send_Raw(std::move(frame));
    return;
  }
  if (message.name == "GET_STATE")
  {
    Synchronize();
    return;
  }
  if (message.name == "STOP")
  {
    Request_Stop();
    return;
  }
  throw std::invalid_argument(
      "Motion commands must be executed from the Robot control page.");
}

Service_Status Robot_Service::Status() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_status;
}

bool Robot_Service::Latest_State(Robot_State *state,
                                 std::uint64_t *revision) const
{
  if (!state)
    return false;
  std::lock_guard<std::mutex> lock(m_mutex);
  if (!m_has_latest_state)
    return false;
  *state = m_latest_state;
  if (revision)
    *revision = m_state_revision;
  return true;
}

bool Robot_Service::Can_Move() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_status.state == Control_State::Ready;
}

Robot_Service::Observer_Token Robot_Service::Subscribe(
    Service_Observer observer)
{
  std::lock_guard<std::mutex> lock(m_observer_mutex);
  const auto token = m_next_observer_token++;
  m_observers.emplace(token, std::move(observer));
  return token;
}

void Robot_Service::Unsubscribe(Observer_Token token)
{
  std::lock_guard<std::mutex> lock(m_observer_mutex);
  m_observers.erase(token);
}

void Robot_Service::On_Connection(bool connected,
                                  const std::string &detail)
{
  if (!connected)
  {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_pending.clear();
    }
    Set_Status(Control_State::Disconnected, 0, detail);
    Notify_Connection(false, detail);
    return;
  }

  Set_Status(Control_State::Connected_Unsynced, 0, "Connected; synchronizing");
  Notify_Connection(true, detail);
  try
  {
    Synchronize();
  }
  catch (const std::exception &error)
  {
    Set_Status(Control_State::Fault, 0, error.what());
  }
}

void Robot_Service::On_Acknowledgement(
    const Acknowledgement &acknowledgement)
{
  Pending_Kind kind = Pending_Kind::Synchronize;
  bool tracked = false;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto found = m_pending.find(acknowledgement.sequence);
    if (found != m_pending.end())
    {
      tracked = true;
      kind = found->second;
    }
  }

  if (tracked)
  {
    if (acknowledgement.state == Acknowledgement_State::Running &&
        kind == Pending_Kind::Motion)
    {
      Set_Status(Control_State::Running,
                 acknowledgement.sequence,
                 acknowledgement.detail);
    }
    else if (acknowledgement.state == Acknowledgement_State::Done)
    {
      if (kind == Pending_Kind::Motion)
      {
        Set_Status(Control_State::Completed,
                   acknowledgement.sequence,
                   "Motion completed; waiting for state");
      }
      std::lock_guard<std::mutex> lock(m_mutex);
      m_pending.erase(acknowledgement.sequence);
    }
    else if (acknowledgement.state == Acknowledgement_State::Rejected ||
             acknowledgement.state == Acknowledgement_State::Failed ||
             acknowledgement.state == Acknowledgement_State::Cancelled)
    {
      {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_pending.erase(acknowledgement.sequence);
      }
      Set_Status(Control_State::Fault,
                 acknowledgement.sequence,
                 acknowledgement.detail);
    }
  }
  Notify_Acknowledgement(acknowledgement);
}

void Robot_Service::On_Robot_State(const Robot_State &state)
{
  std::uint64_t revision = 0;
  bool ready = false;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_latest_state = state;
    m_has_latest_state = true;
    revision = ++m_state_revision;
    ready = state.motion_state == "IDLE" &&
            m_status.state != Control_State::Running &&
            m_status.state != Control_State::Command_Sent;
  }
  Notify_Robot_State(state, revision);
  if (ready)
    Set_Status(Control_State::Ready, 0, "Robot synchronized");
}

void Robot_Service::On_Protocol_Error(const std::string &error,
                                      const std::string &frame)
{
  Notify_Protocol_Error(error, frame);
}

void Robot_Service::Track_Command(std::uint32_t sequence, Pending_Kind kind)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_pending[sequence] = kind;
}

void Robot_Service::Set_Status(Control_State state,
                               std::uint32_t active_sequence,
                               const std::string &detail)
{
  Service_Status status;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_status = {state, active_sequence, detail};
    status = m_status;
  }
  Notify_Service_Status(status);
}

std::vector<Service_Observer> Robot_Service::Observer_Snapshot() const
{
  std::vector<Service_Observer> observers;
  std::lock_guard<std::mutex> lock(m_observer_mutex);
  observers.reserve(m_observers.size());
  for (const auto &entry : m_observers)
    observers.push_back(entry.second);
  return observers;
}

void Robot_Service::Notify_Connection(bool connected,
                                      const std::string &detail)
{
  for (const auto &observer : Observer_Snapshot())
    if (observer.connection)
      observer.connection(connected, detail);
}

void Robot_Service::Notify_Acknowledgement(
    const Acknowledgement &acknowledgement)
{
  for (const auto &observer : Observer_Snapshot())
    if (observer.acknowledgement)
      observer.acknowledgement(acknowledgement);
}

void Robot_Service::Notify_Robot_State(const Robot_State &state,
                                       std::uint64_t revision)
{
  for (const auto &observer : Observer_Snapshot())
    if (observer.robot_state)
      observer.robot_state(state, revision);
}

void Robot_Service::Notify_Service_Status(const Service_Status &status)
{
  for (const auto &observer : Observer_Snapshot())
    if (observer.service_status)
      observer.service_status(status);
}

void Robot_Service::Notify_Protocol_Error(const std::string &error,
                                          const std::string &frame)
{
  for (const auto &observer : Observer_Snapshot())
    if (observer.protocol_error)
      observer.protocol_error(error, frame);
}

const char *To_String(Control_State state)
{
  switch (state)
  {
  case Control_State::Disconnected:
    return "Disconnected";
  case Control_State::Connected_Unsynced:
    return "ConnectedUnsynced";
  case Control_State::Ready:
    return "Ready";
  case Control_State::Command_Sent:
    return "CommandSent";
  case Control_State::Running:
    return "Running";
  case Control_State::Completed:
    return "Completed";
  case Control_State::Fault:
    return "Fault";
  default:
    return "Unknown";
  }
}
} // namespace kuka

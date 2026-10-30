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
  return Submit_Command(
      Pending_Kind::Synchronize,
      [](std::uint32_t sequence) { return Encode_Get_State(sequence); },
      "Waiting for robot state");
}

std::uint32_t Robot_Service::Move_Joint(
    const Axis &target,
    const Joint_Motion_Options &options)
{
  return Submit_Command(
      Pending_Kind::Motion,
      [&](std::uint32_t sequence)
      { return Encode_Move_Joint(sequence, target, options); },
      "MOVEJ sent");
}

std::uint32_t Robot_Service::Move_Pose_Ptp(
    const Pose &target,
    const Cartesian_Motion_Options &options)
{
  return Submit_Command(
      Pending_Kind::Motion,
      [&](std::uint32_t sequence)
      { return Encode_Move_Pose_Ptp(sequence, target, options); },
      "MOVEPTP sent");
}

std::uint32_t Robot_Service::Move_Pose_Ptp(
    const Pose &target,
    const Axis &joint_solution,
    const Cartesian_Motion_Options &options)
{
  return Submit_Command(
      Pending_Kind::Motion,
      [&](std::uint32_t sequence)
      { return Encode_Move_Pose_Ptp(sequence, target, joint_solution, options); },
      "MOVEPTP axis-guided sent");
}

std::uint32_t Robot_Service::Move_Linear(
    const Pose &target,
    const Cartesian_Motion_Options &options)
{
  return Submit_Command(
      Pending_Kind::Motion,
      [&](std::uint32_t sequence)
      { return Encode_Move_Linear(sequence, target, options); },
      "MOVEL sent");
}

std::uint32_t Robot_Service::Request_Stop()
{
  return Submit_Command(
      Pending_Kind::Stop,
      [](std::uint32_t sequence) { return Encode_Stop(sequence, true); },
      "STOP sent");
}

void Robot_Service::Send_Raw(std::string frame)
{
  Protocol_Message message;
  std::string error;
  if (!Decode_Message(frame, &message, &error) ||
      message.type != Message_Type::Command)
    throw std::invalid_argument(
        error.empty() ? "Only KUKA command frames are accepted." : error);

  if (message.name == "PING" || message.name == "PLC_ROTATION")
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
  bool notify = false;
  Service_Status status;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto found = m_pending.find(acknowledgement.sequence);
    if (found != m_pending.end())
    {
      const auto kind = found->second;
      const bool terminal =
          acknowledgement.state == Acknowledgement_State::Done ||
          acknowledgement.state == Acknowledgement_State::Rejected ||
          acknowledgement.state == Acknowledgement_State::Failed ||
          acknowledgement.state == Acknowledgement_State::Cancelled;
      if (kind == Pending_Kind::Stop)
      {
        if (terminal)
        {
          m_pending.erase(found);
          // STOP replies describe only this protocol request. In particular,
          // neither DONE nor FAILED may replace a newer motion's state.
          if (m_latest_stop_sequence == acknowledgement.sequence)
          {
            m_status.stop_sequence = acknowledgement.sequence;
            m_status.stop_result =
                acknowledgement.state == Acknowledgement_State::Done
                    ? Stop_Result::Done : Stop_Result::Failed;
            m_status.stop_detail = acknowledgement.detail;
            notify = true;
          }
        }
      }
      else if (acknowledgement.state == Acknowledgement_State::Running &&
               kind == Pending_Kind::Motion)
      {
        m_status = {Control_State::Running, acknowledgement.sequence,
                    acknowledgement.detail};
        notify = true;
      }
      else if (acknowledgement.state == Acknowledgement_State::Done)
      {
        if (kind == Pending_Kind::Motion)
        {
          m_status = {Control_State::Completed, acknowledgement.sequence,
                      "Motion completed; waiting for state"};
          notify = true;
        }
        m_pending.erase(found);
      }
      else if (acknowledgement.state == Acknowledgement_State::Rejected ||
               acknowledgement.state == Acknowledgement_State::Failed ||
               acknowledgement.state == Acknowledgement_State::Cancelled)
      {
        m_pending.erase(found);
        m_status = {Control_State::Fault, acknowledgement.sequence,
                    acknowledgement.detail};
        notify = true;
      }
      status = m_status;
    }
  }
  if (notify)
    Notify_Service_Status(status);
  Notify_Acknowledgement(acknowledgement);
}

void Robot_Service::On_Robot_State(const Robot_State &state)
{
  std::uint64_t revision = 0;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_latest_state = state;
    m_has_latest_state = true;
    revision = ++m_state_revision;
  }
  Notify_Robot_State(state, revision);

  bool ready = false;
  Service_Status status;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    ready = Is_Connected() && state.motion_state == "IDLE" &&
            m_status.state != Control_State::Running &&
            m_status.state != Control_State::Command_Sent;
    if (ready)
    {
      m_status = {Control_State::Ready, 0, "Robot synchronized"};
      status = m_status;
    }
  }
  if (ready)
    Notify_Service_Status(status);
}

void Robot_Service::On_Protocol_Error(const std::string &error,
                                      const std::string &frame)
{
  Notify_Protocol_Error(error, frame);
}

std::uint32_t Robot_Service::Submit_Command(
    Pending_Kind kind,
    const std::function<std::string(std::uint32_t)> &encoder,
    const std::string &detail)
{
  if (!Is_Connected())
    throw std::runtime_error("KUKA robot is not connected.");

  const auto sequence = m_client->Next_Sequence();
  // Validate/encode before reserving the service state.
  const auto frame = encoder(sequence);
  Service_Status status;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!Is_Connected())
      throw std::runtime_error("KUKA robot is not connected.");
    if (kind == Pending_Kind::Motion && m_status.state != Control_State::Ready)
      throw std::runtime_error(
          std::string("KUKA robot is not ready (") +
          To_String(m_status.state) + ").");
    if (kind == Pending_Kind::Synchronize &&
        (m_status.state == Control_State::Running ||
         m_status.state == Control_State::Command_Sent))
      throw std::runtime_error("KUKA robot is executing a motion command.");

    m_pending.emplace(sequence, kind);
    if (kind == Pending_Kind::Stop)
    {
      m_latest_stop_sequence = sequence;
      m_status.stop_sequence = sequence;
      m_status.stop_result = Stop_Result::None;
      m_status.stop_detail.clear();
    }
    if (kind != Pending_Kind::Stop)
    {
      m_status = {kind == Pending_Kind::Motion
                      ? Control_State::Command_Sent
                      : Control_State::Connected_Unsynced,
                  sequence, detail};
    }
    status = m_status;
  }

  try
  {
    // Publish before enqueueing: a fast reply must never precede Command_Sent.
    // Observers run outside the state lock and may query the service.
    if (kind != Pending_Kind::Stop)
      Notify_Service_Status(status);
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_pending.find(sequence) == m_pending.end())
      throw std::runtime_error("KUKA command cancelled before sending.");
    m_client->Send_Raw(frame);
  }
  catch (...)
  {
    bool notify = false;
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_pending.erase(sequence);
      if (m_status.active_sequence == sequence)
      {
        m_status = {Is_Connected() ? Control_State::Fault
                                   : Control_State::Disconnected,
                    0, "Command submission failed"};
        status = m_status;
        notify = true;
      }
    }
    if (notify)
      Notify_Service_Status(status);
    throw;
  }
  return sequence;
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

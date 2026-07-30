#include "kuka_robot_client.h"

#include <stdexcept>
#include <utility>
#include <vector>

namespace kuka
{
Robot_Client::Robot_Client()
    : m_transport(std::make_shared<tcp_client>())
{
  m_transport->set_status_callback(
      [this](bool connected, const std::string &detail)
      { On_Connection(connected, detail); });
  m_transport->set_recv_callback(
      [this](const std::string &bytes) { On_Bytes(bytes); });
}

Robot_Client::~Robot_Client()
{
  m_transport->set_status_callback(nullptr);
  m_transport->set_recv_callback(nullptr);
  m_transport->disconnect();
}

void Robot_Client::Connect(const std::string &host, unsigned short port)
{
  m_transport->connect(host, port);
}

void Robot_Client::Disconnect()
{
  m_transport->disconnect();
  std::lock_guard<std::mutex> lock(m_state_mutex);
  m_decoder.Reset();
}

bool Robot_Client::Is_Connected() const
{
  return m_transport->is_connected();
}

std::uint32_t Robot_Client::Next_Sequence()
{
  std::uint32_t sequence = m_next_sequence.fetch_add(1);
  if (sequence == 0)
    sequence = m_next_sequence.fetch_add(1);
  return sequence;
}

std::uint32_t Robot_Client::Send_Command(
    const std::function<std::string(std::uint32_t)> &encoder)
{
  if (!Is_Connected())
    throw std::runtime_error("KUKA robot is not connected.");
  const std::uint32_t sequence = Next_Sequence();
  m_transport->send(encoder(sequence));
  return sequence;
}

std::uint32_t Robot_Client::Move_Joint(
    const Axis &target,
    const Joint_Motion_Options &options)
{
  return Send_Command(
      [&](std::uint32_t sequence)
      { return Encode_Move_Joint(sequence, target, options); });
}

std::uint32_t Robot_Client::Move_Pose_Ptp(
    const Pose &target,
    const Cartesian_Motion_Options &options)
{
  return Send_Command(
      [&](std::uint32_t sequence)
      { return Encode_Move_Pose_Ptp(sequence, target, options); });
}

std::uint32_t Robot_Client::Move_Linear(
    const Pose &target,
    const Cartesian_Motion_Options &options)
{
  return Send_Command(
      [&](std::uint32_t sequence)
      { return Encode_Move_Linear(sequence, target, options); });
}

std::uint32_t Robot_Client::Stop(bool controlled)
{
  return Send_Command(
      [&](std::uint32_t sequence)
      { return Encode_Stop(sequence, controlled); });
}

std::uint32_t Robot_Client::Get_State()
{
  return Send_Command(
      [](std::uint32_t sequence) { return Encode_Get_State(sequence); });
}

std::uint32_t Robot_Client::Ping()
{
  return Send_Command(
      [](std::uint32_t sequence) { return Encode_Ping(sequence); });
}

void Robot_Client::Send_Raw(std::string frame)
{
  if (!Is_Connected())
    throw std::runtime_error("KUKA robot is not connected.");
  if (frame.empty())
    throw std::invalid_argument("KUKA frame is empty.");
  if (frame.back() != FRAME_TERMINATOR)
    frame.push_back(FRAME_TERMINATOR);
  if (frame.size() > MAX_FRAME_SIZE + 1)
    throw std::invalid_argument("KUKA frame exceeds the protocol limit.");
  m_transport->send(frame);
}

void Robot_Client::Set_Connection_Callback(Connection_Callback callback)
{
  std::lock_guard<std::mutex> lock(m_callback_mutex);
  m_connection_callback = std::move(callback);
}

void Robot_Client::Set_Acknowledgement_Callback(
    Acknowledgement_Callback callback)
{
  std::lock_guard<std::mutex> lock(m_callback_mutex);
  m_acknowledgement_callback = std::move(callback);
}

void Robot_Client::Set_State_Callback(State_Callback callback)
{
  std::lock_guard<std::mutex> lock(m_callback_mutex);
  m_state_callback = std::move(callback);
}

void Robot_Client::Set_Protocol_Error_Callback(
    Protocol_Error_Callback callback)
{
  std::lock_guard<std::mutex> lock(m_callback_mutex);
  m_protocol_error_callback = std::move(callback);
}

bool Robot_Client::Get_Command_Status(
    std::uint32_t sequence,
    Acknowledgement *acknowledgement) const
{
  if (!acknowledgement)
    return false;
  std::lock_guard<std::mutex> lock(m_state_mutex);
  const auto found = m_command_status.find(sequence);
  if (found == m_command_status.end())
    return false;
  *acknowledgement = found->second;
  return true;
}

bool Robot_Client::Get_Latest_State(Robot_State *state) const
{
  if (!state)
    return false;
  std::lock_guard<std::mutex> lock(m_state_mutex);
  if (!m_has_latest_state)
    return false;
  *state = m_latest_state;
  return true;
}

void Robot_Client::On_Connection(bool connected, const std::string &detail)
{
  if (!connected)
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_decoder.Reset();
  }

  Connection_Callback callback;
  {
    std::lock_guard<std::mutex> lock(m_callback_mutex);
    callback = m_connection_callback;
  }
  if (callback)
    callback(connected, detail);
}

void Robot_Client::On_Bytes(const std::string &bytes)
{
  std::vector<std::string> frames;
  std::string error;
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    if (!m_decoder.Feed(bytes, &frames, &error))
    {
      // Report outside the state lock.
    }
  }
  if (!error.empty())
    Notify_Protocol_Error(error, bytes);
  for (const auto &frame : frames)
    Handle_Frame(frame);
}

void Robot_Client::Handle_Frame(const std::string &frame)
{
  Protocol_Message message;
  std::string error;
  if (!Decode_Message(frame, &message, &error))
  {
    Notify_Protocol_Error(error, frame);
    return;
  }

  if (message.type == Message_Type::Acknowledgement)
  {
    Acknowledgement acknowledgement;
    if (!Parse_Acknowledgement(message, &acknowledgement, &error))
    {
      Notify_Protocol_Error(error, frame);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      m_command_status[acknowledgement.sequence] = acknowledgement;
      if (m_command_status.size() > 1024)
        m_command_status.erase(m_command_status.begin());
    }
    Acknowledgement_Callback callback;
    {
      std::lock_guard<std::mutex> lock(m_callback_mutex);
      callback = m_acknowledgement_callback;
    }
    if (callback)
      callback(acknowledgement);
    return;
  }

  if (message.type == Message_Type::State)
  {
    Robot_State state;
    if (!Parse_Robot_State(message, &state, &error))
    {
      Notify_Protocol_Error(error, frame);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      m_latest_state = state;
      m_has_latest_state = true;
    }
    State_Callback callback;
    {
      std::lock_guard<std::mutex> lock(m_callback_mutex);
      callback = m_state_callback;
    }
    if (callback)
      callback(state);
    return;
  }

  Notify_Protocol_Error("Unexpected message type received from KUKA.", frame);
}

void Robot_Client::Notify_Protocol_Error(const std::string &error,
                                         const std::string &frame)
{
  Protocol_Error_Callback callback;
  {
    std::lock_guard<std::mutex> lock(m_callback_mutex);
    callback = m_protocol_error_callback;
  }
  if (callback)
    callback(error, frame);
}
} // namespace kuka

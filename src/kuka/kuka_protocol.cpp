#include "kuka_protocol.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace kuka
{
namespace
{
void Set_Error(std::string *error_message, const std::string &message)
{
  if (error_message)
    *error_message = message;
}

std::string Trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
    return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::vector<std::string> Split(const std::string &text)
{
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (start <= text.size())
  {
    const auto end = text.find(',', start);
    fields.push_back(Trim(text.substr(
        start, end == std::string::npos ? std::string::npos : end - start)));
    if (end == std::string::npos)
      break;
    start = end + 1;
  }
  return fields;
}

bool Parse_Uint32(const std::string &text, std::uint32_t *value)
{
  if (!value || text.empty())
    return false;
  char *end = nullptr;
  errno = 0;
  const unsigned long parsed = std::strtoul(text.c_str(), &end, 10);
  if (errno != 0 || end != text.c_str() + text.size() ||
      parsed > std::numeric_limits<std::uint32_t>::max())
    return false;
  *value = static_cast<std::uint32_t>(parsed);
  return true;
}

bool Parse_Int(const std::string &text, int *value)
{
  if (!value || text.empty())
    return false;
  char *end = nullptr;
  errno = 0;
  const long parsed = std::strtol(text.c_str(), &end, 10);
  if (errno != 0 || end != text.c_str() + text.size() ||
      parsed < std::numeric_limits<int>::min() ||
      parsed > std::numeric_limits<int>::max())
    return false;
  *value = static_cast<int>(parsed);
  return true;
}

bool Parse_Double(const std::string &text, double *value)
{
  if (!value || text.empty())
    return false;
  std::istringstream input(text);
  input.imbue(std::locale::classic());
  double parsed = 0.0;
  input >> parsed;
  if (!input || input.peek() != std::char_traits<char>::eof() ||
      !std::isfinite(parsed))
    return false;
  *value = parsed;
  return true;
}

std::string Number(double value)
{
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << std::setprecision(10) << std::defaultfloat << value;
  return output.str();
}

std::string Command_Header(std::uint32_t sequence, const char *command)
{
  return std::string(PROTOCOL_VERSION) + ",C," + std::to_string(sequence) +
         "," + command;
}

void Append_Array(std::string *message, const std::array<double, 6> &values)
{
  for (const double value : values)
    *message += "," + Number(value);
}

bool Valid_Percent(double value)
{
  return std::isfinite(value) && value > 0.0 && value <= 100.0;
}

Acknowledgement_State Parse_Ack_State(const std::string &state)
{
  if (state == "ACCEPTED")
    return Acknowledgement_State::Accepted;
  if (state == "RUNNING")
    return Acknowledgement_State::Running;
  if (state == "DONE")
    return Acknowledgement_State::Done;
  if (state == "REJECTED")
    return Acknowledgement_State::Rejected;
  if (state == "FAILED")
    return Acknowledgement_State::Failed;
  if (state == "CANCELLED")
    return Acknowledgement_State::Cancelled;
  return Acknowledgement_State::Unknown;
}
} // namespace

Frame_Decoder::Frame_Decoder(std::size_t max_frame_size)
    : m_max_frame_size(max_frame_size)
{
}

bool Frame_Decoder::Feed(const char *data,
                         std::size_t size,
                         std::vector<std::string> *frames,
                         std::string *error_message)
{
  if (!frames || (!data && size != 0))
  {
    Set_Error(error_message, "Invalid frame decoder arguments.");
    return false;
  }

  if (size != 0)
    m_buffer.append(data, size);

  std::size_t terminator = 0;
  while ((terminator = m_buffer.find(FRAME_TERMINATOR)) != std::string::npos)
  {
    if (terminator > m_max_frame_size)
    {
      m_buffer.erase(0, terminator + 1);
      Set_Error(error_message, "KUKA frame exceeds the configured limit.");
      return false;
    }
    std::string frame = Trim(m_buffer.substr(0, terminator));
    m_buffer.erase(0, terminator + 1);
    if (!frame.empty())
      frames->push_back(std::move(frame));
  }

  if (m_buffer.size() > m_max_frame_size)
  {
    m_buffer.clear();
    Set_Error(error_message, "Unterminated KUKA frame exceeds the configured limit.");
    return false;
  }
  return true;
}

bool Frame_Decoder::Feed(const std::string &data,
                         std::vector<std::string> *frames,
                         std::string *error_message)
{
  return Feed(data.data(), data.size(), frames, error_message);
}

void Frame_Decoder::Reset()
{
  m_buffer.clear();
}

std::size_t Frame_Decoder::Buffered_Size() const
{
  return m_buffer.size();
}

bool Decode_Message(const std::string &raw_frame,
                    Protocol_Message *message,
                    std::string *error_message)
{
  if (!message)
  {
    Set_Error(error_message, "Output message is null.");
    return false;
  }
  std::string frame = Trim(raw_frame);
  if (!frame.empty() && frame.back() == FRAME_TERMINATOR)
    frame.pop_back();
  const auto tokens = Split(frame);
  if (tokens.size() < 4)
  {
    Set_Error(error_message, "Protocol message has fewer than four fields.");
    return false;
  }
  if (tokens[0] != PROTOCOL_VERSION)
  {
    Set_Error(error_message, "Unsupported KUKA protocol version: " + tokens[0]);
    return false;
  }

  Message_Type type;
  if (tokens[1] == "C")
    type = Message_Type::Command;
  else if (tokens[1] == "A")
    type = Message_Type::Acknowledgement;
  else if (tokens[1] == "S")
    type = Message_Type::State;
  else if (tokens[1] == "E")
    type = Message_Type::Event;
  else
  {
    Set_Error(error_message, "Unknown KUKA message type: " + tokens[1]);
    return false;
  }

  std::uint32_t sequence = 0;
  if (!Parse_Uint32(tokens[2], &sequence))
  {
    Set_Error(error_message, "Invalid command sequence: " + tokens[2]);
    return false;
  }
  if (tokens[3].empty())
  {
    Set_Error(error_message, "Message name is empty.");
    return false;
  }

  message->type = type;
  message->sequence = sequence;
  message->name = tokens[3];
  message->fields.assign(tokens.begin() + 4, tokens.end());
  return true;
}

bool Parse_Acknowledgement(const Protocol_Message &message,
                           Acknowledgement *acknowledgement,
                           std::string *error_message)
{
  if (!acknowledgement ||
      message.type != Message_Type::Acknowledgement)
  {
    Set_Error(error_message, "Message is not an acknowledgement.");
    return false;
  }
  const auto state = Parse_Ack_State(message.name);
  if (state == Acknowledgement_State::Unknown)
  {
    Set_Error(error_message, "Unknown acknowledgement state: " + message.name);
    return false;
  }
  if (message.fields.empty())
  {
    Set_Error(error_message, "Acknowledgement has no error code.");
    return false;
  }
  int error_code = 0;
  if (!Parse_Int(message.fields[0], &error_code))
  {
    Set_Error(error_message, "Invalid acknowledgement error code.");
    return false;
  }

  acknowledgement->sequence = message.sequence;
  acknowledgement->state = state;
  acknowledgement->error_code = error_code;
  acknowledgement->detail =
      message.fields.size() > 1 ? message.fields[1] : std::string{};
  return true;
}

bool Parse_Robot_State(const Protocol_Message &message,
                       Robot_State *state,
                       std::string *error_message)
{
  constexpr std::size_t REQUIRED_FIELD_COUNT = 16;
  if (!state || message.type != Message_Type::State)
  {
    Set_Error(error_message, "Message is not a robot state.");
    return false;
  }
  if (message.fields.size() != REQUIRED_FIELD_COUNT)
  {
    Set_Error(error_message, "Robot state must contain exactly 16 data fields.");
    return false;
  }

  Robot_State parsed;
  parsed.request_sequence = message.sequence;
  parsed.motion_state = message.name;
  if (!Parse_Uint32(message.fields[0], &parsed.active_sequence))
  {
    Set_Error(error_message, "Invalid active command sequence.");
    return false;
  }
  for (std::size_t index = 0; index < 6; ++index)
  {
    if (!Parse_Double(message.fields[index + 1], &parsed.axis[index]) ||
        !Parse_Double(message.fields[index + 7], &parsed.pose[index]))
    {
      Set_Error(error_message, "Invalid numeric robot state field.");
      return false;
    }
  }
  if (!Parse_Int(message.fields[13], &parsed.base) ||
      !Parse_Int(message.fields[14], &parsed.tool) ||
      !Parse_Double(message.fields[15], &parsed.override_percent))
  {
    Set_Error(error_message, "Invalid Base, Tool, or override state field.");
    return false;
  }
  *state = parsed;
  return true;
}

bool Validate_Joint_Motion(const Axis &target,
                           const Joint_Motion_Options &options,
                           std::string *error_message)
{
  if (!Valid_Percent(options.velocity_percent) ||
      !Valid_Percent(options.acceleration_percent) ||
      !std::isfinite(options.blend_percent) ||
      options.blend_percent < 0.0 || options.blend_percent > 100.0)
  {
    Set_Error(error_message, "Invalid MOVEJ velocity, acceleration, or blend.");
    return false;
  }
  if (!std::all_of(target.begin(), target.end(),
                   [](double value) { return std::isfinite(value); }))
  {
    Set_Error(error_message, "MOVEJ target contains a non-finite value.");
    return false;
  }
  return true;
}

bool Validate_Cartesian_Motion(const Pose &target,
                               const Cartesian_Motion_Options &options,
                               bool linear_motion,
                               std::string *error_message)
{
  const bool velocity_valid =
      linear_motion ? std::isfinite(options.velocity) &&
                          options.velocity > 0.0 && options.velocity <= 2000.0
                    : Valid_Percent(options.velocity);
  if (!velocity_valid || !Valid_Percent(options.acceleration_percent) ||
      !std::isfinite(options.blend_mm) || options.blend_mm < 0.0 ||
      options.base < 0 || options.base > 32 ||
      options.tool < 0 || options.tool > 32)
  {
    Set_Error(error_message, "Invalid Cartesian motion options.");
    return false;
  }
  if (!std::all_of(target.begin(), target.end(),
                   [](double value) { return std::isfinite(value); }))
  {
    Set_Error(error_message, "Cartesian target contains a non-finite value.");
    return false;
  }
  return true;
}

std::string Encode_Move_Joint(std::uint32_t sequence,
                              const Axis &target,
                              const Joint_Motion_Options &options)
{
  std::string error;
  if (!Validate_Joint_Motion(target, options, &error))
    throw std::invalid_argument(error);
  std::string message = Command_Header(sequence, "MOVEJ");
  message += "," + Number(options.velocity_percent);
  message += "," + Number(options.acceleration_percent);
  message += "," + Number(options.blend_percent);
  Append_Array(&message, target);
  return message + FRAME_TERMINATOR;
}

std::string Encode_Cartesian(const char *command,
                             std::uint32_t sequence,
                             const Pose &target,
                             const Cartesian_Motion_Options &options,
                             bool linear_motion)
{
  std::string error;
  if (!Validate_Cartesian_Motion(target, options, linear_motion, &error))
    throw std::invalid_argument(error);
  std::string message = Command_Header(sequence, command);
  message += "," + Number(options.velocity);
  message += "," + Number(options.acceleration_percent);
  message += "," + Number(options.blend_mm);
  message += "," + std::to_string(options.base);
  message += "," + std::to_string(options.tool);
  Append_Array(&message, target);
  message += options.configuration == Cartesian_Configuration::Exact
                 ? ",EXACT"
                 : ",NEAR";
  message += "," + std::to_string(options.status);
  message += "," + std::to_string(options.turn);
  return message + FRAME_TERMINATOR;
}

std::string Encode_Move_Pose_Ptp(
    std::uint32_t sequence,
    const Pose &target,
    const Cartesian_Motion_Options &options)
{
  return Encode_Cartesian("MOVEPTP", sequence, target, options, false);
}

std::string Encode_Move_Linear(
    std::uint32_t sequence,
    const Pose &target,
    const Cartesian_Motion_Options &options)
{
  return Encode_Cartesian("MOVEL", sequence, target, options, true);
}

std::string Encode_Stop(std::uint32_t sequence, bool controlled)
{
  return Command_Header(sequence, "STOP") +
         (controlled ? ",CONTROLLED;" : ",IMMEDIATE;");
}

std::string Encode_Get_State(std::uint32_t sequence)
{
  return Command_Header(sequence, "GET_STATE") + FRAME_TERMINATOR;
}

std::string Encode_Ping(std::uint32_t sequence)
{
  return Command_Header(sequence, "PING") + FRAME_TERMINATOR;
}

const char *To_String(Acknowledgement_State state)
{
  switch (state)
  {
  case Acknowledgement_State::Accepted:
    return "ACCEPTED";
  case Acknowledgement_State::Running:
    return "RUNNING";
  case Acknowledgement_State::Done:
    return "DONE";
  case Acknowledgement_State::Rejected:
    return "REJECTED";
  case Acknowledgement_State::Failed:
    return "FAILED";
  case Acknowledgement_State::Cancelled:
    return "CANCELLED";
  default:
    return "UNKNOWN";
  }
}
} // namespace kuka

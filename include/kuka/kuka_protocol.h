#ifndef includeguard_kuka_protocol_h_includeguard
#define includeguard_kuka_protocol_h_includeguard

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace kuka
{
constexpr const char *PROTOCOL_VERSION = "V1";
constexpr char FRAME_TERMINATOR = ';';
constexpr std::size_t MAX_FRAME_SIZE = 2047;

using Axis = std::array<double, 6>;
using Pose = std::array<double, 6>;

enum class Message_Type
{
  Command,
  Acknowledgement,
  State,
  Event
};

enum class Acknowledgement_State
{
  Accepted,
  Running,
  Done,
  Rejected,
  Failed,
  Cancelled,
  Unknown
};

enum class Cartesian_Configuration
{
  Nearest,
  Exact
};

struct Joint_Motion_Options
{
  double velocity_percent = 20.0;
  double acceleration_percent = 50.0;
  double blend_percent = 0.0;
};

struct Cartesian_Motion_Options
{
  // MOVEPTP interprets velocity as a percentage. MOVEL interprets it as mm/s.
  double velocity = 100.0;
  double acceleration_percent = 50.0;
  double blend_mm = 0.0;
  int base = 0;
  int tool = 0;
  Cartesian_Configuration configuration = Cartesian_Configuration::Nearest;
  int status = 0;
  int turn = 0;
};

struct Protocol_Message
{
  Message_Type type = Message_Type::Event;
  std::uint32_t sequence = 0;
  std::string name;
  std::vector<std::string> fields;
};

struct Acknowledgement
{
  std::uint32_t sequence = 0;
  Acknowledgement_State state = Acknowledgement_State::Unknown;
  int error_code = 0;
  std::string detail;
};

struct Robot_State
{
  std::uint32_t request_sequence = 0;
  std::string motion_state;
  std::uint32_t active_sequence = 0;
  Axis axis{};
  Pose pose{};
  int base = 0;
  int tool = 0;
  double override_percent = 0.0;
};

class Frame_Decoder
{
public:
  explicit Frame_Decoder(std::size_t max_frame_size = MAX_FRAME_SIZE);

  bool Feed(const char *data,
            std::size_t size,
            std::vector<std::string> *frames,
            std::string *error_message = nullptr);
  bool Feed(const std::string &data,
            std::vector<std::string> *frames,
            std::string *error_message = nullptr);
  void Reset();
  std::size_t Buffered_Size() const;

private:
  std::size_t m_max_frame_size;
  std::string m_buffer;
};

bool Decode_Message(const std::string &frame,
                    Protocol_Message *message,
                    std::string *error_message = nullptr);
bool Parse_Acknowledgement(const Protocol_Message &message,
                           Acknowledgement *acknowledgement,
                           std::string *error_message = nullptr);
bool Parse_Robot_State(const Protocol_Message &message,
                       Robot_State *state,
                       std::string *error_message = nullptr);

std::string Encode_Move_Joint(std::uint32_t sequence,
                              const Axis &target,
                              const Joint_Motion_Options &options = {});
std::string Encode_Move_Pose_Ptp(
    std::uint32_t sequence,
    const Pose &target,
    const Cartesian_Motion_Options &options = {});
std::string Encode_Move_Pose_Ptp(
    std::uint32_t sequence,
    const Pose &target,
    const Axis &joint_solution,
    const Cartesian_Motion_Options &options = {});
std::string Encode_Move_Linear(
    std::uint32_t sequence,
    const Pose &target,
    const Cartesian_Motion_Options &options = {});
std::string Encode_Stop(std::uint32_t sequence, bool controlled = true);
std::string Encode_Get_State(std::uint32_t sequence);
std::string Encode_Ping(std::uint32_t sequence);

bool Validate_Joint_Motion(const Axis &target,
                           const Joint_Motion_Options &options,
                           std::string *error_message = nullptr);
bool Validate_Cartesian_Motion(const Pose &target,
                               const Cartesian_Motion_Options &options,
                               bool linear_motion,
                               std::string *error_message = nullptr);

const char *To_String(Acknowledgement_State state);
} // namespace kuka

#endif

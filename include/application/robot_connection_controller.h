#ifndef includeguard_robot_connection_controller_h_includeguard
#define includeguard_robot_connection_controller_h_includeguard

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace application
{

using Robot_Axis = std::array<double, 6>;
using Robot_Pose = std::array<double, 6>;

enum class Robot_Control_State
{
  Disconnected,
  Connected_Unsynced,
  Ready,
  Command_Sent,
  Running,
  Completed,
  Fault
};

struct Robot_Connection_Config
{
  std::string host = "192.168.1.25";
  unsigned short port = 54600;
  std::string model_id = "KR10_R1100_2";
};

// DONE confirms handling of a STOP protocol command, not physical/safe stop.
enum class Robot_Stop_Result { None, Done, Failed };

struct Robot_Connection_Status
{
  Robot_Control_State state = Robot_Control_State::Disconnected;
  std::uint32_t active_sequence = 0;
  std::string detail;
  std::uint32_t stop_sequence = 0;
  Robot_Stop_Result stop_result = Robot_Stop_Result::None;
  std::string stop_detail;
};

struct Robot_Actual_State
{
  std::uint32_t request_sequence = 0;
  std::string motion_state;
  std::uint32_t active_sequence = 0;
  Robot_Axis axis{};
  Robot_Pose pose{};
  int base = 0;
  int tool = 0;
  double override_percent = 0.0;
};

struct Robot_Joint_Motion_Options
{
  double velocity_percent = 20.0;
  double acceleration_percent = 50.0;
  double blend_percent = 0.0;
};

struct Robot_Cartesian_Motion_Options
{
  double velocity = 100.0;
  double acceleration_percent = 50.0;
  double blend_mm = 0.0;
};

struct Robot_Connection_Observer
{
  std::function<void(bool, const std::string &)> connection;
  std::function<void(const Robot_Actual_State &, std::uint64_t)> robot_state;
  std::function<void(const Robot_Connection_Status &)> status;
  std::function<void(const std::string &)> event_log;
};

struct Robot_Backend_Observer
{
  std::function<void(bool, const std::string &)> connection;
  std::function<void(const Robot_Actual_State &, std::uint64_t)> robot_state;
  std::function<void(const Robot_Connection_Status &)> status;
  std::function<void(const std::string &)> event_log;
};

class Robot_Connection_Backend
{
public:
  virtual ~Robot_Connection_Backend() = default;
  virtual void Set_Observer(Robot_Backend_Observer observer) = 0;
  virtual void Connect(const std::string &host, unsigned short port) = 0;
  virtual void Disconnect() = 0;
  virtual bool Is_Connected() const = 0;
  virtual bool Can_Move() const = 0;
  virtual Robot_Connection_Status Status() const = 0;
  virtual std::uint32_t Synchronize() = 0;
  virtual std::uint32_t Move_Joint(
    const Robot_Axis &target,
    const Robot_Joint_Motion_Options &options) = 0;
  virtual std::uint32_t Move_Pose_Ptp(
    const Robot_Pose &target,
    const Robot_Cartesian_Motion_Options &options) = 0;
  virtual std::uint32_t Move_Pose_Ptp(
    const Robot_Pose &target,
    const Robot_Axis &joint_solution,
    const Robot_Cartesian_Motion_Options &options) = 0;
  virtual std::uint32_t Move_Linear(
    const Robot_Pose &target,
    const Robot_Cartesian_Motion_Options &options) = 0;
  virtual std::uint32_t Request_Stop() = 0;
  virtual void Send_Raw(std::string frame) = 0;
};

class Robot_Connection_Controller
{
public:
  using Observer_Token = std::uint64_t;

  Robot_Connection_Controller();
  explicit Robot_Connection_Controller(
    std::shared_ptr<Robot_Connection_Backend> backend);
  ~Robot_Connection_Controller();

  Robot_Connection_Controller(const Robot_Connection_Controller &) = delete;
  Robot_Connection_Controller &operator=(
    const Robot_Connection_Controller &) = delete;

  bool Load_Configuration(std::string *error_message = nullptr);
  bool Save_Configuration(
    const Robot_Connection_Config &configuration,
    std::string *error_message = nullptr);
  const Robot_Connection_Config &Configuration() const
  {
    return m_configuration;
  }

  void Connect();
  void Connect(const std::string &host, unsigned short port);
  void Disconnect();
  bool Is_Connected() const;
  bool Is_Connecting() const;
  bool Can_Move() const;
  Robot_Connection_Status Status() const;
  bool Latest_State(
    Robot_Actual_State *state,
    std::uint64_t *revision = nullptr) const;

  std::uint32_t Synchronize();
  std::uint32_t Move_Joint(
    const Robot_Axis &target,
    const Robot_Joint_Motion_Options &options = {});
  std::uint32_t Move_Pose_Ptp(
    const Robot_Pose &target,
    const Robot_Cartesian_Motion_Options &options = {});
  std::uint32_t Move_Pose_Ptp(
    const Robot_Pose &target,
    const Robot_Axis &joint_solution,
    const Robot_Cartesian_Motion_Options &options = {});
  std::uint32_t Move_Linear(
    const Robot_Pose &target,
    const Robot_Cartesian_Motion_Options &options = {});
  std::uint32_t Request_Stop();
  void Send_Raw(std::string frame);

  Observer_Token Subscribe(Robot_Connection_Observer observer);
  // Callbacks for this controller are serialized, but may reenter it.
  // On return no callback for token can start or remain on another thread.
  // Self-unsubscription invalidates pending calls; the current call continues.
  // Do not wait in a callback for another thread to dispatch or unsubscribe.
  void Unsubscribe(Observer_Token token);

private:
  struct Observer_Dispatch;
  void Bind_Backend();

private:
  std::shared_ptr<Robot_Connection_Backend> m_backend;
  Robot_Connection_Config m_configuration;
  mutable std::mutex m_state_mutex;
  bool m_connecting = false;
  Robot_Connection_Status m_status;
  Robot_Actual_State m_latest_state;
  bool m_has_latest_state = false;
  std::uint64_t m_state_revision = 0;
  std::shared_ptr<Observer_Dispatch> m_observer_dispatch;
};

const char *To_String(Robot_Control_State state);

} // namespace application

#endif

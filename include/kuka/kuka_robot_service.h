#ifndef includeguard_kuka_robot_service_h_includeguard
#define includeguard_kuka_robot_service_h_includeguard

#include "kuka_robot_client.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace kuka
{
enum class Control_State
{
  Disconnected,
  Connected_Unsynced,
  Ready,
  Command_Sent,
  Running,
  Completed,
  Fault
};

struct Service_Status
{
  Control_State state = Control_State::Disconnected;
  std::uint32_t active_sequence = 0;
  std::string detail;
};

struct Service_Observer
{
  std::function<void(bool, const std::string &)> connection;
  std::function<void(const Acknowledgement &)> acknowledgement;
  std::function<void(const Robot_State &, std::uint64_t)> robot_state;
  std::function<void(const Service_Status &)> service_status;
  std::function<void(const std::string &, const std::string &)> protocol_error;
};

class Robot_Service
{
public:
  using Observer_Token = std::uint64_t;

  Robot_Service();
  ~Robot_Service();

  Robot_Service(const Robot_Service &) = delete;
  Robot_Service &operator=(const Robot_Service &) = delete;

  void Connect(const std::string &host, unsigned short port);
  void Disconnect();
  bool Is_Connected() const;

  std::uint32_t Synchronize();
  std::uint32_t Move_Joint(
      const Axis &target,
      const Joint_Motion_Options &options = {});
  std::uint32_t Move_Pose_Ptp(
      const Pose &target,
      const Cartesian_Motion_Options &options = {});
  std::uint32_t Move_Pose_Ptp(
      const Pose &target,
      const Axis &joint_solution,
      const Cartesian_Motion_Options &options = {});
  std::uint32_t Move_Linear(
      const Pose &target,
      const Cartesian_Motion_Options &options = {});
  std::uint32_t Request_Stop();
  void Send_Raw(std::string frame);

  Service_Status Status() const;
  bool Latest_State(Robot_State *state,
                    std::uint64_t *revision = nullptr) const;
  bool Can_Move() const;

  Observer_Token Subscribe(Service_Observer observer);
  void Unsubscribe(Observer_Token token);

private:
  enum class Pending_Kind
  {
    Synchronize,
    Motion,
    Stop
  };

  void On_Connection(bool connected, const std::string &detail);
  void On_Acknowledgement(const Acknowledgement &acknowledgement);
  void On_Robot_State(const Robot_State &state);
  void On_Protocol_Error(const std::string &error,
                         const std::string &frame);

  void Set_Status(Control_State state,
                  std::uint32_t active_sequence,
                  const std::string &detail);
  void Track_Command(std::uint32_t sequence, Pending_Kind kind);
  void Require_Ready() const;

  void Notify_Connection(bool connected, const std::string &detail);
  void Notify_Acknowledgement(const Acknowledgement &acknowledgement);
  void Notify_Robot_State(const Robot_State &state, std::uint64_t revision);
  void Notify_Service_Status(const Service_Status &status);
  void Notify_Protocol_Error(const std::string &error,
                             const std::string &frame);
  std::vector<Service_Observer> Observer_Snapshot() const;

private:
  std::shared_ptr<Robot_Client> m_client;

  mutable std::mutex m_mutex;
  Service_Status m_status;
  Robot_State m_latest_state;
  bool m_has_latest_state = false;
  std::uint64_t m_state_revision = 0;
  std::unordered_map<std::uint32_t, Pending_Kind> m_pending;

  mutable std::mutex m_observer_mutex;
  std::unordered_map<Observer_Token, Service_Observer> m_observers;
  Observer_Token m_next_observer_token = 1;
};

const char *To_String(Control_State state);
} // namespace kuka

#endif

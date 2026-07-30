#ifndef includeguard_kuka_robot_client_h_includeguard
#define includeguard_kuka_robot_client_h_includeguard

#include "kuka_protocol.h"
#include "tcp_client.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace kuka
{
class Robot_Client
{
public:
  using Connection_Callback =
      std::function<void(bool connected, const std::string &detail)>;
  using Acknowledgement_Callback =
      std::function<void(const Acknowledgement &acknowledgement)>;
  using State_Callback = std::function<void(const Robot_State &state)>;
  using Protocol_Error_Callback =
      std::function<void(const std::string &error, const std::string &frame)>;

  Robot_Client();
  ~Robot_Client();

  Robot_Client(const Robot_Client &) = delete;
  Robot_Client &operator=(const Robot_Client &) = delete;

  void Connect(const std::string &host, unsigned short port);
  void Disconnect();
  bool Is_Connected() const;

  std::uint32_t Move_Joint(
      const Axis &target,
      const Joint_Motion_Options &options = {});
  std::uint32_t Move_Pose_Ptp(
      const Pose &target,
      const Cartesian_Motion_Options &options = {});
  std::uint32_t Move_Linear(
      const Pose &target,
      const Cartesian_Motion_Options &options = {});
  std::uint32_t Stop(bool controlled = true);
  std::uint32_t Get_State();
  std::uint32_t Ping();
  void Send_Raw(std::string frame);

  void Set_Connection_Callback(Connection_Callback callback);
  void Set_Acknowledgement_Callback(Acknowledgement_Callback callback);
  void Set_State_Callback(State_Callback callback);
  void Set_Protocol_Error_Callback(Protocol_Error_Callback callback);

  bool Get_Command_Status(std::uint32_t sequence,
                          Acknowledgement *acknowledgement) const;
  bool Get_Latest_State(Robot_State *state) const;

private:
  std::uint32_t Next_Sequence();
  std::uint32_t Send_Command(
      const std::function<std::string(std::uint32_t)> &encoder);
  void On_Connection(bool connected, const std::string &detail);
  void On_Bytes(const std::string &bytes);
  void Handle_Frame(const std::string &frame);
  void Notify_Protocol_Error(const std::string &error,
                             const std::string &frame);

private:
  std::shared_ptr<tcp_client> m_transport;
  std::atomic<std::uint32_t> m_next_sequence{1};

  mutable std::mutex m_state_mutex;
  Frame_Decoder m_decoder;
  std::unordered_map<std::uint32_t, Acknowledgement> m_command_status;
  Robot_State m_latest_state;
  bool m_has_latest_state = false;

  mutable std::mutex m_callback_mutex;
  Connection_Callback m_connection_callback;
  Acknowledgement_Callback m_acknowledgement_callback;
  State_Callback m_state_callback;
  Protocol_Error_Callback m_protocol_error_callback;
};
} // namespace kuka

#endif

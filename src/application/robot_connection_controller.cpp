#include "robot_connection_controller.h"

#include "kuka_connection_config.h"
#include "kuka_robot_service.h"

#include <sstream>
#include <stdexcept>
#include <utility>
#include <unordered_map>
#include <vector>

namespace application
{
namespace
{
Robot_Control_State Convert_Control_State(kuka::Control_State state)
{
  switch( state )
  {
    case kuka::Control_State::Disconnected:
      return Robot_Control_State::Disconnected;
    case kuka::Control_State::Connected_Unsynced:
      return Robot_Control_State::Connected_Unsynced;
    case kuka::Control_State::Ready:
      return Robot_Control_State::Ready;
    case kuka::Control_State::Command_Sent:
      return Robot_Control_State::Command_Sent;
    case kuka::Control_State::Running:
      return Robot_Control_State::Running;
    case kuka::Control_State::Completed:
      return Robot_Control_State::Completed;
    case kuka::Control_State::Fault:
      return Robot_Control_State::Fault;
  }
  return Robot_Control_State::Fault;
}

Robot_Connection_Status Convert_Status(const kuka::Service_Status &status)
{
  return {Convert_Control_State(status.state),
          status.active_sequence, status.detail, status.stop_sequence,
          status.stop_result == kuka::Stop_Result::Done
            ? Robot_Stop_Result::Done
            : status.stop_result == kuka::Stop_Result::Failed
              ? Robot_Stop_Result::Failed : Robot_Stop_Result::None,
          status.stop_detail};
}

Robot_Actual_State Convert_State(const kuka::Robot_State &state)
{
  Robot_Actual_State result;
  result.request_sequence = state.request_sequence;
  result.motion_state = state.motion_state;
  result.active_sequence = state.active_sequence;
  result.axis = state.axis;
  result.pose = state.pose;
  result.base = state.base;
  result.tool = state.tool;
  result.override_percent = state.override_percent;
  return result;
}

// Service snapshots can outlive the backend. Keep their forwarding state alive
// independently, and never hold its mutex while calling application code.
struct Backend_Observer_Bridge
{
  Robot_Backend_Observer Snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex);
    return observer;
  }

  void Set(Robot_Backend_Observer value)
  {
    std::lock_guard<std::mutex> lock(mutex);
    std::swap(observer, value);
  }

  mutable std::mutex mutex;
  Robot_Backend_Observer observer;
};

class Kuka_Robot_Backend final : public Robot_Connection_Backend
{
public:
  Kuka_Robot_Backend()
    : m_service(std::make_shared<kuka::Robot_Service>())
  {
    const auto bridge = m_observer_bridge;
    kuka::Service_Observer observer;
    observer.connection = [bridge](bool connected, const std::string &detail)
    {
      const auto target = bridge->Snapshot();
      if( target.connection ) target.connection(connected, detail);
    };
    observer.service_status = [bridge](const kuka::Service_Status &status)
    {
      const auto target = bridge->Snapshot();
      if( target.status ) target.status(Convert_Status(status));
    };
    observer.robot_state =
      [bridge](const kuka::Robot_State &state, std::uint64_t revision)
    {
      const auto target = bridge->Snapshot();
      if( target.robot_state )
        target.robot_state(Convert_State(state), revision);
      if( target.event_log )
      {
        std::ostringstream log;
        log << "STATE " << state.motion_state
            << " active=" << state.active_sequence
            << " A=[";
        for( std::size_t i = 0; i < state.axis.size(); ++i )
        {
          if( i ) log << ',';
          log << state.axis[i];
        }
        log << "] P=[";
        for( std::size_t i = 0; i < state.pose.size(); ++i )
        {
          if( i ) log << ',';
          log << state.pose[i];
        }
        log << "] base=" << state.base << " tool=" << state.tool
            << " ov=" << state.override_percent << '%';
        target.event_log(log.str());
      }
    };
    observer.acknowledgement = [bridge](const kuka::Acknowledgement &ack)
    {
      const auto target = bridge->Snapshot();
      if( !target.event_log ) return;
      std::ostringstream log;
      log << "ACK seq=" << ack.sequence
          << " state=" << kuka::To_String(ack.state)
          << " code=" << ack.error_code
          << " detail=" << ack.detail;
      target.event_log(log.str());
    };
    observer.protocol_error =
      [bridge](const std::string &error, const std::string &frame)
    {
      const auto target = bridge->Snapshot();
      if( target.event_log )
        target.event_log("Protocol error: " + error + " frame=" + frame);
    };
    m_observer_token = m_service->Subscribe(std::move(observer));
  }

  ~Kuka_Robot_Backend() override
  {
    m_observer_bridge->Set({});
    m_service->Unsubscribe(m_observer_token);
    m_service->Disconnect();
  }

  void Set_Observer(Robot_Backend_Observer observer) override
  {
    m_observer_bridge->Set(std::move(observer));
  }
  void Connect(const std::string &host, unsigned short port) override
  {
    m_service->Connect(host, port);
  }
  void Disconnect() override { m_service->Disconnect(); }
  bool Is_Connected() const override { return m_service->Is_Connected(); }
  bool Can_Move() const override { return m_service->Can_Move(); }
  Robot_Connection_Status Status() const override
  {
    return Convert_Status(m_service->Status());
  }
  std::uint32_t Synchronize() override { return m_service->Synchronize(); }
  std::uint32_t Move_Joint(
    const Robot_Axis &target,
    const Robot_Joint_Motion_Options &options) override
  {
    kuka::Joint_Motion_Options converted;
    converted.velocity_percent = options.velocity_percent;
    converted.acceleration_percent = options.acceleration_percent;
    converted.blend_percent = options.blend_percent;
    return m_service->Move_Joint(target, converted);
  }
  std::uint32_t Move_Pose_Ptp(
    const Robot_Pose &target,
    const Robot_Cartesian_Motion_Options &options) override
  {
    return m_service->Move_Pose_Ptp(target, Convert_Options(options));
  }
  std::uint32_t Move_Pose_Ptp(
    const Robot_Pose &target,
    const Robot_Axis &joint_solution,
    const Robot_Cartesian_Motion_Options &options) override
  {
    return m_service->Move_Pose_Ptp(
      target, joint_solution, Convert_Options(options));
  }
  std::uint32_t Move_Linear(
    const Robot_Pose &target,
    const Robot_Cartesian_Motion_Options &options) override
  {
    return m_service->Move_Linear(target, Convert_Options(options));
  }
  std::uint32_t Request_Stop() override { return m_service->Request_Stop(); }
  void Send_Raw(std::string frame) override
  {
    m_service->Send_Raw(std::move(frame));
  }

private:
  static kuka::Cartesian_Motion_Options Convert_Options(
    const Robot_Cartesian_Motion_Options &options)
  {
    kuka::Cartesian_Motion_Options result;
    result.velocity = options.velocity;
    result.acceleration_percent = options.acceleration_percent;
    result.blend_mm = options.blend_mm;
    return result;
  }

  std::shared_ptr<kuka::Robot_Service> m_service;
  kuka::Robot_Service::Observer_Token m_observer_token = 0;
  std::shared_ptr<Backend_Observer_Bridge> m_observer_bridge =
    std::make_shared<Backend_Observer_Bridge>();
};
} // namespace

// One recursive gate for the entire dispatch domain avoids lock-order cycles
// when callbacks unsubscribe themselves or each other. Snapshots contain tokens,
// not callbacks: a callback removed by an earlier notification is never started.
// Backend callbacks own this state, not the controller, so stale backend copies
// can check owner under the gate without dereferencing a destroyed controller.
struct Robot_Connection_Controller::Observer_Dispatch
{
  template<typename Callback, typename... Arguments>
  void Notify(Callback Robot_Connection_Observer::*member,
              const Arguments &...arguments)
  {
    std::vector<Observer_Token> snapshot;
    snapshot.reserve(observers.size());
    for( const auto &entry : observers ) snapshot.push_back(entry.first);
    for( const auto token : snapshot )
    {
      const auto found = observers.find(token);
      if( found == observers.end() ) continue;
      // Keep the callable alive if it unsubscribes itself while executing.
      const auto observer = found->second;
      if( observer.get()->*member ) (observer.get()->*member)(arguments...);
    }
  }

  std::recursive_mutex mutex;
  Robot_Connection_Controller *owner = nullptr;
  std::unordered_map<Observer_Token,
                     std::shared_ptr<Robot_Connection_Observer>> observers;
  Observer_Token next_token = 1;
};

Robot_Connection_Controller::Robot_Connection_Controller()
  : Robot_Connection_Controller(std::make_shared<Kuka_Robot_Backend>())
{
}

Robot_Connection_Controller::Robot_Connection_Controller(
  std::shared_ptr<Robot_Connection_Backend> backend)
  : m_backend(std::move(backend)),
    m_observer_dispatch(std::make_shared<Observer_Dispatch>())
{
  if( !m_backend )
    throw std::invalid_argument("Robot connection backend is null.");
  m_status = m_backend->Status();
  Bind_Backend();
}

Robot_Connection_Controller::~Robot_Connection_Controller()
{
  {
    const auto dispatch = m_observer_dispatch;
    std::lock_guard<std::recursive_mutex> lock(dispatch->mutex);
    dispatch->owner = nullptr;
    dispatch->observers.clear();
  }
  // Disconnect may join a network thread: never hold the dispatch gate here.
  m_backend->Set_Observer({});
  m_backend->Disconnect();
}

void Robot_Connection_Controller::Bind_Backend()
{
  const auto dispatch = m_observer_dispatch;
  dispatch->owner = this;
  Robot_Backend_Observer observer;
  observer.connection = [dispatch](bool connected, const std::string &detail)
  {
    std::lock_guard<std::recursive_mutex> gate(dispatch->mutex);
    auto *owner = dispatch->owner;
    if( !owner ) return;
    {
      std::lock_guard<std::mutex> lock(owner->m_state_mutex);
      owner->m_connecting = false;
      if( !connected ) owner->m_has_latest_state = false;
    }
    dispatch->Notify(&Robot_Connection_Observer::connection, connected, detail);
  };
  observer.status = [dispatch](const Robot_Connection_Status &status)
  {
    std::lock_guard<std::recursive_mutex> gate(dispatch->mutex);
    auto *owner = dispatch->owner;
    if( !owner ) return;
    {
      std::lock_guard<std::mutex> lock(owner->m_state_mutex);
      owner->m_status = status;
      if( status.state == Robot_Control_State::Disconnected )
        owner->m_has_latest_state = false;
    }
    dispatch->Notify(&Robot_Connection_Observer::status, status);
  };
  observer.robot_state =
    [dispatch](const Robot_Actual_State &state, std::uint64_t revision)
  {
    std::lock_guard<std::recursive_mutex> gate(dispatch->mutex);
    auto *owner = dispatch->owner;
    if( !owner ) return;
    {
      std::lock_guard<std::mutex> lock(owner->m_state_mutex);
      if( revision <= owner->m_state_revision ) return;
      owner->m_latest_state = state;
      owner->m_has_latest_state = true;
      owner->m_state_revision = revision;
    }
    dispatch->Notify(&Robot_Connection_Observer::robot_state, state, revision);
  };
  observer.event_log = [dispatch](const std::string &message)
  {
    std::lock_guard<std::recursive_mutex> gate(dispatch->mutex);
    if( !dispatch->owner ) return;
    dispatch->Notify(&Robot_Connection_Observer::event_log, message);
  };
  m_backend->Set_Observer(std::move(observer));
}

bool Robot_Connection_Controller::Load_Configuration(std::string *error_message)
{
  kuka::Connection_Config configuration;
  if( !kuka::Load_Connection_Config(
        kuka::Connection_Config_Path(), &configuration, error_message) )
  {
    m_configuration = {};
    return false;
  }
  m_configuration = {configuration.host, configuration.port,
                     configuration.model_id};
  return true;
}

bool Robot_Connection_Controller::Save_Configuration(
  const Robot_Connection_Config &configuration,
  std::string *error_message)
{
  const kuka::Connection_Config converted = {
    configuration.host, configuration.port, configuration.model_id};
  if( !kuka::Save_Connection_Config(
        kuka::Connection_Config_Path(), converted, error_message) )
    return false;
  if( Is_Connected() || Is_Connecting() ) Disconnect();
  m_configuration = configuration;
  return true;
}

void Robot_Connection_Controller::Connect()
{
  Connect(m_configuration.host, m_configuration.port);
}

void Robot_Connection_Controller::Connect(
  const std::string &host,
  unsigned short port)
{
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_connecting = true;
  }
  try
  {
    m_backend->Connect(host, port);
  }
  catch( ... )
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_connecting = false;
    throw;
  }
}

void Robot_Connection_Controller::Disconnect()
{
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_connecting = false;
    m_has_latest_state = false;
  }
  m_backend->Disconnect();
}

bool Robot_Connection_Controller::Is_Connected() const
{
  return m_backend->Is_Connected();
}

bool Robot_Connection_Controller::Is_Connecting() const
{
  std::lock_guard<std::mutex> lock(m_state_mutex);
  return m_connecting;
}

bool Robot_Connection_Controller::Can_Move() const
{
  return m_backend->Can_Move();
}

Robot_Connection_Status Robot_Connection_Controller::Status() const
{
  std::lock_guard<std::mutex> lock(m_state_mutex);
  return m_status;
}

bool Robot_Connection_Controller::Latest_State(
  Robot_Actual_State *state,
  std::uint64_t *revision) const
{
  if( !state ) return false;
  std::lock_guard<std::mutex> lock(m_state_mutex);
  if( !m_has_latest_state ) return false;
  *state = m_latest_state;
  if( revision ) *revision = m_state_revision;
  return true;
}

std::uint32_t Robot_Connection_Controller::Synchronize()
{
  return m_backend->Synchronize();
}

std::uint32_t Robot_Connection_Controller::Move_Joint(
  const Robot_Axis &target,
  const Robot_Joint_Motion_Options &options)
{
  return m_backend->Move_Joint(target, options);
}

std::uint32_t Robot_Connection_Controller::Move_Pose_Ptp(
  const Robot_Pose &target,
  const Robot_Cartesian_Motion_Options &options)
{
  return m_backend->Move_Pose_Ptp(target, options);
}

std::uint32_t Robot_Connection_Controller::Move_Pose_Ptp(
  const Robot_Pose &target,
  const Robot_Axis &joint_solution,
  const Robot_Cartesian_Motion_Options &options)
{
  return m_backend->Move_Pose_Ptp(target, joint_solution, options);
}

std::uint32_t Robot_Connection_Controller::Move_Linear(
  const Robot_Pose &target,
  const Robot_Cartesian_Motion_Options &options)
{
  return m_backend->Move_Linear(target, options);
}

std::uint32_t Robot_Connection_Controller::Request_Stop()
{
  return m_backend->Request_Stop();
}

void Robot_Connection_Controller::Send_Raw(std::string frame)
{
  m_backend->Send_Raw(std::move(frame));
}

Robot_Connection_Controller::Observer_Token
Robot_Connection_Controller::Subscribe(Robot_Connection_Observer observer)
{
  const auto dispatch = m_observer_dispatch;
  std::lock_guard<std::recursive_mutex> lock(dispatch->mutex);
  const auto token = dispatch->next_token++;
  dispatch->observers.emplace(token,
    std::make_shared<Robot_Connection_Observer>(std::move(observer)));
  return token;
}

void Robot_Connection_Controller::Unsubscribe(Observer_Token token)
{
  const auto dispatch = m_observer_dispatch;
  std::lock_guard<std::recursive_mutex> lock(dispatch->mutex);
  dispatch->observers.erase(token);
}

const char *To_String(Robot_Control_State state)
{
  switch( state )
  {
    case Robot_Control_State::Disconnected: return "Disconnected";
    case Robot_Control_State::Connected_Unsynced: return "ConnectedUnsynced";
    case Robot_Control_State::Ready: return "Ready";
    case Robot_Control_State::Command_Sent: return "CommandSent";
    case Robot_Control_State::Running: return "Running";
    case Robot_Control_State::Completed: return "Completed";
    case Robot_Control_State::Fault: return "Fault";
  }
  return "Unknown";
}

} // namespace application

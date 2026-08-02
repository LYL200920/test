#include "robot_connection_controller.h"

#include "kuka_connection_config.h"
#include "kuka_robot_service.h"

#include <sstream>
#include <stdexcept>
#include <utility>

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
          status.active_sequence, status.detail};
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

class Kuka_Robot_Backend final : public Robot_Connection_Backend
{
public:
  Kuka_Robot_Backend()
    : m_service(std::make_shared<kuka::Robot_Service>())
  {
    kuka::Service_Observer observer;
    observer.connection = [this](bool connected, const std::string &detail)
    {
      if( m_observer.connection ) m_observer.connection(connected, detail);
    };
    observer.service_status = [this](const kuka::Service_Status &status)
    {
      if( m_observer.status ) m_observer.status(Convert_Status(status));
    };
    observer.robot_state =
      [this](const kuka::Robot_State &state, std::uint64_t revision)
    {
      if( m_observer.robot_state )
        m_observer.robot_state(Convert_State(state), revision);
      if( m_observer.event_log )
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
        m_observer.event_log(log.str());
      }
    };
    observer.acknowledgement = [this](const kuka::Acknowledgement &ack)
    {
      if( !m_observer.event_log ) return;
      std::ostringstream log;
      log << "ACK seq=" << ack.sequence
          << " state=" << kuka::To_String(ack.state)
          << " code=" << ack.error_code
          << " detail=" << ack.detail;
      m_observer.event_log(log.str());
    };
    observer.protocol_error =
      [this](const std::string &error, const std::string &frame)
    {
      if( m_observer.event_log )
        m_observer.event_log("Protocol error: " + error + " frame=" + frame);
    };
    m_observer_token = m_service->Subscribe(std::move(observer));
  }

  ~Kuka_Robot_Backend() override
  {
    m_service->Unsubscribe(m_observer_token);
    m_service->Disconnect();
  }

  void Set_Observer(Robot_Backend_Observer observer) override
  {
    m_observer = std::move(observer);
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
  Robot_Backend_Observer m_observer;
};
} // namespace

Robot_Connection_Controller::Robot_Connection_Controller()
  : Robot_Connection_Controller(std::make_shared<Kuka_Robot_Backend>())
{
}

Robot_Connection_Controller::Robot_Connection_Controller(
  std::shared_ptr<Robot_Connection_Backend> backend)
  : m_backend(std::move(backend))
{
  if( !m_backend )
    throw std::invalid_argument("Robot connection backend is null.");
  m_status = m_backend->Status();
  Bind_Backend();
}

Robot_Connection_Controller::~Robot_Connection_Controller()
{
  m_backend->Set_Observer({});
  m_backend->Disconnect();
}

void Robot_Connection_Controller::Bind_Backend()
{
  Robot_Backend_Observer observer;
  observer.connection = [this](bool connected, const std::string &detail)
  {
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      m_connecting = false;
      if( !connected ) m_has_latest_state = false;
    }
    for( const auto &entry : Observer_Snapshot() )
      if( entry.connection ) entry.connection(connected, detail);
  };
  observer.status = [this](const Robot_Connection_Status &status)
  {
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      m_status = status;
      if( status.state == Robot_Control_State::Disconnected )
        m_has_latest_state = false;
    }
    for( const auto &entry : Observer_Snapshot() )
      if( entry.status ) entry.status(status);
  };
  observer.robot_state =
    [this](const Robot_Actual_State &state, std::uint64_t revision)
  {
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      if( revision <= m_state_revision ) return;
      m_latest_state = state;
      m_has_latest_state = true;
      m_state_revision = revision;
    }
    for( const auto &entry : Observer_Snapshot() )
      if( entry.robot_state ) entry.robot_state(state, revision);
  };
  observer.event_log = [this](const std::string &message)
  {
    for( const auto &entry : Observer_Snapshot() )
      if( entry.event_log ) entry.event_log(message);
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
  std::lock_guard<std::mutex> lock(m_observer_mutex);
  const auto token = m_next_observer_token++;
  m_observers.emplace(token, std::move(observer));
  return token;
}

void Robot_Connection_Controller::Unsubscribe(Observer_Token token)
{
  std::lock_guard<std::mutex> lock(m_observer_mutex);
  m_observers.erase(token);
}

std::vector<Robot_Connection_Observer>
Robot_Connection_Controller::Observer_Snapshot() const
{
  std::vector<Robot_Connection_Observer> result;
  std::lock_guard<std::mutex> lock(m_observer_mutex);
  result.reserve(m_observers.size());
  for( const auto &entry : m_observers ) result.push_back(entry.second);
  return result;
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

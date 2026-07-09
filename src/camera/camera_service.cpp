#include "camera_service.h"

#include <wx/app.h>

#include <utility>

namespace
{
wxDEFINE_EVENT (wxEVT_CAMERA_WORKER_RESULT, wxThreadEvent);

constexpr std::uint32_t FRAME_FETCH_TIMEOUT_MS = 40;
} // namespace

wxDEFINE_EVENT (wxEVT_CAMERA_UPDATED, wxThreadEvent);

Camera_Service::Camera_Service ( )
{
  Bind (
    wxEVT_CAMERA_WORKER_RESULT,
    &Camera_Service::On_Worker_Result,
    this);
  m_worker = std::thread (&Camera_Service::Worker_Loop, this);
}

Camera_Service::~Camera_Service ( )
{
  {
    std::lock_guard<std::mutex> lock (m_command_mutex);
    m_exit_requested = true;
    m_commands.clear ( );
  }
  m_command_ready.notify_all ( );
  if( m_worker.joinable ( ) )
  {
    m_worker.join ( );
  }

  DeletePendingEvents ( );
  Unbind (
    wxEVT_CAMERA_WORKER_RESULT,
    &Camera_Service::On_Worker_Result,
    this);
}

std::uint64_t Camera_Service::Request_Refresh_Devices ( )
{
  return Enqueue (Camera_Operation::Refresh_Devices);
}

std::uint64_t Camera_Service::Request_Select_Device (
  const std::string& serial_number)
{
  return Enqueue (Camera_Operation::Select_Device, serial_number);
}

std::uint64_t Camera_Service::Request_Open_Selected_Device ( )
{
  return Enqueue (Camera_Operation::Open_Device);
}

std::uint64_t Camera_Service::Request_Get_Device_Info ( )
{
  return Enqueue (Camera_Operation::Get_Device_Info);
}

std::uint64_t Camera_Service::Request_Get_Stream_Configuration ( )
{
  return Enqueue (Camera_Operation::Get_Stream_Configuration);
}

std::uint64_t Camera_Service::Request_Load_Parameters ( )
{
  return Enqueue (Camera_Operation::Load_Parameters);
}

std::uint64_t Camera_Service::Request_Apply_Parameters (
  std::vector<Camera_Parameter_Update> updates)
{
  return Enqueue (
    Camera_Operation::Apply_Parameters, {}, std::move (updates));
}

std::uint64_t Camera_Service::Request_Close_Device ( )
{
  return Enqueue (Camera_Operation::Close_Device);
}

std::uint64_t Camera_Service::Request_Start_Acquisition ( )
{
  return Enqueue (Camera_Operation::Start_Acquisition);
}

std::uint64_t Camera_Service::Request_Stop_Acquisition ( )
{
  return Enqueue (Camera_Operation::Stop_Acquisition);
}

std::uint64_t Camera_Service::Enqueue (
  Camera_Operation operation,
  std::string serial_number,
  std::vector<Camera_Parameter_Update> parameter_updates)
{
  const auto request_id = m_next_request_id.fetch_add (1);
  {
    std::lock_guard<std::mutex> lock (m_command_mutex);
    if( m_exit_requested )
    {
      return 0;
    }
    m_commands.push_back ({
      request_id,
      operation,
      std::move (serial_number),
      std::move (parameter_updates) });
  }

  ++m_pending_request_count;
  m_command_ready.notify_one ( );
  return request_id;
}

void Camera_Service::Worker_Loop ( )
{
  Camera_Manager core;

  while( true )
  {
    Command command;
    bool has_command = false;
    {
      std::unique_lock<std::mutex> lock (m_command_mutex);
      if( m_commands.empty ( ) &&
          core.State ( ) != Camera_State::Grabbing &&
          !m_exit_requested )
      {
        m_command_ready.wait (
          lock,
          [this] { return m_exit_requested || !m_commands.empty ( ); });
      }

      if( m_exit_requested )
      {
        break;
      }
      if( !m_commands.empty ( ) )
      {
        command = std::move (m_commands.front ( ));
        m_commands.pop_front ( );
        has_command = true;
      }
    }

    if( has_command )
    {
      Post_Worker_Result (Execute_Command (core, command));
      continue;
    }

    if( core.State ( ) == Camera_State::Grabbing )
    {
      Camera_Frame frame;
      bool no_data = false;
      std::string error;
      if( core.Fetch_Frame (
            frame, FRAME_FETCH_TIMEOUT_MS, &no_data, &error) )
      {
        m_frame_buffer.Publish (std::move (frame));
      }
      else if( !no_data )
      {
        Command frame_error;
        frame_error.operation = Camera_Operation::Frame_Error;
        Post_Worker_Result (
          Build_Result (core, frame_error, false, std::move (error)));
      }
    }
  }

  core.Shut_Down ( );
  m_frame_buffer.Clear ( );
}

Camera_Service::Worker_Result Camera_Service::Execute_Command (
  Camera_Manager& core,
  const Command& command)
{
  bool success = false;
  std::string error;

  switch( command.operation )
  {
  case Camera_Operation::Refresh_Devices:
    m_frame_buffer.Clear ( );
    success = core.Refresh_Devices (&error);
    break;
  case Camera_Operation::Select_Device:
    success = core.Select_Device (command.serial_number);
    if( !success )
    {
      error = "Unable to select the requested camera device";
    }
    break;
  case Camera_Operation::Open_Device:
    success = core.Open_Selected_Device (&error);
    break;
  case Camera_Operation::Get_Device_Info:
    success = core.Refresh_Current_Device_Info (&error);
    break;
  case Camera_Operation::Get_Stream_Configuration:
    success = core.Refresh_Stream_Configuration (&error);
    break;
  case Camera_Operation::Load_Parameters:
    success = core.Refresh_Parameters (&error);
    break;
  case Camera_Operation::Apply_Parameters:
  {
    bool stream_config_changed = false;
    success = core.Apply_Parameters (
      command.parameter_updates, &stream_config_changed, &error);
    if( stream_config_changed )
    {
      m_frame_buffer.Clear ( );
      std::string stream_error;
      if( !core.Refresh_Stream_Configuration (&stream_error) )
      {
        if( success )
        {
          success = false;
          error = std::move (stream_error);
        }
        else if( !stream_error.empty ( ) )
        {
          error += "; " + stream_error;
        }
      }
    }
    break;
  }
  case Camera_Operation::Close_Device:
    success = core.Close_Device (&error);
    if( success )
    {
      m_frame_buffer.Clear ( );
    }
    break;
  case Camera_Operation::Start_Acquisition:
    m_frame_buffer.Clear ( );
    success = core.Start_Acquisition (&error);
    break;
  case Camera_Operation::Stop_Acquisition:
    success = core.Stop_Acquisition (&error);
    break;
  case Camera_Operation::None:
  case Camera_Operation::Frame_Error:
    error = "Unsupported camera command";
    break;
  }

  return Build_Result (core, command, success, std::move (error));
}

Camera_Service::Worker_Result Camera_Service::Build_Result (
  const Camera_Manager& core,
  const Command& command,
  bool success,
  std::string error) const
{
  Worker_Result result;
  result.request_id = command.request_id;
  result.operation = command.operation;
  result.success = success;
  result.state = core.State ( );
  result.error = error.empty ( ) ? core.Last_Error ( ) : std::move (error);
  result.initialized = core.Is_Initialized ( );
  result.device_open = core.Is_Device_Open ( );
  result.can_refresh_devices = core.Can_Refresh_Devices ( );
  result.can_select_device = core.Can_Select_Device ( );
  result.version = core.Version ( );
  result.devices = core.Devices ( );
  result.device_detail = core.Device_Detail ( );
  result.parameters = core.Parameters ( );
  result.parameters_loaded = core.Parameters_Loaded ( );
  result.parameter_status = core.Parameter_Status ( );
  result.stream_configuration = core.Stream_Configuration ( );
  result.stream_configuration_loaded =
    core.Stream_Configuration_Loaded ( );
  result.selected_serial_number = core.Selected_Serial_Number ( );
  return result;
}

void Camera_Service::Post_Worker_Result (Worker_Result result)
{
  auto* event = new wxThreadEvent (wxEVT_CAMERA_WORKER_RESULT);
  event->SetPayload (std::move (result));
  wxQueueEvent (this, event);
}

void Camera_Service::On_Worker_Result (wxThreadEvent& event)
{
  const auto result = event.GetPayload<Worker_Result> ( );
  m_state = result.state;
  m_last_error = result.error;
  m_initialized = result.initialized;
  m_device_open = result.device_open;
  m_can_refresh_devices = result.can_refresh_devices;
  m_can_select_device = result.can_select_device;
  m_version = result.version;
  m_devices = result.devices;
  m_device_detail = result.device_detail;
  m_parameters = result.parameters;
  m_parameters_loaded = result.parameters_loaded;
  m_parameter_status = result.parameter_status;
  m_stream_configuration = result.stream_configuration;
  m_stream_configuration_loaded = result.stream_configuration_loaded;
  m_selected_serial_number = result.selected_serial_number;
  m_last_operation = result.operation;
  m_last_operation_succeeded = result.success;
  m_last_completed_request = result.request_id;
  if( result.request_id != 0 && m_pending_request_count > 0 )
  {
    --m_pending_request_count;
  }

  if( result.operation == Camera_Operation::Open_Device &&
      result.success && result.device_open )
  {
    Request_Get_Device_Info ( );
    Request_Get_Stream_Configuration ( );
  }

  wxThreadEvent updated (wxEVT_CAMERA_UPDATED);
  ProcessEvent (updated);
}

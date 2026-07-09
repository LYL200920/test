#ifndef includeguard_camera_service_h_includeguard
#define includeguard_camera_service_h_includeguard

#include <wx/event.h>

#include "camera_frame.h"
#include "camera_manager.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

wxDECLARE_EVENT (wxEVT_CAMERA_UPDATED, wxThreadEvent);

enum class Camera_Operation
{
  None,
  Refresh_Devices,
  Select_Device,
  Open_Device,
  Get_Device_Info,
  Get_Stream_Configuration,
  Load_Parameters,
  Apply_Parameters,
  Close_Device,
  Start_Acquisition,
  Stop_Acquisition,
  Frame_Error
};

class Camera_Service : public wxEvtHandler
{
public:
  Camera_Service ( );
  ~Camera_Service ( ) override;

  std::uint64_t Request_Refresh_Devices ( );
  std::uint64_t Request_Select_Device (const std::string& serial_number);
  std::uint64_t Request_Open_Selected_Device ( );
  std::uint64_t Request_Get_Device_Info ( );
  std::uint64_t Request_Get_Stream_Configuration ( );
  std::uint64_t Request_Load_Parameters ( );
  std::uint64_t Request_Apply_Parameters (
    std::vector<Camera_Parameter_Update> updates);
  std::uint64_t Request_Close_Device ( );
  std::uint64_t Request_Start_Acquisition ( );
  std::uint64_t Request_Stop_Acquisition ( );

  Camera_State State ( ) const { return m_state; }
  const std::string& Last_Error ( ) const { return m_last_error; }
  bool Is_Initialized ( ) const { return m_initialized; }
  bool Is_Device_Open ( ) const { return m_device_open; }
  bool Can_Refresh_Devices ( ) const { return m_can_refresh_devices; }
  bool Can_Select_Device ( ) const { return m_can_select_device; }
  bool Is_Busy ( ) const { return m_pending_request_count != 0; }

  const SDK_VERSION& Version ( ) const { return m_version; }
  const std::vector<CAMERA_DEVICE_INFO>& Devices ( ) const { return m_devices; }
  const std::optional<Camera_Device_Detail>& Device_Detail ( ) const
  {
    return m_device_detail;
  }
  const std::vector<Camera_Parameter>& Parameters ( ) const
  {
    return m_parameters;
  }
  bool Parameters_Loaded ( ) const { return m_parameters_loaded; }
  const std::string& Parameter_Status ( ) const { return m_parameter_status; }
  const std::vector<Camera_Stream_Config>& Stream_Configuration ( ) const
  {
    return m_stream_configuration;
  }
  bool Stream_Configuration_Loaded ( ) const
  {
    return m_stream_configuration_loaded;
  }
  const std::string& Selected_Serial_Number ( ) const
  {
    return m_selected_serial_number;
  }

  Camera_Operation Last_Operation ( ) const { return m_last_operation; }
  bool Last_Operation_Succeeded ( ) const { return m_last_operation_succeeded; }
  std::uint64_t Last_Completed_Request ( ) const
  {
    return m_last_completed_request;
  }

  std::shared_ptr<const Camera_Frame> Latest_Frame ( ) const
  {
    return m_frame_buffer.Latest ( );
  }

private:
  struct Command
  {
    std::uint64_t request_id = 0;
    Camera_Operation operation = Camera_Operation::None;
    std::string serial_number;
    std::vector<Camera_Parameter_Update> parameter_updates;
  };

  struct Worker_Result
  {
    std::uint64_t request_id = 0;
    Camera_Operation operation = Camera_Operation::None;
    bool success = false;
    Camera_State state = Camera_State::Uninitialized;
    std::string error;
    bool initialized = false;
    bool device_open = false;
    bool can_refresh_devices = true;
    bool can_select_device = false;
    SDK_VERSION version{};
    std::vector<CAMERA_DEVICE_INFO> devices;
    std::optional<Camera_Device_Detail> device_detail;
    std::vector<Camera_Parameter> parameters;
    bool parameters_loaded = false;
    std::string parameter_status;
    std::vector<Camera_Stream_Config> stream_configuration;
    bool stream_configuration_loaded = false;
    std::string selected_serial_number;
  };

  std::uint64_t Enqueue (
    Camera_Operation operation,
    std::string serial_number = {},
    std::vector<Camera_Parameter_Update> parameter_updates = {});
  void Worker_Loop ( );
  Worker_Result Execute_Command (
    Camera_Manager& core,
    const Command& command);
  Worker_Result Build_Result (
    const Camera_Manager& core,
    const Command& command,
    bool success,
    std::string error) const;
  void Post_Worker_Result (Worker_Result result);
  void On_Worker_Result (wxThreadEvent& event);

private:
  std::thread m_worker;
  mutable std::mutex m_command_mutex;
  std::condition_variable m_command_ready;
  std::deque<Command> m_commands;
  bool m_exit_requested = false;
  std::atomic<std::uint64_t> m_next_request_id{1};

  Camera_Frame_Buffer m_frame_buffer;

  Camera_State m_state = Camera_State::Uninitialized;
  std::string m_last_error;
  bool m_initialized = false;
  bool m_device_open = false;
  bool m_can_refresh_devices = true;
  bool m_can_select_device = false;
  SDK_VERSION m_version{};
  std::vector<CAMERA_DEVICE_INFO> m_devices;
  std::optional<Camera_Device_Detail> m_device_detail;
  std::vector<Camera_Parameter> m_parameters;
  bool m_parameters_loaded = false;
  std::string m_parameter_status;
  std::vector<Camera_Stream_Config> m_stream_configuration;
  bool m_stream_configuration_loaded = false;
  std::string m_selected_serial_number;
  std::size_t m_pending_request_count = 0;
  Camera_Operation m_last_operation = Camera_Operation::None;
  bool m_last_operation_succeeded = false;
  std::uint64_t m_last_completed_request = 0;
};

#endif

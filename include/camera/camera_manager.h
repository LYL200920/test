#ifndef includeguard_camera_manager_h_includeguard
#define includeguard_camera_manager_h_includeguard

#include "mv3d_rgbd.h"

#include <optional>
#include <string>
#include <vector>

enum class Camera_State
{
  Uninitialized,
  Initialized,
  Opened,
  Grabbing,
  Error
};

class Camera_Manager
{
public:
  ~Camera_Manager();

  bool Ensure_Initialized(std::string *error = nullptr);
  bool Refresh_Devices(std::string *error = nullptr);
  bool Open_Selected_Device(std::string *error = nullptr);
  bool Refresh_Current_Device_Info(std::string *error = nullptr);
  bool Refresh_Parameters(std::string *error = nullptr);
  bool Apply_Parameters(const std::vector<Camera_Parameter_Update> &updates,
                        bool *stream_config_changed = nullptr,
                        std::string *error = nullptr);
  bool Refresh_Stream_Configuration(std::string *error = nullptr);
  bool Close_Device(std::string *error = nullptr);
  bool Start_Acquisition(std::string *error = nullptr);
  bool Stop_Acquisition(std::string *error = nullptr);
  bool Fetch_Frame(Camera_Frame &frame,
                   std::uint32_t timeout_ms,
                   bool *no_data = nullptr,
                   std::string *error = nullptr);
  void Shut_Down();

  Camera_State State() const { return m_state; }
  const std::string &Last_Error() const { return m_last_error; }
  bool Is_Initialized() const;
  bool Is_Device_Open() const;
  bool Can_Refresh_Devices() const;
  bool Can_Select_Device() const;
  const SDK_VERSION &Version() const { return m_version; }
  const std::vector<CAMERA_DEVICE_INFO> &Devices() const { return m_devices; }
  const std::optional<Camera_Device_Detail> &Device_Detail() const
  {
    return m_device_detail;
  }
  const std::vector<Camera_Parameter> &Parameters() const
  {
    return m_parameters;
  }
  bool Parameters_Loaded() const { return m_parameters_loaded; }
  const std::string &Parameter_Status() const { return m_parameter_status; }
  const std::vector<Camera_Stream_Config> &Stream_Configuration() const
  {
    return m_stream_configuration;
  }
  bool Stream_Configuration_Loaded() const
  {
    return m_stream_configuration_loaded;
  }

  bool Select_Device(const std::string &serial_number);
  const std::string &Selected_Serial_Number() const
  {
    return m_selected_serial_number;
  }

private:
  Camera_State Lifecycle_State() const;
  void Restore_Lifecycle_State();
  void Clear_Parameters();
  void Clear_Stream_Configuration();
  bool Set_Error(Camera_State recovery_state,
                 const std::string &message,
                 std::string *error);
  void Clear_Error();

private:
  Mv3d_Rgbd_Camera m_camera;
  Camera_State m_state = Camera_State::Uninitialized;
  Camera_State m_recovery_state = Camera_State::Uninitialized;
  std::string m_last_error;
  SDK_VERSION m_version{};
  std::vector<CAMERA_DEVICE_INFO> m_devices;
  std::vector<MV3D_RGBD_DEVICE_INFO> m_sdk_devices;
  std::optional<MV3D_RGBD_DEVICE_INFO> m_selected_sdk_device;
  std::optional<Camera_Device_Detail> m_device_detail;
  std::vector<Camera_Parameter> m_parameters;
  bool m_parameters_loaded = false;
  std::string m_parameter_status;
  std::vector<Camera_Stream_Config> m_stream_configuration;
  bool m_stream_configuration_loaded = false;
  std::string m_selected_serial_number;
};

#endif

#include "camera_manager.h"

#include "camera_parameter_registry.h"

#include <algorithm>
#include <sstream>
#include <utility>

namespace
{
bool Return_Error(const std::string &message, std::string *error)
{
  if (error)
    *error = message;
  return false;
}
} // namespace

Camera_Manager::~Camera_Manager()
{
  Shut_Down();
}

bool Camera_Manager::Ensure_Initialized(std::string *error)
{
  if (error)
    error->clear();

  const auto lifecycle_state = Lifecycle_State();
  if (lifecycle_state != Camera_State::Uninitialized)
  {
    Restore_Lifecycle_State();
    Clear_Error();
    return true;
  }

  std::string operation_error;
  if (!m_camera.Initialize(&operation_error))
    return Set_Error(Camera_State::Uninitialized, operation_error, error);

  SDK_VERSION version{};
  if (!m_camera.Get_Version(version, &operation_error))
  {
    m_camera.Shut_Down();
    return Set_Error(Camera_State::Uninitialized, operation_error, error);
  }

  m_version = version;
  m_state = Camera_State::Initialized;
  m_recovery_state = Camera_State::Initialized;
  Clear_Error();
  return true;
}

bool Camera_Manager::Refresh_Devices(std::string *error)
{
  if (error)
    error->clear();

  if (!Ensure_Initialized(error))
  {
    m_devices.clear();
    m_sdk_devices.clear();
    m_selected_sdk_device.reset();
    m_device_detail.reset();
    Clear_Parameters();
    Clear_Stream_Configuration();
    return false;
  }
  if (m_state != Camera_State::Initialized)
  {
    return Return_Error(
        "Device enumeration requires the camera to be closed", error);
  }

  std::vector<CAMERA_DEVICE_INFO> devices;
  std::vector<MV3D_RGBD_DEVICE_INFO> sdk_devices;
  std::string operation_error;
  if (!m_camera.Enumerate_Devices(devices, sdk_devices, &operation_error))
  {
    m_devices.clear();
    m_sdk_devices.clear();
    m_selected_sdk_device.reset();
    m_device_detail.reset();
    Clear_Parameters();
    Clear_Stream_Configuration();
    return Set_Error(Camera_State::Initialized, operation_error, error);
  }

  m_devices = std::move(devices);
  m_sdk_devices = std::move(sdk_devices);
  m_selected_sdk_device.reset();
  m_device_detail.reset();
  Clear_Parameters();
  Clear_Stream_Configuration();
  if (m_selected_serial_number.empty())
  {
    Clear_Error();
    return true;
  }

  const auto selected = std::find_if(
      m_devices.begin(), m_devices.end(),
      [this](const CAMERA_DEVICE_INFO &device)
      {
        return device.serial_number == m_selected_serial_number;
      });
  if (selected == m_devices.end())
  {
    m_selected_serial_number.clear();
    Clear_Error();
    return true;
  }

  const auto selected_index = static_cast<std::size_t>(
      std::distance(m_devices.begin(), selected));
  if (selected_index >= m_sdk_devices.size())
  {
    m_selected_serial_number.clear();
    Clear_Error();
    return true;
  }

  m_selected_sdk_device = m_sdk_devices[selected_index];
  Clear_Error();
  return true;
}

bool Camera_Manager::Select_Device(const std::string &serial_number)
{
  if (!Can_Select_Device())
    return false;

  const auto selected = std::find_if(
      m_devices.begin(), m_devices.end(),
      [&serial_number](const CAMERA_DEVICE_INFO &device)
      {
        return device.serial_number == serial_number;
      });
  if (selected == m_devices.end())
    return false;

  const auto selected_index = static_cast<std::size_t>(
      std::distance(m_devices.begin(), selected));
  if (selected_index >= m_sdk_devices.size())
    return false;

  m_selected_serial_number = serial_number;
  m_selected_sdk_device = m_sdk_devices[selected_index];
  m_device_detail.reset();
  Clear_Parameters();
  Clear_Stream_Configuration();
  return true;
}

bool Camera_Manager::Open_Selected_Device(std::string *error)
{
  if (error)
    error->clear();

  Restore_Lifecycle_State();
  if (m_state == Camera_State::Opened || m_state == Camera_State::Grabbing)
  {
    Clear_Error();
    return true;
  }
  if (m_state != Camera_State::Initialized)
    return Return_Error("SDK is not initialized", error);
  if (!m_selected_sdk_device)
    return Return_Error("No camera device is selected", error);

  m_device_detail.reset();
  Clear_Parameters();
  Clear_Stream_Configuration();
  std::string operation_error;
  if (!m_camera.Open_Device(*m_selected_sdk_device, &operation_error))
    return Set_Error(Camera_State::Initialized, operation_error, error);

  m_state = Camera_State::Opened;
  m_recovery_state = Camera_State::Opened;
  Clear_Error();
  return true;
}

bool Camera_Manager::Refresh_Parameters(std::string *error)
{
  if (error)
    error->clear();

  Restore_Lifecycle_State();
  if (m_state != Camera_State::Opened && m_state != Camera_State::Grabbing)
    return Return_Error("Camera device is not open", error);

  std::vector<Camera_Parameter> parameters;
  const auto &definitions = First_Stage_Camera_Parameter_Definitions();
  parameters.reserve(definitions.size());
  std::size_t unavailable_count = 0;
  std::string first_operation_error;

  for (const auto &definition : definitions)
  {
    Camera_Parameter parameter;
    std::string operation_error;
    if (!m_camera.Get_Parameter(definition.key, parameter, &operation_error))
    {
      ++unavailable_count;
      if (first_operation_error.empty())
        first_operation_error = definition.display_name + ": " + operation_error;
      continue;
    }

    parameter.display_name = definition.display_name;
    parameter.group = definition.group;
    parameter.affects_stream_config = definition.affects_stream_config;
    if (parameter.type == Camera_Parameter_Type::Enum)
    {
      for (const auto value : parameter.enum_values)
      {
        parameter.choices.push_back({
            static_cast<std::int64_t>(value),
            Camera_Parameter_Enum_Label(parameter.key, value)});
      }
    }
    else
    {
      parameter.choices = definition.choices;
    }
    if (!parameter.choices.empty() &&
        (parameter.type == Camera_Parameter_Type::Int ||
         parameter.type == Camera_Parameter_Type::Enum))
    {
      const auto current_value = parameter.type == Camera_Parameter_Type::Int
                                     ? std::get<std::int64_t>(parameter.current_value)
                                     : static_cast<std::int64_t>(
                                           std::get<std::uint32_t>(
                                               parameter.current_value));
      const auto choice_found = std::find_if(
          parameter.choices.begin(), parameter.choices.end(),
          [current_value](const Camera_Parameter_Choice &choice)
          { return choice.value == current_value; });
      if (choice_found == parameter.choices.end())
      {
        parameter.choices.push_back({current_value, "当前设备值"});
      }
    }
    parameters.push_back(std::move(parameter));
  }

  if (parameters.empty())
  {
    Clear_Parameters();
    const auto message = first_operation_error.empty()
                             ? "No supported camera parameters were found"
                             : first_operation_error;
    m_parameter_status = message;
    return Return_Error(message, error);
  }

  m_parameters = std::move(parameters);
  m_parameters_loaded = true;
  std::ostringstream status;
  status << "已读取 " << m_parameters.size() << " / " << definitions.size()
         << " 项参数";
  if (unavailable_count > 0)
    status << "，" << unavailable_count << " 项当前设备不可用";
  m_parameter_status = status.str();
  Clear_Error();
  return true;
}

bool Camera_Manager::Apply_Parameters(
    const std::vector<Camera_Parameter_Update> &updates,
    bool *stream_config_changed,
    std::string *error)
{
  if (error)
    error->clear();
  if (stream_config_changed)
    *stream_config_changed = false;

  Restore_Lifecycle_State();
  if (m_state == Camera_State::Grabbing)
    return Return_Error("Stop acquisition before changing camera parameters", error);
  if (m_state != Camera_State::Opened)
    return Return_Error("Camera device is not open", error);
  if (updates.empty())
    return true;

  if (!m_parameters_loaded && !Refresh_Parameters(error))
    return false;

  bool changed_stream_config = false;
  std::size_t applied_count = 0;
  std::string apply_error;
  for (const auto &update : updates)
  {
    const auto current = std::find_if(
        m_parameters.begin(), m_parameters.end(),
        [&update](const Camera_Parameter &parameter)
        { return parameter.key == update.key; });
    if (current == m_parameters.end())
    {
      apply_error = update.key + ": parameter is not available";
      break;
    }

    std::string operation_error;
    if (!Validate_Camera_Parameter_Update(*current, update, &operation_error) ||
        !m_camera.Set_Parameter(update, &operation_error))
    {
      apply_error = current->display_name + ": " + operation_error;
      break;
    }

    changed_stream_config =
        changed_stream_config || current->affects_stream_config;
    ++applied_count;
  }

  std::string refresh_error;
  const bool refreshed = Refresh_Parameters(&refresh_error);
  if (stream_config_changed)
    *stream_config_changed = changed_stream_config;
  if (changed_stream_config)
    Clear_Stream_Configuration();

  if (!apply_error.empty())
    return Return_Error(apply_error, error);
  if (!refreshed)
    return Return_Error(refresh_error, error);

  std::ostringstream status;
  status << "已应用 " << applied_count << " 项参数更改";
  m_parameter_status = status.str();
  return true;
}

bool Camera_Manager::Refresh_Stream_Configuration(std::string *error)
{
  if (error)
    error->clear();

  Restore_Lifecycle_State();
  if (m_state != Camera_State::Opened)
    return Return_Error(
        "Stream configuration can only be read while acquisition is stopped",
        error);

  std::vector<Camera_Stream_Config> streams;
  std::string operation_error;
  if (!m_camera.Get_Stream_Configuration(streams, &operation_error))
  {
    Clear_Stream_Configuration();
    return Return_Error(operation_error, error);
  }

  m_stream_configuration = std::move(streams);
  m_stream_configuration_loaded = true;
  Clear_Error();
  return true;
}

bool Camera_Manager::Refresh_Current_Device_Info(std::string *error)
{
  if (error)
    error->clear();

  Restore_Lifecycle_State();
  if (m_state != Camera_State::Opened && m_state != Camera_State::Grabbing)
    return Return_Error("Camera device is not open", error);

  Camera_Device_Detail detail;
  std::string operation_error;
  if (!m_camera.Get_Device_Info(detail, &operation_error))
  {
    m_device_detail.reset();
    return Return_Error(operation_error, error);
  }

  m_device_detail = std::move(detail);
  Clear_Error();
  return true;
}

bool Camera_Manager::Close_Device(std::string *error)
{
  if (error)
    error->clear();

  Restore_Lifecycle_State();
  if (m_state == Camera_State::Grabbing && !Stop_Acquisition(error))
    return false;
  if (m_state == Camera_State::Uninitialized ||
      m_state == Camera_State::Initialized)
  {
    m_device_detail.reset();
    Clear_Parameters();
    Clear_Stream_Configuration();
    Clear_Error();
    return true;
  }
  if (m_state != Camera_State::Opened)
    return Return_Error("Camera device is not open", error);

  std::string operation_error;
  if (!m_camera.Close_Device(&operation_error))
    return Set_Error(Camera_State::Opened, operation_error, error);

  m_state = Camera_State::Initialized;
  m_recovery_state = Camera_State::Initialized;
  m_device_detail.reset();
  Clear_Parameters();
  Clear_Stream_Configuration();
  Clear_Error();
  return true;
}

bool Camera_Manager::Start_Acquisition(std::string *error)
{
  if (error)
    error->clear();

  Restore_Lifecycle_State();
  if (m_state == Camera_State::Grabbing)
  {
    Clear_Error();
    return true;
  }
  if (m_state != Camera_State::Opened)
    return Return_Error("Camera device is not open", error);

  std::string operation_error;
  if (!Refresh_Stream_Configuration(&operation_error))
    return Return_Error(operation_error, error);
  if (!m_camera.Start(&operation_error))
    return Set_Error(Camera_State::Opened, operation_error, error);

  m_state = Camera_State::Grabbing;
  m_recovery_state = Camera_State::Grabbing;
  Clear_Error();
  return true;
}

bool Camera_Manager::Stop_Acquisition(std::string *error)
{
  if (error)
    error->clear();

  Restore_Lifecycle_State();
  if (m_state == Camera_State::Opened)
  {
    Clear_Error();
    return true;
  }
  if (m_state != Camera_State::Grabbing)
    return Return_Error("Camera acquisition is not running", error);

  std::string operation_error;
  if (!m_camera.Stop(&operation_error))
    return Set_Error(Camera_State::Grabbing, operation_error, error);

  m_state = Camera_State::Opened;
  m_recovery_state = Camera_State::Opened;
  Clear_Error();
  return true;
}

bool Camera_Manager::Fetch_Frame(
    Camera_Frame &frame,
    std::uint32_t timeout_ms,
    bool *no_data,
    std::string *error)
{
  if (error)
    error->clear();
  if (no_data)
    *no_data = false;

  Restore_Lifecycle_State();
  if (m_state != Camera_State::Grabbing)
    return Return_Error("Camera acquisition is not running", error);

  MV3D_RGBD_STATUS status = MV3D_RGBD_OK;
  std::string operation_error;
  if (!m_camera.Fetch_Frame(frame, timeout_ms, &status, &operation_error))
  {
    if (status == MV3D_RGBD_E_NODATA)
    {
      if (no_data)
        *no_data = true;
      return false;
    }
    return Set_Error(Camera_State::Grabbing, operation_error, error);
  }

  Clear_Error();
  return true;
}

void Camera_Manager::Shut_Down()
{
  m_camera.Shut_Down();
  m_state = Camera_State::Uninitialized;
  m_recovery_state = Camera_State::Uninitialized;
  m_version = {};
  m_devices.clear();
  m_sdk_devices.clear();
  m_selected_sdk_device.reset();
  m_device_detail.reset();
  Clear_Parameters();
  Clear_Stream_Configuration();
  m_selected_serial_number.clear();
  Clear_Error();
}

bool Camera_Manager::Is_Initialized() const
{
  return Lifecycle_State() != Camera_State::Uninitialized;
}

bool Camera_Manager::Is_Device_Open() const
{
  const auto state = Lifecycle_State();
  return state == Camera_State::Opened || state == Camera_State::Grabbing;
}

bool Camera_Manager::Can_Refresh_Devices() const
{
  const auto state = Lifecycle_State();
  return state == Camera_State::Uninitialized ||
         state == Camera_State::Initialized;
}

bool Camera_Manager::Can_Select_Device() const
{
  return m_state == Camera_State::Initialized;
}

Camera_State Camera_Manager::Lifecycle_State() const
{
  return m_state == Camera_State::Error ? m_recovery_state : m_state;
}

void Camera_Manager::Restore_Lifecycle_State()
{
  if (m_state == Camera_State::Error)
    m_state = m_recovery_state;
}

void Camera_Manager::Clear_Parameters()
{
  m_parameters.clear();
  m_parameters_loaded = false;
  m_parameter_status.clear();
}

void Camera_Manager::Clear_Stream_Configuration()
{
  m_stream_configuration.clear();
  m_stream_configuration_loaded = false;
}

bool Camera_Manager::Set_Error(
    Camera_State recovery_state,
    const std::string &message,
    std::string *error)
{
  m_recovery_state = recovery_state;
  m_state = Camera_State::Error;
  m_last_error = message.empty() ? "Unknown camera SDK error" : message;
  if (error)
    *error = m_last_error;
  return false;
}

void Camera_Manager::Clear_Error()
{
  m_last_error.clear();
}

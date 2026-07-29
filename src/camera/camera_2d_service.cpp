#include "camera_2d_service.h"

#include "camera_dahua.h"

#include <cstring>
#include <utility>

namespace
{
std::string Camera_Error(const char *operation)
{
  return std::string(operation) + "失败，请检查相机连接和华睿驱动";
}
}

Camera_2D_Service::~Camera_2D_Service()
{
  Close(nullptr);
}

bool Camera_2D_Service::Refresh_Devices(std::string *error_message)
{
  const auto keys = jutze_camera::camera_dahua::discover_devices();
  std::vector<Camera_2D_Device> devices;
  devices.reserve(keys.size());
  for (std::size_t index = 0; index < keys.size(); ++index)
  {
    Camera_2D_Device device;
    device.key = keys[index];
    device.display_name =
      "华睿相机 " + std::to_string(index + 1) + "  [" + keys[index] + "]";
    devices.push_back(std::move(device));
  }

  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_devices = std::move(devices);
  }
  if (keys.empty())
  {
    if (error_message)
    {
      *error_message = "未发现华睿相机，请检查供电、网络和相机网卡";
    }
    return true;
  }
  if (error_message)
  {
    error_message->clear();
  }
  return true;
}

std::vector<Camera_2D_Device> Camera_2D_Service::Devices() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_devices;
}

bool Camera_2D_Service::Open(
  const std::string &device_key,
  std::string *error_message)
{
  if (device_key.empty())
  {
    return Fail("请选择要打开的华睿相机", error_message);
  }
  if (Is_Open() && !Close(error_message))
  {
    return false;
  }

  auto camera = std::make_shared<jutze_camera::camera_dahua>();
  if (!camera->open(const_cast<char *>(device_key.c_str())))
  {
    return Fail(Camera_Error("打开相机"), error_message);
  }
  if (!camera->register_callback(
        [this](const jutze_camera::camera_frame &frame)
        {
          Publish_Frame(frame);
        }))
  {
    camera->close();
    return Fail(Camera_Error("注册采集回调"), error_message);
  }

  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_camera = std::move(camera);
    m_status = Camera_2D_Status{};
    m_status.state = Camera_2D_State::Opened;
    m_fps_window_start = std::chrono::steady_clock::now();
    m_fps_window_frames = 0;
  }
  if (!Update_Status_From_Camera(error_message))
  {
    Close(nullptr);
    return false;
  }
  if (error_message)
  {
    error_message->clear();
  }
  return true;
}

bool Camera_2D_Service::Close(std::string *error_message)
{
  std::shared_ptr<jutze_camera::camera_dahua> camera;
  bool was_grabbing = false;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    camera = m_camera;
    was_grabbing = camera && camera->is_grabbing();
  }
  if (!camera)
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_status = Camera_2D_Status{};
    m_latest_frame.reset();
    if (error_message)
    {
      error_message->clear();
    }
    return true;
  }

  bool success = true;
  if (was_grabbing && !camera->stop_grabbing())
  {
    success = false;
  }
  if (!camera->close())
  {
    success = false;
  }
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_camera.reset();
    m_latest_frame.reset();
    m_status = Camera_2D_Status{};
  }
  if (!success)
  {
    return Fail(Camera_Error("关闭相机"), error_message);
  }
  if (error_message)
  {
    error_message->clear();
  }
  return true;
}

bool Camera_2D_Service::Start(std::string *error_message)
{
  std::shared_ptr<jutze_camera::camera_dahua> camera;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    camera = m_camera;
    if (m_status.state == Camera_2D_State::Grabbing)
    {
      return true;
    }
  }
  if (!camera)
  {
    return Fail("请先打开2D相机", error_message);
  }
  if (!camera->start_grabbing())
  {
    return Fail(Camera_Error("开始采集"), error_message);
  }
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_status.state = Camera_2D_State::Grabbing;
    m_status.last_error.clear();
    m_status.frames_per_second = 0.0;
    m_fps_window_start = std::chrono::steady_clock::now();
    m_fps_window_frames = 0;
  }
  if (error_message)
  {
    error_message->clear();
  }
  return true;
}

bool Camera_2D_Service::Stop(std::string *error_message)
{
  std::shared_ptr<jutze_camera::camera_dahua> camera;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    camera = m_camera;
    if (m_status.state != Camera_2D_State::Grabbing)
    {
      return true;
    }
  }
  if (!camera || !camera->stop_grabbing())
  {
    return Fail(Camera_Error("停止采集"), error_message);
  }
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_status.state = Camera_2D_State::Opened;
    m_status.frames_per_second = 0.0;
    m_status.last_error.clear();
  }
  if (error_message)
  {
    error_message->clear();
  }
  return true;
}

bool Camera_2D_Service::Software_Trigger(std::string *error_message)
{
  std::shared_ptr<jutze_camera::camera_dahua> camera;
  Camera_2D_Status status;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    camera = m_camera;
    status = m_status;
  }
  if (!camera)
  {
    return Fail("请先打开2D相机", error_message);
  }
  if (status.state != Camera_2D_State::Grabbing)
  {
    return Fail("软触发前请先开始采集", error_message);
  }
  if (status.trigger_mode != jutze_camera::camera_trigger_mode::soft_trigger)
  {
    return Fail("当前相机不是软触发模式", error_message);
  }
  if (!camera->soft_trigger_commad())
  {
    return Fail(Camera_Error("软触发"), error_message);
  }
  if (error_message)
  {
    error_message->clear();
  }
  return true;
}

bool Camera_2D_Service::Apply_Configuration(
  unsigned int width,
  unsigned int height,
  jutze_camera::camera_trigger_mode trigger_mode,
  std::string *error_message)
{
  std::shared_ptr<jutze_camera::camera_dahua> camera;
  bool restart = false;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    camera = m_camera;
    restart = m_status.state == Camera_2D_State::Grabbing;
  }
  if (!camera)
  {
    return Fail("请先打开2D相机", error_message);
  }
  if (width == 0 || height == 0)
  {
    return Fail("图像宽度和高度必须大于0", error_message);
  }
  if (trigger_mode == jutze_camera::camera_trigger_mode::unknown_mode)
  {
    return Fail("请选择有效的触发模式", error_message);
  }

  if (restart && !Stop(error_message))
  {
    return false;
  }
  if (!camera->set_width(width) ||
      !camera->set_height(height) ||
      !camera->set_trigger_mode(trigger_mode))
  {
    if (restart)
    {
      Start(nullptr);
    }
    return Fail(
      "应用相机配置失败，数值可能不满足相机的步进或范围要求",
      error_message);
  }
  if (!Update_Status_From_Camera(error_message))
  {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_status.trigger_mode = trigger_mode;
  }
  Clear_Frame();
  if (restart && !Start(error_message))
  {
    return false;
  }
  if (error_message)
  {
    error_message->clear();
  }
  return true;
}

Camera_2D_Status Camera_2D_Service::Status() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_status;
}

std::shared_ptr<const jutze_camera::camera_frame>
Camera_2D_Service::Latest_Frame() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_latest_frame;
}

bool Camera_2D_Service::Is_Open() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return static_cast<bool>(m_camera);
}

bool Camera_2D_Service::Is_Grabbing() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_status.state == Camera_2D_State::Grabbing;
}

void Camera_2D_Service::Publish_Frame(
  const jutze_camera::camera_frame &frame)
{
  auto published =
    std::make_shared<jutze_camera::camera_frame>(frame);
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(m_mutex);
  m_latest_frame = std::move(published);
  m_status.width = frame.m_width;
  m_status.height = frame.m_height;
  m_status.pixel_format =
    static_cast<jutze_camera::pixel_format>(frame.m_pixel_type);
  ++m_fps_window_frames;
  const auto elapsed =
    std::chrono::duration<double>(now - m_fps_window_start).count();
  if (elapsed >= 1.0)
  {
    m_status.frames_per_second =
      static_cast<double>(m_fps_window_frames) / elapsed;
    m_fps_window_frames = 0;
    m_fps_window_start = now;
  }
}

bool Camera_2D_Service::Update_Status_From_Camera(
  std::string *error_message)
{
  std::shared_ptr<jutze_camera::camera_dahua> camera;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    camera = m_camera;
  }
  if (!camera)
  {
    return Fail("2D相机未打开", error_message);
  }

  const auto configuration = camera->get_config();
  const auto width = camera->get_width();
  const auto height = camera->get_height();
  const auto pixel_format = camera->get_pixel_format();
  const auto trigger_mode = camera->get_trigger_mode();
  if (!width || !height || !pixel_format || !trigger_mode)
  {
    return Fail(Camera_Error("读取相机配置"), error_message);
  }

  std::lock_guard<std::mutex> lock(m_mutex);
  m_status.device_name = configuration.device_name;
  m_status.serial_number = configuration.device_serial;
  m_status.width = *width;
  m_status.height = *height;
  m_status.pixel_format = *pixel_format;
  IMV_String trigger_mode_value{};
  const int trigger_mode_result = IMV_GetEnumFeatureSymbol(
    camera->get_handle(), "TriggerMode", &trigger_mode_value);
  m_status.trigger_mode =
    trigger_mode_result == IMV_OK &&
        std::strncmp(trigger_mode_value.str, "Off", 3) == 0
      ? jutze_camera::camera_trigger_mode::auto_trigger
      : *trigger_mode;
  m_status.last_error.clear();
  return true;
}

bool Camera_2D_Service::Fail(
  const std::string &message,
  std::string *error_message)
{
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_status.last_error = message;
    if (m_camera)
    {
      m_status.state = Camera_2D_State::Error;
    }
  }
  if (error_message)
  {
    *error_message = message;
  }
  return false;
}

void Camera_2D_Service::Clear_Frame()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_latest_frame.reset();
}

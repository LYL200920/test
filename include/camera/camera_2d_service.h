#ifndef includeguard_camera_2d_service_h_includeguard
#define includeguard_camera_2d_service_h_includeguard

#include "camera_params.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace jutze_camera
{
class camera_dahua;
struct camera_frame;
}

enum class Camera_2D_State
{
  Closed,
  Opened,
  Grabbing,
  Error
};

struct Camera_2D_Device
{
  std::string key;
  std::string display_name;
};

struct Camera_2D_Status
{
  Camera_2D_State state = Camera_2D_State::Closed;
  std::string device_name;
  std::string serial_number;
  unsigned int width = 0;
  unsigned int height = 0;
  jutze_camera::pixel_format pixel_format =
    jutze_camera::pixel_format::unknown;
  jutze_camera::camera_trigger_mode trigger_mode =
    jutze_camera::camera_trigger_mode::unknown_mode;
  double frames_per_second = 0.0;
  std::string last_error;
};

class Camera_2D_Service
{
public:
  Camera_2D_Service() = default;
  ~Camera_2D_Service();

  Camera_2D_Service(const Camera_2D_Service &) = delete;
  Camera_2D_Service &operator=(const Camera_2D_Service &) = delete;

  bool Refresh_Devices(std::string *error_message = nullptr);
  std::vector<Camera_2D_Device> Devices() const;

  bool Open(const std::string &device_key,
            std::string *error_message = nullptr);
  bool Close(std::string *error_message = nullptr);
  bool Start(std::string *error_message = nullptr);
  bool Stop(std::string *error_message = nullptr);
  bool Software_Trigger(std::string *error_message = nullptr);
  bool Apply_Configuration(
    unsigned int width,
    unsigned int height,
    jutze_camera::camera_trigger_mode trigger_mode,
    std::string *error_message = nullptr);

  Camera_2D_Status Status() const;
  std::shared_ptr<const jutze_camera::camera_frame> Latest_Frame() const;
  bool Is_Open() const;
  bool Is_Grabbing() const;

private:
  void Publish_Frame(const jutze_camera::camera_frame &frame);
  bool Update_Status_From_Camera(std::string *error_message);
  bool Fail(const std::string &message, std::string *error_message);
  void Clear_Frame();

  mutable std::mutex m_mutex;
  std::vector<Camera_2D_Device> m_devices;
  std::shared_ptr<jutze_camera::camera_dahua> m_camera;
  std::shared_ptr<const jutze_camera::camera_frame> m_latest_frame;
  Camera_2D_Status m_status;
  std::chrono::steady_clock::time_point m_fps_window_start;
  unsigned int m_fps_window_frames = 0;
};

#endif

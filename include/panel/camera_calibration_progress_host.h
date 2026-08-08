#ifndef PANEL_CAMERA_CALIBRATION_PROGRESS_HOST_H_
#define PANEL_CAMERA_CALIBRATION_PROGRESS_HOST_H_

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>

namespace jutze_camera
{
struct camera_frame;
}

struct Camera_Calibration_Progress_Summary
{
  std::filesystem::path path;
  std::string robot_model_id;
  std::size_t total_point_count = 0;
  std::size_t capture_point_count = 0;
  std::size_t transition_point_count = 0;
};

struct Camera_Calibration_Progress_Options
{
  int linear_velocity_mm_s = 20;
  int settling_time_ms = 500;
  int image_timeout_ms = 3000;
  std::size_t maximum_detection_retries = 1;
};

struct Camera_Calibration_Progress_Update
{
  bool active = false;
  bool stopping = false;
  std::size_t point_index = 0;
  std::size_t point_count = 0;
  std::size_t point_id = 0;
  std::size_t retry_index = 0;
  std::size_t accepted_image_count = 0;
  std::size_t rejected_point_count = 0;
  std::string message;
};

class Camera_Calibration_Progress_Observer
{
public:
  virtual ~Camera_Calibration_Progress_Observer() = default;
  virtual void On_Calibration_Progress_Update(
    const Camera_Calibration_Progress_Update &update) = 0;
  virtual void On_Calibration_Progress_Frame(
    std::shared_ptr<const jutze_camera::camera_frame> frame,
    std::size_t point_index,
    std::size_t point_id,
    std::size_t retry_index) = 0;
  virtual void On_Calibration_Progress_Finished(
    bool success,
    const std::string &message) = 0;
};

class Camera_Calibration_Progress_Host
{
public:
  virtual ~Camera_Calibration_Progress_Host() = default;
  virtual bool Inspect_Calibration_Progress(
    const std::filesystem::path &path,
    Camera_Calibration_Progress_Summary *summary,
    std::string *error_message) const = 0;
  virtual bool Start_Calibration_Progress(
    const std::filesystem::path &path,
    const Camera_Calibration_Progress_Options &options,
    Camera_Calibration_Progress_Observer *observer,
    std::string *error_message) = 0;
  virtual void Complete_Calibration_Image_Processing(
    bool accepted,
    const std::string &error_message) = 0;
  virtual void Request_Calibration_Progress_Stop() = 0;
  virtual bool Is_Calibration_Progress_Active() const = 0;
  virtual void Detach_Calibration_Progress_Observer(
    Camera_Calibration_Progress_Observer *observer) = 0;
};

#endif // PANEL_CAMERA_CALIBRATION_PROGRESS_HOST_H_

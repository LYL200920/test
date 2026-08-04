#ifndef APPLICATION_CAMERA_INTRINSIC_CALIBRATION_WORKFLOW_H_
#define APPLICATION_CAMERA_INTRINSIC_CALIBRATION_WORKFLOW_H_

#include "intrinsic_calibration.h"

#include <filesystem>
#include <string>
#include <vector>

namespace application
{

struct Camera_Calibration_Capture
{
  camera_calibration::Image_Detection_Result detection;
  std::filesystem::path source_path;
  bool enabled = true;
};

struct Camera_Calibration_Readiness
{
  std::size_t total_image_count = 0;
  std::size_t detected_image_count = 0;
  std::size_t enabled_image_count = 0;
  std::size_t covered_zone_count = 0;
  bool ready = false;
  std::vector<std::string> messages;
};

class Camera_Intrinsic_Calibration_Workflow
{
public:
  explicit Camera_Intrinsic_Calibration_Workflow(
    camera_calibration::Chessboard_Configuration chessboard = {});

  void Reset(camera_calibration::Chessboard_Configuration chessboard);
  const camera_calibration::Chessboard_Configuration &Chessboard() const;

  void Add_Capture(Camera_Calibration_Capture capture);
  bool Remove_Capture(std::size_t index);
  bool Set_Capture_Enabled(std::size_t index, bool enabled);
  const std::vector<Camera_Calibration_Capture> &Captures() const;

  Camera_Calibration_Readiness Readiness(
    std::size_t minimum_view_count = 10) const;

  bool Calibrate(
    const camera_calibration::Calibration_Options &options,
    camera_calibration::Camera_Intrinsics *intrinsics,
    std::string *error_message = nullptr) const;

private:
  camera_calibration::Chessboard_Configuration chessboard_;
  std::vector<Camera_Calibration_Capture> captures_;
};

} // namespace application

#endif // APPLICATION_CAMERA_INTRINSIC_CALIBRATION_WORKFLOW_H_

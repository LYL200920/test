#include "camera_intrinsic_calibration_workflow.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <utility>

namespace application
{

Camera_Intrinsic_Calibration_Workflow::
Camera_Intrinsic_Calibration_Workflow(
  camera_calibration::Chessboard_Configuration chessboard)
  : chessboard_(std::move(chessboard))
{
}

void Camera_Intrinsic_Calibration_Workflow::Reset(
  camera_calibration::Chessboard_Configuration chessboard)
{
  chessboard_ = std::move(chessboard);
  captures_.clear();
}

const camera_calibration::Chessboard_Configuration &
Camera_Intrinsic_Calibration_Workflow::Chessboard() const
{
  return chessboard_;
}

void Camera_Intrinsic_Calibration_Workflow::Add_Capture(
  Camera_Calibration_Capture capture)
{
  capture.enabled = capture.enabled && capture.detection.chessboard_found;
  captures_.push_back(std::move(capture));
}

bool Camera_Intrinsic_Calibration_Workflow::Remove_Capture(
  std::size_t index)
{
  if (index >= captures_.size())
  {
    return false;
  }
  captures_.erase(captures_.begin() + static_cast<std::ptrdiff_t>(index));
  return true;
}

bool Camera_Intrinsic_Calibration_Workflow::Set_Capture_Enabled(
  std::size_t index,
  bool enabled)
{
  if (index >= captures_.size() ||
      (enabled && !captures_[index].detection.chessboard_found))
  {
    return false;
  }
  captures_[index].enabled = enabled;
  return true;
}

const std::vector<Camera_Calibration_Capture> &
Camera_Intrinsic_Calibration_Workflow::Captures() const
{
  return captures_;
}

Camera_Calibration_Readiness
Camera_Intrinsic_Calibration_Workflow::Readiness(
  std::size_t minimum_view_count) const
{
  Camera_Calibration_Readiness result;
  result.total_image_count = captures_.size();
  std::array<bool, 9> covered_zones{};
  bool small_board_seen = false;
  for (const Camera_Calibration_Capture &capture : captures_)
  {
    const auto &detection = capture.detection;
    if (!detection.chessboard_found)
    {
      continue;
    }
    ++result.detected_image_count;
    if (!capture.enabled)
    {
      continue;
    }
    ++result.enabled_image_count;
    if (detection.corners.empty() || detection.image_width <= 0 ||
        detection.image_height <= 0)
    {
      continue;
    }
    double minimum_x = detection.corners.front().x;
    double maximum_x = minimum_x;
    double minimum_y = detection.corners.front().y;
    double maximum_y = minimum_y;
    for (const auto &corner : detection.corners)
    {
      minimum_x = std::min(minimum_x, corner.x);
      maximum_x = std::max(maximum_x, corner.x);
      minimum_y = std::min(minimum_y, corner.y);
      maximum_y = std::max(maximum_y, corner.y);
    }
    const double center_x = (minimum_x + maximum_x) * 0.5 /
                            detection.image_width;
    const double center_y = (minimum_y + maximum_y) * 0.5 /
                            detection.image_height;
    const int zone_x = std::clamp(static_cast<int>(center_x * 3.0), 0, 2);
    const int zone_y = std::clamp(static_cast<int>(center_y * 3.0), 0, 2);
    covered_zones[static_cast<std::size_t>(zone_y * 3 + zone_x)] = true;
    const double width_ratio =
      (maximum_x - minimum_x) / detection.image_width;
    const double height_ratio =
      (maximum_y - minimum_y) / detection.image_height;
    small_board_seen = small_board_seen ||
      width_ratio < 0.15 || height_ratio < 0.15;
  }
  result.covered_zone_count = static_cast<std::size_t>(std::count(
    covered_zones.begin(), covered_zones.end(), true));
  if (result.enabled_image_count < minimum_view_count)
  {
    result.messages.push_back(
      "有效图像不足，至少需要 " + std::to_string(minimum_view_count) + " 张");
  }
  if (result.covered_zone_count < 5)
  {
    result.messages.push_back("棋盘中心至少需要覆盖九宫格中的 5 个区域");
  }
  if (small_board_seen)
  {
    result.messages.push_back(
      "部分图像中的棋盘偏小，建议棋盘覆盖图像宽高的 20% 以上");
  }
  result.ready = result.enabled_image_count >= minimum_view_count &&
                 result.covered_zone_count >= 5;
  return result;
}

bool Camera_Intrinsic_Calibration_Workflow::Calibrate(
  const camera_calibration::Calibration_Options &options,
  camera_calibration::Camera_Intrinsics *intrinsics,
  std::string *error_message) const
{
  camera_calibration::Intrinsic_Calibration_Session session(chessboard_);
  for (const Camera_Calibration_Capture &capture : captures_)
  {
    if (!capture.enabled || !capture.detection.chessboard_found)
    {
      continue;
    }
    if (!session.Add_Observation(
          capture.detection.image_width,
          capture.detection.image_height,
          capture.detection.corners,
          capture.detection.image_id,
          error_message))
    {
      return false;
    }
  }
  return session.Calibrate(options, intrinsics, error_message);
}

} // namespace application

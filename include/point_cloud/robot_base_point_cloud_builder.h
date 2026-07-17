#ifndef includeguard_robot_base_point_cloud_builder_h_includeguard
#define includeguard_robot_base_point_cloud_builder_h_includeguard

#include "camera_frame.h"
#include "eye_to_hand_calibration.h"
#include "point_cloud_data.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace point_cloud
{

struct Robot_Base_Point_Cloud
{
  Point_Cloud_Data data;
  std::uint32_t source_frame_number = 0;
  std::size_t source_point_count = 0;
  std::size_t filtered_point_count = 0;
  double median_depth_mm = 0.0;
  std::array<double, 6> world_bounds_mm = {};
};

bool Calculate_Point_Cloud_Bounds (
  const Point_Cloud_Data& cloud,
  std::array<double, 6>* bounds,
  std::string* error_message = nullptr);

bool Are_Robot_Base_Bounds_Reasonable (
  const std::array<double, 6>& bounds);

bool Build_Robot_Base_Point_Cloud (
  const Camera_Frame& frame,
  const robot_model::Eye_To_Hand_Calibration& calibration,
  Robot_Base_Point_Cloud* output,
  std::string* error_message = nullptr);

} // namespace point_cloud

#endif

#ifndef includeguard_eye_to_hand_calibration_h_includeguard
#define includeguard_eye_to_hand_calibration_h_includeguard

#include "pose_transform.h"

#include <cstdint>
#include <filesystem>
#include <string>

namespace robot_model
{

struct Eye_To_Hand_Calibration
{
  Matrix4 camera_to_robot = { };
  std::uint32_t camera_coordinate_type = 0;
  double point_cloud_unit_to_mm = 1.0;

  Matrix4 World_From_Camera ( ) const;
};

bool Load_Eye_To_Hand_Calibration (
  const std::filesystem::path& path,
  Eye_To_Hand_Calibration* calibration,
  std::string* error_message = nullptr);

} // namespace robot_model

#endif

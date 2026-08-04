#ifndef CAMERA_CALIBRATION_CAMERA_INTRINSICS_REPOSITORY_H_
#define CAMERA_CALIBRATION_CAMERA_INTRINSICS_REPOSITORY_H_

#include "intrinsic_calibration.h"

#include <filesystem>
#include <string>

namespace camera_calibration
{

bool Save_Camera_Intrinsics(
  const std::filesystem::path &path,
  const Chessboard_Configuration &chessboard,
  const Camera_Intrinsics &intrinsics,
  std::string *error_message = nullptr);

bool Load_Camera_Intrinsics(
  const std::filesystem::path &path,
  Chessboard_Configuration *chessboard,
  Camera_Intrinsics *intrinsics,
  std::string *error_message = nullptr);

} // namespace camera_calibration

#endif // CAMERA_CALIBRATION_CAMERA_INTRINSICS_REPOSITORY_H_

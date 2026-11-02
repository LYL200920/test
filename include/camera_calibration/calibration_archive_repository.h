#ifndef CAMERA_CALIBRATION_ARCHIVE_REPOSITORY_H_
#define CAMERA_CALIBRATION_ARCHIVE_REPOSITORY_H_

#include "calibration_geometry.h"
#include <filesystem>

namespace camera_calibration
{
// Separate versioned format. Never reads/writes legacy intrinsics, Tool or 3D
// hand-eye files implicitly. Save refuses an existing destination (no overwrite).
bool Save_Calibration_Archive(const std::filesystem::path &path,
  const Calibration_Archive &archive, std::string *error = nullptr);
// Strict schema/numeric parsing. Output remains unchanged if loading fails.
bool Load_Calibration_Archive(const std::filesystem::path &path,
  Calibration_Archive *archive, std::string *error = nullptr);
}
#endif

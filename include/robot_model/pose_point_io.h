#ifndef includeguard_pose_point_io_h_includeguard
#define includeguard_pose_point_io_h_includeguard

#include "pose_transform.h"

#include <filesystem>
#include <string>
#include <vector>

namespace robot_model
{

struct Pose_Point_File
{
  XyzabcPose default_pose = { };
  std::vector<XyzabcPose> transform_points;
};

bool Load_Pose_Point_File (const std::filesystem::path& path,
                           Pose_Point_File* point_file,
                           std::string* error_message);

} // namespace robot_model

#endif

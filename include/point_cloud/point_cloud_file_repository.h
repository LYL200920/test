#ifndef includeguard_point_cloud_file_repository_h_includeguard
#define includeguard_point_cloud_file_repository_h_includeguard

#include "point_cloud_data.h"
#include "point_cloud_persistence.h"

#include <cstdint>
#include <filesystem>
#include <string>

namespace point_cloud
{

struct Point_Cloud_File_Save_Result
{
  bool success = false;
  std::filesystem::path path;
  std::string error_message;
};

std::filesystem::path Point_Cloud_Resource_Root ( );

Point_Cloud_File_Save_Result Save_Robot_Base_Point_Cloud_To_File (
  const std::filesystem::path& path,
  const Point_Cloud_Data& cloud,
  const std::string& robot_model_id,
  std::uint32_t source_frame_number);

Point_Cloud_File_Save_Result Save_Robot_Base_Point_Cloud_To_Resource (
  const Point_Cloud_Data& cloud,
  const std::string& robot_model_id,
  std::uint32_t source_frame_number);

bool Load_Robot_Base_Point_Cloud_For_Model (
  const std::filesystem::path& path,
  const std::string& expected_robot_model,
  Point_Cloud_Data* cloud,
  Point_Cloud_File_Metadata* metadata,
  std::string* error_message = nullptr);

} // namespace point_cloud

#endif

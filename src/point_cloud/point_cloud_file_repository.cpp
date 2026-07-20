#include "point_cloud_file_repository.h"

#include <chrono>
#include <cctype>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace point_cloud
{
namespace
{

std::string safe_file_component (const std::string& value)
{
  std::string result = value;
  for( char& character : result )
  {
    const auto unsigned_character = static_cast<unsigned char> (character);
    if( !std::isalnum (unsigned_character) && character != '-' &&
        character != '_' )
    {
      character = '_';
    }
  }
  return result.empty ( ) ? "robot" : result;
}

std::string timestamp_for_file_name (std::chrono::system_clock::time_point now)
{
  const std::time_t time = std::chrono::system_clock::to_time_t (now);
  std::tm local_time = { };
  localtime_s (&local_time, &time);
  std::ostringstream text;
  text << std::put_time (&local_time, "%Y%m%d_%H%M%S");
  return text.str ( );
}

} // namespace

std::filesystem::path Point_Cloud_Resource_Root ( )
{
  return std::filesystem::path (POINT_CLOUD_RESOURCE_ROOT);
}

Point_Cloud_File_Save_Result Save_Robot_Base_Point_Cloud_To_File (
  const std::filesystem::path& path,
  const Point_Cloud_Data& cloud,
  const std::string& robot_model_id,
  std::uint32_t source_frame_number)
{
  Point_Cloud_File_Save_Result result;
  if( path.empty ( ) )
  {
    result.error_message = "Point-cloud save path is empty";
    return result;
  }

  std::error_code directory_error;
  if( !path.parent_path ( ).empty ( ) )
  {
    std::filesystem::create_directories (path.parent_path ( ), directory_error);
  }
  if( directory_error )
  {
    result.error_message = "Unable to create point-cloud directory: " +
      directory_error.message ( );
    return result;
  }

  Point_Cloud_File_Metadata metadata;
  metadata.coordinate_frame = "robot_base";
  metadata.unit = "millimeter";
  metadata.robot_model = robot_model_id;
  metadata.source_frame_number = source_frame_number;
  metadata.saved_unix_milliseconds =
    std::chrono::duration_cast<std::chrono::milliseconds> (
      std::chrono::system_clock::now ( ).time_since_epoch ( )).count ( );
  if( !Save_Robot_Base_Point_Cloud_Ply (
        path, cloud, metadata, &result.error_message) )
  {
    return result;
  }
  result.success = true;
  result.path = path;
  return result;
}

Point_Cloud_File_Save_Result Save_Robot_Base_Point_Cloud_To_Resource (
  const Point_Cloud_Data& cloud,
  const std::string& robot_model_id,
  std::uint32_t source_frame_number)
{
  Point_Cloud_File_Save_Result result;
  const auto resource_root = Point_Cloud_Resource_Root ( );
  std::error_code directory_error;
  std::filesystem::create_directories (resource_root, directory_error);
  if( directory_error )
  {
    result.error_message = "Unable to create Resource/PointCloud: " +
      directory_error.message ( );
    return result;
  }

  const auto now = std::chrono::system_clock::now ( );
  const std::string base_name = safe_file_component (robot_model_id) + "_" +
    timestamp_for_file_name (now) + "_frame_" +
    std::to_string (source_frame_number);
  auto path = resource_root / ( base_name + ".ply" );
  for( int suffix = 2; std::filesystem::exists (path); ++suffix )
  {
    path = resource_root /
      ( base_name + "_" + std::to_string (suffix) + ".ply" );
  }

  return Save_Robot_Base_Point_Cloud_To_File (
    path, cloud, robot_model_id, source_frame_number);
}

bool Load_Robot_Base_Point_Cloud_For_Model (
  const std::filesystem::path& path,
  const std::string& expected_robot_model,
  Point_Cloud_Data* cloud,
  Point_Cloud_File_Metadata* metadata,
  std::string* error_message)
{
  if( !Load_Robot_Base_Point_Cloud_Ply (
        path, cloud, metadata, error_message) )
  {
    return false;
  }
  if( !expected_robot_model.empty ( ) &&
      metadata->robot_model != expected_robot_model )
  {
    if( cloud ) *cloud = {};
    if( error_message )
    {
      *error_message = "Point-cloud file belongs to robot " +
        metadata->robot_model + ", current robot is " +
        expected_robot_model;
    }
    return false;
  }
  return true;
}

} // namespace point_cloud

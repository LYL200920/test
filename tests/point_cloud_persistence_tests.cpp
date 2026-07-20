#include "point_cloud_file_repository.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

std::filesystem::path temporary_ply_path (const char* prefix)
{
  const auto suffix = std::chrono::high_resolution_clock::now ( )
    .time_since_epoch ( ).count ( );
  return std::filesystem::temp_directory_path ( ) /
    ( std::string (prefix) + std::to_string (suffix) + ".ply" );
}

void require (bool condition, const std::string& message)
{
  if( !condition ) throw std::runtime_error (message);
}

void test_binary_ply_round_trip ( )
{
  const auto path = temporary_ply_path ("robot_base_point_cloud_test_");

  point_cloud::Point_Cloud_Data source;
  source.xyz = { 100.5f, -20.25f, 900.0f, 101.5f, -19.25f, 901.0f };
  source.rgb = { 10, 20, 30, 40, 50, 60 };
  point_cloud::Point_Cloud_File_Metadata source_metadata;
  source_metadata.coordinate_frame = "robot_base";
  source_metadata.unit = "millimeter";
  source_metadata.robot_model = "KR10_R1100_2";
  source_metadata.source_frame_number = 17;
  source_metadata.saved_unix_milliseconds = 123456789;

  std::string error;
  require (point_cloud::Save_Robot_Base_Point_Cloud_Ply (
             path, source, source_metadata, &error),
           "Saving robot-base PLY failed: " + error);
  require (!point_cloud::Save_Robot_Base_Point_Cloud_Ply (
             path, source, source_metadata, &error),
           "Existing robot-base PLY was overwritten");

  point_cloud::Point_Cloud_Data loaded;
  point_cloud::Point_Cloud_File_Metadata loaded_metadata;
  require (point_cloud::Load_Robot_Base_Point_Cloud_Ply (
             path, &loaded, &loaded_metadata, &error),
           "Loading robot-base PLY failed: " + error);
  require (loaded.xyz == source.xyz, "PLY XYZ round trip mismatch");
  require (loaded.rgb == source.rgb, "PLY RGB round trip mismatch");
  require (loaded_metadata.coordinate_frame == "robot_base" &&
             loaded_metadata.unit == "millimeter" &&
             loaded_metadata.robot_model == "KR10_R1100_2" &&
             loaded_metadata.source_frame_number == 17 &&
             loaded_metadata.saved_unix_milliseconds == 123456789,
           "PLY metadata round trip mismatch");

  point_cloud::Point_Cloud_Data wrong_model_cloud;
  point_cloud::Point_Cloud_File_Metadata wrong_model_metadata;
  require (!point_cloud::Load_Robot_Base_Point_Cloud_For_Model (
             path, "OTHER_ROBOT", &wrong_model_cloud,
             &wrong_model_metadata, &error),
           "Point-cloud repository accepted a file for another robot");

  std::filesystem::remove (path);
}

void test_plain_ply_is_rejected_for_overlay ( )
{
  const auto path = temporary_ply_path ("plain_point_cloud_test_");
  {
    std::ofstream file (path, std::ios::binary);
    file << "ply\nformat ascii 1.0\nelement vertex 1\n"
            "property float x\nproperty float y\nproperty float z\n"
            "end_header\n0 0 0\n";
  }
  point_cloud::Point_Cloud_Data cloud;
  point_cloud::Point_Cloud_File_Metadata metadata;
  std::string error;
  require (!point_cloud::Load_Robot_Base_Point_Cloud_Ply (
             path, &cloud, &metadata, &error),
           "Plain PLY was incorrectly accepted as a robot-base overlay");
  require (!error.empty ( ), "Rejected plain PLY did not report an error");
  std::filesystem::remove (path);
}

void test_repository_saves_to_user_selected_path ( )
{
  const auto path = temporary_ply_path ("custom_point_cloud_name_");
  point_cloud::Point_Cloud_Data source;
  source.xyz = { 10.0f, 20.0f, 30.0f };

  const auto saved = point_cloud::Save_Robot_Base_Point_Cloud_To_File (
    path, source, "TEST_ROBOT", 42);
  require (saved.success, "Repository custom-path save failed: " +
                          saved.error_message);
  require (saved.path == path, "Repository changed the selected save path");

  point_cloud::Point_Cloud_Data loaded;
  point_cloud::Point_Cloud_File_Metadata metadata;
  std::string error;
  require (point_cloud::Load_Robot_Base_Point_Cloud_For_Model (
             path, "TEST_ROBOT", &loaded, &metadata, &error),
           "Custom-path point cloud could not be loaded: " + error);
  require (loaded.xyz == source.xyz, "Custom-path point cloud data mismatch");
  require (metadata.source_frame_number == 42,
           "Custom-path point cloud frame metadata mismatch");
  std::filesystem::remove (path);
}

} // namespace

int main ( )
{
  try
  {
    test_binary_ply_round_trip ( );
    test_plain_ply_is_rejected_for_overlay ( );
    test_repository_saves_to_user_selected_path ( );
  }
  catch( const std::exception& error )
  {
    std::cerr << "FAILED: " << error.what ( ) << '\n';
    return 1;
  }
  std::cout << "Point-cloud persistence tests passed.\n";
  return 0;
}

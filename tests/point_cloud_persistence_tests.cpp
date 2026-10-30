#include "point_cloud_file_repository.h"
#include "point_cloud_renderer.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
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

void test_strict_ply_validation ( )
{
  const std::string prefix =
    "ply\nformat binary_little_endian 1.0\n"
    "comment application robot_point_cloud_overlay\n"
    "comment coordinate_frame robot_base\ncomment unit millimeter\n";
  const std::string xyz_properties =
    "property float x\nproperty float y\nproperty float z\n";
  const std::string header = prefix + "element vertex 1\n" +
    xyz_properties + "end_header\n";
  const float xyz[] = { 1.0f, 2.0f, 3.0f };
  const std::string data (reinterpret_cast<const char*> (xyz), sizeof (xyz));
  const auto check = [&] (const std::string& contents, bool expected,
                         const std::string& scenario)
  {
    const auto path = temporary_ply_path ("strict_ply_test_");
    {
      std::ofstream file (path, std::ios::binary);
      file.write (contents.data ( ), contents.size ( ));
    }
    point_cloud::Point_Cloud_Data cloud;
    cloud.xyz = { 9.0f, 9.0f, 9.0f };
    point_cloud::Point_Cloud_File_Metadata metadata;
    metadata.robot_model = "old output";
    std::string error;
    const bool result = point_cloud::Load_Robot_Base_Point_Cloud_Ply (
      path, &cloud, &metadata, &error);
    std::filesystem::remove (path);
    require (result == expected, scenario + ": " + error);
    if( !expected )
    {
      require (!error.empty ( ), scenario + " did not report an error");
      require (cloud.xyz.empty ( ) && cloud.rgb.empty ( ) &&
                 metadata.robot_model.empty ( ) &&
                 metadata.coordinate_frame.empty ( ),
               scenario + " left partial output");
    }
    else
    {
      require (cloud.xyz == std::vector<float> ({ 1.0f, 2.0f, 3.0f }),
               scenario + " changed coordinates");
    }
  };

  check (header + data, true, "Valid uncolored layout");
  check (prefix + "comment source_frame_number 4294967295\n"
           "comment saved_unix_milliseconds 9223372036854775807\n"
           "element vertex 1\n" + xyz_properties + "end_header\n" + data,
         true, "Maximum numeric metadata");
  std::string crlf_header;
  for( const char character : header )
  {
    if( character == '\n' ) crlf_header += '\r';
    crlf_header += character;
  }
  check (crlf_header + data, true, "CRLF header");
  check (prefix + "comment optional extension\nelement vertex 1\n" +
           xyz_properties + "end_header\n" + data,
         true, "Unknown comment");

  const std::vector<std::string> invalid_layouts = {
    "property float y\nproperty float x\nproperty float z\n",
    "property double x\nproperty double y\nproperty double z\n",
    "property float x\nproperty float x\nproperty float z\n",
    "property list uchar float x\nproperty float y\nproperty float z\n",
    xyz_properties + "property uchar red\n",
    xyz_properties + "property uchar blue\nproperty uchar green\nproperty uchar red\n",
    xyz_properties + "property float red\nproperty uchar green\nproperty uchar blue\n",
    xyz_properties + "property float intensity\n"
  };
  for( const auto& properties : invalid_layouts )
    check (prefix + "element vertex 1\n" + properties + "end_header\n" + data,
           false, "Unsupported properties");
  check (prefix + xyz_properties + "element vertex 1\nend_header\n" + data,
         false, "Properties outside vertex element");
  check (prefix + "element face 1\nelement vertex 1\n" + xyz_properties +
           "end_header\n" + data, false, "Additional element before vertex");
  check (prefix + "element vertex 1\n" + xyz_properties +
           "element face 0\nend_header\n" + data,
         false, "Additional element after vertex");
  check (prefix + "element vertex 1\nelement vertex 1\n" + xyz_properties +
           "end_header\n" + data, false, "Repeated vertex element");
  for( const auto* count : { "0", "-1", "1junk", "10000001", "18446744073709551616" } )
    check (prefix + "element vertex " + count + "\n" + xyz_properties +
             "end_header\n" + data, false, "Invalid point count");
  for( const auto* value : { "-1", "4294967296", "17junk", "" } )
    check (prefix + "comment source_frame_number " + value +
             "\nelement vertex 1\n" + xyz_properties + "end_header\n" + data,
           false, "Invalid frame metadata");
  for( const auto* value : { "123junk", "9223372036854775808", "" } )
    check (prefix + "comment saved_unix_milliseconds " + value +
             "\nelement vertex 1\n" + xyz_properties + "end_header\n" + data,
           false, "Invalid time metadata");
  check (prefix + "element vertex 1\n" + xyz_properties,
         false, "Missing end_header");
  check (prefix + "end_header\n" + data, false, "Missing vertex layout");
  check (header + data.substr (0, data.size ( ) - 1), false, "Truncated data");
  check (header + data + "x", false, "Trailing data");
  check ("comment before signature\n" + header + data, false, "Missing first-line signature");
  check (prefix + "format ascii 1.0\nelement vertex 1\n" + xyz_properties +
           "end_header\n" + data, false, "Conflicting format");
  check (prefix + "comment " + std::string (65536, 'x') + "\n" +
           "element vertex 1\n" + xyz_properties + "end_header\n" + data,
         false, "Oversized header line");
  check (prefix + "comment hidden" + std::string (1, '\0') +
           " data\nelement vertex 1\n" + xyz_properties + "end_header\n" + data,
         false, "Null byte in header");
  std::string oversized_header = prefix;
  for( int index = 0; index < 7000; ++index )
    oversized_header += "comment x\n";
  check (oversized_header + "element vertex 1\n" + xyz_properties +
           "end_header\n" + data, false, "Oversized multi-line header");
  check (prefix + "element vertex 10000000\n" + xyz_properties +
           "end_header\n" + data, false, "Large count with short payload");
  for( const auto value : { std::numeric_limits<float>::quiet_NaN ( ),
                           std::numeric_limits<float>::infinity ( ),
                           -std::numeric_limits<float>::infinity ( ) } )
  {
    const float invalid_xyz[] = { 1.0f, value, 3.0f };
    check (header + std::string (reinterpret_cast<const char*> (invalid_xyz),
                                sizeof (invalid_xyz)),
           false, "Non-finite coordinate");
  }
}

void test_save_rejects_unreadable_metadata ( )
{
  const auto path = temporary_ply_path ("invalid_ply_metadata_");
  point_cloud::Point_Cloud_Data cloud;
  cloud.xyz = { 1.0f, 2.0f, 3.0f };
  point_cloud::Point_Cloud_File_Metadata metadata;
  metadata.coordinate_frame = "robot_base";
  metadata.unit = "millimeter";
  std::string error;
  for( const auto& name : { std::string (65536, 'x'),
                           std::string ("robot\0hidden", 12) } )
  {
    metadata.robot_model = name;
    require (!point_cloud::Save_Robot_Base_Point_Cloud_Ply (
               path, cloud, metadata, &error),
             "Save accepted metadata the loader cannot read");
    require (!std::filesystem::exists (path) &&
               !std::filesystem::exists (path.string ( ) + ".tmp"),
             "Rejected metadata left an output file");
  }
}

void test_null_output_clears_other_output ( )
{
  point_cloud::Point_Cloud_Data cloud;
  cloud.xyz = { 1.0f, 2.0f, 3.0f };
  point_cloud::Point_Cloud_File_Metadata metadata;
  metadata.robot_model = "previous result";
  std::string error;
  require (!point_cloud::Load_Robot_Base_Point_Cloud_Ply (
             { }, &cloud, nullptr, &error) && cloud.xyz.empty ( ),
           "Null metadata output left stale point data");
  require (!point_cloud::Load_Robot_Base_Point_Cloud_Ply (
             { }, nullptr, &metadata, &error) && metadata.robot_model.empty ( ),
           "Null cloud output left stale metadata");
}

void test_interactive_render_lod ( )
{
  constexpr std::size_t point_count = 200001;
  std::vector<float> xyz;
  xyz.reserve (point_count * 3);
  for( std::size_t index = 0; index < point_count; ++index )
  {
    xyz.push_back (static_cast<float> (index + 1));
    xyz.push_back (1.0f);
    xyz.push_back (2.0f);
  }

  point_cloud::Point_Cloud_Renderer renderer;
  std::string error;
  require (renderer.Set_Point_Data (xyz, { }, &error),
           "LOD point cloud was rejected: " + error);
  require (renderer.Displayed_Point_Count ( ) == point_count,
           "Full point count was not initially displayed");
  renderer.Set_Interactive_LOD (true);
  require (renderer.Interactive_LOD_Enabled ( ),
           "Interactive point-cloud LOD was not enabled");
  require (renderer.Displayed_Point_Count ( ) <= 100000 &&
           renderer.Displayed_Point_Count ( ) < point_count,
           "Interactive point-cloud LOD did not reduce display points");
  renderer.Set_Interactive_LOD (false);
  require (renderer.Displayed_Point_Count ( ) == point_count,
           "Full point cloud was not restored after interaction");
}

} // namespace

int main ( )
{
  try
  {
    test_binary_ply_round_trip ( );
    test_plain_ply_is_rejected_for_overlay ( );
    test_repository_saves_to_user_selected_path ( );
    test_strict_ply_validation ( );
    test_save_rejects_unreadable_metadata ( );
    test_null_output_clears_other_output ( );
    test_interactive_render_lod ( );
  }
  catch( const std::exception& error )
  {
    std::cerr << "FAILED: " << error.what ( ) << '\n';
    return 1;
  }
  std::cout << "Point-cloud persistence tests passed.\n";
  return 0;
}

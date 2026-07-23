#ifndef includeguard_point_cloud_persistence_h_includeguard
#define includeguard_point_cloud_persistence_h_includeguard

#include "point_cloud_data.h"

#include <cstdint>
#include <filesystem>
#include <string>

namespace point_cloud
{

  struct Point_Cloud_File_Metadata
  {
    std::string coordinate_frame;
    std::string unit;
    std::string robot_model;
    std::uint32_t source_frame_number = 0;
    std::int64_t saved_unix_milliseconds = 0;
  };

  bool Save_Robot_Base_Point_Cloud_Ply(const std::filesystem::path &path,
                                       const Point_Cloud_Data &cloud,
                                       const Point_Cloud_File_Metadata &metadata,
                                       std::string *error_message = nullptr);

  bool Load_Robot_Base_Point_Cloud_Ply(const std::filesystem::path &path,
                                       Point_Cloud_Data *cloud,
                                       Point_Cloud_File_Metadata *metadata,
                                       std::string *error_message = nullptr);

} // namespace point_cloud

#endif

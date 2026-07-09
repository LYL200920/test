#ifndef includeguard_camera_point_cloud_converter_h_includeguard
#define includeguard_camera_point_cloud_converter_h_includeguard

#include "camera_frame.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct Camera_Point_Cloud
{
  std::uint32_t source_image_type = 0;
  std::uint32_t source_frame_number = 0;
  std::uint32_t source_width = 0;
  std::uint32_t source_height = 0;
  std::size_t source_point_count = 0;
  std::vector<float> xyz;
  std::vector<std::uint8_t> rgb;

  std::size_t Point_Count ( ) const { return xyz.size ( ) / 3; }
  bool Has_Color ( ) const
  {
    return !rgb.empty ( ) && rgb.size ( ) == Point_Count ( ) * 3;
  }
};

bool Convert_Camera_Frame_To_Point_Cloud (
  const Camera_Frame& frame,
  Camera_Point_Cloud* destination,
  std::string* error = nullptr,
  std::size_t maximum_points = 500000);

#endif

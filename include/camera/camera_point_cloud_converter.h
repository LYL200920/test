#ifndef includeguard_camera_point_cloud_converter_h_includeguard
#define includeguard_camera_point_cloud_converter_h_includeguard

#include "camera_frame.h"
#include "point_cloud_data.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct Camera_Point_Cloud : public point_cloud::Point_Cloud_Data
{
  std::uint32_t source_image_type = 0;
  std::uint32_t source_frame_number = 0;
  std::uint32_t source_coordinate_type = 0;
  std::uint32_t source_width = 0;
  std::uint32_t source_height = 0;
  std::size_t source_point_count = 0;
};

struct Camera_Point_Cloud_Filter_Options
{
  double coordinate_to_mm = 1.0;
  double minimum_depth_mm = 50.0;
  double maximum_depth_mm = 10000.0;
  double maximum_absolute_coordinate_mm = 20000.0;
  std::size_t maximum_points = 150000;
};

struct Camera_Point_Cloud_Filter_Stats
{
  std::size_t input_points = 0;
  std::size_t valid_points = 0;
  std::size_t output_points = 0;
  std::size_t rejected_points = 0;
  double raw_positive_depth_min = 0.0;
  double raw_positive_depth_median = 0.0;
  double raw_positive_depth_max = 0.0;
  double median_depth_mm = 0.0;
  std::array<double, 6> camera_bounds_mm = {};
};

bool Convert_Camera_Frame_To_Point_Cloud (
  const Camera_Frame& frame,
  Camera_Point_Cloud* destination,
  std::string* error = nullptr,
  std::size_t maximum_points = 500000,
  std::uint32_t required_coordinate_type = 0);

bool Prepare_Camera_Point_Cloud_For_Overlay (
  Camera_Point_Cloud* cloud,
  const Camera_Point_Cloud_Filter_Options& options,
  Camera_Point_Cloud_Filter_Stats* stats = nullptr,
  std::string* error = nullptr);

#endif

#include "camera_point_cloud_converter.h"

#include "Mv3dRgbdDefine.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
void Require (bool condition, const char* message)
{
  if( !condition )
  {
    throw std::runtime_error (message);
  }
}

void Append_Float (std::vector<std::uint8_t>* data, float value)
{
  const auto old_size = data->size ( );
  data->resize (old_size + sizeof (value));
  std::memcpy (data->data ( ) + old_size, &value, sizeof (value));
}

void Append_Xyz (std::vector<std::uint8_t>* data, float x, float y, float z)
{
  Append_Float (data, x);
  Append_Float (data, y);
  Append_Float (data, z);
}

void Test_Xyz_Filters_Invalid_Points ( )
{
  Camera_Frame frame;
  Camera_Image_Frame image;
  image.image_type = static_cast<std::uint32_t> (ImageType_PointCloud);
  image.width = 3;
  image.height = 1;
  image.frame_number = 42;
  image.coordinate_type = static_cast<std::uint32_t> (CoordinateType_Depth);
  Append_Xyz (&image.data, 1.0f, 2.0f, 3.0f);
  Append_Xyz (&image.data, 0.0f, 0.0f, 0.0f);
  Append_Xyz (&image.data, 4.0f, 5.0f, 6.0f);
  frame.images.push_back (std::move (image));

  Camera_Point_Cloud cloud;
  Require (
    Convert_Camera_Frame_To_Point_Cloud (frame, &cloud),
    "XYZ point-cloud conversion failed");
  Require (cloud.Point_Count ( ) == 2, "Invalid XYZ point was not filtered");
  Require (cloud.xyz[3] == 4.0f && cloud.xyz[5] == 6.0f,
           "XYZ coordinates were decoded incorrectly");
  Require (!cloud.Has_Color ( ), "Plain XYZ cloud unexpectedly has color");
  Require (cloud.source_frame_number == 42, "Frame metadata was not copied");
  Require (cloud.source_coordinate_type ==
             static_cast<std::uint32_t> (CoordinateType_Depth),
           "Point-cloud coordinate type was not copied");
}

void Test_Textured_Cloud_Is_Preferred ( )
{
  Camera_Frame frame;
  Camera_Image_Frame plain;
  plain.image_type = static_cast<std::uint32_t> (ImageType_PointCloud);
  plain.width = plain.height = 1;
  Append_Xyz (&plain.data, 9.0f, 9.0f, 9.0f);
  frame.images.push_back (std::move (plain));

  Camera_Image_Frame textured;
  textured.image_type =
    static_cast<std::uint32_t> (ImageType_TexturedPointCloud);
  textured.width = textured.height = 1;
  textured.coordinate_type = static_cast<std::uint32_t> (CoordinateType_RGB);
  Append_Xyz (&textured.data, 1.0f, 2.0f, 3.0f);
  textured.data.insert (textured.data.end ( ), { 10, 20, 30, 255 });
  frame.images.push_back (std::move (textured));

  Camera_Point_Cloud cloud;
  Require (
    Convert_Camera_Frame_To_Point_Cloud (frame, &cloud),
    "Textured point-cloud conversion failed");
  Require (cloud.Point_Count ( ) == 1, "Unexpected textured point count");
  Require (cloud.xyz[0] == 1.0f, "Textured cloud was not preferred");
  Require (cloud.Has_Color ( ), "Textured cloud color is missing");
  Require (cloud.source_coordinate_type ==
             static_cast<std::uint32_t> (CoordinateType_RGB),
           "Preferred point-cloud coordinate type was not copied");
  Require (cloud.rgb == std::vector<std::uint8_t> ({ 10, 20, 30 }),
           "Textured cloud color was decoded incorrectly");
}

void Test_Point_Limit_Downsamples ( )
{
  Camera_Frame frame;
  Camera_Image_Frame image;
  image.image_type = static_cast<std::uint32_t> (ImageType_PointCloud);
  image.width = 5;
  image.height = 1;
  for( int i = 1; i <= 5; ++i )
  {
    Append_Xyz (&image.data, static_cast<float> (i), 1.0f, 1.0f);
  }
  frame.images.push_back (std::move (image));

  Camera_Point_Cloud cloud;
  Require (
    Convert_Camera_Frame_To_Point_Cloud (frame, &cloud, nullptr, 2),
    "Downsampled point-cloud conversion failed");
  Require (cloud.Point_Count ( ) == 2, "Point limit was not enforced");
  Require (cloud.xyz[0] == 1.0f && cloud.xyz[3] == 4.0f,
           "Point downsampling step is incorrect");
}

void Test_Required_Depth_Coordinate_Is_Selected ( )
{
  Camera_Frame frame;
  Camera_Image_Frame depth;
  depth.image_type = static_cast<std::uint32_t> (ImageType_PointCloud);
  depth.coordinate_type =
    static_cast<std::uint32_t> (CoordinateType_Depth);
  depth.width = depth.height = 1;
  Append_Xyz (&depth.data, 100.0f, 200.0f, 1000.0f);
  frame.images.push_back (std::move (depth));

  Camera_Image_Frame rgb;
  rgb.image_type =
    static_cast<std::uint32_t> (ImageType_TexturedPointCloud);
  rgb.coordinate_type = static_cast<std::uint32_t> (CoordinateType_RGB);
  rgb.width = rgb.height = 1;
  Append_Xyz (&rgb.data, 9.0f, 9.0f, 9.0f);
  rgb.data.insert (rgb.data.end ( ), { 10, 20, 30, 255 });
  frame.images.push_back (std::move (rgb));

  Camera_Point_Cloud cloud;
  Require (
    Convert_Camera_Frame_To_Point_Cloud (
      frame, &cloud, nullptr, 500000,
      static_cast<std::uint32_t> (CoordinateType_Depth)),
    "Required Depth point cloud was not selected");
  Require (cloud.source_coordinate_type ==
             static_cast<std::uint32_t> (CoordinateType_Depth),
           "Required Depth coordinate type mismatch");
  Require (cloud.xyz[2] == 1000.0f,
           "RGB point cloud was selected instead of Depth point cloud");
  Require (!cloud.Has_Color ( ),
           "Depth point cloud unexpectedly inherited RGB color");
}

void Test_Overlay_Preparation_Filters_Scales_And_Downsamples ( )
{
  Camera_Point_Cloud cloud;
  cloud.xyz = {
    0.1f, 0.2f, 1.0f,
    0.0f, 0.0f, -1.0f,
    0.0f, 0.0f, 12.0f,
    30.0f, 0.0f, 1.0f,
    0.2f, 0.3f, 2.0f };
  cloud.rgb = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

  Camera_Point_Cloud_Filter_Options options;
  options.coordinate_to_mm = 1000.0;
  options.maximum_points = 1;
  Camera_Point_Cloud_Filter_Stats stats;
  std::string error;
  Require (
    Prepare_Camera_Point_Cloud_For_Overlay (
      &cloud, options, &stats, &error),
    "Overlay point-cloud preparation failed");
  Require (stats.input_points == 5 && stats.valid_points == 2 &&
             stats.rejected_points == 3 && stats.output_points == 1,
           "Overlay filtering statistics are incorrect");
  Require (std::abs (stats.raw_positive_depth_min - 1.0) < 0.001 &&
             std::abs (stats.raw_positive_depth_median - 2.0) < 0.001 &&
             std::abs (stats.raw_positive_depth_max - 12.0) < 0.001,
           "Raw positive Depth statistics are incorrect");
  Require (cloud.Point_Count ( ) == 1 && cloud.xyz[0] == 100.0f &&
             cloud.xyz[1] == 200.0f && cloud.xyz[2] == 1000.0f,
           "Overlay coordinates were not scaled or downsampled correctly");
  Require (cloud.rgb == std::vector<std::uint8_t> ({ 1, 2, 3 }),
           "Overlay color did not stay aligned with its point");
  Require (std::abs (stats.median_depth_mm - 2000.0) < 0.001,
           "Overlay median Depth is incorrect");
  Require (std::abs (stats.camera_bounds_mm[0] - 100.0) < 0.01 &&
             std::abs (stats.camera_bounds_mm[1] - 200.0) < 0.01 &&
             std::abs (stats.camera_bounds_mm[4] - 1000.0) < 0.01 &&
             std::abs (stats.camera_bounds_mm[5] - 2000.0) < 0.01,
           "Overlay camera-coordinate bounds are incorrect");
}

void Test_Mv3d_Micrometre_Coordinates_Are_Converted_To_Millimetres ( )
{
  Camera_Point_Cloud cloud;
  cloud.xyz = { 100000.0f, 200000.0f, 1200000.0f };

  Camera_Point_Cloud_Filter_Options options;
  options.coordinate_to_mm = 0.001;
  Camera_Point_Cloud_Filter_Stats stats;
  std::string error;
  Require (
    Prepare_Camera_Point_Cloud_For_Overlay (
      &cloud, options, &stats, &error),
    "MV3D micrometre point was rejected after unit conversion");
  Require (cloud.Point_Count ( ) == 1,
           "MV3D micrometre point count changed unexpectedly");
  Require (std::abs (cloud.xyz[0] - 100.0f) < 0.01f &&
             std::abs (cloud.xyz[1] - 200.0f) < 0.01f &&
             std::abs (cloud.xyz[2] - 1200.0f) < 0.01f,
           "MV3D micrometre coordinates were not converted to millimetres");
  Require (std::abs (stats.median_depth_mm - 1200.0) < 0.01,
           "Converted MV3D median Depth is incorrect");
}
} // namespace

int main ( )
{
  try
  {
    Test_Xyz_Filters_Invalid_Points ( );
    Test_Textured_Cloud_Is_Preferred ( );
    Test_Point_Limit_Downsamples ( );
    Test_Required_Depth_Coordinate_Is_Selected ( );
    Test_Overlay_Preparation_Filters_Scales_And_Downsamples ( );
    Test_Mv3d_Micrometre_Coordinates_Are_Converted_To_Millimetres ( );
  }
  catch( const std::exception& error )
  {
    std::cerr << "FAILED: " << error.what ( ) << '\n';
    return 1;
  }

  std::cout << "Camera point-cloud converter tests passed.\n";
  return 0;
}

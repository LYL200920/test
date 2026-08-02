#include "robot_base_point_cloud_builder.h"
#include "camera_formats.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{

void require (bool condition, const char* message)
{
  if( !condition ) throw std::runtime_error (message);
}

void append_float (std::vector<std::uint8_t>* data, float value)
{
  const auto old_size = data->size ( );
  data->resize (old_size + sizeof (value));
  std::memcpy (data->data ( ) + old_size, &value, sizeof (value));
}

void test_complete_camera_to_robot_base_pipeline ( )
{
  Camera_Frame frame;
  Camera_Image_Frame image;
  image.image_type = camera_formats::Point_Cloud;
  image.coordinate_type = camera_formats::Depth_Coordinates;
  image.width = image.height = 1;
  image.frame_number = 42;
  append_float (&image.data, 100000.0f);
  append_float (&image.data, 200000.0f);
  append_float (&image.data, 1200000.0f);
  frame.images.push_back (std::move (image));

  robot_model::Eye_To_Hand_Calibration calibration;
  for( int index = 0; index < 4; ++index )
    calibration.camera_to_robot[index][index] = 1.0;
  calibration.camera_to_robot[0][3] = 10.0;
  calibration.camera_to_robot[1][3] = 20.0;
  calibration.camera_to_robot[2][3] = 30.0;
  calibration.camera_coordinate_type =
    camera_formats::Depth_Coordinates;
  calibration.point_cloud_unit_to_mm = 0.001;

  point_cloud::Robot_Base_Point_Cloud output;
  std::string error;
  require (point_cloud::Build_Robot_Base_Point_Cloud (
             frame, calibration, &output, &error),
           "Robot-base pipeline failed");
  require (output.data.Point_Count ( ) == 1,
           "Robot-base pipeline point count mismatch");
  require (std::abs (output.data.xyz[0] - 110.0f) < 0.01f &&
             std::abs (output.data.xyz[1] - 220.0f) < 0.01f &&
             std::abs (output.data.xyz[2] - 1230.0f) < 0.01f,
           "Unit conversion and hand-eye transform were not composed correctly");
  require (output.source_frame_number == 42 &&
             output.source_point_count == 1 &&
             output.filtered_point_count == 1,
           "Robot-base pipeline metadata mismatch");
  require (std::abs (output.median_depth_mm - 1200.0) < 0.01,
           "Robot-base pipeline median Depth mismatch");
  require (point_cloud::Are_Robot_Base_Bounds_Reasonable (
             output.world_bounds_mm),
           "Robot-base pipeline bounds were rejected");
}

void test_micrometre_textured_cloud_unit_is_detected ( )
{
  Camera_Frame frame;
  Camera_Image_Frame image;
  image.image_type =
    camera_formats::Textured_Point_Cloud;
  image.coordinate_type =
    camera_formats::Depth_Coordinates;
  image.width = image.height = 1;
  append_float (&image.data, 100000.0f);
  append_float (&image.data, 200000.0f);
  append_float (&image.data, 1200000.0f);
  image.data.insert (image.data.end ( ), { 10, 20, 30, 255 });
  frame.images.push_back (std::move (image));

  robot_model::Eye_To_Hand_Calibration calibration;
  for( int index = 0; index < 4; ++index )
    calibration.camera_to_robot[index][index] = 1.0;
  calibration.camera_coordinate_type =
    camera_formats::Depth_Coordinates;
  calibration.point_cloud_unit_to_mm = 1.0;

  point_cloud::Robot_Base_Point_Cloud output;
  std::string error;
  require (point_cloud::Build_Robot_Base_Point_Cloud (
             frame, calibration, &output, &error),
           "Micrometre textured point cloud was not auto-detected");
  require (std::abs (output.data.xyz[0] - 100.0f) < 0.01f &&
             std::abs (output.data.xyz[1] - 200.0f) < 0.01f &&
             std::abs (output.data.xyz[2] - 1200.0f) < 0.01f,
           "Micrometre textured point cloud was not converted to millimetres");
  require (output.data.rgb == std::vector<std::uint8_t> ({ 10, 20, 30 }),
           "Textured point-cloud colour was not preserved");
}

} // namespace

int main ( )
{
  try
  {
    test_complete_camera_to_robot_base_pipeline ( );
    test_micrometre_textured_cloud_unit_is_detected ( );
  }
  catch( const std::exception& error )
  {
    std::cerr << "FAILED: " << error.what ( ) << '\n';
    return 1;
  }
  std::cout << "Robot-base point-cloud builder tests passed.\n";
  return 0;
}

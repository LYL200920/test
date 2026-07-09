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
} // namespace

int main ( )
{
  try
  {
    Test_Xyz_Filters_Invalid_Points ( );
    Test_Textured_Cloud_Is_Preferred ( );
    Test_Point_Limit_Downsamples ( );
  }
  catch( const std::exception& error )
  {
    std::cerr << "FAILED: " << error.what ( ) << '\n';
    return 1;
  }

  std::cout << "Camera point-cloud converter tests passed.\n";
  return 0;
}

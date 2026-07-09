#include "camera_image_converter.h"

#include "Mv3dRgbdDefine.h"

#include <cstdint>
#include <iostream>
#include <stdexcept>

namespace
{
void Require (bool condition, const char* message)
{
  if( !condition )
  {
    throw std::runtime_error (message);
  }
}

Camera_Image_Frame Make_Image (
  Mv3dRgbdImageType type,
  std::uint32_t width,
  std::uint32_t height)
{
  Camera_Image_Frame image;
  image.image_type = static_cast<std::uint32_t> (type);
  image.width = width;
  image.height = height;
  return image;
}

void Test_Rgb_Planar ( )
{
  Camera_Frame frame;
  auto image = Make_Image (ImageType_RGB8_Planar, 2, 1);
  image.data = { 10, 20, 30, 40, 50, 60 };
  frame.images.push_back (std::move (image));

  Camera_Display_Image output;
  Require (
    Convert_Camera_Frame_To_Display_Image (
      frame, Camera_Image_Display_Mode::Color, &output),
    "RGB Planar conversion failed");
  Require (
    output.rgb == std::vector<std::uint8_t> ({ 10, 30, 50, 20, 40, 60 }),
    "RGB Planar channel order mismatch");
}

void Test_Yuyv ( )
{
  Camera_Frame frame;
  auto image = Make_Image (ImageType_YUV422, 2, 1);
  image.data = { 40, 128, 200, 128 };
  frame.images.push_back (std::move (image));

  Camera_Display_Image output;
  Require (
    Convert_Camera_Frame_To_Display_Image (
      frame, Camera_Image_Display_Mode::Color, &output),
    "YUYV conversion failed");
  Require (
    output.rgb[0] == 40 && output.rgb[1] == 40 && output.rgb[2] == 40,
    "First neutral YUYV pixel mismatch");
  Require (
    output.rgb[3] == 200 && output.rgb[4] == 200 && output.rgb[5] == 200,
    "Second neutral YUYV pixel mismatch");
}

void Test_Nv12 ( )
{
  Camera_Frame frame;
  auto image = Make_Image (ImageType_YUV420SP_NV12, 2, 2);
  image.data = { 10, 20, 30, 40, 128, 128 };
  frame.images.push_back (std::move (image));

  Camera_Display_Image output;
  Require (
    Convert_Camera_Frame_To_Display_Image (
      frame, Camera_Image_Display_Mode::Color, &output),
    "NV12 conversion failed");
  Require (
    output.rgb[0] == 10 && output.rgb[9] == 40,
    "NV12 luminance conversion mismatch");
}

void Test_Automatic_Prefers_Color ( )
{
  Camera_Frame frame;
  auto depth = Make_Image (ImageType_Depth, 1, 1);
  depth.data = { 100, 0 };
  frame.images.push_back (std::move (depth));
  auto color = Make_Image (ImageType_RGB8_Planar, 1, 1);
  color.data = { 1, 2, 3 };
  frame.images.push_back (std::move (color));

  Camera_Display_Image output;
  Require (
    Convert_Camera_Frame_To_Display_Image (
      frame, Camera_Image_Display_Mode::Automatic, &output),
    "Automatic conversion failed");
  Require (
    output.source_image_type ==
      static_cast<std::uint32_t> (ImageType_RGB8_Planar),
    "Automatic mode did not prefer color");
}
} // namespace

int main ( )
{
  try
  {
    Test_Rgb_Planar ( );
    Test_Yuyv ( );
    Test_Nv12 ( );
    Test_Automatic_Prefers_Color ( );
  }
  catch( const std::exception& error )
  {
    std::cerr << "FAILED: " << error.what ( ) << '\n';
    return 1;
  }

  std::cout << "Camera image converter tests passed.\n";
  return 0;
}

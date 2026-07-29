#include "camera_2d_image_converter.h"
#include "camera_2d_cross_template.h"
#include "camera_params.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

namespace
{
jutze_camera::camera_frame Make_Frame(
  unsigned int width,
  unsigned int height,
  jutze_camera::pixel_format format,
  const std::vector<std::uint8_t> &bytes)
{
  return jutze_camera::camera_frame(
    "test",
    "serial",
    width,
    height,
    static_cast<unsigned int>(format),
    42,
    bytes.data(),
    bytes.size());
}

void Test_Mono8()
{
  const auto frame = Make_Frame(
    2, 1, jutze_camera::pixel_format::mono8, {0, 255});
  Camera_2D_Display_Image image;
  std::string error;
  assert(Convert_Camera_2D_Frame(frame, &image, &error));
  assert(error.empty());
  assert(image.width == 2 && image.height == 1);
  assert(image.frame_number == 42);
  assert((image.rgb == std::vector<std::uint8_t>{
    0, 0, 0, 255, 255, 255}));
}

void Test_Rgb8()
{
  const auto frame = Make_Frame(
    1, 1, jutze_camera::pixel_format::rgb8, {11, 22, 33});
  Camera_2D_Display_Image image;
  assert(Convert_Camera_2D_Frame(frame, &image));
  assert((image.rgb == std::vector<std::uint8_t>{11, 22, 33}));
}

void Test_Bgr8()
{
  const auto frame = Make_Frame(
    1, 1, jutze_camera::pixel_format::bgr8, {11, 22, 33});
  Camera_2D_Display_Image image;
  assert(Convert_Camera_2D_Frame(frame, &image));
  assert((image.rgb == std::vector<std::uint8_t>{33, 22, 11}));
}

void Test_Mono16()
{
  const auto frame = Make_Frame(
    2, 1, jutze_camera::pixel_format::mono16,
    {0x00, 0x00, 0xff, 0xff});
  Camera_2D_Display_Image image;
  assert(Convert_Camera_2D_Frame(frame, &image));
  assert((image.rgb == std::vector<std::uint8_t>{
    0, 0, 0, 255, 255, 255}));
}

void Test_Mono10()
{
  const auto frame = Make_Frame(
    2, 1, jutze_camera::pixel_format::mono10,
    {0x00, 0x00, 0xff, 0x03});
  Camera_2D_Display_Image image;
  assert(Convert_Camera_2D_Frame(frame, &image));
  assert((image.rgb == std::vector<std::uint8_t>{
    0, 0, 0, 255, 255, 255}));
}

void Test_Bayer_Rg8()
{
  const auto frame = Make_Frame(
    2, 2, jutze_camera::pixel_format::bayer_rg8,
    {200, 100, 100, 20});
  Camera_2D_Display_Image image;
  assert(Convert_Camera_2D_Frame(frame, &image));
  assert(image.rgb.size() == 12);
  assert(image.rgb[0] == 200);
  assert(image.rgb[1] == 100);
  assert(image.rgb[2] == 20);
}

void Test_Truncated_Buffer()
{
  const auto frame = Make_Frame(
    2, 2, jutze_camera::pixel_format::rgb8, {1, 2, 3});
  Camera_2D_Display_Image image;
  std::string error;
  assert(!Convert_Camera_2D_Frame(frame, &image, &error));
  assert(!error.empty());
}

void Test_Cross_Detection()
{
  Camera_2D_Display_Image image;
  image.width = 100;
  image.height = 100;
  image.rgb.assign(100 * 100 * 3, 255);
  for (int y = 25; y <= 75; ++y)
  {
    for (int x = 47; x <= 53; ++x)
    {
      const auto index = (static_cast<std::size_t>(y) * 100 + x) * 3;
      image.rgb[index] = image.rgb[index + 1] = image.rgb[index + 2] = 0;
    }
  }
  for (int y = 47; y <= 53; ++y)
  {
    for (int x = 25; x <= 75; ++x)
    {
      const auto index = (static_cast<std::size_t>(y) * 100 + x) * 3;
      image.rgb[index] = image.rgb[index + 1] = image.rgb[index + 2] = 0;
    }
  }
  Camera_2D_Cross_Template cross_template;
  cross_template.id = "test";
  cross_template.name = "test";
  cross_template.roi = {20, 20, 61, 61};
  cross_template.reference_width = 100;
  cross_template.reference_height = 100;
  cross_template.dark_on_light = true;
  cross_template.foreground_ratio = 0.18;
  Camera_2D_Cross_Detection detection;
  std::string error;
  assert(Detect_Camera_2D_Cross(
    image, cross_template, &detection, &error));
  assert(error.empty());
  assert(detection.found);
  assert(std::abs(detection.center_x - 50.0) < 1.0);
  assert(std::abs(detection.center_y - 50.0) < 1.0);
  assert(detection.confidence > 0.5);
  assert(!detection.outline.empty());
}
}

int main()
{
  Test_Mono8();
  Test_Rgb8();
  Test_Bgr8();
  Test_Mono16();
  Test_Mono10();
  Test_Bayer_Rg8();
  Test_Truncated_Buffer();
  Test_Cross_Detection();
  std::cout << "camera_2d_image_converter_tests passed\n";
  return 0;
}

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

std::vector<std::uint8_t> Gray_Roi(
  const Camera_2D_Display_Image &image,
  int x0,
  int y0,
  int width,
  int height)
{
  std::vector<std::uint8_t> result(
    static_cast<std::size_t>(width) * height);
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x)
      result[static_cast<std::size_t>(y) * width + x] =
        image.rgb[
          (static_cast<std::size_t>(y0 + y) * image.width + x0 + x) * 3];
  return result;
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
  Camera_2D_Display_Image reference;
  reference.width = 61;
  reference.height = 61;
  reference.rgb.assign(61 * 61 * 3, 255);
  for (int y = 5; y <= 55; ++y)
  {
    for (int x = 27; x <= 33; ++x)
    {
      const auto index = (static_cast<std::size_t>(y) * 61 + x) * 3;
      reference.rgb[index] =
        reference.rgb[index + 1] = reference.rgb[index + 2] = 0;
    }
  }
  for (int y = 27; y <= 33; ++y)
  {
    for (int x = 5; x <= 55; ++x)
    {
      const auto index = (static_cast<std::size_t>(y) * 61 + x) * 3;
      reference.rgb[index] =
        reference.rgb[index + 1] = reference.rgb[index + 2] = 0;
    }
  }
  Camera_2D_Display_Image image;
  image.width = 140;
  image.height = 120;
  image.rgb.assign(140 * 120 * 3, 255);
  for (int y = 45; y <= 95; ++y)
    for (int x = 92; x <= 98; ++x)
    {
      const auto index = (static_cast<std::size_t>(y) * 140 + x) * 3;
      image.rgb[index] = image.rgb[index + 1] = image.rgb[index + 2] = 0;
    }
  for (int y = 67; y <= 73; ++y)
    for (int x = 70; x <= 120; ++x)
    {
      const auto index = (static_cast<std::size_t>(y) * 140 + x) * 3;
      image.rgb[index] = image.rgb[index + 1] = image.rgb[index + 2] = 0;
    }
  Camera_2D_Cross_Template cross_template;
  cross_template.id = "test";
  cross_template.name = "test";
  cross_template.roi = {0, 0, 61, 61};
  cross_template.reference_width = 140;
  cross_template.reference_height = 120;
  cross_template.dark_on_light = true;
  cross_template.foreground_ratio = 0.18;
  cross_template.feature_center_x = 30.0;
  cross_template.feature_center_y = 30.0;
  cross_template.reference_gray = Gray_Roi(reference, 0, 0, 61, 61);
  cross_template.reference_outline = {
    {30, 5}, {30, 55}, {5, 30}, {55, 30}};
  Camera_2D_Cross_Detection detection;
  std::string error;
  assert(Detect_Camera_2D_Cross(
    image, cross_template, &detection, &error));
  assert(error.empty());
  assert(detection.found);
  assert(std::abs(detection.center_x - 95.0) < 1.0);
  assert(std::abs(detection.center_y - 70.0) < 1.0);
  assert(detection.confidence > 0.5);
  assert(!detection.outline.empty());
}

void Test_Rotated_Cross_Direction()
{
  Camera_2D_Display_Image reference;
  reference.width = 81;
  reference.height = 81;
  reference.rgb.assign(81 * 81 * 3, 255);
  for (int y = 6; y <= 74; ++y)
    for (int x = 37; x <= 43; ++x)
    {
      const auto index = (static_cast<std::size_t>(y) * 81 + x) * 3;
      reference.rgb[index] =
        reference.rgb[index + 1] = reference.rgb[index + 2] = 0;
    }
  for (int y = 37; y <= 43; ++y)
    for (int x = 6; x <= 74; ++x)
    {
      const auto index = (static_cast<std::size_t>(y) * 81 + x) * 3;
      reference.rgb[index] =
        reference.rgb[index + 1] = reference.rgb[index + 2] = 0;
    }
  Camera_2D_Display_Image image;
  image.width = 120;
  image.height = 120;
  image.rgb.assign(120 * 120 * 3, 255);
  constexpr double expected_angle = 13.0;
  const double radians =
    expected_angle * 3.14159265358979323846 / 180.0;
  const double cosine = std::cos(radians);
  const double sine = std::sin(radians);
  for (int y = 0; y < 120; ++y)
  {
    for (int x = 0; x < 120; ++x)
    {
      const double dx = x - 60.0;
      const double dy = y - 60.0;
      const double along = dx * cosine + dy * sine;
      const double across = -dx * sine + dy * cosine;
      const bool on_cross =
        (std::abs(across) <= 3.0 && std::abs(along) <= 34.0) ||
        (std::abs(along) <= 3.0 && std::abs(across) <= 34.0);
      if (!on_cross) continue;
      const auto index =
        (static_cast<std::size_t>(y) * 120 + x) * 3;
      image.rgb[index] = image.rgb[index + 1] = image.rgb[index + 2] = 0;
    }
  }
  Camera_2D_Cross_Template cross_template;
  cross_template.id = "rotated";
  cross_template.name = "rotated";
  cross_template.roi = {20, 20, 81, 81};
  cross_template.reference_width = 120;
  cross_template.reference_height = 120;
  cross_template.dark_on_light = true;
  cross_template.foreground_ratio = 0.13;
  cross_template.feature_center_x = 40.0;
  cross_template.feature_center_y = 40.0;
  cross_template.reference_angle_deg = 0.0;
  cross_template.reference_gray = Gray_Roi(reference, 0, 0, 81, 81);
  Camera_2D_Cross_Detection detection;
  assert(Detect_Camera_2D_Cross(
    image, cross_template, &detection, nullptr));
  assert(detection.found);
  assert(std::abs(detection.angle_deg - expected_angle) < 1.0);
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
  Test_Rotated_Cross_Direction();
  std::cout << "camera_2d_image_converter_tests passed\n";
  return 0;
}

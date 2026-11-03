#include "intrinsic_undistortion.h"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using camera_calibration::Camera_Intrinsics;
using camera_calibration::Image_View;
using camera_calibration::Pixel_Format;
using camera_calibration::Undistort_Intrinsic_Image;

void Require(bool condition, const char *message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

Camera_Intrinsics Make_Intrinsics(int width, int height)
{
  Camera_Intrinsics intrinsics;
  intrinsics.image_width = width;
  intrinsics.image_height = height;
  intrinsics.camera_matrix = {
    60.0, 0.0, (width - 1) * 0.5,
    0.0, 62.0, (height - 1) * 0.5,
    0.0, 0.0, 1.0};
  return intrinsics;
}

void Test_Zero_Distortion_Formats_And_Strides()
{
  constexpr int width = 19;
  constexpr int height = 13;
  const auto intrinsics = Make_Intrinsics(width, height);
  for (const auto format :
       {Pixel_Format::Gray8, Pixel_Format::RGB8, Pixel_Format::BGR8})
  {
    for (const std::size_t padding : {0U, 7U})
    {
      const std::size_t channels = format == Pixel_Format::Gray8 ? 1 : 3;
      const std::size_t stride = width * channels + padding;
      std::vector<std::uint8_t> source(stride * height, 231);
      std::vector<std::uint8_t> expected(width * height * 3);
      for (int y = 0; y < height; ++y)
      {
        for (int x = 0; x < width; ++x)
        {
          const std::array<std::uint8_t, 3> color{
            static_cast<std::uint8_t>((x * 11 + y * 3) % 256),
            static_cast<std::uint8_t>((x * 3 + y * 17) % 256),
            static_cast<std::uint8_t>((x * 7 + y * 5) % 256)};
          const auto input = static_cast<std::size_t>(y) * stride + x * channels;
          const auto output = static_cast<std::size_t>(y * width + x) * 3;
          for (std::size_t channel = 0; channel < 3; ++channel)
          {
            expected[output + channel] = channels == 1 ? color[0] : color[channel];
          }
          if (channels == 1)
          {
            source[input] = color[0];
          }
          else
          {
            for (std::size_t channel = 0; channel < 3; ++channel)
            {
              source[input + channel] =
                color[format == Pixel_Format::BGR8 ? 2 - channel : channel];
            }
          }
        }
      }
      const auto original = source;
      const Image_View image{source.data(), width, height, stride, format};
      std::vector<std::uint8_t> rgb{9, 8, 7};
      std::string error = "stale error";
      Require(Undistort_Intrinsic_Image(image, intrinsics, &rgb, &error),
        "Zero-distortion conversion must succeed");
      Require(error.empty(), "Success must clear an old error");
      Require(rgb == expected, "Zero distortion must preserve pixels and RGB order exactly");
      Require(source == original, "Input pixels and padding must not be changed");
      Require(Undistort_Intrinsic_Image(image, intrinsics, &rgb),
        "The error output must be optional");
    }
  }
}

// Independent Brown-Conrady projection; no OpenCV mapping in the oracle.
std::array<double, 2> Distort(double x, double y, const Camera_Intrinsics &intrinsics)
{
  const auto &d = intrinsics.distortion;
  const double r2 = x * x + y * y;
  const double radial = 1.0 + r2 * (d[0] + r2 * (d[1] + r2 * d[4]));
  return {x * radial + 2.0 * d[2] * x * y + d[3] * (r2 + 2.0 * x * x),
          y * radial + d[2] * (r2 + 2.0 * y * y) + 2.0 * d[3] * x * y};
}

std::uint8_t Ideal_Pixel(double x, double y, int channel)
{
  return static_cast<std::uint8_t>(std::lround(
    120.0 + 45.0 * std::sin(x / 8.0 + channel * 0.7) +
    40.0 * std::cos(y / 10.0 - channel * 0.5)));
}

void Test_Synthetic_Distortion()
{
  constexpr int width = 96;
  constexpr int height = 72;
  auto intrinsics = Make_Intrinsics(width, height);
  intrinsics.distortion = {0.22, -0.035, 0.004, -0.006, 0.015};
  const auto original_intrinsics = intrinsics;
  const auto &k = intrinsics.camera_matrix;
  constexpr std::size_t stride = width * 3 + 5;
  std::vector<std::uint8_t> source(stride * height, 239);
  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      const double xd = (x - k[2]) / k[0];
      const double yd = (y - k[5]) / k[4];
      double xu = xd;
      double yu = yd;
      for (int iteration = 0; iteration < 80; ++iteration)
      {
        const auto projected = Distort(xu, yu, intrinsics);
        xu += xd - projected[0];
        yu += yd - projected[1];
      }
      const auto projected = Distort(xu, yu, intrinsics);
      Require(std::hypot(projected[0] - xd, projected[1] - yd) < 1.0e-10,
        "Synthetic inverse projection must converge");
      for (int channel = 0; channel < 3; ++channel)
      {
        source[y * stride + x * 3 + channel] =
          Ideal_Pixel(xu * k[0] + k[2], yu * k[4] + k[5], channel);
      }
    }
  }
  const auto original = source;
  const Image_View image{source.data(), width, height, stride, Pixel_Format::RGB8};
  std::vector<std::uint8_t> rgb;
  std::string error;
  Require(Undistort_Intrinsic_Image(image, intrinsics, &rgb, &error),
    "Synthetic distorted image must be corrected");
  Require(rgb.size() == width * height * 3, "Output must retain original dimensions");
  Require(source == original, "Synthetic source must remain unchanged");
  Require(intrinsics.camera_matrix == original_intrinsics.camera_matrix &&
          intrinsics.distortion == original_intrinsics.distortion,
    "Calibration must not be modified");
  double corrected_error = 0.0;
  double distorted_error = 0.0;
  std::size_t samples = 0;
  std::size_t black_pixels = 0;
  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      const auto projected = Distort((x - k[2]) / k[0], (y - k[5]) / k[4], intrinsics);
      const double sx = projected[0] * k[0] + k[2];
      const double sy = projected[1] * k[4] + k[5];
      const auto index = static_cast<std::size_t>(y * width + x) * 3;
      if (sx < -1.0 || sx > width || sy < -1.0 || sy > height)
      {
        Require(rgb[index] == 0 && rgb[index + 1] == 0 && rgb[index + 2] == 0,
          "Out-of-frame output pixels must be black");
        ++black_pixels;
      }
      if (sx < 1.0 || sx >= width - 2.0 || sy < 1.0 || sy >= height - 2.0)
      {
        continue;
      }
      for (int channel = 0; channel < 3; ++channel)
      {
        const int expected = Ideal_Pixel(x, y, channel);
        const int difference = std::abs(static_cast<int>(rgb[index + channel]) - expected);
        Require(difference <= 3, "Corrected synthetic pixels must match unchanged K geometry");
        corrected_error += difference;
        distorted_error += std::abs(static_cast<int>(source[y * stride + x * 3 + channel]) - expected);
        ++samples;
      }
    }
  }
  Require(samples > width * height, "Synthetic comparison must cover substantial image area");
  Require(black_pixels > 0, "Synthetic case must exercise uncropped black borders");
  Require(corrected_error / samples < 1.0, "Correction mean error must be below one intensity level");
  Require(distorted_error / samples > 1.0, "Synthetic distortion must visibly change the source");
  Require(corrected_error < distorted_error * 0.25, "Correction must remove most synthetic distortion");
}

void Expect_Failure(const Image_View &image, const Camera_Intrinsics &intrinsics)
{
  std::vector<std::uint8_t> rgb{17, 29, 43, 71};
  const auto original = rgb;
  std::string error;
  Require(!Undistort_Intrinsic_Image(image, intrinsics, &rgb, &error),
    "Invalid input must fail");
  Require(!error.empty(), "Failure must provide an error message");
  Require(rgb == original, "Failure must leave existing RGB output unchanged");
  Require(!Undistort_Intrinsic_Image(image, intrinsics, &rgb),
    "Invalid input must also fail without an error output");
  Require(rgb == original, "Failure without an error output must preserve RGB");
}

void Test_Invalid_Inputs()
{
  auto intrinsics = Make_Intrinsics(8, 6);
  std::vector<std::uint8_t> source(8 * 6 * 3, 127);
  const auto original = source;
  const Image_View image{source.data(), 8, 6, 24, Pixel_Format::RGB8};
  std::string error;
  Require(!Undistort_Intrinsic_Image(image, intrinsics, nullptr, &error),
    "Null output must fail");
  Require(!error.empty(), "Null output must report an error");
  Require(!Undistort_Intrinsic_Image(image, intrinsics, nullptr),
    "Null output must fail without an error output");

  auto invalid_image = image;
  invalid_image.data = nullptr;
  Expect_Failure(invalid_image, intrinsics);
  for (const int dimension : {0, -1, 7})
  {
    invalid_image = image;
    invalid_image.width = dimension;
    Expect_Failure(invalid_image, intrinsics);
    invalid_image = image;
    invalid_image.height = dimension;
    Expect_Failure(invalid_image, intrinsics);
  }
  for (const auto format : {Pixel_Format::Gray8, Pixel_Format::RGB8, Pixel_Format::BGR8})
  {
    invalid_image = image;
    invalid_image.pixel_format = format;
    const std::size_t minimum_stride = format == Pixel_Format::Gray8 ? 8 : 24;
    for (const auto stride : {std::size_t{0}, minimum_stride - 1,
         std::numeric_limits<std::size_t>::max()})
    {
      invalid_image.row_stride_bytes = stride;
      Expect_Failure(invalid_image, intrinsics);
    }
  }
  invalid_image = image;
  invalid_image.pixel_format = static_cast<Pixel_Format>(999);
  Expect_Failure(invalid_image, intrinsics);

  for (const double nonfinite : {std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::infinity()})
  {
    for (std::size_t index = 0; index < intrinsics.camera_matrix.size(); ++index)
    {
      auto invalid = intrinsics;
      invalid.camera_matrix[index] = nonfinite;
      Expect_Failure(image, invalid);
    }
    for (std::size_t index = 0; index < intrinsics.distortion.size(); ++index)
    {
      auto invalid = intrinsics;
      invalid.distortion[index] = nonfinite;
      Expect_Failure(image, invalid);
    }
  }
  for (const std::size_t index : {0U, 4U})
  {
    for (const double focal : {0.0, -1.0})
    {
      auto invalid = intrinsics;
      invalid.camera_matrix[index] = focal;
      Expect_Failure(image, invalid);
    }
  }
  for (const std::size_t index : {1U, 3U, 6U, 7U, 8U})
  {
    auto invalid = intrinsics;
    invalid.camera_matrix[index] = index == 8 ? 0.0 : 0.1;
    Expect_Failure(image, invalid);
  }
  auto invalid = intrinsics;
  invalid.image_width = 0;
  Expect_Failure(image, invalid);
  invalid = intrinsics;
  invalid.image_height = -1;
  Expect_Failure(image, invalid);
  invalid = intrinsics;
  invalid.image_width += 1;
  Expect_Failure(image, invalid);
  invalid = intrinsics;
  invalid.image_height += 1;
  Expect_Failure(image, invalid);
  Require(source == original, "Rejected inputs must not modify source pixels");
}

} // namespace

int main()
{
  try
  {
    Test_Zero_Distortion_Formats_And_Strides();
    Test_Synthetic_Distortion();
    Test_Invalid_Inputs();
    return 0;
  }
  catch (const std::exception &exception)
  {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}

#include "intrinsic_undistortion.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <exception>
#include <limits>

namespace camera_calibration
{

bool Undistort_Intrinsic_Image(
  const Image_View &image,
  const Camera_Intrinsics &intrinsics,
  std::vector<std::uint8_t> *rgb,
  std::string *error)
{
  if (error)
  {
    error->clear();
  }
  const auto fail = [error](const std::string &message)
  {
    if (error)
    {
      *error = message;
    }
    return false;
  };
  if (!rgb)
  {
    return fail("Undistorted RGB output is null");
  }
  if (!image.data || image.width <= 0 || image.height <= 0)
  {
    return fail("Undistortion input image is empty or has invalid dimensions");
  }
  if (!Validate_Camera_Intrinsics(intrinsics, error))
  {
    return false;
  }
  if (image.width != intrinsics.image_width ||
      image.height != intrinsics.image_height)
  {
    return fail("Image dimensions do not match the intrinsic calibration");
  }
  const auto &k = intrinsics.camera_matrix;
  if (k[1] != 0.0 || k[3] != 0.0 || k[6] != 0.0 ||
      k[7] != 0.0 || k[8] != 1.0)
  {
    return fail("Camera matrix must use the standard OpenCV pinhole form");
  }

  std::size_t channels = 0;
  switch (image.pixel_format)
  {
    case Pixel_Format::Gray8:
      channels = 1;
      break;
    case Pixel_Format::RGB8:
    case Pixel_Format::BGR8:
      channels = 3;
      break;
    default:
      return fail("Unsupported undistortion input pixel format");
  }
  const auto width = static_cast<std::size_t>(image.width);
  const auto height = static_cast<std::size_t>(image.height);
  const auto maximum_span = static_cast<std::size_t>(
    std::numeric_limits<std::ptrdiff_t>::max());
  // Check before multiplication, including the span used by cv::Mat's step.
  if (width > maximum_span / 3 ||
      image.row_stride_bytes < width * channels ||
      image.row_stride_bytes > maximum_span / height)
  {
    return fail("Input image row stride is invalid or its buffer span overflows");
  }
  const std::size_t rgb_stride = width * 3;
  if (height > maximum_span / rgb_stride ||
      height > rgb->max_size() / rgb_stride)
  {
    return fail("Undistorted RGB image size overflows");
  }

  try
  {
    const cv::Mat source(
      image.height, image.width, channels == 1 ? CV_8UC1 : CV_8UC3,
      const_cast<std::uint8_t *>(image.data), image.row_stride_bytes);
    cv::Mat source_rgb;
    if (image.pixel_format == Pixel_Format::Gray8)
    {
      cv::cvtColor(source, source_rgb, cv::COLOR_GRAY2RGB);
    }
    else if (image.pixel_format == Pixel_Format::BGR8)
    {
      cv::cvtColor(source, source_rgb, cv::COLOR_BGR2RGB);
    }
    else
    {
      source_rgb = source;
    }

    const cv::Mat camera_matrix(3, 3, CV_64F,
      const_cast<double *>(intrinsics.camera_matrix.data()));
    const cv::Mat distortion(1, 5, CV_64F,
      const_cast<double *>(intrinsics.distortion.data()));
    std::vector<std::uint8_t> result(rgb_stride * height);
    cv::Mat destination(image.height, image.width, CV_8UC3,
      result.data(), rgb_stride);
    // Explicitly retain K; undistort uses linear interpolation and a black
    // constant border. All work is local until success, including conversion.
    cv::undistort(source_rgb, destination, camera_matrix, distortion,
      camera_matrix);
    rgb->swap(result);
    return true;
  }
  catch (const std::exception &exception)
  {
    return fail(exception.what());
  }
}

} // namespace camera_calibration

#include "intrinsic_calibration.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <utility>

namespace camera_calibration
{
namespace
{

void Set_Error(std::string *error_message, const std::string &message)
{
  if (error_message)
  {
    *error_message = message;
  }
}

bool Is_Finite(double value)
{
  return std::isfinite(value) != 0;
}

struct Observation
{
  std::string image_id;
  std::vector<cv::Point2f> corners;
};

std::vector<cv::Point3f> Make_Object_Points(
  const Chessboard_Configuration &configuration)
{
  std::vector<cv::Point3f> points;
  points.reserve(static_cast<std::size_t>(
    configuration.inner_corner_columns * configuration.inner_corner_rows));
  for (int row = 0; row < configuration.inner_corner_rows; ++row)
  {
    for (int column = 0;
         column < configuration.inner_corner_columns;
         ++column)
    {
      points.emplace_back(
        static_cast<float>(column * configuration.square_size_mm),
        static_cast<float>(row * configuration.square_size_mm),
        0.0F);
    }
  }
  return points;
}

} // namespace

class Intrinsic_Calibration_Session::Impl
{
public:
  explicit Impl(Chessboard_Configuration board)
    : configuration(std::move(board))
  {
  }

  Chessboard_Configuration configuration;
  int image_width = 0;
  int image_height = 0;
  std::vector<Observation> observations;
};

bool Validate_Chessboard_Configuration(
  const Chessboard_Configuration &configuration,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (configuration.inner_corner_columns < 2 ||
      configuration.inner_corner_rows < 2)
  {
    Set_Error(error_message, "Chessboard inner-corner dimensions must be at least 2 x 2");
    return false;
  }
  if (!Is_Finite(configuration.square_size_mm) ||
      configuration.square_size_mm <= 0.0)
  {
    Set_Error(error_message, "Chessboard square size must be positive and finite");
    return false;
  }
  return true;
}

bool Validate_Camera_Intrinsics(
  const Camera_Intrinsics &intrinsics,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (intrinsics.image_width <= 0 || intrinsics.image_height <= 0)
  {
    Set_Error(error_message, "Intrinsic image dimensions are invalid");
    return false;
  }
  for (double value : intrinsics.camera_matrix)
  {
    if (!Is_Finite(value))
    {
      Set_Error(error_message, "Camera matrix contains a non-finite value");
      return false;
    }
  }
  for (double value : intrinsics.distortion)
  {
    if (!Is_Finite(value))
    {
      Set_Error(error_message, "Distortion coefficients contain a non-finite value");
      return false;
    }
  }
  if (intrinsics.camera_matrix[0] <= 0.0 ||
      intrinsics.camera_matrix[4] <= 0.0 ||
      !Is_Finite(intrinsics.overall_rms_px) ||
      intrinsics.overall_rms_px < 0.0)
  {
    Set_Error(error_message, "Intrinsic focal length or RMS is invalid");
    return false;
  }
  return true;
}

Intrinsic_Calibration_Session::Intrinsic_Calibration_Session(
  Chessboard_Configuration configuration)
  : impl_(std::make_unique<Impl>(std::move(configuration)))
{
}

Intrinsic_Calibration_Session::~Intrinsic_Calibration_Session() = default;
Intrinsic_Calibration_Session::Intrinsic_Calibration_Session(
  Intrinsic_Calibration_Session &&) noexcept = default;
Intrinsic_Calibration_Session &Intrinsic_Calibration_Session::operator=(
  Intrinsic_Calibration_Session &&) noexcept = default;

bool Intrinsic_Calibration_Session::Add_Image(
  const Image_View &image,
  const std::string &image_id,
  Image_Detection_Result *detection,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (!detection)
  {
    Set_Error(error_message, "Image detection output is null");
    return false;
  }
  *detection = {};
  detection->image_id = image_id;
  detection->image_width = image.width;
  detection->image_height = image.height;

  std::string configuration_error;
  if (!Validate_Chessboard_Configuration(
        impl_->configuration, &configuration_error))
  {
    Set_Error(error_message, configuration_error);
    return false;
  }
  if (!image.data || image.width <= 0 || image.height <= 0)
  {
    Set_Error(error_message, "Calibration image is empty");
    return false;
  }

  int type = CV_8UC1;
  std::size_t bytes_per_pixel = 1;
  if (image.pixel_format == Pixel_Format::RGB8 ||
      image.pixel_format == Pixel_Format::BGR8)
  {
    type = CV_8UC3;
    bytes_per_pixel = 3;
  }
  if (image.row_stride_bytes <
      static_cast<std::size_t>(image.width) * bytes_per_pixel)
  {
    Set_Error(error_message, "Calibration image row stride is too small");
    return false;
  }

  try
  {
    const cv::Mat source(
      image.height,
      image.width,
      type,
      const_cast<std::uint8_t *>(image.data),
      image.row_stride_bytes);
    cv::Mat gray;
    if (image.pixel_format == Pixel_Format::RGB8)
    {
      cv::cvtColor(source, gray, cv::COLOR_RGB2GRAY);
    }
    else if (image.pixel_format == Pixel_Format::BGR8)
    {
      cv::cvtColor(source, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
      gray = source;
    }

    constexpr int maximum_detection_extent = 2048;
    const int source_extent = std::max(gray.cols, gray.rows);
    const double detection_scale = source_extent > maximum_detection_extent
      ? static_cast<double>(maximum_detection_extent) / source_extent
      : 1.0;
    cv::Mat detection_image;
    if (detection_scale < 1.0)
    {
      cv::resize(
        gray,
        detection_image,
        cv::Size(),
        detection_scale,
        detection_scale,
        cv::INTER_AREA);
    }
    else
    {
      detection_image = gray;
    }

    std::vector<cv::Point2f> corners;
    const int flags = cv::CALIB_CB_NORMALIZE_IMAGE |
                      cv::CALIB_CB_EXHAUSTIVE |
                      cv::CALIB_CB_ACCURACY;
    detection->chessboard_found = cv::findChessboardCornersSB(
      detection_image,
      cv::Size(
        impl_->configuration.inner_corner_columns,
        impl_->configuration.inner_corner_rows),
      corners,
      flags);
    if (!detection->chessboard_found)
    {
      const int classic_flags = cv::CALIB_CB_ADAPTIVE_THRESH |
                                cv::CALIB_CB_NORMALIZE_IMAGE |
                                cv::CALIB_CB_FILTER_QUADS;
      detection->chessboard_found = cv::findChessboardCorners(
        detection_image,
        cv::Size(
          impl_->configuration.inner_corner_columns,
          impl_->configuration.inner_corner_rows),
        corners,
        classic_flags);
    }
    if (!detection->chessboard_found)
    {
      return true;
    }

    if (detection_scale < 1.0)
    {
      const float inverse_scale = static_cast<float>(1.0 / detection_scale);
      for (cv::Point2f &corner : corners)
      {
        corner *= inverse_scale;
      }
    }
    cv::cornerSubPix(
      gray,
      corners,
      cv::Size(5, 5),
      cv::Size(-1, -1),
      cv::TermCriteria(
        cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
        40,
        1.0e-4));

    detection->corners.reserve(corners.size());
    for (const cv::Point2f &corner : corners)
    {
      detection->corners.push_back({corner.x, corner.y});
    }
    return Add_Observation(
      image.width,
      image.height,
      detection->corners,
      image_id,
      error_message);
  }
  catch (const cv::Exception &exception)
  {
    Set_Error(error_message, std::string("OpenCV corner detection failed: ") +
                             exception.what());
    return false;
  }
}

bool Intrinsic_Calibration_Session::Add_Observation(
  int image_width,
  int image_height,
  const std::vector<Image_Point> &corners,
  const std::string &image_id,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  std::string configuration_error;
  if (!Validate_Chessboard_Configuration(
        impl_->configuration, &configuration_error))
  {
    Set_Error(error_message, configuration_error);
    return false;
  }
  if (image_width <= 0 || image_height <= 0)
  {
    Set_Error(error_message, "Observation image dimensions are invalid");
    return false;
  }
  const std::size_t expected_corner_count = static_cast<std::size_t>(
    impl_->configuration.inner_corner_columns *
    impl_->configuration.inner_corner_rows);
  if (corners.size() != expected_corner_count)
  {
    Set_Error(error_message, "Observation corner count does not match the chessboard");
    return false;
  }
  if (impl_->image_width != 0 &&
      (impl_->image_width != image_width ||
       impl_->image_height != image_height))
  {
    Set_Error(error_message, "All calibration images must have the same resolution");
    return false;
  }

  Observation observation;
  observation.image_id = image_id;
  observation.corners.reserve(corners.size());
  for (const Image_Point &corner : corners)
  {
    if (!Is_Finite(corner.x) || !Is_Finite(corner.y))
    {
      Set_Error(error_message, "Observation contains a non-finite corner");
      return false;
    }
    observation.corners.emplace_back(
      static_cast<float>(corner.x), static_cast<float>(corner.y));
  }

  impl_->image_width = image_width;
  impl_->image_height = image_height;
  impl_->observations.push_back(std::move(observation));
  return true;
}

std::size_t Intrinsic_Calibration_Session::Observation_Count() const
{
  return impl_->observations.size();
}

bool Intrinsic_Calibration_Session::Calibrate(
  const Calibration_Options &options,
  Camera_Intrinsics *intrinsics,
  std::string *error_message) const
{
  if (error_message)
  {
    error_message->clear();
  }
  if (!intrinsics)
  {
    Set_Error(error_message, "Camera intrinsics output is null");
    return false;
  }
  *intrinsics = {};
  if (options.minimum_view_count < 3)
  {
    Set_Error(error_message, "Minimum calibration view count must be at least three");
    return false;
  }
  if (!Is_Finite(options.maximum_view_rms_px) ||
      options.maximum_view_rms_px < 0.0)
  {
    Set_Error(error_message, "Maximum per-view RMS must be finite and non-negative");
    return false;
  }
  if (!Is_Finite(options.maximum_focal_length_to_image_ratio) ||
      options.maximum_focal_length_to_image_ratio < 0.0)
  {
    Set_Error(error_message, "Maximum focal-length ratio must be finite and non-negative");
    return false;
  }
  if (impl_->observations.size() < options.minimum_view_count)
  {
    Set_Error(error_message, "Not enough valid chessboard views for calibration");
    return false;
  }

  try
  {
    std::vector<std::size_t> active_indices(impl_->observations.size());
    std::iota(active_indices.begin(), active_indices.end(), 0);
    std::vector<View_Calibration_Error> rejected_views;
    const std::vector<cv::Point3f> board_points =
      Make_Object_Points(impl_->configuration);

    cv::Mat camera_matrix;
    cv::Mat distortion;
    cv::Mat intrinsic_deviations;
    cv::Mat per_view_errors;
    double overall_rms = 0.0;
    double final_worst_rms = 0.0;

    for (std::size_t iteration = 0;; ++iteration)
    {
      std::vector<std::vector<cv::Point3f>> object_points(
        active_indices.size(), board_points);
      std::vector<std::vector<cv::Point2f>> image_points;
      image_points.reserve(active_indices.size());
      for (std::size_t index : active_indices)
      {
        image_points.push_back(impl_->observations[index].corners);
      }

      camera_matrix = cv::Mat::eye(3, 3, CV_64F);
      distortion = cv::Mat::zeros(5, 1, CV_64F);
      std::vector<cv::Mat> rotation_vectors;
      std::vector<cv::Mat> translation_vectors;
      cv::Mat extrinsic_deviations;
      int flags = 0;
      if (options.zero_tangential_distortion)
      {
        flags |= cv::CALIB_ZERO_TANGENT_DIST;
      }
      if (options.fix_k3)
      {
        flags |= cv::CALIB_FIX_K3;
      }
      overall_rms = cv::calibrateCamera(
        object_points,
        image_points,
        cv::Size(impl_->image_width, impl_->image_height),
        camera_matrix,
        distortion,
        rotation_vectors,
        translation_vectors,
        intrinsic_deviations,
        extrinsic_deviations,
        per_view_errors,
        flags,
        cv::TermCriteria(
          cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
          100,
          1.0e-10));

      double worst_rms = -1.0;
      std::size_t worst_position = 0;
      for (std::size_t position = 0;
           position < active_indices.size();
           ++position)
      {
        const double rms = per_view_errors.at<double>(
          static_cast<int>(position), 0);
        if (rms > worst_rms)
        {
          worst_rms = rms;
          worst_position = position;
        }
      }

      const bool may_remove = options.maximum_view_rms_px > 0.0 &&
        worst_rms > options.maximum_view_rms_px &&
        iteration < options.maximum_outlier_removals &&
        active_indices.size() > options.minimum_view_count;
      if (!may_remove)
      {
        final_worst_rms = worst_rms;
        break;
      }
      const std::size_t rejected_index = active_indices[worst_position];
      rejected_views.push_back({
        impl_->observations[rejected_index].image_id,
        worst_rms});
      active_indices.erase(active_indices.begin() +
                           static_cast<std::ptrdiff_t>(worst_position));
    }

    Camera_Intrinsics result;
    result.image_width = impl_->image_width;
    result.image_height = impl_->image_height;
    result.overall_rms_px = overall_rms;
    for (int row = 0; row < 3; ++row)
    {
      for (int column = 0; column < 3; ++column)
      {
        result.camera_matrix[static_cast<std::size_t>(row * 3 + column)] =
          camera_matrix.at<double>(row, column);
      }
    }
    for (std::size_t index = 0; index < result.distortion.size(); ++index)
    {
      result.distortion[index] = distortion.at<double>(
        static_cast<int>(index), 0);
    }
    for (std::size_t index = 0;
         index < result.intrinsic_standard_deviations.size();
         ++index)
    {
      if (static_cast<int>(index) < intrinsic_deviations.rows)
      {
        result.intrinsic_standard_deviations[index] =
          intrinsic_deviations.at<double>(static_cast<int>(index), 0);
      }
    }
    for (std::size_t position = 0;
         position < active_indices.size();
         ++position)
    {
      result.accepted_views.push_back({
        impl_->observations[active_indices[position]].image_id,
        per_view_errors.at<double>(static_cast<int>(position), 0)});
    }
    result.rejected_views = std::move(rejected_views);

    if (options.maximum_view_rms_px > 0.0 &&
        final_worst_rms > options.maximum_view_rms_px)
    {
      std::ostringstream message;
      message << "Calibration quality failed: worst view RMS is "
              << final_worst_rms << " px, above the "
              << options.maximum_view_rms_px << " px limit";
      Set_Error(error_message, message.str());
      return false;
    }

    const double image_extent = static_cast<double>(
      std::max(result.image_width, result.image_height));
    if (options.maximum_focal_length_to_image_ratio > 0.0 &&
        (result.camera_matrix[0] >
           options.maximum_focal_length_to_image_ratio * image_extent ||
         result.camera_matrix[4] >
           options.maximum_focal_length_to_image_ratio * image_extent))
    {
      std::ostringstream message;
      message << "Pinhole calibration is degenerate (fx="
              << result.camera_matrix[0] << ", fy="
              << result.camera_matrix[4]
              << "): focal length is far larger than the image. Use a larger "
                 "board with stronger out-of-plane tilt, and verify whether "
                 "the lens is telecentric";
      Set_Error(error_message, message.str());
      return false;
    }

    std::string validation_error;
    if (!Validate_Camera_Intrinsics(result, &validation_error))
    {
      Set_Error(error_message, validation_error);
      return false;
    }
    *intrinsics = std::move(result);
    return true;
  }
  catch (const cv::Exception &exception)
  {
    Set_Error(error_message, std::string("OpenCV intrinsic calibration failed: ") +
                             exception.what());
    return false;
  }
}

} // namespace camera_calibration

#include "camera_intrinsics_repository.h"
#include "intrinsic_calibration_dataset.h"

#include <cstdlib>
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

namespace
{

void Print_Usage(const char *executable)
{
  std::cerr
    << "Usage: " << executable
    << " <image-directory> <output.xml>"
       " [inner-columns=17] [inner-rows=19] [square-mm=0.5]\n";
}

std::size_t Print_Detection_Summary(
  const camera_calibration::Dataset_Calibration_Report &report)
{
  std::size_t detected_count = 0;
  for (const camera_calibration::Image_Detection_Result &detection :
       report.detections)
  {
    if (detection.chessboard_found)
    {
      ++detected_count;
      const auto x_range = std::minmax_element(
        detection.corners.begin(), detection.corners.end(),
        [](const auto &left, const auto &right)
        {
          return left.x < right.x;
        });
      const auto y_range = std::minmax_element(
        detection.corners.begin(), detection.corners.end(),
        [](const auto &left, const auto &right)
        {
          return left.y < right.y;
        });
      std::cout << "Board: " << detection.image_id
                << " [" << x_range.first->x << ", " << y_range.first->y
                << "] - [" << x_range.second->x << ", "
                << y_range.second->y << "]\n";
    }
    else
    {
      std::cout << "No board: " << detection.image_id << '\n';
    }
  }
  std::cout << "Images: " << report.detections.size()
            << ", detected: " << detected_count << '\n';
  return detected_count;
}

} // namespace

int main(int argc, char **argv)
{
  if (argc < 3 || argc > 6)
  {
    Print_Usage(argv[0]);
    return 2;
  }

  camera_calibration::Dataset_Calibration_Request request;
  request.image_directory = std::filesystem::u8path(argv[1]);
  if (argc >= 4)
  {
    request.chessboard.inner_corner_columns = std::atoi(argv[3]);
  }
  if (argc >= 5)
  {
    request.chessboard.inner_corner_rows = std::atoi(argv[4]);
  }
  if (argc >= 6)
  {
    request.chessboard.square_size_mm = std::atof(argv[5]);
  }
  request.options.minimum_view_count = 10;
  request.options.maximum_view_rms_px = 1.0;
  request.options.maximum_outlier_removals = 6;

  camera_calibration::Dataset_Calibration_Report report;
  std::string error;
  const bool calibrated = camera_calibration::Calibrate_Image_Directory(
    request, &report, &error);
  const std::size_t detected_count = Print_Detection_Summary(report);
  if (!calibrated)
  {
    std::cerr << "Calibration failed: " << error << '\n';
    return 1;
  }

  const std::filesystem::path output_path =
    std::filesystem::u8path(argv[2]);
  if (!camera_calibration::Save_Camera_Intrinsics(
        output_path, request.chessboard, report.intrinsics, &error))
  {
    std::cerr << "Saving calibration failed: " << error << '\n';
    return 1;
  }

  const auto &value = report.intrinsics;
  std::cout << std::setprecision(12)
            << "Detected: " << detected_count
            << ", accepted: " << value.accepted_views.size()
            << ", rejected as outliers: " << value.rejected_views.size()
            << '\n'
            << "RMS: " << value.overall_rms_px << " px\n"
            << "fx=" << value.camera_matrix[0]
            << ", fy=" << value.camera_matrix[4]
            << ", cx=" << value.camera_matrix[2]
            << ", cy=" << value.camera_matrix[5] << '\n'
            << "k1=" << value.distortion[0]
            << ", k2=" << value.distortion[1]
            << ", p1=" << value.distortion[2]
            << ", p2=" << value.distortion[3]
            << ", k3=" << value.distortion[4] << '\n'
            << "Saved: " << output_path.string() << '\n';
  return 0;
}

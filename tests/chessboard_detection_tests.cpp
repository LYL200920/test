#include "intrinsic_calibration.h"
#include "intrinsic_calibration_dataset.h"
#include "telecentric_planar_calibration.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using namespace camera_calibration;
void Require(bool value, const char *message)
{
  if (!value) throw std::runtime_error(message);
}
void Geometry()
{
  Chessboard_Configuration board{9, 7, 0.5};
  std::vector<Image_Point> points;
  for (int y = 0; y < 7; ++y)
    for (int x = 0; x < 9; ++x)
    {
      const double denominator = 1 + 0.015 * x + 0.02 * y;
      points.push_back({(100 + 35 * x + 2 * y) / denominator,
                        (100 + 3 * x + 31 * y) / denominator});
    }
  std::vector<std::size_t> suspects;
  std::string error;
  Require(Validate_Detected_Chessboard(board, points, 640, 480, &suspects, &error), "Perspective grid rejected");
  auto reversed = points;
  std::reverse(reversed.begin(), reversed.end());
  Require(Validate_Detected_Chessboard(board, reversed, 640, 480, &suspects), "Reverse order rejected");
  for (std::size_t index : {std::size_t(0), std::size_t(8), std::size_t(31), std::size_t(62)})
  {
    auto bad = points; bad[index].x += 14;
    Require(!Validate_Detected_Chessboard(board, bad, 640, 480, &suspects), "Gross corner displacement accepted");
    Require(std::find(suspects.begin(), suspects.end(), index) != suspects.end(), "Suspect corner omitted");
  }
  auto bad = points; bad[20] = bad[21];
  Require(!Validate_Detected_Chessboard(board, bad, 640, 480, &suspects), "Duplicate corner accepted");
  bad = points; std::swap(bad[20], bad[21]);
  Require(!Validate_Detected_Chessboard(board, bad, 640, 480, &suspects), "Crossed grid accepted");
  bad = points; bad[20].x = std::numeric_limits<double>::quiet_NaN();
  Require(!Validate_Detected_Chessboard(board, bad, 640, 480, &suspects), "NaN accepted");
  bad = points; bad.pop_back();
  Require(!Validate_Detected_Chessboard(board, bad, 640, 480, &suspects), "Incomplete grid accepted");
}
void Synthetic()
{
  const Chessboard_Configuration board{9, 7, 0.5};
  const int width = 2501, height = 1601, cell = 32, left = 1200, top = 550;
  std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height, 235);
  for (int y = 0; y < 8 * cell; ++y)
    for (int x = 0; x < 10 * cell; ++x)
      pixels[static_cast<std::size_t>(top + y) * width + left + x] = ((x / cell + y / cell) % 2) ? 235 : 20;
  Intrinsic_Calibration_Session session(board);
  Image_Detection_Result result;
  std::string error;
  Require(session.Add_Image({pixels.data(), width, height, static_cast<std::size_t>(width), Pixel_Format::Gray8},
    "synthetic", &result, &error), "Synthetic detection operation failed");
  Require(result.chessboard_found && result.corners.size() == 63 && session.Observation_Count() == 1,
    "Synthetic board not detected");
  Require(result.detection_method.find("ROI-") == 0 && !result.diagnostic.empty(), "ROI diagnostics missing");
  const bool reverse = result.corners.front().x > result.corners.back().x;
  for (std::size_t i = 0; i < result.corners.size(); ++i)
  {
    const auto index = reverse ? result.corners.size() - 1 - i : i;
    const auto &p = result.corners[index];
    Require(std::hypot(p.x - (left + (i % 9 + 1) * cell - 0.5),
      p.y - (top + (i / 9 + 1) * cell - 0.5)) < 0.5, "Original-resolution coordinate recovery inaccurate");
  }
  std::fill(pixels.begin(), pixels.end(), 127);
  Require(session.Add_Image({pixels.data(), width, height, static_cast<std::size_t>(width), Pixel_Format::Gray8},
    "blank", &result, &error) && !result.chessboard_found && result.corners.empty() &&
    session.Observation_Count() == 1 && !result.diagnostic.empty(), "Blank image accepted or stale output");
}
void Dataset(const std::filesystem::path &directory)
{
  std::vector<std::filesystem::path> files;
  for (const auto &entry : std::filesystem::directory_iterator(directory))
    if (entry.path().extension() == ".png") files.push_back(entry.path());
  std::sort(files.begin(), files.end());
  Require(!files.empty(), "Dataset is empty");
  std::vector<Image_Detection_Result> views;
  for (const auto &file : files)
  {
    Image_Detection_Result result;
    std::string error;
    const bool detected = Detect_Calibration_Image_File(file, {}, &result, &error);
    Require(detected, error.c_str());
    std::cout << file.filename().u8string() << " " << result.diagnostic << std::endl;
    Require(result.chessboard_found, "Dataset image rejected");
    views.push_back(std::move(result));
  }
  Telecentric_Planar_Result result;
  Telecentric_Planar_Options options;
  options.progress = [](const std::string &text) { std::cout << text << std::endl; };
  std::string error;
  const bool calibrated = Calibrate_Telecentric_Plane({}, views, options, &result, &error);
  Require(calibrated, error.c_str());
  std::cout << "DATASET PASS views=" << views.size() << " RMS=" << result.rms_px
            << " max=" << result.maximum_error_px << " linear=" << result.maximum_linear_deviation_percent << std::endl;
}
}
int main(int argc, char **argv)
{
  try
  {
    Geometry();
    Synthetic();
    if (argc == 2) Dataset(std::filesystem::u8path(argv[1]));
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}

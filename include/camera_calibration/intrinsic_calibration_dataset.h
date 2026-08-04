#ifndef CAMERA_CALIBRATION_INTRINSIC_CALIBRATION_DATASET_H_
#define CAMERA_CALIBRATION_INTRINSIC_CALIBRATION_DATASET_H_

#include "intrinsic_calibration.h"

#include <filesystem>
#include <string>
#include <vector>

namespace camera_calibration
{

struct Dataset_Calibration_Request
{
  std::filesystem::path image_directory;
  Chessboard_Configuration chessboard;
  Calibration_Options options;
};

struct Dataset_Calibration_Report
{
  std::vector<Image_Detection_Result> detections;
  Camera_Intrinsics intrinsics;
};

bool Detect_Calibration_Image_File(
  const std::filesystem::path &image_path,
  const Chessboard_Configuration &chessboard,
  Image_Detection_Result *detection,
  std::string *error_message = nullptr);

bool Detect_Calibration_Image_Directory(
  const std::filesystem::path &image_directory,
  const Chessboard_Configuration &chessboard,
  std::vector<Image_Detection_Result> *detections,
  std::string *error_message = nullptr);

bool Save_Calibration_Image(
  const std::filesystem::path &image_path,
  const Image_View &image,
  std::string *error_message = nullptr);

bool Calibrate_Image_Directory(
  const Dataset_Calibration_Request &request,
  Dataset_Calibration_Report *report,
  std::string *error_message = nullptr);

} // namespace camera_calibration

#endif // CAMERA_CALIBRATION_INTRINSIC_CALIBRATION_DATASET_H_

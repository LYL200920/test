#include "camera_intrinsics_repository.h"

#include "pugixml.hpp"

#include <system_error>
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

void Append_View(
  pugi::xml_node parent,
  const char *name,
  const View_Calibration_Error &view)
{
  pugi::xml_node node = parent.append_child(name);
  node.append_attribute("image") = view.image_id.c_str();
  node.append_attribute("rmsPx") = view.rms_px;
}

} // namespace

bool Save_Camera_Intrinsics(
  const std::filesystem::path &path,
  const Chessboard_Configuration &chessboard,
  const Camera_Intrinsics &intrinsics,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  std::string validation_error;
  if (!Validate_Chessboard_Configuration(chessboard, &validation_error) ||
      !Validate_Camera_Intrinsics(intrinsics, &validation_error))
  {
    Set_Error(error_message, validation_error);
    return false;
  }
  if (path.empty())
  {
    Set_Error(error_message, "Camera intrinsics path is empty");
    return false;
  }

  if (!path.parent_path().empty())
  {
    std::error_code filesystem_error;
    std::filesystem::create_directories(path.parent_path(), filesystem_error);
    if (filesystem_error)
    {
      Set_Error(error_message, "Unable to create camera intrinsics directory");
      return false;
    }
  }

  pugi::xml_document document;
  pugi::xml_node root = document.append_child("CameraIntrinsicCalibration");
  root.append_attribute("version") = 1;
  root.append_attribute("model") = "pinhole-brown5";
  root.append_attribute("cameraSerial") =
    intrinsics.camera_serial_number.c_str();
  root.append_attribute("cameraModel") = intrinsics.camera_model.c_str();
  root.append_attribute("pixelFormat") = intrinsics.pixel_format.c_str();
  root.append_attribute("imageWidth") = intrinsics.image_width;
  root.append_attribute("imageHeight") = intrinsics.image_height;
  root.append_attribute("overallRmsPx") = intrinsics.overall_rms_px;

  pugi::xml_node board = root.append_child("Chessboard");
  board.append_attribute("innerCornerColumns") =
    chessboard.inner_corner_columns;
  board.append_attribute("innerCornerRows") = chessboard.inner_corner_rows;
  board.append_attribute("squareSizeMm") = chessboard.square_size_mm;

  pugi::xml_node matrix = root.append_child("CameraMatrix");
  matrix.append_attribute("fx") = intrinsics.camera_matrix[0];
  matrix.append_attribute("skew") = intrinsics.camera_matrix[1];
  matrix.append_attribute("cx") = intrinsics.camera_matrix[2];
  matrix.append_attribute("fy") = intrinsics.camera_matrix[4];
  matrix.append_attribute("cy") = intrinsics.camera_matrix[5];

  pugi::xml_node distortion = root.append_child("Distortion");
  distortion.append_attribute("k1") = intrinsics.distortion[0];
  distortion.append_attribute("k2") = intrinsics.distortion[1];
  distortion.append_attribute("p1") = intrinsics.distortion[2];
  distortion.append_attribute("p2") = intrinsics.distortion[3];
  distortion.append_attribute("k3") = intrinsics.distortion[4];

  pugi::xml_node deviations = root.append_child("StandardDeviations");
  static const char *names[] = {
    "fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3"};
  for (std::size_t index = 0;
       index < intrinsics.intrinsic_standard_deviations.size();
       ++index)
  {
    deviations.append_attribute(names[index]) =
      intrinsics.intrinsic_standard_deviations[index];
  }

  pugi::xml_node views = root.append_child("Views");
  for (const View_Calibration_Error &view : intrinsics.accepted_views)
  {
    Append_View(views, "Accepted", view);
  }
  for (const View_Calibration_Error &view : intrinsics.rejected_views)
  {
    Append_View(views, "Rejected", view);
  }

  if (!document.save_file(path.wstring().c_str(), "  "))
  {
    Set_Error(error_message, "Unable to save camera intrinsics");
    return false;
  }
  return true;
}

bool Load_Camera_Intrinsics(
  const std::filesystem::path &path,
  Chessboard_Configuration *chessboard,
  Camera_Intrinsics *intrinsics,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (!chessboard || !intrinsics)
  {
    Set_Error(error_message, "Camera intrinsics load output is null");
    return false;
  }
  *chessboard = {};
  *intrinsics = {};

  pugi::xml_document document;
  if (!document.load_file(path.wstring().c_str()))
  {
    Set_Error(error_message, "Unable to parse camera intrinsics file");
    return false;
  }
  const pugi::xml_node root = document.child("CameraIntrinsicCalibration");
  if (!root || root.attribute("version").as_int() != 1 ||
      std::string(root.attribute("model").as_string()) != "pinhole-brown5")
  {
    Set_Error(error_message, "Camera intrinsics file has an unsupported format");
    return false;
  }

  const pugi::xml_node board = root.child("Chessboard");
  chessboard->inner_corner_columns =
    board.attribute("innerCornerColumns").as_int();
  chessboard->inner_corner_rows =
    board.attribute("innerCornerRows").as_int();
  chessboard->square_size_mm = board.attribute("squareSizeMm").as_double();

  intrinsics->image_width = root.attribute("imageWidth").as_int();
  intrinsics->image_height = root.attribute("imageHeight").as_int();
  intrinsics->camera_serial_number =
    root.attribute("cameraSerial").as_string();
  intrinsics->camera_model = root.attribute("cameraModel").as_string();
  intrinsics->pixel_format = root.attribute("pixelFormat").as_string();
  intrinsics->overall_rms_px = root.attribute("overallRmsPx").as_double();
  const pugi::xml_node matrix = root.child("CameraMatrix");
  intrinsics->camera_matrix = {
    matrix.attribute("fx").as_double(),
    matrix.attribute("skew").as_double(),
    matrix.attribute("cx").as_double(),
    0.0,
    matrix.attribute("fy").as_double(),
    matrix.attribute("cy").as_double(),
    0.0,
    0.0,
    1.0};
  const pugi::xml_node distortion = root.child("Distortion");
  intrinsics->distortion = {
    distortion.attribute("k1").as_double(),
    distortion.attribute("k2").as_double(),
    distortion.attribute("p1").as_double(),
    distortion.attribute("p2").as_double(),
    distortion.attribute("k3").as_double()};

  const pugi::xml_node deviations = root.child("StandardDeviations");
  static const char *names[] = {
    "fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3"};
  for (std::size_t index = 0;
       index < intrinsics->intrinsic_standard_deviations.size();
       ++index)
  {
    intrinsics->intrinsic_standard_deviations[index] =
      deviations.attribute(names[index]).as_double();
  }

  const pugi::xml_node views = root.child("Views");
  for (const pugi::xml_node node : views.children("Accepted"))
  {
    intrinsics->accepted_views.push_back({
      node.attribute("image").as_string(),
      node.attribute("rmsPx").as_double()});
  }
  for (const pugi::xml_node node : views.children("Rejected"))
  {
    intrinsics->rejected_views.push_back({
      node.attribute("image").as_string(),
      node.attribute("rmsPx").as_double()});
  }

  std::string validation_error;
  if (!Validate_Chessboard_Configuration(*chessboard, &validation_error) ||
      !Validate_Camera_Intrinsics(*intrinsics, &validation_error))
  {
    *chessboard = {};
    *intrinsics = {};
    Set_Error(error_message, validation_error);
    return false;
  }
  return true;
}

} // namespace camera_calibration

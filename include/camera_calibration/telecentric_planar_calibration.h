#ifndef TELECENTRIC_PLANAR_CALIBRATION_H_
#define TELECENTRIC_PLANAR_CALIBRATION_H_

#include "calibration_geometry.h"
#include <filesystem>

namespace camera_calibration
{
struct Telecentric_Planar_View
{
  std::string image_id;
  double rms_px = 0;
  double maximum_error_px = 0;
  double linear_deviation_percent = 0;
};
struct Telecentric_Planar_Result
{
  int width = 0, height = 0;
  Chessboard_Configuration board;
  // [du,dv] = A * [dX,dY], row-major. X/Y are the fixed board-plane axes,
  // not robot axes. Absolute position/depth and nonlinear distortion unsolved.
  std::array<double, 4> pixels_from_plane_mm{};
  double rms_px = 0;
  double maximum_error_px = 0;
  double maximum_linear_deviation_percent = 0;
  std::vector<Telecentric_Planar_View> views;
};
struct Telecentric_Planar_Options
{
  std::size_t minimum_views = 10;
  double maximum_view_rms_px = 0.75;
  double maximum_corner_error_px = 2.0;
  double maximum_linear_deviation_percent = 1.0;
  std::function<void(const std::string &)> progress;
};
struct Telecentric_Planar_Profile
{
  Calibration_Identity identity;
  Camera_Image_Configuration image;
  std::string fixed_setup_id; // Same plane, working distance and relative orientation.
  Telecentric_Planar_Result result;
};
// Fits one shared linear map and a separate translation per image. Requires a
// stationary board, fixed relative orientation and in-plane translations only.
// 180-degree corner-order ambiguity is normalized to the first view, not claimed
// as an absolute physical board origin. No automatic removal of tilted views.
bool Calibrate_Telecentric_Plane(const Chessboard_Configuration &board,
  const std::vector<Image_Detection_Result> &views, const Telecentric_Planar_Options &options,
  Telecentric_Planar_Result *result, std::string *error = nullptr);
bool Validate_Telecentric_Planar_Result(const Telecentric_Planar_Result &result,
  std::string *error = nullptr);
bool Telecentric_Pixel_Displacement_To_Plane(const Telecentric_Planar_Result &result,
  const Image_Point &pixels, Image_Point *millimetres, std::string *error = nullptr);
bool Save_Telecentric_Planar_Profile(const std::filesystem::path &path,
  const Telecentric_Planar_Profile &profile, std::string *error = nullptr);
bool Load_Telecentric_Planar_Profile(const std::filesystem::path &path,
  Telecentric_Planar_Profile *profile, std::string *error = nullptr);
bool Match_Telecentric_Planar_Profile(const Telecentric_Planar_Profile &profile,
  const Camera_Image_Configuration &image, const std::string &fixed_setup_id,
  std::string *error = nullptr);
}
#endif

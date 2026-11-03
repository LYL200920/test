#include "telecentric_planar_calibration.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>

namespace camera_calibration
{
namespace
{
bool Fail(std::string *error, const std::string &message)
{
  if (error) *error = message;
  return false;
}
bool Finite_Nonnegative(double value)
{
  return std::isfinite(value) && value >= 0;
}
bool Meaningful_Text(const std::string &value)
{
  return value.find('\0') == std::string::npos &&
    value.find_first_not_of(" \t\r\n") != std::string::npos;
}
} // namespace

bool Validate_Telecentric_Planar_Result(const Telecentric_Planar_Result &value,
  std::string *error)
{
  if (error) error->clear();
  if (!Validate_Chessboard_Configuration(value.board, error)) return false;
  if (value.width <= 0 || value.height <= 0 || value.views.size() < 3 ||
      !Finite_Nonnegative(value.rms_px) || !Finite_Nonnegative(value.maximum_error_px) ||
      !Finite_Nonnegative(value.maximum_linear_deviation_percent))
    return Fail(error, "Invalid planar result");
  const auto &a = value.pixels_from_plane_mm;
  double norm = 0;
  for (double coefficient : a)
  {
    if (!std::isfinite(coefficient)) return Fail(error, "Nonfinite planar matrix");
    norm += coefficient * coefficient;
  }
  const double det = a[0] * a[3] - a[1] * a[2];
  if (!std::isfinite(norm) || !std::isfinite(det) || norm <= 0 || std::abs(det) <= norm * 1e-8)
    return Fail(error, "Singular or ill-conditioned planar matrix");
  std::set<std::string> ids;
  for (const auto &view : value.views)
  {
    if (!Meaningful_Text(view.image_id) || !ids.insert(view.image_id).second ||
        !Finite_Nonnegative(view.rms_px) || !Finite_Nonnegative(view.maximum_error_px) ||
        !Finite_Nonnegative(view.linear_deviation_percent))
      return Fail(error, "Invalid planar view statistics");
  }
  return true;
}

bool Calibrate_Telecentric_Plane(const Chessboard_Configuration &board,
  const std::vector<Image_Detection_Result> &views, const Telecentric_Planar_Options &options,
  Telecentric_Planar_Result *output, std::string *error)
{
  if (error) error->clear();
  if (!output) return Fail(error, "Planar output is null");
  if (!Validate_Chessboard_Configuration(board, error)) return false;
  if (options.minimum_views < 3 || views.size() < options.minimum_views ||
      !std::isfinite(options.maximum_view_rms_px) || options.maximum_view_rms_px <= 0 ||
      !std::isfinite(options.maximum_corner_error_px) || options.maximum_corner_error_px <= 0 ||
      !std::isfinite(options.maximum_linear_deviation_percent) || options.maximum_linear_deviation_percent <= 0)
    return Fail(error, "Insufficient views or invalid planar quality thresholds");
  const std::size_t count = static_cast<std::size_t>(board.inner_corner_columns) * board.inner_corner_rows;
  // Reject inconsistent input before allocating a potentially enormous board grid.
  for (const auto &view : views)
    if (view.corners.size() != count)
      return Fail(error, "Corner count does not match the board: " + view.image_id);
  std::vector<Image_Point> model;
  double xx = 0, yy = 0;
  for (int row = 0; row < board.inner_corner_rows; ++row)
    for (int column = 0; column < board.inner_corner_columns; ++column)
    {
      Image_Point point{
        (column - (board.inner_corner_columns - 1) * 0.5) * board.square_size_mm,
        (row - (board.inner_corner_rows - 1) * 0.5) * board.square_size_mm};
      model.push_back(point);
      xx += point.x * point.x;
      yy += point.y * point.y;
    }
  if (!std::isfinite(xx) || !std::isfinite(yy) || xx <= 0 || yy <= 0)
    return Fail(error, "Board dimensions overflow or underflow");

  Telecentric_Planar_Result result;
  result.board = board;
  result.width = views.front().image_width;
  result.height = views.front().image_height;
  std::vector<std::array<double, 4>> matrices;
  std::vector<std::vector<Image_Point>> centred;
  std::set<std::string> ids;
  for (const auto &view : views)
  {
    if (options.progress) options.progress("检查平面样本：" + view.image_id);
    if (!view.chessboard_found || view.image_width != result.width ||
        view.image_height != result.height || !Meaningful_Text(view.image_id) || !ids.insert(view.image_id).second)
      return Fail(error, "Invalid, duplicate or mixed-size planar view: " + view.image_id);
    Image_Point mean{};
    for (const auto &point : view.corners)
    {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          point.x < 0 || point.y < 0 || point.x >= result.width || point.y >= result.height)
        return Fail(error, "Invalid corner: " + view.image_id);
      mean.x += point.x / count;
      mean.y += point.y / count;
    }
    std::vector<Image_Point> points = view.corners;
    for (auto &point : points)
    {
      point.x -= mean.x;
      point.y -= mean.y;
    }
    // The complete, centred rectangular grid has sum(X*Y)=0, so its
    // least-squares normal matrix is diagonal after removing translation.
    std::array<double, 4> matrix{};
    for (std::size_t j = 0; j < count; ++j)
    {
      matrix[0] += model[j].x * points[j].x / xx;
      matrix[1] += model[j].y * points[j].x / yy;
      matrix[2] += model[j].x * points[j].y / xx;
      matrix[3] += model[j].y * points[j].y / yy;
    }
    if (!matrices.empty())
    {
      double alignment = 0;
      for (int j = 0; j < 4; ++j) alignment += matrix[j] * matrices.front()[j];
      // A symmetric board can be detected in reverse order. This does not
      // resolve an absolute board frame or certify that physical rotation was absent.
      if (alignment < 0)
      {
        std::reverse(points.begin(), points.end());
        for (auto &coefficient : matrix) coefficient = -coefficient;
      }
    }
    matrices.push_back(matrix);
    centred.push_back(std::move(points));
  }
  for (const auto &matrix : matrices)
    for (int j = 0; j < 4; ++j)
      result.pixels_from_plane_mm[j] += matrix[j] / matrices.size();
  const auto &a = result.pixels_from_plane_mm;
  double norm = 0;
  for (double coefficient : a) norm += coefficient * coefficient;
  if (!std::isfinite(norm) || norm <= 0) return Fail(error, "Invalid planar scale");

  std::string failures;
  for (std::size_t i = 0; i < views.size(); ++i)
  {
    Telecentric_Planar_View stat;
    stat.image_id = views[i].image_id;
    double delta = 0;
    for (int j = 0; j < 4; ++j) delta += std::pow(matrices[i][j] - a[j], 2);
    stat.linear_deviation_percent = 100 * std::sqrt(delta / norm);
    double squared = 0;
    for (std::size_t j = 0; j < count; ++j)
    {
      const double du = a[0] * model[j].x + a[1] * model[j].y - centred[i][j].x;
      const double dv = a[2] * model[j].x + a[3] * model[j].y - centred[i][j].y;
      const double residual = std::hypot(du, dv);
      squared += residual * residual;
      stat.maximum_error_px = std::max(stat.maximum_error_px, residual);
    }
    stat.rms_px = std::sqrt(squared / count);
    result.rms_px += squared / count / views.size();
    result.maximum_error_px = std::max(result.maximum_error_px, stat.maximum_error_px);
    result.maximum_linear_deviation_percent = std::max(
      result.maximum_linear_deviation_percent, stat.linear_deviation_percent);
    std::ostringstream message;
    message << stat.image_id << ": RMS=" << stat.rms_px << " px, max=" << stat.maximum_error_px
            << " px, linear change=" << stat.linear_deviation_percent << "%";
    if (options.progress) options.progress(message.str());
    if (stat.rms_px > options.maximum_view_rms_px || stat.maximum_error_px > options.maximum_corner_error_px ||
        stat.linear_deviation_percent > options.maximum_linear_deviation_percent)
      failures += message.str() + "\n";
    result.views.push_back(stat);
  }
  result.rms_px = std::sqrt(result.rms_px);
  if (!failures.empty())
    return Fail(error, "远心固定平面数据不一致：请排除倾斜/变高度图，检查异常角点、畸变和目标平整度。不会自动删除样本。\n" + failures);
  if (!Validate_Telecentric_Planar_Result(result, error)) return false;
  *output = std::move(result);
  return true;
}

bool Telecentric_Pixel_Displacement_To_Plane(const Telecentric_Planar_Result &result,
  const Image_Point &pixels, Image_Point *output, std::string *error)
{
  if (!output || !std::isfinite(pixels.x) || !std::isfinite(pixels.y))
    return Fail(error, "Invalid displacement");
  if (!Validate_Telecentric_Planar_Result(result, error)) return false;
  const auto &a = result.pixels_from_plane_mm;
  const double det = a[0] * a[3] - a[1] * a[2];
  Image_Point point{(a[3] * pixels.x - a[1] * pixels.y) / det,
                    (-a[2] * pixels.x + a[0] * pixels.y) / det};
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) return Fail(error, "Displacement overflow");
  *output = point;
  return true;
}

bool Match_Telecentric_Planar_Profile(const Telecentric_Planar_Profile &profile,
  const Camera_Image_Configuration &image, const std::string &setup, std::string *error)
{
  if (!Meaningful_Text(profile.identity.id) || !Meaningful_Text(profile.identity.created_utc) ||
      !Meaningful_Text(profile.fixed_setup_id) || setup != profile.fixed_setup_id)
    return Fail(error, "Fixed setup identity mismatch");
  if (!Validate_Telecentric_Planar_Result(profile.result, error) ||
      !Image_Configurations_Match(profile.image, image, error)) return false;
  if (profile.image.width != profile.result.width || profile.image.height != profile.result.height)
    return Fail(error, "Planar profile image size mismatch");
  return true;
}
} // namespace camera_calibration

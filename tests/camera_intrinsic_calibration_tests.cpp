#include "camera_intrinsics_repository.h"
#include "intrinsic_calibration.h"
#include "intrinsic_calibration_dataset.h"

#include <array>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

namespace
{

using Matrix3 = std::array<double, 9>;

Matrix3 Multiply(const Matrix3 &left, const Matrix3 &right)
{
  Matrix3 result{};
  for (int row = 0; row < 3; ++row)
  {
    for (int column = 0; column < 3; ++column)
    {
      for (int index = 0; index < 3; ++index)
      {
        result[static_cast<std::size_t>(row * 3 + column)] +=
          left[static_cast<std::size_t>(row * 3 + index)] *
          right[static_cast<std::size_t>(index * 3 + column)];
      }
    }
  }
  return result;
}

Matrix3 Euler_Rotation(double x, double y, double z)
{
  const double cx = std::cos(x);
  const double sx = std::sin(x);
  const double cy = std::cos(y);
  const double sy = std::sin(y);
  const double cz = std::cos(z);
  const double sz = std::sin(z);
  const Matrix3 rx{1, 0, 0, 0, cx, -sx, 0, sx, cx};
  const Matrix3 ry{cy, 0, sy, 0, 1, 0, -sy, 0, cy};
  const Matrix3 rz{cz, -sz, 0, sz, cz, 0, 0, 0, 1};
  return Multiply(Multiply(rz, ry), rx);
}

std::vector<camera_calibration::Image_Point> Make_View(
  const camera_calibration::Chessboard_Configuration &board,
  const Matrix3 &rotation,
  double tx,
  double ty,
  double tz)
{
  constexpr double fx = 3200.0;
  constexpr double fy = 3180.0;
  constexpr double cx = 2048.0;
  constexpr double cy = 1500.0;
  std::vector<camera_calibration::Image_Point> points;
  for (int row = 0; row < board.inner_corner_rows; ++row)
  {
    for (int column = 0; column < board.inner_corner_columns; ++column)
    {
      const double x = column * board.square_size_mm;
      const double y = row * board.square_size_mm;
      const double camera_x = rotation[0] * x + rotation[1] * y + tx;
      const double camera_y = rotation[3] * x + rotation[4] * y + ty;
      const double camera_z = rotation[6] * x + rotation[7] * y + tz;
      points.push_back({
        fx * camera_x / camera_z + cx,
        fy * camera_y / camera_z + cy});
    }
  }
  return points;
}

void Test_Synthetic_Calibration()
{
  const camera_calibration::Chessboard_Configuration board;
  camera_calibration::Intrinsic_Calibration_Session session(board);
  for (int index = 0; index < 14; ++index)
  {
    const double rx = (-0.20 + 0.035 * index);
    const double ry = (index % 2 == 0 ? -0.16 : 0.14) + 0.006 * index;
    const double rz = -0.12 + 0.018 * index;
    const auto corners = Make_View(
      board,
      Euler_Rotation(rx, ry, rz),
      -3.5 + (index % 5) * 1.5,
      -3.0 + (index % 4) * 1.7,
      105.0 + index * 2.5);
    std::string error;
    assert(session.Add_Observation(
      4096, 3000, corners, "view_" + std::to_string(index), &error));
  }

  camera_calibration::Calibration_Options options;
  options.minimum_view_count = 10;
  camera_calibration::Camera_Intrinsics result;
  std::string error;
  assert(session.Calibrate(options, &result, &error));
  assert(std::abs(result.camera_matrix[0] - 3200.0) < 2.0);
  assert(std::abs(result.camera_matrix[4] - 3180.0) < 2.0);
  assert(std::abs(result.camera_matrix[2] - 2048.0) < 2.0);
  assert(std::abs(result.camera_matrix[5] - 1500.0) < 2.0);
  assert(result.overall_rms_px < 0.01);
}

void Test_Repository_Round_Trip()
{
  camera_calibration::Chessboard_Configuration board;
  camera_calibration::Camera_Intrinsics value;
  value.camera_serial_number = "SN-123";
  value.camera_model = "Test Camera";
  value.pixel_format = "Mono8";
  value.image_width = 4096;
  value.image_height = 3000;
  value.camera_matrix = {3200.0, 0.0, 2048.0,
                         0.0, 3180.0, 1500.0,
                         0.0, 0.0, 1.0};
  value.distortion = {-0.1, 0.02, 0.001, -0.002, -0.003};
  value.intrinsic_standard_deviations =
    {1.0, 1.1, 0.5, 0.6, 0.01, 0.02, 0.001, 0.001, 0.03};
  value.overall_rms_px = 0.25;
  value.accepted_views.push_back({"P002_frame_1.png", 0.22});
  value.rejected_views.push_back({"P010_frame_9.png", 1.25});

  const std::filesystem::path path =
    std::filesystem::temp_directory_path() /
    "camera_intrinsic_calibration_tests.xml";
  std::string error;
  assert(camera_calibration::Save_Camera_Intrinsics(
    path, board, value, &error));

  camera_calibration::Chessboard_Configuration loaded_board;
  camera_calibration::Camera_Intrinsics loaded;
  assert(camera_calibration::Load_Camera_Intrinsics(
    path, &loaded_board, &loaded, &error));
  assert(loaded_board.inner_corner_columns == 17);
  assert(loaded_board.inner_corner_rows == 19);
  assert(std::abs(loaded_board.square_size_mm - 0.5) < 1.0e-12);
  assert(loaded.image_width == value.image_width);
  assert(loaded.image_height == value.image_height);
  assert(loaded.camera_serial_number == value.camera_serial_number);
  assert(loaded.camera_model == value.camera_model);
  assert(loaded.pixel_format == value.pixel_format);
  assert(std::abs(loaded.camera_matrix[0] - value.camera_matrix[0]) < 1.0e-12);
  assert(std::abs(loaded.distortion[4] - value.distortion[4]) < 1.0e-12);
  assert(loaded.accepted_views.size() == 1);
  assert(loaded.rejected_views.size() == 1);
  std::filesystem::remove(path);
}

void Test_Invalid_Board_Is_Rejected()
{
  camera_calibration::Chessboard_Configuration board;
  board.square_size_mm = 0.0;
  std::string error;
  assert(!camera_calibration::Validate_Chessboard_Configuration(
    board, &error));
  assert(!error.empty());
}

void Test_Degenerate_Focal_Length_Is_Rejected()
{
  const camera_calibration::Chessboard_Configuration board;
  camera_calibration::Intrinsic_Calibration_Session session(board);
  for (int index = 0; index < 10; ++index)
  {
    const auto corners = Make_View(
      board,
      Euler_Rotation(0.0, 0.0, index * 0.02),
      -2.0 + index * 0.4,
      -1.0 + index * 0.2,
      105.0);
    std::string error;
    assert(session.Add_Observation(
      4096, 3000, corners, "flat_" + std::to_string(index), &error));
  }
  camera_calibration::Calibration_Options options;
  camera_calibration::Camera_Intrinsics result;
  std::string error;
  assert(!session.Calibrate(options, &result, &error));
  assert(!error.empty());
}

void Test_Unicode_Image_Path_Round_Trip()
{
  const std::filesystem::path path =
    std::filesystem::temp_directory_path() /
    std::filesystem::path(L"相机标定图像测试.png");
  std::vector<std::uint8_t> pixels(64U * 64U * 3U, 127U);
  const camera_calibration::Image_View image{
    pixels.data(), 64, 64, 64U * 3U,
    camera_calibration::Pixel_Format::RGB8};
  std::string error;
  assert(camera_calibration::Save_Calibration_Image(
    path, image, &error));
  camera_calibration::Image_Detection_Result detection;
  assert(camera_calibration::Detect_Calibration_Image_File(
    path, {}, &detection, &error));
  assert(!detection.chessboard_found);
  std::filesystem::remove(path);
}

} // namespace

int main()
{
  Test_Synthetic_Calibration();
  Test_Repository_Round_Trip();
  Test_Invalid_Board_Is_Rejected();
  Test_Degenerate_Focal_Length_Is_Rejected();
  Test_Unicode_Image_Path_Round_Trip();
  return 0;
}

#ifndef CAMERA_CALIBRATION_INTRINSIC_CALIBRATION_H_
#define CAMERA_CALIBRATION_INTRINSIC_CALIBRATION_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace camera_calibration
{

enum class Pixel_Format
{
  Gray8,
  RGB8,
  BGR8
};

struct Image_View
{
  const std::uint8_t *data = nullptr;
  int width = 0;
  int height = 0;
  std::size_t row_stride_bytes = 0;
  Pixel_Format pixel_format = Pixel_Format::Gray8;
};

struct Image_Point
{
  double x = 0.0;
  double y = 0.0;
};

// The dimensions are inner-corner counts, not black/white square counts.
// The CC10-17X19-0.5 board used by this project exposes 17 x 19 corners.
struct Chessboard_Configuration
{
  int inner_corner_columns = 17;
  int inner_corner_rows = 19;
  double square_size_mm = 0.5;
};

struct Calibration_Options
{
  std::size_t minimum_view_count = 10;
  bool zero_tangential_distortion = false;
  bool fix_k3 = false;

  // Set to zero to retain every successfully detected view. When positive,
  // the worst view above this RMS is removed and calibration is repeated.
  double maximum_view_rms_px = 0.0;
  std::size_t maximum_outlier_removals = 0;

  // A very large focal length relative to the sensor is usually the pinhole
  // solver's degenerate affine/telecentric limit. Set to zero to disable.
  double maximum_focal_length_to_image_ratio = 20.0;
};

struct Image_Detection_Result
{
  std::string image_id;
  bool chessboard_found = false;
  int image_width = 0;
  int image_height = 0;
  std::vector<Image_Point> corners;
};

struct View_Calibration_Error
{
  std::string image_id;
  double rms_px = 0.0;
};

struct Camera_Intrinsics
{
  std::string camera_serial_number;
  std::string camera_model;
  std::string pixel_format;
  int image_width = 0;
  int image_height = 0;

  // Row-major 3 x 3 camera matrix.
  std::array<double, 9> camera_matrix{};

  // Brown-Conrady coefficients in OpenCV order: k1, k2, p1, p2, k3.
  std::array<double, 5> distortion{};
  std::array<double, 9> intrinsic_standard_deviations{};

  double overall_rms_px = 0.0;
  std::vector<View_Calibration_Error> accepted_views;
  std::vector<View_Calibration_Error> rejected_views;
};

bool Validate_Chessboard_Configuration(
  const Chessboard_Configuration &configuration,
  std::string *error_message = nullptr);

bool Validate_Camera_Intrinsics(
  const Camera_Intrinsics &intrinsics,
  std::string *error_message = nullptr);

class Intrinsic_Calibration_Session
{
public:
  explicit Intrinsic_Calibration_Session(
    Chessboard_Configuration configuration);
  ~Intrinsic_Calibration_Session();

  Intrinsic_Calibration_Session(Intrinsic_Calibration_Session &&) noexcept;
  Intrinsic_Calibration_Session &operator=(
    Intrinsic_Calibration_Session &&) noexcept;
  Intrinsic_Calibration_Session(
    const Intrinsic_Calibration_Session &) = delete;
  Intrinsic_Calibration_Session &operator=(
    const Intrinsic_Calibration_Session &) = delete;

  bool Add_Image(
    const Image_View &image,
    const std::string &image_id,
    Image_Detection_Result *detection,
    std::string *error_message = nullptr);

  // Allows a different corner detector, or deterministic tests, to feed the
  // same calibration solver. Corners must be row-major across the board.
  bool Add_Observation(
    int image_width,
    int image_height,
    const std::vector<Image_Point> &corners,
    const std::string &image_id,
    std::string *error_message = nullptr);

  std::size_t Observation_Count() const;

  bool Calibrate(
    const Calibration_Options &options,
    Camera_Intrinsics *intrinsics,
    std::string *error_message = nullptr) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace camera_calibration

#endif // CAMERA_CALIBRATION_INTRINSIC_CALIBRATION_H_

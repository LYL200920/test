#ifndef CAMERA_CALIBRATION_GEOMETRY_H_
#define CAMERA_CALIBRATION_GEOMETRY_H_

#include "intrinsic_calibration.h"
#include "pose_transform.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace camera_calibration
{

// A_from_B maps column vectors from B to A. Translation is in millimetres.
// R is an explicitly named robot reference, not an implicit WORLD or BASE.
// F is the flange (never an active TCP); C is the OpenCV optical frame:
// +X right, +Y down, +Z forward. P has +Z normal and right-handed X/Y axes.
struct Calibration_Identity
{
  std::string id;
  std::string created_utc;
};

struct Camera_Image_Configuration
{
  std::string camera_serial;
  std::string lens_configuration_id; // Identifies unchanged lens/focus/aperture.
  int width = 0;
  int height = 0;
  // Sensor-pixel ROI before binning. Cropping/decimation beyond this model
  // requires a different configuration; equal output dimensions are insufficient.
  int roi_x = 0;
  int roi_y = 0;
  int roi_width = 0;
  int roi_height = 0;
  int binning_x = 1;
  int binning_y = 1;
};

struct Robot_Reference
{
  std::string robot_model_id;
  std::string reference_frame_id;
};

struct Intrinsic_Profile
{
  Calibration_Identity identity;
  Camera_Image_Configuration image;
  Chessboard_Configuration board;
  Camera_Intrinsics intrinsics;
};

struct Hand_Eye_Profile
{
  Calibration_Identity identity;
  Robot_Reference robot;
  Camera_Image_Configuration image;
  std::string intrinsic_id;
  robot_model::Matrix4 flange_from_camera{};
  std::string method;
  std::size_t training_samples = 0;
  std::size_t validation_samples = 0;
  double translation_rms_mm = 0;
  double rotation_rms_deg = 0;
  double reprojection_rms_px = 0;
};

enum class Plane_Source { Calibration_Board, Fixed_3D_Camera, Manual };

struct Work_Plane_Profile
{
  Calibration_Identity identity;
  Robot_Reference robot;
  Plane_Source source = Plane_Source::Manual;
  // For 3D: identifies the fixed camera extrinsic calibration version.
  // For board/manual: identifies the measurement/session record.
  std::string source_calibration_id;
  robot_model::Matrix4 reference_from_plane{};
  double min_x_mm = 0;
  double max_x_mm = 0;
  double min_y_mm = 0;
  double max_y_mm = 0;
  double fit_rms_mm = 0;
};

enum class Sample_Status { Pending, Accepted, Rejected };

struct Hand_Eye_Sample
{
  std::string session_id;
  std::string sample_id;
  std::string captured_utc;
  std::string intrinsic_id;
  std::string board_frame_id; // Same fixed, consistently oriented board per session.
  Robot_Reference robot;
  Camera_Image_Configuration image;
  std::size_t point_id = 0;
  std::uint32_t motion_sequence = 0;
  std::uint32_t state_request_sequence = 0;
  std::uint64_t state_revision = 0;
  // Nanoseconds from a session-local steady-clock origin; not wall/device time.
  std::uint64_t state_received_ns = 0;
  std::uint64_t trigger_requested_ns = 0;
  std::uint64_t frame_received_ns = 0;
  int reported_base = -1;
  int reported_tool = -1;
  // Already converted to the named R frame and to the flange, not raw $POS_ACT.
  robot_model::Matrix4 reference_from_flange{};
  robot_model::XyzabcPose actual_joints_deg{};
  // True only after acquisition has verified state identity/freshness/stability.
  bool pose_verified = false;
  std::uint64_t camera_frame_id = 0;
  // Raw device ticks; no claim of synchronization with robot/host clocks.
  std::optional<std::uint64_t> camera_timestamp_ticks;
  std::string camera_clock_id;
  double camera_tick_period_ns = 0;
  std::string image_path; // UTF-8, relative to the archive; loader never opens it.
  std::vector<Image_Point> corners;
  Sample_Status status = Sample_Status::Pending;
  std::string rejection_reason;
};

// Profiles may be added incrementally. A sample collection needs its intrinsic
// profile, but does not require a hand-eye result that has not yet been solved.
struct Calibration_Archive
{
  std::optional<Intrinsic_Profile> intrinsic;
  std::optional<Hand_Eye_Profile> hand_eye;
  std::optional<Work_Plane_Profile> work_plane;
  std::vector<Hand_Eye_Sample> samples;
};

bool Validate_Rigid_Transform(const robot_model::Matrix4 &matrix,
  std::string *error = nullptr);
bool Validate_Image_Configuration(const Camera_Image_Configuration &image,
  std::string *error = nullptr);
bool Image_Configurations_Match(const Camera_Image_Configuration &expected,
  const Camera_Image_Configuration &actual, std::string *error = nullptr);
bool Validate_Calibration_Archive(const Calibration_Archive &archive,
  std::string *error = nullptr);
// Strict runtime gate for the future calibrated mosaic path. No legacy fallback.
bool Match_Calibration_Context(const Calibration_Archive &archive,
  const Camera_Image_Configuration &image, const Robot_Reference &robot,
  std::string *error = nullptr);
// Output remains unchanged on failure.
bool Build_Reference_From_Camera(const robot_model::Matrix4 &reference_from_flange,
  const robot_model::Matrix4 &flange_from_camera,
  robot_model::Matrix4 *reference_from_camera, std::string *error = nullptr);

} // namespace camera_calibration
#endif

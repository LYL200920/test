#include "calibration_geometry.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

namespace camera_calibration
{
namespace
{
bool Fail(std::string *error, const std::string &text)
{
  if (error) *error = text;
  return false;
}
bool Text(const std::string &value)
{
  return !value.empty() && value.find('\0') == std::string::npos &&
    value.find_first_not_of(" \t\r\n") != std::string::npos;
}
bool Identity(const Calibration_Identity &value)
{
  return Text(value.id) && Text(value.created_utc);
}
bool Reference(const Robot_Reference &value)
{
  return Text(value.robot_model_id) && Text(value.reference_frame_id);
}
bool Same_Reference(const Robot_Reference &a, const Robot_Reference &b)
{
  return a.robot_model_id == b.robot_model_id &&
    a.reference_frame_id == b.reference_frame_id;
}
bool Relative_Image_Path(const std::string &value)
{
  if (!Text(value) || value.front() == '/' || value.find('\\') != std::string::npos ||
      value.find(':') != std::string::npos) return false;
  std::size_t begin = 0;
  while (begin <= value.size())
  {
    const auto end = value.find('/', begin);
    const auto part = value.substr(begin, end == std::string::npos ? end : end - begin);
    if (part.empty() || part == "." || part == "..") return false;
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  return true;
}
bool Nonnegative(double value)
{
  return std::isfinite(value) && value >= 0;
}
}

bool Validate_Rigid_Transform(const robot_model::Matrix4 &m, std::string *error)
{
  if (error) error->clear();
  for (const auto &row : m)
    for (double value : row)
      if (!std::isfinite(value)) return Fail(error, "Transform contains nonfinite values");
  constexpr double tolerance = 1e-6;
  for (int col = 0; col < 4; ++col)
    if (std::abs(m[3][col] - (col == 3 ? 1.0 : 0.0)) > tolerance)
      return Fail(error, "Invalid homogeneous transform bottom row");
  for (int a = 0; a < 3; ++a)
    for (int b = 0; b < 3; ++b)
    {
      double dot = 0;
      for (int k = 0; k < 3; ++k) dot += m[k][a] * m[k][b];
      if (std::abs(dot - (a == b ? 1.0 : 0.0)) > tolerance)
        return Fail(error, "Transform rotation is not orthonormal");
    }
  const double determinant =
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  if (std::abs(determinant - 1.0) > tolerance)
    return Fail(error, "Transform rotation must have determinant +1");
  return true;
}

bool Validate_Image_Configuration(const Camera_Image_Configuration &v, std::string *error)
{
  if (error) error->clear();
  if (!Text(v.camera_serial) || !Text(v.lens_configuration_id))
    return Fail(error, "Camera serial and lens configuration are required");
  if (v.width <= 0 || v.height <= 0 || v.roi_x < 0 || v.roi_y < 0 ||
      v.roi_width <= 0 || v.roi_height <= 0 || v.binning_x <= 0 || v.binning_y <= 0 ||
      static_cast<std::int64_t>(v.width) * v.binning_x != v.roi_width ||
      static_cast<std::int64_t>(v.height) * v.binning_y != v.roi_height)
    return Fail(error, "Invalid ROI, image dimensions or binning");
  return true;
}

bool Image_Configurations_Match(const Camera_Image_Configuration &a,
  const Camera_Image_Configuration &b, std::string *error)
{
  if (!Validate_Image_Configuration(a, error) || !Validate_Image_Configuration(b, error))
    return false;
  if (a.camera_serial != b.camera_serial || a.lens_configuration_id != b.lens_configuration_id ||
      a.width != b.width || a.height != b.height || a.roi_x != b.roi_x || a.roi_y != b.roi_y ||
      a.roi_width != b.roi_width || a.roi_height != b.roi_height ||
      a.binning_x != b.binning_x || a.binning_y != b.binning_y)
    return Fail(error, "Camera/lens/ROI/binning configuration mismatch");
  return true;
}

bool Validate_Calibration_Archive(const Calibration_Archive &archive, std::string *error)
{
  if (error) error->clear();
  if (!archive.intrinsic && !archive.hand_eye && !archive.work_plane && archive.samples.empty())
    return Fail(error, "Calibration archive is empty");
  if (archive.intrinsic)
  {
    const auto &v = *archive.intrinsic;
    if (!Identity(v.identity)) return Fail(error, "Intrinsic identity is incomplete");
    if (!Validate_Image_Configuration(v.image, error) ||
        !Validate_Chessboard_Configuration(v.board, error) ||
        !Validate_Camera_Intrinsics(v.intrinsics, error)) return false;
    const auto &k = v.intrinsics.camera_matrix;
    if (v.intrinsics.camera_serial_number != v.image.camera_serial ||
        v.intrinsics.image_width != v.image.width || v.intrinsics.image_height != v.image.height ||
        k[3] != 0 || k[6] != 0 || k[7] != 0 || k[8] != 1)
      return Fail(error, "Intrinsic camera identity, dimensions or matrix layout mismatch");
    for (double value : v.intrinsics.intrinsic_standard_deviations)
      if (!Nonnegative(value)) return Fail(error, "Invalid intrinsic standard deviation");
    for (const auto *views : {&v.intrinsics.accepted_views, &v.intrinsics.rejected_views})
      for (const auto &view : *views)
        if (!Text(view.image_id) || !Nonnegative(view.rms_px))
          return Fail(error, "Invalid intrinsic view error");
  }
  if (archive.hand_eye)
  {
    const auto &v = *archive.hand_eye;
    if (!Identity(v.identity) || !Reference(v.robot) || !Text(v.intrinsic_id) ||
        !Text(v.method) || v.training_samples < 3 || v.validation_samples == 0 ||
        !Nonnegative(v.translation_rms_mm) || !Nonnegative(v.rotation_rms_deg) ||
        !Nonnegative(v.reprojection_rms_px))
      return Fail(error, "Invalid hand-eye identity or validation statistics");
    if (!Validate_Image_Configuration(v.image, error) ||
        !Validate_Rigid_Transform(v.flange_from_camera, error)) return false;
    if (archive.intrinsic && (v.intrinsic_id != archive.intrinsic->identity.id ||
        !Image_Configurations_Match(archive.intrinsic->image, v.image, error)))
      return Fail(error, "Hand-eye profile does not match the intrinsic profile");
  }
  if (archive.work_plane)
  {
    const auto &v = *archive.work_plane;
    if (!Identity(v.identity) || !Reference(v.robot) || !Text(v.source_calibration_id) ||
        !std::isfinite(v.min_x_mm) || !std::isfinite(v.max_x_mm) ||
        !std::isfinite(v.min_y_mm) || !std::isfinite(v.max_y_mm) ||
        v.min_x_mm >= v.max_x_mm || v.min_y_mm >= v.max_y_mm || !Nonnegative(v.fit_rms_mm) ||
        (v.source != Plane_Source::Calibration_Board && v.source != Plane_Source::Fixed_3D_Camera &&
         v.source != Plane_Source::Manual))
      return Fail(error, "Invalid work-plane metadata or bounds");
    if (!Validate_Rigid_Transform(v.reference_from_plane, error)) return false;
    if (archive.hand_eye && !Same_Reference(v.robot, archive.hand_eye->robot))
      return Fail(error, "Work plane and hand-eye reference frames differ");
  }
  std::set<std::pair<std::string, std::string>> ids;
  const Hand_Eye_Sample *first = nullptr;
  for (const auto &s : archive.samples)
  {
    if (!archive.intrinsic) return Fail(error, "Samples require an intrinsic profile");
    if (!Text(s.session_id) || !Text(s.sample_id) || !Text(s.captured_utc) ||
        !Text(s.board_frame_id) || !Relative_Image_Path(s.image_path) || !Reference(s.robot) ||
        !ids.emplace(s.session_id, s.sample_id).second ||
        s.intrinsic_id != archive.intrinsic->identity.id)
      return Fail(error, "Invalid or duplicate sample identity");
    if (!Image_Configurations_Match(archive.intrinsic->image, s.image, error) ||
        !Validate_Rigid_Transform(s.reference_from_flange, error)) return false;
    for (double joint : s.actual_joints_deg)
      if (!std::isfinite(joint)) return Fail(error, "Invalid actual joint value");
    if (s.reported_base < 0 || s.reported_tool < 0 || s.motion_sequence == 0 ||
        s.state_request_sequence == 0 || s.state_revision == 0 || s.point_id == 0)
      return Fail(error, "Sample is missing robot state provenance");
    if (s.state_received_ns > s.trigger_requested_ns || s.trigger_requested_ns > s.frame_received_ns)
      return Fail(error, "Invalid sample host timestamp order");
    if (s.camera_timestamp_ticks && (!Text(s.camera_clock_id) ||
        !std::isfinite(s.camera_tick_period_ns) || s.camera_tick_period_ns <= 0))
      return Fail(error, "Camera timestamp requires clock identity and tick units");
    if (!s.camera_timestamp_ticks && (!s.camera_clock_id.empty() || s.camera_tick_period_ns != 0))
      return Fail(error, "Camera clock metadata without timestamp");
    const auto &board = archive.intrinsic->board;
    const auto corner_count = static_cast<std::size_t>(board.inner_corner_columns) * board.inner_corner_rows;
    if (s.status != Sample_Status::Pending && s.status != Sample_Status::Accepted &&
        s.status != Sample_Status::Rejected) return Fail(error, "Invalid sample status");
    if (s.status == Sample_Status::Accepted && (!s.pose_verified || s.corners.size() != corner_count ||
        s.frame_received_ns == 0 || !s.rejection_reason.empty())) return Fail(error, "Accepted sample lacks verified pose or corners");
    if (s.status == Sample_Status::Rejected && !Text(s.rejection_reason))
      return Fail(error, "Rejected sample requires a reason");
    for (const auto &p : s.corners)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
          p.x >= s.image.width || p.y >= s.image.height)
        return Fail(error, "Sample corner outside the image");
    if (first && (s.session_id != first->session_id || s.board_frame_id != first->board_frame_id ||
        !Same_Reference(s.robot, first->robot)))
      return Fail(error, "Sample collection mixes sessions, boards or robot references");
    if (archive.hand_eye && !Same_Reference(s.robot, archive.hand_eye->robot))
      return Fail(error, "Sample and hand-eye reference frames differ");
    if (archive.work_plane && !Same_Reference(s.robot, archive.work_plane->robot))
      return Fail(error, "Sample and work-plane reference frames differ");
    first = &s;
  }
  return true;
}

bool Match_Calibration_Context(const Calibration_Archive &archive,
  const Camera_Image_Configuration &image, const Robot_Reference &robot, std::string *error)
{
  if (!Validate_Calibration_Archive(archive, error)) return false;
  if (!archive.intrinsic || !archive.hand_eye || !archive.work_plane)
    return Fail(error, "Calibrated projection requires intrinsics, hand-eye and work plane");
  if (!Image_Configurations_Match(archive.intrinsic->image, image, error)) return false;
  if (!Same_Reference(archive.hand_eye->robot, robot))
    return Fail(error, "Runtime robot reference does not match calibration");
  return true;
}

bool Build_Reference_From_Camera(const robot_model::Matrix4 &reference_from_flange,
  const robot_model::Matrix4 &flange_from_camera, robot_model::Matrix4 *output, std::string *error)
{
  if (!output) return Fail(error, "Transform output is null");
  if (!Validate_Rigid_Transform(reference_from_flange, error) ||
      !Validate_Rigid_Transform(flange_from_camera, error)) return false;
  auto result = robot_model::Multiply_Matrices(reference_from_flange, flange_from_camera);
  if (!Validate_Rigid_Transform(result, error)) return false;
  *output = result;
  return true;
}
} // namespace camera_calibration

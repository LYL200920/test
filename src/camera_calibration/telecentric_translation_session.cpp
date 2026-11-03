#include "telecentric_translation_session.h"
#include <cmath>
#include <set>

namespace camera_calibration
{
namespace
{
bool Fail(std::string *error, const std::string &text)
{
  if (error) *error = text;
  return false;
}
bool Text(const std::string &text)
{
  return text.find('\0') == std::string::npos && text.find_first_not_of(" \t\r\n") != std::string::npos;
}
bool Pose(const Translation_Pose_Snapshot &pose)
{
  for (double value : pose.reported_pose) if (!std::isfinite(value)) return false;
  for (double value : pose.joints_deg) if (!std::isfinite(value)) return false;
  return pose.motion_state.find('\0') == std::string::npos &&
    pose.reported_base >= -1 && pose.reported_tool >= -1 &&
    (!pose.available || (pose.revision > 0 && pose.received_ns > 0));
}
}
bool Check_Translation_Capture_Timing(const Translation_Capture_Evidence &v, std::string *error)
{
  if (error) error->clear();
  if (!v.software_triggered || !v.before.available || !v.after.available || !Pose(v.before) || !Pose(v.after))
    return Fail(error, "Missing software-trigger or actual-state evidence");
  if (!v.before.matched_state_request || !v.after.matched_state_request ||
      v.before.active_sequence != 0 || v.after.active_sequence != 0 ||
      v.before.motion_state != "IDLE" || v.after.motion_state != "IDLE")
    return Fail(error, "Idle responses must be correlated to explicit sampling requests");
  if (v.before.request_sequence == 0 || v.after.request_sequence == 0 ||
      v.before.request_sequence == v.after.request_sequence || v.after.revision <= v.before.revision)
    return Fail(error, "Two distinct correlated state responses are required");
  if (v.before.received_ns > v.trigger_requested_ns || v.trigger_requested_ns > v.frame_observed_ns ||
      v.frame_observed_ns > v.after.received_ns ||
      v.trigger_requested_ns - v.before.received_ns > 250000000ULL ||
      v.after.received_ns - v.frame_observed_ns > 250000000ULL)
    return Fail(error, "State responses do not freshly bracket the capture");
  if (v.before.reported_base < 0 || v.before.reported_tool < 0 ||
      v.before.reported_base != v.after.reported_base || v.before.reported_tool != v.after.reported_tool)
    return Fail(error, "Controller BASE/TOOL identity is unknown or changed");
  return true;
}
bool Validate_Telecentric_Translation_Session(const Telecentric_Translation_Session &v, std::string *error)
{
  if (error) error->clear();
  if (!Text(v.identity.id) || !Text(v.identity.created_utc) || !Text(v.robot.robot_model_id) ||
      !Text(v.robot.reference_frame_id) || !Text(v.board_frame_id))
    return Fail(error, "Session, robot reference and fixed board identities are required");
  if (!Match_Telecentric_Planar_Profile(v.planar_profile, v.planar_profile.image,
        v.planar_profile.fixed_setup_id, error)) return false;
  std::set<std::string> ids, paths;
  std::set<std::uint64_t> frames;
  for (const auto &s : v.samples)
  {
    if (!Text(s.id) || !Text(s.captured_utc) || !ids.insert(s.id).second ||
        !Text(s.image_path) || !paths.insert(s.image_path).second ||
        s.image_path.find_first_of(":\\") != std::string::npos || s.image_path.front() == '/' ||
        (s.role != Translation_Sample_Role::Training && s.role != Translation_Sample_Role::Validation))
      return Fail(error, "Invalid or duplicate session sample metadata");
    const auto path = std::filesystem::u8path(s.image_path);
    for (const auto &part : path)
      if (part == ".." || part == ".") return Fail(error, "Sample image path must stay within the session");
    if (!Pose(s.evidence.before) || !Pose(s.evidence.after) || !s.evidence.software_triggered ||
        s.evidence.trigger_requested_ns == 0 || s.evidence.frame_observed_ns < s.evidence.trigger_requested_ns ||
        !frames.insert(s.evidence.camera_frame_id).second)
      return Fail(error, "Invalid or duplicate capture evidence");
    const auto &d = s.detection;
    if (!Text(d.image_id) || d.image_width != v.planar_profile.image.width ||
        d.image_height != v.planar_profile.image.height)
      return Fail(error, "Sample image size/identity does not match planar profile");
    if (d.chessboard_found)
    {
      if (!Validate_Detected_Chessboard(v.planar_profile.result.board, d.corners,
            d.image_width, d.image_height, nullptr, error)) return false;
    }
    else if (!d.corners.empty()) return Fail(error, "Rejected detection must not contain accepted corners");
    if (d.diagnostic.find('\0') != std::string::npos || d.detection_method.find('\0') != std::string::npos)
      return Fail(error, "Invalid detection diagnostic text");
  }
  return true;
}
}

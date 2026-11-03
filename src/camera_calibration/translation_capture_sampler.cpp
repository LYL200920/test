#include "translation_capture_sampler.h"
#include <algorithm>
#include <cmath>

namespace camera_calibration
{
namespace
{
constexpr std::uint64_t state_timeout_ns = 250000000;
bool Fail_Check(std::string *error, const char *message)
{
  if (error) *error = message;
  return false;
}
}
bool Check_Translation_Capture_Stability(const Translation_Capture_Evidence &v, std::string *error)
{
  if (!Check_Translation_Capture_Timing(v, error)) return false;
  double squared = 0;
  for (int i = 0; i < 3; ++i)
    squared += std::pow(v.before.reported_pose[i] - v.after.reported_pose[i], 2);
  if (std::sqrt(squared) > 0.05) return Fail_Check(error, "Cartesian endpoints differ by more than 0.05 mm");
  const auto before = robot_model::Build_Zyx_Pose_Matrix(v.before.reported_pose);
  const auto after = robot_model::Build_Zyx_Pose_Matrix(v.after.reported_pose);
  double trace = 0;
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) trace += before[i][j] * after[i][j];
  const double degrees = std::acos(std::clamp((trace - 1) * 0.5, -1.0, 1.0)) * 180 / std::acos(-1.0);
  if (degrees > 0.02) return Fail_Check(error, "Orientation endpoints differ by more than 0.02 degree");
  for (int i = 0; i < 6; ++i)
    if (std::abs(v.before.joints_deg[i] - v.after.joints_deg[i]) > 0.02)
      return Fail_Check(error, "Joint endpoints differ by more than 0.02 degree");
  return true;
}
void Translation_Capture_Sampler::Reset()
{
  state_ = Translation_Sampling_State::Idle; evidence_ = {}; expected_ = 0;
  requested_ns_ = minimum_revision_ = 0; error_.clear();
}
void Translation_Capture_Sampler::Fail(const std::string &message)
{
  error_ = message; state_ = Translation_Sampling_State::Failed;
}
void Translation_Capture_Sampler::Begin(std::uint32_t motion, std::uint64_t minimum_revision,
  std::uint32_t request, std::uint64_t requested_ns)
{
  Reset();
  if (!motion || !request || motion == request || !requested_ns)
  { Fail("Invalid pre-capture state request identity"); return; }
  evidence_.motion_sequence = motion; minimum_revision_ = minimum_revision;
  expected_ = request; requested_ns_ = requested_ns; state_ = Translation_Sampling_State::Before;
}
void Translation_Capture_Sampler::Observe(const Translation_Pose_Snapshot &pose)
{
  if (state_ != Translation_Sampling_State::Before && state_ != Translation_Sampling_State::After) return;
  if (pose.request_sequence != expected_) return;
  if (!pose.available || pose.revision <= minimum_revision_ || pose.received_ns < requested_ns_ ||
      pose.received_ns - requested_ns_ > state_timeout_ns || pose.active_sequence != 0 ||
      pose.motion_state != "IDLE" || pose.reported_base < 0 || pose.reported_tool < 0)
  { Fail("Invalid, stale or non-idle correlated robot state"); return; }
  for (double x : pose.reported_pose) if (!std::isfinite(x)) { Fail("Nonfinite robot pose"); return; }
  for (double x : pose.joints_deg) if (!std::isfinite(x)) { Fail("Nonfinite robot joints"); return; }
  auto matched = pose; matched.matched_state_request = true;
  if (state_ == Translation_Sampling_State::Before)
  {
    evidence_.before = matched; state_ = Translation_Sampling_State::Trigger_Ready;
  }
  else
  {
    evidence_.after = matched;
    if (!Check_Translation_Capture_Stability(evidence_, &error_)) state_ = Translation_Sampling_State::Failed;
    else state_ = Translation_Sampling_State::Complete;
  }
}
bool Translation_Capture_Sampler::Trigger(std::uint64_t now)
{
  if (state_ != Translation_Sampling_State::Trigger_Ready) return false;
  if (now < evidence_.before.received_ns || now - evidence_.before.received_ns > state_timeout_ns)
  { Fail("Pre-trigger robot state expired"); return false; }
  evidence_.trigger_requested_ns = now; evidence_.software_triggered = true;
  state_ = Translation_Sampling_State::Image;
  return true;
}
bool Translation_Capture_Sampler::Image(std::uint64_t frame, std::uint64_t observed_ns,
  std::uint32_t request, std::uint64_t requested_ns)
{
  if (state_ != Translation_Sampling_State::Image) return false;
  if (!request || request == evidence_.before.request_sequence || request == evidence_.motion_sequence ||
      observed_ns < evidence_.trigger_requested_ns || requested_ns < observed_ns)
  { Fail("Invalid post-capture state request identity or timing"); return false; }
  evidence_.camera_frame_id = frame; evidence_.frame_observed_ns = observed_ns;
  expected_ = request; requested_ns_ = requested_ns; minimum_revision_ = evidence_.before.revision;
  state_ = Translation_Sampling_State::After;
  return true;
}
void Translation_Capture_Sampler::Tick(std::uint64_t now)
{
  if ((state_ == Translation_Sampling_State::Before || state_ == Translation_Sampling_State::After) &&
      now >= requested_ns_ && now - requested_ns_ > state_timeout_ns)
    Fail("Timed out waiting for correlated GET_STATE response");
}
}

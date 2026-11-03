#include "translation_capture_sampler.h"
#include <iostream>
#include <stdexcept>
#include <limits>
using namespace camera_calibration;
void Require(bool condition, const char *message) { if (!condition) throw std::runtime_error(message); }
Translation_Pose_Snapshot Pose(std::uint32_t request, std::uint64_t revision, std::uint64_t ns)
{
  Translation_Pose_Snapshot p;
  p.available = true; p.request_sequence = request; p.revision = revision; p.received_ns = ns;
  p.motion_state = "IDLE"; p.reported_base = p.reported_tool = 0;
  return p;
}
void Test()
{
  using State = Translation_Sampling_State;
  Translation_Capture_Sampler s;
  s.Begin(1, 5, 2, 1000000000);
  s.Observe(Pose(1, 6, 1010000000));
  Require(s.State() == State::Before, "Motion state mistaken for sampling response");
  s.Observe(Pose(2, 6, 1010000000));
  Require(s.State() == State::Trigger_Ready, "Pre-state not matched");
  Require(s.Trigger(1020000000), "Trigger refused");
  Require(s.Image(42, 1050000000, 3, 1050000000), "Frame refused");
  s.Observe(Pose(2, 7, 1060000000));
  Require(s.State() == State::After, "Late duplicate accepted as post-state");
  s.Observe(Pose(3, 8, 1070000000));
  Require(s.State() == State::Complete && s.Evidence().after.matched_state_request, "Bracket not completed");
  auto v = s.Evidence();
  Require(Check_Translation_Capture_Stability(v), "Stable endpoints rejected");
  v.after.reported_pose[0] = 0.051;
  Require(!Check_Translation_Capture_Stability(v), "Translation change accepted");
  v = s.Evidence(); v.after.reported_pose[3] = 0.021;
  Require(!Check_Translation_Capture_Stability(v), "Rotation change accepted");
  v = s.Evidence(); v.before.reported_pose[3] = 180; v.after.reported_pose[3] = -180;
  Require(Check_Translation_Capture_Stability(v), "Euler wrap rejected");
  v = s.Evidence(); v.after.joints_deg[0] = 0.021;
  Require(!Check_Translation_Capture_Stability(v), "Joint change accepted");
  s.Begin(1, 5, 2, 1000000000); s.Tick(1250000001);
  Require(s.State() == State::Failed, "State timeout missed");
  s.Reset(); s.Observe(Pose(2, 6, 1010000000));
  Require(s.State() == State::Idle, "Canceled session revived");
  s.Begin(1, 5, 2, 1000000000); s.Observe(Pose(2, 5, 1010000000));
  Require(s.State() == State::Failed, "Stale revision accepted");
  s.Begin(1, 5, 2, 1000000000); auto bad = Pose(2, 6, 1010000000); bad.active_sequence = 99;
  s.Observe(bad); Require(s.State() == State::Failed, "Moving state accepted");
  s.Begin(1, 5, 2, 1000000000); bad = Pose(2, 6, 1010000000); bad.reported_pose[0] = std::numeric_limits<double>::quiet_NaN();
  s.Observe(bad); Require(s.State() == State::Failed, "Nonfinite pose accepted");
  s.Begin(1, 5, 2, 1000000000); s.Observe(Pose(2, 6, 1010000000));
  Require(!s.Trigger(1300000000) && s.State() == State::Failed, "Expired pre-state accepted");
  s.Begin(1, 5, 2, 1000000000); s.Observe(Pose(2, 6, 1010000000)); s.Trigger(1020000000);
  s.Image(42, 1050000000, 3, 1050000000); s.Tick(1300000001);
  Require(s.State() == State::Failed, "Post-state timeout missed");
  s.Reset(); s.Observe(Pose(3, 8, 1070000000));
  Require(s.State() == State::Idle, "Late post-state revived canceled session");
  // A callback buffered during Synchronize is delivered after request registration.
  const auto early = Pose(4, 9, 2010000000);
  s.Begin(1, 8, 4, 2000000000); s.Observe(early);
  Require(s.State() == State::Trigger_Ready, "Buffered synchronous reply lost");
  s.Trigger(2020000000);
  Require(!s.Image(43, 2050000000, 4, 2050000000), "Reused request sequence accepted");
  s.Begin(1, 5, 2, 1000000000); s.Observe(Pose(2, 6, 1010000000)); s.Trigger(1020000000);
  s.Image(42, 1050000000, 3, 1050000000); bad = Pose(3, 8, 1070000000); bad.reported_tool = 1;
  s.Observe(bad); Require(s.State() == State::Failed, "Tool change accepted");
}
int main()
{
  try { Test(); return 0; }
  catch (const std::exception &e) { std::cerr << e.what() << std::endl; return 1; }
}

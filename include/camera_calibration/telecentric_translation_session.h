#ifndef TELECENTRIC_TRANSLATION_SESSION_H_
#define TELECENTRIC_TRANSLATION_SESSION_H_

#include "telecentric_planar_calibration.h"

namespace camera_calibration
{
// Host steady-clock nanoseconds, shared by both snapshots and capture events.
// Raw controller Cartesian feedback is NOT assumed to be flange-in-reference.
struct Translation_Pose_Snapshot
{
  bool available = false;
  std::uint64_t revision = 0;
  std::uint32_t request_sequence = 0;
  std::uint64_t received_ns = 0;
  std::string motion_state;
  std::uint32_t active_sequence = 0;
  bool matched_state_request = false;
  robot_model::XyzabcPose reported_pose{};
  robot_model::XyzabcPose joints_deg{};
  int reported_base = -1;
  int reported_tool = -1;
};

struct Translation_Capture_Evidence
{
  std::uint32_t motion_sequence = 0;
  Translation_Pose_Snapshot before;
  Translation_Pose_Snapshot after;
  std::uint64_t trigger_requested_ns = 0;
  std::uint64_t frame_observed_ns = 0; // UI observation, not SDK receipt/exposure.
  std::uint64_t camera_frame_id = 0;
  // Host cannot infer hardware exposure time from frame delivery time.
  bool software_triggered = false;
};

enum class Translation_Sample_Role { Training, Validation };
struct Telecentric_Translation_Sample
{
  std::string id;
  std::string captured_utc;
  std::size_t point_id = 0;
  std::size_t retry_index = 0;
  Translation_Sample_Role role = Translation_Sample_Role::Training;
  std::string image_path;
  Image_Detection_Result detection;
  Translation_Capture_Evidence evidence;
  // Acquisition-only record. No checkbox can promote raw feedback to a
  // verified flange pose; that requires a future reference-aware verifier.
};
struct Telecentric_Translation_Session
{
  Calibration_Identity identity;
  Telecentric_Planar_Profile planar_profile;
  Robot_Reference robot;
  std::string board_frame_id;
  std::vector<Telecentric_Translation_Sample> samples;
};

// Structural validation only. Missing state metadata is permitted and remains
// explicitly unavailable; this is an acquisition archive, not a solved profile.
bool Validate_Telecentric_Translation_Session(const Telecentric_Translation_Session &session,
  std::string *error = nullptr);
// Reports the evidence still needed before a sample can be used by a solver.
// Passing does not establish coordinate semantics or exposure synchronization.
bool Check_Translation_Capture_Timing(const Translation_Capture_Evidence &evidence,
  std::string *error = nullptr);
bool Save_Telecentric_Translation_Session(const std::filesystem::path &path,
  const Telecentric_Translation_Session &session, std::string *error = nullptr);
bool Load_Telecentric_Translation_Session(const std::filesystem::path &path,
  Telecentric_Translation_Session *session, std::string *error = nullptr);
}
#endif

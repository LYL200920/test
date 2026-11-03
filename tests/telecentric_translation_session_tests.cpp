#include "telecentric_translation_session.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace
{
using namespace camera_calibration;
void Require(bool value, const char *message)
{
  if (!value) throw std::runtime_error(message);
}
Telecentric_Translation_Session Fixture()
{
  Telecentric_Translation_Session s;
  s.identity = {"session1", "2026-11-03T00:00:00Z"};
  s.robot = {"KR10", "controller-reference-unverified"};
  s.board_frame_id = "fixed-board";
  auto &p = s.planar_profile;
  p.identity = {"planar1", s.identity.created_utc};
  p.fixed_setup_id = "setup1";
  p.image = {"SN1", "lens1", 640, 480, 0, 0, 640, 480, 1, 1};
  p.result.width = 640; p.result.height = 480;
  p.result.board = {3, 3, 0.5};
  p.result.pixels_from_plane_mm = {30, 0, 0, 30};
  p.result.views = {{"v1", 0, 0, 0}, {"v2", 0, 0, 0}, {"v3", 0, 0, 0}};
  Telecentric_Translation_Sample sample;
  sample.id = "sample1"; sample.captured_utc = s.identity.created_utc;
  sample.point_id = 2; sample.image_path = "images/sample1.png";
  sample.detection.image_id = "sample1.png"; sample.detection.image_width = 640;
  sample.detection.image_height = 480; sample.detection.chessboard_found = true;
  for (int y = 0; y < 3; ++y) for (int x = 0; x < 3; ++x)
    sample.detection.corners.push_back({100.0 + x * 15, 100.0 + y * 15});
  sample.evidence.software_triggered = true;
  sample.evidence.trigger_requested_ns = 1000000000;
  sample.evidence.frame_observed_ns = 1100000000;
  sample.evidence.camera_frame_id = 42;
  s.samples.push_back(sample);
  return s;
}
void Test()
{
  auto session = Fixture();
  std::string error;
  Require(Validate_Telecentric_Translation_Session(session, &error), "Unverified capture must be archivable");
  Require(!Check_Translation_Capture_Timing(session.samples[0].evidence, &error), "Missing poses accepted as bracket");
  auto evidence = session.samples[0].evidence;
  evidence.before.available = evidence.after.available = true;
  evidence.before.matched_state_request = evidence.after.matched_state_request = true;
  evidence.before.motion_state = evidence.after.motion_state = "IDLE";
  evidence.before.revision = 1; evidence.after.revision = 2;
  evidence.before.request_sequence = 10; evidence.after.request_sequence = 11;
  evidence.before.received_ns = 950000000; evidence.after.received_ns = 1150000000;
  evidence.before.reported_base = evidence.after.reported_base = 0;
  evidence.before.reported_tool = evidence.after.reported_tool = 0;
  Require(Check_Translation_Capture_Timing(evidence, &error), "Valid host timing rejected");
  auto bad = evidence; bad.after = bad.before;
  Require(!Check_Translation_Capture_Timing(bad), "Repeated cached state accepted");
  bad = evidence; bad.before.received_ns = 100;
  Require(!Check_Translation_Capture_Timing(bad), "Stale state accepted");
  bad = evidence; bad.before.reported_tool = -1;
  Require(!Check_Translation_Capture_Timing(bad), "Unknown tool accepted");
  bad = evidence; bad.after.received_ns = bad.frame_observed_ns - 1;
  Require(!Check_Translation_Capture_Timing(bad), "Pre-frame state accepted as post-frame");
  auto invalid = session; invalid.samples.push_back(invalid.samples[0]);
  Require(!Validate_Telecentric_Translation_Session(invalid), "Duplicate sample accepted");
  invalid = session; invalid.samples[0].image_path = "../outside.png";
  Require(!Validate_Telecentric_Translation_Session(invalid), "Escaping path accepted");
  invalid = session; invalid.samples[0].detection.image_width = 4096;
  Require(!Validate_Telecentric_Translation_Session(invalid), "Mismatched image accepted");

  const auto directory = std::filesystem::temp_directory_path() / ("translation-session-test-" +
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  Require(std::filesystem::create_directory(directory), "Cannot reserve test directory");
  struct Cleanup { std::filesystem::path path; ~Cleanup() { std::error_code ec; std::filesystem::remove_all(path, ec); } } cleanup{directory};
  const auto path = directory / "session.xml";
  Require(Save_Telecentric_Translation_Session(path, session, &error), "Save failed");
  Require(!Save_Telecentric_Translation_Session(path, session, &error), "Overwrite permitted");
  Telecentric_Translation_Session loaded;
  Require(Load_Telecentric_Translation_Session(path, &loaded, &error), "Load failed");
  Require(loaded.samples.size() == 1 && loaded.samples[0].evidence.camera_frame_id == 42 &&
    !loaded.samples[0].evidence.before.available && loaded.planar_profile.identity.id == "planar1", "Roundtrip promoted or lost evidence");
  const auto malformed = directory / "malformed.xml";
  std::ifstream input(path); std::string xml((std::istreambuf_iterator<char>(input)), {});
  const auto pos = xml.find("acquisition-only-unverified");
  Require(pos != std::string::npos, "Missing unverified convention");
  xml.replace(pos, std::string("acquisition-only-unverified").size(), "verified");
  { std::ofstream output(malformed); output << xml; }
  Require(!Load_Telecentric_Translation_Session(malformed, &loaded, &error) && loaded.identity.id == session.identity.id,
    "False verified convention accepted or output mutated");
}
}
int main()
{
  try { Test(); return 0; }
  catch (const std::exception &error) { std::cerr << error.what() << std::endl; return 1; }
}

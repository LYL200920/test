#include "calibration_archive_repository.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using namespace camera_calibration;
void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}
struct Temporary_Directory
{
  std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("calibration_geometry_" + std::to_string(
      std::chrono::high_resolution_clock::now().time_since_epoch().count()));
  Temporary_Directory() { std::filesystem::create_directory(path); }
  ~Temporary_Directory()
  {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
  }
};
Calibration_Archive Fixture()
{
  Calibration_Archive a;
  Intrinsic_Profile i;
  i.identity = {"intrinsic-1", "2026-11-02T10:00:00Z"};
  i.image = {"SN-123", "lens-fixed-focus-1", 640, 480, 8, 12, 1280, 960, 2, 2};
  i.board = {2, 2, 10.0};
  i.intrinsics.camera_serial_number = i.image.camera_serial;
  i.intrinsics.camera_model = "Test camera";
  i.intrinsics.pixel_format = "Mono8";
  i.intrinsics.image_width = 640;
  i.intrinsics.image_height = 480;
  i.intrinsics.camera_matrix = {800, 0, 320, 0, 810, 240, 0, 0, 1};
  i.intrinsics.distortion = {-0.12, 0.03, 0.001, -0.002, 0.004};
  i.intrinsics.overall_rms_px = 0.25;
  i.intrinsics.accepted_views.push_back({"棋盘图.png", 0.24});
  i.intrinsics.rejected_views.push_back({"rejected.png", 1.4});
  a.intrinsic = i;
  Hand_Eye_Profile h;
  h.identity = {"hand-eye-1", "2026-11-02T11:00:00Z"};
  h.robot = {"KR10", "robot-world-calibrated-1"};
  h.image = i.image;
  h.intrinsic_id = i.identity.id;
  h.flange_from_camera = robot_model::Build_Zyx_Pose_Matrix({10, 20, 100, 15, -5, 90});
  h.method = "synthetic-test-not-a-solver";
  h.training_samples = 15;
  h.validation_samples = 5;
  h.translation_rms_mm = 0.1;
  h.rotation_rms_deg = 0.05;
  h.reprojection_rms_px = 0.3;
  a.hand_eye = h;
  Work_Plane_Profile p;
  p.identity = {"plane-1", "2026-11-02T12:00:00Z"};
  p.robot = h.robot;
  p.source = Plane_Source::Fixed_3D_Camera;
  p.source_calibration_id = "fixed-3d-eye-to-hand-1";
  p.reference_from_plane = robot_model::Build_Zyx_Pose_Matrix({400, 0, 700, 90, 0, 0});
  p.min_x_mm = -100; p.max_x_mm = 100;
  p.min_y_mm = -50; p.max_y_mm = 50;
  p.fit_rms_mm = 0.12;
  a.work_plane = p;
  Hand_Eye_Sample s;
  s.session_id = "session-1"; s.sample_id = "sample-1";
  s.captured_utc = "2026-11-02T10:30:00Z";
  s.intrinsic_id = i.identity.id; s.board_frame_id = "fixed-board-origin-mark-1";
  s.robot = h.robot; s.image = i.image;
  s.point_id = 42; s.motion_sequence = 12; s.state_request_sequence = 13; s.state_revision = 99;
  s.state_received_ns = 100; s.trigger_requested_ns = 200; s.frame_received_ns = 300;
  s.reported_base = 0; s.reported_tool = 0;
  s.reference_from_flange = robot_model::Build_Zyx_Pose_Matrix({400, -200, 500, 90, 20, 10});
  s.actual_joints_deg = {1, 2, 3, 4, 5, 6};
  s.pose_verified = true;
  s.camera_frame_id = std::numeric_limits<std::uint64_t>::max();
  s.camera_timestamp_ticks = 123456789;
  s.camera_clock_id = "camera-boot-1"; s.camera_tick_period_ns = 8;
  s.image_path = "images/棋盘图.png";
  s.corners = {{100, 100}, {200, 100}, {100, 200}, {200, 200}};
  s.status = Sample_Status::Accepted;
  a.samples.push_back(s);
  return a;
}
void Test_Transforms()
{
  const auto r_f = robot_model::Build_Zyx_Pose_Matrix({100, 0, 0, 90, 0, 0});
  const auto f_c = robot_model::Build_Zyx_Pose_Matrix({10, 0, 0, 0, 0, 0});
  robot_model::Matrix4 r_c{};
  std::string error;
  Require(Build_Reference_From_Camera(r_f, f_c, &r_c, &error), "Transform composition failed");
  const auto point = robot_model::Transform_Position(r_c, {1, 0, 0});
  Require(std::abs(point[0] - 100) < 1e-9 && std::abs(point[1] - 11) < 1e-9,
    "Transform direction/order or ABC degrees convention is wrong");
  const auto original = robot_model::Transform_Position(robot_model::Invert_Rigid_Matrix(r_c), point);
  Require(std::abs(original[0] - 1) < 1e-9 && std::abs(original[1]) < 1e-9,
    "Rigid inverse round-trip failed");
  auto bad = f_c; bad[0][0] = -1;
  Require(!Validate_Rigid_Transform(bad, &error), "Reflection accepted as rotation");
  bad = f_c; bad[0][0] = 2;
  Require(!Validate_Rigid_Transform(bad, &error), "Scale accepted as rotation");
  bad = f_c; bad[3][0] = 1;
  Require(!Validate_Rigid_Transform(bad, &error), "Invalid homogeneous row accepted");
  bad = f_c; bad[0][3] = std::numeric_limits<double>::quiet_NaN();
  const auto previous = r_c;
  Require(!Build_Reference_From_Camera(r_f, bad, &r_c, &error) && r_c == previous,
    "Invalid composition changed output");
  Require(!Build_Reference_From_Camera(r_f, f_c, nullptr, &error), "Null output accepted");
}
void Test_Validation()
{
  const auto good = Fixture();
  std::string error;
  Require(Validate_Calibration_Archive(good, &error), "Fixture invalid");
  Require(Match_Calibration_Context(good, good.intrinsic->image, good.hand_eye->robot, &error),
    "Matching context rejected");
  auto image = good.intrinsic->image; ++image.roi_x;
  Require(!Match_Calibration_Context(good, image, good.hand_eye->robot, &error),
    "Same dimensions with changed ROI accepted");
  image = good.intrinsic->image; image.camera_serial = "different";
  Require(!Match_Calibration_Context(good, image, good.hand_eye->robot, &error), "Wrong camera accepted");
  image = good.intrinsic->image; image.lens_configuration_id = "refocused";
  Require(!Image_Configurations_Match(good.intrinsic->image, image, &error), "Changed lens accepted");
  auto robot = good.hand_eye->robot; robot.reference_frame_id = "BASE1";
  Require(!Match_Calibration_Context(good, good.intrinsic->image, robot, &error), "Wrong robot frame accepted");
  auto bad = good; bad.hand_eye->intrinsic_id = "another-version";
  Require(!Validate_Calibration_Archive(bad, &error), "Mixed intrinsic versions accepted");
  bad = good; bad.work_plane->robot.reference_frame_id = "BASE1";
  Require(!Validate_Calibration_Archive(bad, &error), "Mixed plane reference accepted");
  bad = good; bad.samples[0].pose_verified = false;
  Require(!Validate_Calibration_Archive(bad, &error), "Unverified accepted pose allowed");
  bad = good; bad.samples.push_back(bad.samples[0]);
  Require(!Validate_Calibration_Archive(bad, &error), "Duplicate sample accepted");
  bad = good; bad.samples[0].corners[0].x = 640;
  Require(!Validate_Calibration_Archive(bad, &error), "Out-of-bounds corner accepted");
  bad = good; bad.samples[0].image_path = "../outside.png";
  Require(!Validate_Calibration_Archive(bad, &error), "Escaping image path accepted");
  bad = good; bad.samples[0].trigger_requested_ns = 400;
  Require(!Validate_Calibration_Archive(bad, &error), "Reversed host timestamp order accepted");
  bad = good; bad.samples[0].camera_clock_id.clear();
  Require(!Validate_Calibration_Archive(bad, &error), "Timestamp without clock identity accepted");
  bad = good; bad.samples[0].status = Sample_Status::Rejected;
  Require(!Validate_Calibration_Archive(bad, &error), "Rejection without reason accepted");
  bad = good; bad.intrinsic->intrinsics.camera_serial_number = "wrong";
  Require(!Validate_Calibration_Archive(bad, &error), "Embedded intrinsic mismatch accepted");
  auto collection = good; collection.hand_eye.reset(); collection.work_plane.reset();
  Require(Validate_Calibration_Archive(collection, &error), "Pre-solve sample collection rejected");
  Require(!Match_Calibration_Context(collection, good.intrinsic->image, robot, &error),
    "Incomplete projection profiles accepted");
}
std::string Read_Text(const std::filesystem::path &path)
{
  std::ifstream stream(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}
void Test_Repository()
{
  Temporary_Directory directory;
  const auto path = directory.path / std::filesystem::path(L"标定归档.xml");
  auto good = Fixture();
  std::string error;
  Require(Save_Calibration_Archive(path, good, &error), "Archive save failed");
  const auto original = Read_Text(path);
  Calibration_Archive loaded;
  Require(Load_Calibration_Archive(path, &loaded, &error), "Archive load failed");
  Require(loaded.hand_eye->flange_from_camera == good.hand_eye->flange_from_camera &&
    loaded.work_plane->reference_from_plane == good.work_plane->reference_from_plane &&
    loaded.intrinsic->intrinsics.distortion == good.intrinsic->intrinsics.distortion &&
    loaded.samples[0].camera_frame_id == good.samples[0].camera_frame_id &&
    loaded.samples[0].image_path == good.samples[0].image_path &&
    loaded.samples[0].camera_timestamp_ticks == good.samples[0].camera_timestamp_ticks &&
    loaded.samples[0].corners.size() == 4, "Archive round-trip lost precision or metadata");
  const auto second = directory.path / "second.xml";
  Require(Save_Calibration_Archive(second, loaded, &error) && Read_Text(second) == original,
    "Round-trip did not retain all fields");
  Require(!Save_Calibration_Archive(path, good, &error) && Read_Text(path) == original,
    "Existing calibration overwritten");
  const auto corrupt = directory.path / "corrupt.xml";
  const auto reject = [&](std::string text)
  {
    std::ofstream out(corrupt, std::ios::binary); out << text; out.close();
    loaded.intrinsic->identity.id = "sentinel";
    Require(!Load_Calibration_Archive(corrupt, &loaded, &error) &&
      loaded.intrinsic->identity.id == "sentinel" && !error.empty(),
      "Malformed archive accepted or output changed on failure");
  };
  const auto mutate = [&](const std::string &from, const std::string &to)
  {
    auto text = original;
    const auto position = text.find(from);
    Require(position != std::string::npos, "Mutation fixture token missing");
    text.replace(position, from.size(), to);
    reject(text);
  };
  mutate("lengthUnit=\"mm\"", "lengthUnit=\"m\"");
  mutate("version=\"1\"", "version=\"99\"");
  mutate("poseVerified=\"true\"", "poseVerified=\"1\"");
  mutate("roiX=\"8\"", "roiX=\"8junk\"");
  mutate("roiX=\"8\"", "roiX=\"999999999999999999999999\"");
  mutate("fitRmsMm=\"0.12\"", "fitRmsMm=\"0x1p2\"");
  mutate("fitRmsMm=\"0.12\"", "fitRmsMm=\"nan\"");
  mutate("fitRmsMm=\"0.12\"", "fitRmsMm=\"1e9999\"");
  mutate("source=\"fixed-3d\"", "source=\"unknown\"");
  mutate("<HandEye ", "<HandEye unknown=\"1\" ");
  mutate("</CalibrationArchive>", "<Unknown /></CalibrationArchive>");
  mutate("</CalibrationArchive>", "<HandEye /></CalibrationArchive>");
  mutate("robotPose=\"reference-from-flange\"", "robotPose=\"reference-from-tcp\"");
  mutate("roiX=\"8\"", "");
  reject(original + "<Extra />");
  reject(original.substr(0, original.size() / 2));
  reject(std::string("\0", 1) + original);
  Require(!Load_Calibration_Archive(path, nullptr, &error), "Null archive output accepted");
  good.samples[0].camera_timestamp_ticks.reset();
  good.samples[0].camera_clock_id.clear(); good.samples[0].camera_tick_period_ns = 0;
  good.hand_eye.reset(); good.work_plane.reset();
  const auto collection = directory.path / "collection.xml";
  Require(Save_Calibration_Archive(collection, good, &error) &&
    Load_Calibration_Archive(collection, &loaded, &error) &&
    !loaded.hand_eye && !loaded.work_plane && !loaded.samples[0].camera_timestamp_ticks,
    "Optional profiles or unknown camera timestamp did not round-trip");
}
}
int main()
{
  try { Test_Transforms(); Test_Validation(); Test_Repository(); }
  catch (const std::exception &error)
  {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }
  std::cout << "Calibration geometry and archive tests passed.\n";
  return 0;
}

#include "robot_joint_state_builder.h"
#include "robot_part_transform.h"
#include "eye_to_hand_calibration.h"
#include "pose_point_io.h"
#include "pose_transform.h"
#include "robot_forward_kinematics.h"
#include "robot_inverse_kinematics.h"
#include "robot_trajectory_io.h"
#include "robot_trajectory_planner.h"
#include "robot_trajectory_session.h"

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

void require (bool condition, const std::string& message)
{
  if( !condition )
  {
    throw std::runtime_error (message);
  }
}

void require_near (double actual, double expected, const std::string& message)
{
  require (std::abs (actual - expected) < 1.0e-9, message);
}

void require_point_near (
  const std::array<double, 6>& actual,
  const std::array<double, 6>& expected,
  const std::string& message)
{
  for( size_t i = 0; i < actual.size ( ); ++i )
  {
    if( std::abs (actual[i] - expected[i]) >= 1.0e-9 )
    {
      throw std::runtime_error (message);
    }
  }
}

void test_joint_state_builder ( )
{
  robot_model::Robot_Kinematic_Params params;
  params.joint_offsets = { 10.0, -20.0, 30.0 };
  params.joint_directions = { 1, -1, 0 };

  const std::array<double, 6> input = { 5.0, 10.0, 1.0, 2.0, 3.0, 4.0 };
  const auto state = robot_model::Build_Joint_State_From_Input_Angles (
    params, input);

  require_near (state.input_angles_deg[0], 5.0, "A1 input mismatch");
  require_near (state.effective_angles_deg[0], 15.0, "A1 effective mismatch");
  require_near (state.delta_angles_deg[0], 15.0, "A1 delta mismatch");

  require_near (state.effective_angles_deg[1], -30.0, "A2 effective mismatch");
  require_near (state.delta_angles_deg[1], -30.0, "A2 delta mismatch");

  require_near (state.effective_angles_deg[2], 31.0, "A3 effective mismatch");
  require_near (state.delta_angles_deg[2], 31.0, "A3 delta mismatch");
}

void test_trajectory_planner ( )
{
  const std::array<double, 6> start = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  const std::array<double, 6> target = { 10.0, 20.0, 30.0, 40.0, 50.0, 60.0 };

  const auto frames = robot_model::Build_Joint_Ptp_Trajectory (
    start, target, 3);
  require (frames.size ( ) == 3, "PTP frame count mismatch");
  require_point_near (frames[0], start, "PTP start frame mismatch");
  require_point_near (
    frames[1],
    { 5.0, 10.0, 15.0, 20.0, 25.0, 30.0 },
    "PTP midpoint mismatch");
  require_point_near (frames[2], target, "PTP target frame mismatch");

  const robot_model::Robot_Joint_Trajectory points =
  {
    start,
    target,
    { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 }
  };
  const auto multi_frames =
    robot_model::Build_Multi_Point_Joint_Ptp_Trajectory (points, 3);
  require (multi_frames.size ( ) == 5, "Multi-point frame count mismatch");
  require_point_near (multi_frames.front ( ), points.front ( ),
                      "Multi-point start mismatch");
  require_point_near (multi_frames.back ( ), points.back ( ),
                      "Multi-point end mismatch");
}

void test_trajectory_session ( )
{
  robot_model::Robot_Trajectory_Session session;
  const std::array<double, 6> p0 = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  const std::array<double, 6> p1 = { 10.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  const std::array<double, 6> p2 = { 10.0, 10.0, 0.0, 0.0, 0.0, 0.0 };

  session.Add_Point (p0);
  session.Add_Point (p1);
  session.Add_Point (p2);
  require (session.Point_Count ( ) == 3, "Session point count mismatch");
  require (session.Has_Playable_Path ( ), "Session should be playable");
  require (session.Start_Playback (3), "Session playback should start");

  size_t stepped_frames = 0;
  while( session.Step ( ) != nullptr )
  {
    ++stepped_frames;
  }
  require (stepped_frames == 5, "Session playback frame count mismatch");
  require (session.Is_Finished ( ), "Session should be finished");

  require (session.Start_Go_To (p0, 1, 3), "Session Go To should start");
  const auto* first = session.Step ( );
  require (first != nullptr, "Session Go To first frame missing");
  require_point_near (*first, p0, "Session Go To start frame mismatch");
}

void test_trajectory_csv_io ( )
{
  const auto path = std::filesystem::temp_directory_path ( ) /
    "test_robot_trajectory_io_test.csv";
  const std::vector<std::array<double, 6>> points =
  {
    { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 },
    { -1.5, 0.0, 2.5, 3.5, 4.5, 5.5 }
  };

  std::string error;
  require (robot_model::Save_Joint_Trajectory_Csv (path, points, &error),
           "CSV save failed: " + error);

  std::vector<std::array<double, 6>> loaded_points;
  require (robot_model::Load_Joint_Trajectory_Csv (
             path, &loaded_points, &error),
           "CSV load failed: " + error);
  require (loaded_points.size ( ) == points.size ( ),
           "CSV loaded point count mismatch");
  require_point_near (loaded_points[0], points[0], "CSV first point mismatch");
  require_point_near (loaded_points[1], points[1], "CSV second point mismatch");

  {
    std::ofstream invalid_file (path);
    invalid_file << "A1,A2,A3,A4,A5,A6\n1,2,3\n";
  }
  require (!robot_model::Load_Joint_Trajectory_Csv (
             path, &loaded_points, &error),
           "Invalid CSV should fail");

  std::filesystem::remove (path);
}

void test_pose_transform_logic ( )
{
  robot_model::XyzabcPose parsed_pose = { };
  std::string error;
  require (robot_model::Parse_Xyzabc_Pose_Text (
             "pose: 10, 20, 30, 90, 0, 0", &parsed_pose, &error),
           "Pose text parse failed: " + error);
  require_point_near (
    parsed_pose,
    { 10.0, 20.0, 30.0, 90.0, 0.0, 0.0 },
    "Parsed pose mismatch");
  require (
    robot_model::Format_Xyzabc_Pose_Csv(parsed_pose) ==
      "10.000000,20.000000,30.000000,90.000000,0.000000,0.000000",
    "Pose CSV format mismatch");

  const robot_model::XyzabcPose identity =
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  const robot_model::XyzabcPose target =
    { 10.0, 20.0, 30.0, 0.0, 0.0, 0.0 };
  const auto translate_transform =
    robot_model::Build_Relative_Pose_Transform (identity, target);
  const auto translated = robot_model::Transform_Xyzabc_Pose (
    translate_transform,
    { 1.0, 2.0, 3.0, 0.0, 0.0, 0.0 });
  require_point_near (
    translated,
    { 11.0, 22.0, 33.0, 0.0, 0.0, 0.0 },
    "Translated pose mismatch");

  const auto rotate_transform = robot_model::Build_Relative_Pose_Transform (
    identity,
    { 0.0, 0.0, 0.0, 90.0, 0.0, 0.0 });
  const auto rotated = robot_model::Transform_Xyzabc_Pose (
    rotate_transform,
    { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
  require_near (rotated[0], 0.0, "Rotated pose X mismatch");
  require_near (rotated[1], 1.0, "Rotated pose Y mismatch");
  require_near (rotated[2], 0.0, "Rotated pose Z mismatch");
  require_near (rotated[3], 90.0, "Rotated pose A mismatch");
  require_near (rotated[4], 0.0, "Rotated pose B mismatch");
  require_near (rotated[5], 0.0, "Rotated pose C mismatch");
}

void test_pose_point_file_io ( )
{
  const auto path = std::filesystem::temp_directory_path ( ) /
    "test_pose_point_file_io.txt";
  {
    std::ofstream file (path);
    file << "# default pose + transform points\n";
    file << "x,y,z,a,b,c\n";
    file << "1,2,3,4,5,6\n";
    file << "7,8,9,10,11,12\n";
    file << "13 14 15 16 17 18\n";
  }

  robot_model::Pose_Point_File point_file;
  std::string error;
  require (robot_model::Load_Pose_Point_File (path, &point_file, &error),
           "Point file load failed: " + error);
  require_point_near (
    point_file.default_pose,
    { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 },
    "Default point mismatch");
  require (point_file.transform_points.size ( ) == 2,
           "Transform point count mismatch");
  require_point_near (
    point_file.transform_points[0],
    { 7.0, 8.0, 9.0, 10.0, 11.0, 12.0 },
    "First transform point mismatch");
  require_point_near (
    point_file.transform_points[1],
    { 13.0, 14.0, 15.0, 16.0, 17.0, 18.0 },
    "Second transform point mismatch");

  std::filesystem::remove (path);
}

void test_eye_to_hand_calibration ( )
{
  const auto path = std::filesystem::temp_directory_path ( ) /
    "test_eye_to_hand_calibration.txt";
  {
    std::ofstream file (path);
    file << "RotMat_cam2robot = [\n";
    file << "  0.0029242793573063426  0.92028524175874993 "
            "-0.39123714632032508\n";
    file << "  0.99998141562059728 -0.00059780162156294328 "
            "0.0060681302921512387\n";
    file << "  0.0053505264238772193 -0.39124761801406271 "
            "-0.92026988215636063 ]\n";
    file << "TranMat_cam2robot = [1225.5965333023571 "
            "198.01430682455043 1177.1407695305254]\n";
  }

  robot_model::Eye_To_Hand_Calibration calibration;
  std::string error;
  require (robot_model::Load_Eye_To_Hand_Calibration (
             path, &calibration, &error),
           "Eye-to-hand calibration load failed: " + error);
  const auto matrix = calibration.World_From_Camera ( );
  require_near (matrix[0][0], 0.0029242793573063426,
                "Eye-to-hand R00 mismatch");
  require_near (matrix[0][1], 0.92028524175874993,
                "Eye-to-hand R01 mismatch");
  require_near (matrix[1][2], 0.0060681302921512387,
                "Eye-to-hand R12 mismatch");
  require_near (matrix[2][2], -0.92026988215636063,
                "Eye-to-hand R22 mismatch");
  require_near (matrix[0][3], 1225.5965333023571,
                "Eye-to-hand matrix tx mismatch");
  require_near (matrix[2][3], 1177.1407695305254,
                "Eye-to-hand matrix tz mismatch");

  const auto camera_origin = robot_model::Transform_Position (
    matrix, { 0.0, 0.0, 0.0 });
  require_near (camera_origin[0], 1225.5965333023571,
                "Transformed camera origin X mismatch");
  require_near (camera_origin[1], 198.01430682455043,
                "Transformed camera origin Y mismatch");
  require_near (camera_origin[2], 1177.1407695305254,
                "Transformed camera origin Z mismatch");

  const auto camera_x_axis_end = robot_model::Transform_Position (
    matrix, { 300.0, 0.0, 0.0 });
  require_near (camera_x_axis_end[0],
                1225.5965333023571 + matrix[0][0] * 300.0,
                "Camera X axis endpoint X mismatch");
  require_near (camera_x_axis_end[1],
                198.01430682455043 + matrix[1][0] * 300.0,
                "Camera X axis endpoint Y mismatch");
  require_near (camera_x_axis_end[2],
                1177.1407695305254 + matrix[2][0] * 300.0,
                "Camera X axis endpoint Z mismatch");

  const auto scaled_matrix = calibration.World_From_Camera (0.001);
  require_near (scaled_matrix[0][0], matrix[0][0] * 0.001,
                "Scaled eye-to-hand R00 mismatch");
  require_near (scaled_matrix[1][2], matrix[1][2] * 0.001,
                "Scaled eye-to-hand R12 mismatch");
  require_near (scaled_matrix[0][3], 1225.5965333023571,
                "Scaled eye-to-hand translation X changed");
  require_near (scaled_matrix[2][3], 1177.1407695305254,
                "Scaled eye-to-hand translation Z changed");

  const auto scaled_x_point = robot_model::Transform_Position (
    scaled_matrix, { 1000.0, 0.0, 0.0 });
  const auto one_mm_x_point = robot_model::Transform_Position (
    matrix, { 1.0, 0.0, 0.0 });
  require_near (scaled_x_point[0], one_mm_x_point[0],
                "1000 raw units did not convert to 1 mm on X");
  require_near (scaled_x_point[1], one_mm_x_point[1],
                "Scaled X rotation contribution mismatch on Y");
  require_near (scaled_x_point[2], one_mm_x_point[2],
                "Scaled X rotation contribution mismatch on Z");

  std::filesystem::remove (path);
}

robot_model::Matrix4 identity_matrix ( )
{
  robot_model::Matrix4 matrix = { };
  for( std::size_t index = 0; index < 4; ++index )
  {
    matrix[index][index] = 1.0;
  }
  return matrix;
}

void test_forward_kinematics_flange_pose ( )
{
  robot_model::Robot_Forward_Kinematics_Model model;
  model.joint_axes = {
    robot_model::Point3 { 0.0, 0.0, 1.0 },
    robot_model::Point3 { 1.0, 0.0, 0.0 },
    robot_model::Point3 { 0.0, 1.0, 0.0 },
    robot_model::Point3 { 1.0, 0.0, 0.0 },
    robot_model::Point3 { 0.0, 1.0, 0.0 },
    robot_model::Point3 { 1.0, 0.0, 0.0 }
  };
  model.neutral_world_from_parts.assign (7, identity_matrix ( ));
  model.neutral_world_from_parts.back ( )[0][3] = 100.0;
  model.flange_from_last_part = identity_matrix ( );
  model.has_flange = true;

  robot_model::Robot_Joint_State zero_state;
  const auto zero_result = robot_model::Compute_Forward_Kinematics (
    model, zero_state);
  require (zero_result.has_flange, "Zero-state flange pose missing");
  require_near (zero_result.world_from_flange[0][3], 100.0,
                "Zero-state flange X mismatch");
  require_near (zero_result.world_from_flange[1][3], 0.0,
                "Zero-state flange Y mismatch");

  robot_model::Robot_Joint_State rotated_state;
  rotated_state.delta_angles_deg[0] = 90.0;
  const auto rotated_result = robot_model::Compute_Forward_Kinematics (
    model, rotated_state);
  require_near (rotated_result.world_from_flange[0][3], 0.0,
                "Rotated flange X mismatch");
  require_near (rotated_result.world_from_flange[1][3], 100.0,
                "Rotated flange Y mismatch");
  require_near (rotated_result.joint_axes_world[1][0], 0.0,
                "Child joint world axis X mismatch");
  require_near (rotated_result.joint_axes_world[1][1], 1.0,
                "Child joint world axis Y mismatch");
}

void test_part_calibration_uses_pure_matrix_math ( )
{
  robot_model::Robot_Part_Calibration calibration;
  calibration.rotate_xyz[0] = 90.0;
  calibration.translate[1] = 10.0;

  const auto matrix = robot_model::Build_Part_Calibration_Matrix (calibration);
  const auto origin = robot_model::Transform_Position (
    matrix, { 0.0, 0.0, 0.0 });
  require_near (origin[0], 0.0, "Part calibration X mismatch");
  require_near (origin[1], 0.0, "Part calibration Y mismatch");
  require_near (origin[2], 10.0, "Part calibration rotation order mismatch");

  const auto transformed = robot_model::Transform_Part_Point (
    calibration, { 1.0, 2.0, 3.0 });
  require_near (transformed[0], 1.0, "Part point X mismatch");
  require_near (transformed[1], -3.0, "Part point Y mismatch");
  require_near (transformed[2], 12.0, "Part point Z mismatch");
}

robot_model::Robot_Forward_Kinematics_Model build_planar_ik_test_model ( )
{
  robot_model::Robot_Forward_Kinematics_Model model;
  model.joint_axes.fill ({ 0.0, 0.0, 1.0 });
  model.neutral_world_from_parts.assign (7, identity_matrix ( ));
  model.neutral_world_from_parts.back ( )[0][3] = 100.0;
  model.flange_from_last_part = identity_matrix ( );
  model.has_flange = true;
  return model;
}

robot_model::Robot_Kinematic_Params build_planar_ik_test_params ( )
{
  robot_model::Robot_Kinematic_Params params;
  params.joint_directions.assign (6, 1);
  params.joint_offsets.assign (6, 0.0);
  params.joint_mins = { -180.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  params.joint_maxs = { 180.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  return params;
}

void test_position_ik_converges ( )
{
  const auto model = build_planar_ik_test_model ( );
  const auto params = build_planar_ik_test_params ( );
  robot_model::Robot_Joint_State initial_state;
  robot_model::Robot_Position_IK_Options options;
  options.position_tolerance_mm = 1.0e-6;
  options.damping_mm = 1.0;
  options.max_joint_step_deg = 10.0;
  options.max_iterations = 40;
  options.time_budget_ms = 0.0;
  const double coordinate = 100.0 / std::sqrt (2.0);

  const auto result = robot_model::Solve_Flange_Position_IK (
    model, params, initial_state, { coordinate, coordinate, 0.0 }, options);
  require (result.Converged ( ), "Reachable position IK did not converge");
  require (std::abs (result.joint_state.input_angles_deg[0] - 45.0) < 1.0e-5,
           "Position IK joint angle mismatch");
  require (result.position_error_mm <= options.position_tolerance_mm,
           "Position IK final error exceeds tolerance");
}

void test_position_ik_respects_joint_limits ( )
{
  const auto model = build_planar_ik_test_model ( );
  auto params = build_planar_ik_test_params ( );
  params.joint_mins[0] = -30.0;
  params.joint_maxs[0] = 30.0;
  robot_model::Robot_Joint_State initial_state;
  robot_model::Robot_Position_IK_Options options;
  options.damping_mm = 1.0;
  options.max_joint_step_deg = 10.0;
  options.max_iterations = 20;
  options.time_budget_ms = 0.0;

  const auto result = robot_model::Solve_Flange_Position_IK (
    model, params, initial_state, { 0.0, 100.0, 0.0 }, options);
  require (!result.Converged ( ),
           "Joint-limited unreachable IK unexpectedly converged");
  require_near (result.joint_state.input_angles_deg[0], 30.0,
                "Position IK did not stop at the upper joint limit");
}

void test_pose_ik_converges_orientation ( )
{
  auto model = build_planar_ik_test_model ( );
  model.joint_pivots[5] = { 100.0, 0.0, 0.0 };
  auto params = build_planar_ik_test_params ( );
  params.joint_mins = { 0.0, 0.0, 0.0, 0.0, 0.0, -180.0 };
  params.joint_maxs = { 0.0, 0.0, 0.0, 0.0, 0.0, 180.0 };
  robot_model::Matrix4 target = identity_matrix ( );
  const double radians = 45.0 * 3.14159265358979323846 / 180.0;
  target[0][0] = std::cos (radians);
  target[0][1] = -std::sin (radians);
  target[1][0] = std::sin (radians);
  target[1][1] = std::cos (radians);
  target[0][3] = 100.0;

  robot_model::Robot_Joint_State initial_state;
  robot_model::Robot_Pose_IK_Options options;
  options.position_tolerance_mm = 1.0e-6;
  options.orientation_tolerance_deg = 1.0e-6;
  options.orientation_weight_mm = 100.0;
  options.damping_mm = 1.0;
  options.max_joint_step_deg = 10.0;
  options.max_iterations = 40;
  options.time_budget_ms = 0.0;
  const auto result = robot_model::Solve_Flange_Pose_IK (
    model, params, initial_state, target, options);

  require (result.Converged ( ), "Reachable pose IK did not converge");
  require (std::abs (result.joint_state.input_angles_deg[5] - 45.0) < 1.0e-5,
           "Pose IK wrist angle mismatch");
  require (result.position_error_mm <= options.position_tolerance_mm,
           "Pose IK changed the fixed flange position");
  require (result.orientation_error_deg <= options.orientation_tolerance_deg,
           "Pose IK orientation error exceeds tolerance");
}

void test_ik_realtime_budget ( )
{
  const auto model = build_planar_ik_test_model ( );
  const auto params = build_planar_ik_test_params ( );
  robot_model::Robot_Joint_State initial_state;

  robot_model::Robot_Position_IK_Options position_options;
  position_options.max_iterations = 100000;
  position_options.time_budget_ms = 1.0e-9;
  const auto position_result = robot_model::Solve_Flange_Position_IK (
    model, params, initial_state, { 1000.0, 1000.0, 0.0 }, position_options);
  require (
    position_result.status == robot_model::Robot_IK_Status::Time_Budget_Exceeded,
    "Position IK did not stop at its realtime budget");
  require (std::isfinite (position_result.position_error_mm),
           "Budgeted position IK did not preserve its best result");

  auto target_pose = identity_matrix ( );
  target_pose[0][3] = 1000.0;
  robot_model::Robot_Pose_IK_Options pose_options;
  pose_options.max_iterations = 100000;
  pose_options.time_budget_ms = 1.0e-9;
  const auto pose_result = robot_model::Solve_Flange_Pose_IK (
    model, params, initial_state, target_pose, pose_options);
  require (
    pose_result.status == robot_model::Robot_IK_Status::Time_Budget_Exceeded,
    "Pose IK did not stop at its realtime budget");
  require (std::isfinite (pose_result.position_error_mm),
           "Budgeted pose IK did not preserve its best result");
}

} // namespace

int main ( )
{
  try
  {
    test_joint_state_builder ( );
    test_trajectory_planner ( );
    test_trajectory_session ( );
    test_trajectory_csv_io ( );
    test_pose_transform_logic ( );
    test_pose_point_file_io ( );
    test_eye_to_hand_calibration ( );
    test_forward_kinematics_flange_pose ( );
    test_part_calibration_uses_pure_matrix_math ( );
    test_position_ik_converges ( );
    test_position_ik_respects_joint_limits ( );
    test_pose_ik_converges_orientation ( );
    test_ik_realtime_budget ( );
  }
  catch( const std::exception& e )
  {
    std::cerr << "FAILED: " << e.what ( ) << '\n';
    return 1;
  }

  std::cout << "All robot model logic tests passed.\n";
  return 0;
}

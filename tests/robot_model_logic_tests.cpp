#include "robot_joint_state_builder.h"
#include "robot_part_transform.h"
#include "eye_to_hand_calibration.h"
#include "pose_point_io.h"
#include "pose_transform.h"
#include "robot_forward_kinematics.h"
#include "robot_inverse_kinematics.h"
#include "robot_model_config_loader.h"
#include "robot_model_repository.h"
#include "robot_trajectory_io.h"
#include "robot_trajectory_planner.h"
#include "robot_trajectory_session.h"
#include "robot_teach_point_store.h"

#include <algorithm>
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

void test_teach_point_store ( )
{
  robot_model::Robot_Teach_Point_Store store;
  const std::array<double, 6> joints_a = { 1, 2, 3, 4, 5, 6 };
  const robot_model::XyzabcPose pose_a = { 10, 20, 30, 40, 50, 60 };
  const auto first = store.Add_Point ("robot-a", joints_a, pose_a);
  require (first.id == 1, "First teach point id mismatch");
  require (robot_model::Format_Teach_Point_Name (first.id) == "P[1]",
           "Teach point name format mismatch");
  require (robot_model::Format_Teach_Point_Name (101) == "P[101]",
           "Three-digit teach point name format mismatch");
  require (first.joint_angles_deg == joints_a,
           "Teach point joint state mismatch");
  require (first.world_pose == pose_a && first.has_world_pose,
           "Teach point world pose mismatch");

  store.Add_Point ("robot-a", joints_a, pose_a);
  require (store.Delete_Point ("robot-a", 0),
           "Teach point deletion failed");
  const auto third = store.Add_Point ("robot-a", joints_a, pose_a);
  require (third.id == 3,
           "Deleted teach point id was unexpectedly reused");
  store.Clear_Points ("robot-a");
  const auto fourth = store.Add_Point ("robot-a", joints_a, pose_a);
  require (fourth.id == 4,
           "Cleared teach point id was unexpectedly reused");

  store.Add_Point ("robot-b", joints_a, pose_a);
  require (store.Point_Count ("robot-a") == 1,
           "Robot A teach point count mismatch");
  require (store.Point_Count ("robot-b") == 1,
           "Teach points were not isolated by robot model");
}

void test_robot_resources_use_source_directory ( )
{
  const auto actual_root = std::filesystem::weakly_canonical (
    robot_model::Find_Robot_Root ( ));
  const auto expected_root = std::filesystem::weakly_canonical (
    std::filesystem::path (EXPECTED_ROBOT_RESOURCE_ROOT));

  require (!actual_root.empty ( ), "Robot resource root was not found");
  require (actual_root == expected_root,
           "Robot resources are not loaded from source Resource/Robot");
  require (std::filesystem::is_regular_file (actual_root / "point.txt"),
           "Robot point.txt was not found in source Resource/Robot");
  require (std::filesystem::is_regular_file (
             actual_root / "RT_eyetohand.txt"),
           "Robot calibration was not found in source Resource/Robot");

  const auto models = robot_model::Scan_Models_In_Directory (actual_root);
  require (!models.empty ( ), "No robot models were found in source resources");
  for( const auto& model : models )
  {
    require (std::filesystem::weakly_canonical (model.model_dir.parent_path ( )) ==
               actual_root,
             "A robot model was loaded from outside source Resource/Robot");
  }
}

void test_kr10_r1100_2_resource_config ( )
{
  const auto root = robot_model::Find_Robot_Root ( );
  const auto models = robot_model::Scan_Models_In_Directory (root);
  const auto model = std::find_if (
    models.begin ( ), models.end ( ),
    [] (const robot_model::Robot_Model_Info& info)
    {
      return info.model_dir.filename ( ) == "KR10_R1100_2";
    });

  require (model != models.end ( ), "KR10_R1100_2 model was not found");
  require (model->stl_files.size ( ) == 7,
           "KR10_R1100_2 must load all seven STL files");
  const std::array<std::string, 7> expected_stl_order = {
    "0.STL", "5.STL", "1.STL", "3.STL", "4.STL", "2.STL", "6.STL"
  };
  for( size_t index = 0; index < expected_stl_order.size ( ); ++index )
  {
    require (model->stl_files[index].filename ( ) == expected_stl_order[index],
             "KR10_R1100_2 STL hierarchy order mismatch");
  }
  const auto params = robot_model::Load_Robot_Kinematic_Params (
    model->xml_path, model->display_name);
  require (params.model_name == "KR10_R1100_2",
           "KR10_R1100_2 model name mismatch");
  require (params.link_lengths.size ( ) == 6,
           "KR10_R1100_2 link lengths were not parsed");
  require_near (params.link_lengths[0], 400.0,
                "KR10_R1100_2 base height mismatch");
  require_near (params.link_lengths[1], 560.0,
                "KR10_R1100_2 upper-arm length mismatch");
  require_near (params.joint_mins[1], -190.0,
                "KR10_R1100_2 A2 lower limit mismatch");
  require_near (params.joint_maxs[2], 156.0,
                "KR10_R1100_2 A3 upper limit mismatch");
  require (params.joint_frames[0].has_pivot &&
             params.joint_frames[0].has_axis,
           "KR10_R1100_2 A1 frame was not parsed");
  require_near (params.joint_frames[0].pivot[0], 0.0,
                "KR10_R1100_2 A1 pivot X mismatch");
  require_near (params.joint_frames[0].pivot[1], 0.0,
                "KR10_R1100_2 A1 pivot Y mismatch");
  require_near (params.joint_frames[0].pivot[2], 219.70,
                "KR10_R1100_2 A1 pivot Z mismatch");
  require_near (params.joint_frames[5].pivot[0], 611.89,
                "KR10_R1100_2 A6 pivot mismatch");
  require (params.has_neutral_flange_pose,
           "KR10_R1100_2 neutral flange pose was not parsed");
  require_near (params.neutral_flange_pose[0], 629.89,
                "KR10_R1100_2 flange X mismatch");
  require_near (params.neutral_flange_pose[1], 0.0,
                "KR10_R1100_2 flange Y mismatch");
  require_near (params.neutral_flange_pose[2], 985.0,
                "KR10_R1100_2 flange Z mismatch");

  const std::array<std::array<double, 3>, 7> expected_part_offsets = { {
    { -383.44, -109.50, -2.83 },
    { -113.09, -139.63, 0.0 },
    { -83.15, -131.53, 0.0 },
    { -62.38, -75.50, 0.0 },
    { -0.11, -78.60, 0.0 },
    { -0.11, -54.50, 0.0 },
    { -0.11, -41.61, 0.0 }
  } };
  robot_model::Robot_Assembly_Calibration calibration;
  for( size_t part = 0; part < expected_part_offsets.size ( ); ++part )
  {
    for( size_t axis = 0; axis < 3; ++axis )
    {
      require_near (params.manual_part_calibration[part].translate[axis],
                    expected_part_offsets[part][axis],
                    "KR10_R1100_2 part assembly offset mismatch");
      calibration.parts[part].translate[axis] =
        params.manual_part_calibration[part].translate[axis];
    }
  }

  // Adjacent circular interfaces must share the same center line after the
  // per-STL assembly translations are applied.
  const auto assembled_center = [&calibration] (
    size_t part, const std::array<double, 3>& local)
  {
    std::array<double, 3> result = local;
    for( size_t axis = 0; axis < 3; ++axis )
    {
      result[axis] += calibration.parts[part].translate[axis];
    }
    return result;
  };
  const auto a1_parent = assembled_center (0, { 383.44, 109.50, 222.53 });
  const auto a1_child = assembled_center (1, { 113.09, 139.63, 219.70 });
  require_near (a1_parent[0], a1_child[0],
                "KR10_R1100_2 A1 mesh centers do not align in X");
  require_near (a1_parent[1], a1_child[1],
                "KR10_R1100_2 A1 mesh centers do not align in Y");
  require_near (a1_parent[2], a1_child[2],
                "KR10_R1100_2 A1 mesh centers do not align in Z");
  const auto a2_parent = assembled_center (1, { 138.00, 109.50, 400.00 });
  const auto a2_child = assembled_center (2, { 108.06, 109.50, 400.00 });
  require_near (a2_parent[0], a2_child[0],
                "KR10_R1100_2 A2 mesh centers do not align");
  require_near (a2_parent[2], a2_child[2],
                "KR10_R1100_2 A2 mesh heights do not align");
  const auto a3_parent = assembled_center (2, { 108.06, 109.50, 960.00 });
  const auto a3_child = assembled_center (3, { 87.29, 109.50, 960.00 });
  require_near (a3_parent[0], a3_child[0],
                "KR10_R1100_2 A3 mesh centers do not align");
  const auto a4_parent = assembled_center (3, { 109.50, 75.50, 985.00 });
  const auto a4_child = assembled_center (4, { 109.50, 78.60, 985.00 });
  require_near (a4_parent[1], a4_child[1],
                "KR10_R1100_2 A4 mesh centers do not align");
  const auto a5_parent = assembled_center (4, { 540.00, 109.50, 985.00 });
  const auto a5_child = assembled_center (5, { 540.00, 109.50, 985.00 });
  require_near (a5_parent[0], a5_child[0],
                "KR10_R1100_2 A5 mesh centers do not align in X");
  require_near (a5_parent[2], a5_child[2],
                "KR10_R1100_2 A5 mesh centers do not align in Z");
  const auto a6_parent = assembled_center (5, { 607.00, 54.50, 985.00 });
  const auto a6_child = assembled_center (6, { 607.00, 41.61, 985.00 });
  require_near (a6_parent[1], a6_child[1],
                "KR10_R1100_2 A6 mesh centers do not align");

  const auto forward_model = robot_model::Build_Forward_Kinematics_Model (
    7, calibration, params);
  const std::array<double, 6> neutral_input = {
    0.0, -90.0, 90.0, 0.0, 0.0, 0.0
  };
  for( size_t index = 0; index < neutral_input.size ( ); ++index )
  {
    require_near (
      robot_model::Neutral_Joint_Input_At (params, index),
      neutral_input[index],
      "KR10_R1100_2 reset input does not match its loaded neutral pose");
  }
  const auto neutral_state =
    robot_model::Build_Joint_State_From_Input_Angles (
      params, neutral_input);
  const auto neutral_forward = robot_model::Compute_Forward_Kinematics (
    forward_model, neutral_state);
  require (neutral_forward.has_flange,
           "KR10_R1100_2 flange transform was not built");
  require_near (neutral_forward.world_from_flange[0][3], 629.89,
                "KR10_R1100_2 neutral flange world X mismatch");
  require_near (neutral_forward.world_from_flange[1][3], 0.0,
                "KR10_R1100_2 neutral flange world Y mismatch");
  require_near (neutral_forward.world_from_flange[2][3], 985.0,
                "KR10_R1100_2 neutral flange world Z mismatch");

  auto rotated_input = neutral_input;
  rotated_input[0] = 90.0;
  const auto rotated_state =
    robot_model::Build_Joint_State_From_Input_Angles (
      params, rotated_input);
  const auto rotated_forward = robot_model::Compute_Forward_Kinematics (
    forward_model, rotated_state);
  require_near (rotated_forward.world_from_flange[0][3], 0.0,
                "KR10_R1100_2 flange did not follow A1 in X");
  require_near (rotated_forward.world_from_flange[1][3], -629.89,
                "KR10_R1100_2 flange did not follow A1 in Y");
  require_near (rotated_forward.world_from_flange[2][3], 985.0,
                "KR10_R1100_2 flange did not preserve Z under A1");
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

  const robot_model::XyzabcPose cartesian_target =
    { 520.0, -135.0, 880.0, 35.0, -20.0, 70.0 };
  const auto cartesian_matrix = robot_model::Build_Zyx_Pose_Matrix (
    cartesian_target);
  const auto cartesian_round_trip =
    robot_model::Build_Xyzabc_From_Zyx_Matrix (cartesian_matrix);
  require_point_near (
    cartesian_round_trip,
    cartesian_target,
    "Cartesian XYZABC target round-trip mismatch");
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
    file << "CameraCoordinateType = [1]\n";
    file << "PointCloudUnitToMm = [1.0]\n";
  }

  robot_model::Eye_To_Hand_Calibration calibration;
  std::string error;
  require (robot_model::Load_Eye_To_Hand_Calibration (
             path, &calibration, &error),
           "Eye-to-hand calibration load failed: " + error);
  require (calibration.camera_coordinate_type == 1,
           "Eye-to-hand camera coordinate type mismatch");
  require_near (calibration.point_cloud_unit_to_mm, 1.0,
                "Eye-to-hand point-cloud unit mismatch");
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

  const auto one_meter_camera_z = robot_model::Transform_Position (
    matrix, { 0.0, 0.0, 1000.0 });
  require_near (one_meter_camera_z[0],
                1225.5965333023571 + matrix[0][2] * 1000.0,
                "1000 mm camera Z was incorrectly rescaled in X");
  require_near (one_meter_camera_z[1],
                198.01430682455043 + matrix[1][2] * 1000.0,
                "1000 mm camera Z was incorrectly rescaled in Y");
  require_near (one_meter_camera_z[2],
                1177.1407695305254 + matrix[2][2] * 1000.0,
                "1000 mm camera Z was incorrectly rescaled in Z");

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

  const auto micro_target = robot_model::Build_Zyx_Pose_Matrix (
    { 100.0, 0.0, 0.0, 0.01, 0.0, 0.0 });
  const auto micro_result = robot_model::Solve_Flange_Pose_IK (
    model, params, initial_state, micro_target, options);
  require (micro_result.Converged ( ),
           "Pose IK did not converge for a 0.01 degree fine adjustment");
  require (
    std::abs (micro_result.joint_state.input_angles_deg[5] - 0.01) < 1.0e-5,
    "Pose IK lost the 0.01 degree fine adjustment");
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
    test_robot_resources_use_source_directory ( );
    test_kr10_r1100_2_resource_config ( );
    test_joint_state_builder ( );
    test_trajectory_planner ( );
    test_trajectory_session ( );
    test_teach_point_store ( );
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

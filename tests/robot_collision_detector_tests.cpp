#include "robot_collision_detector.h"
#include "robot_calibration_builder.h"
#include "robot_forward_kinematics.h"
#include "robot_joint_state_builder.h"
#include "robot_mesh_loader.h"
#include "robot_model_config_loader.h"
#include "robot_model_repository.h"
#include "robot_render_controller.h"

#include <vtkCubeSource.h>

#include <cmath>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

void require (bool condition, const std::string& message)
{
  if( !condition ) throw std::runtime_error (message);
}

robot_model::Matrix4 identity_matrix ( )
{
  robot_model::Matrix4 matrix = { };
  for( int index = 0; index < 4; ++index ) matrix[index][index] = 1.0;
  return matrix;
}

robot_model::Matrix4 translated_matrix (double x, double y, double z)
{
  auto matrix = identity_matrix ( );
  matrix[0][3] = x;
  matrix[1][3] = y;
  matrix[2][3] = z;
  return matrix;
}

robot_model::Robot_Visual_Part cube_part ( )
{
  auto cube = vtkSmartPointer<vtkCubeSource>::New ( );
  cube->SetXLength (100.0);
  cube->SetYLength (100.0);
  cube->SetZLength (100.0);
  cube->Update ( );
  robot_model::Robot_Visual_Part part;
  part.mesh_data = cube->GetOutput ( );
  return part;
}

} // namespace

int main ( )
{
  try
  {
    robot_model::Robot_Collision_Detector detector;
    detector.Set_Robot_Parts ({ cube_part ( ) });
    require (detector.Has_Robot_Geometry ( ),
             "Robot collision mesh was not indexed");

    std::string error;
    robot_model::Robot_Collision_Point_Cloud_Options filter_options;
    filter_options.voxel_size_mm = 1.0;
    filter_options.robot_exclusion_distance_mm = 10.0;
    robot_model::Robot_Collision_Point_Cloud_Stats filter_stats;
    require (detector.Set_Obstacle_Points (
               { 55.0f, 0.0f, 0.0f, 80.0f, 0.0f, 0.0f },
               filter_options, { identity_matrix ( ) },
               &filter_stats, &error),
             "Robot point filtering failed: " + error);
    require (filter_stats.excluded_robot_point_count == 1,
             "Robot surface point was not excluded");
    require (filter_stats.collision_point_count == 1,
             "Environment point was removed with robot points");

    require (detector.Set_Obstacle_Points (
               { 55.0f, 0.0f, 0.0f }, filter_options,
               { identity_matrix ( ) }, &filter_stats, &error),
             "Fully filtered robot point cloud was rejected: " + error);
    require (!detector.Has_Obstacle_Points ( ),
             "Fully filtered robot point cloud retained collision points");
    require (filter_stats.excluded_robot_point_count == 1 &&
             filter_stats.collision_point_count == 0,
             "Fully filtered robot point-cloud statistics are wrong");

    filter_options.exclude_robot_points = false;
    require (detector.Set_Obstacle_Points (
               { 55.0f, 0.0f, 0.0f }, filter_options,
               { identity_matrix ( ) }, &filter_stats, &error),
             "Disabled robot point filtering failed: " + error);
    require (filter_stats.excluded_robot_point_count == 0 &&
             filter_stats.collision_point_count == 1,
             "Disabled robot point filtering still removed a point");

    require (detector.Set_Obstacle_Points ({ 55.0f, 0.0f, 0.0f }, &error),
             "Obstacle point was rejected: " + error);
    require (detector.Obstacle_Point_Count ( ) == 1,
             "Obstacle point count was not retained");

    require (detector.Set_Obstacle_Points (
               { 55.0f, 0.0f, 0.0f, 57.0f, 1.0f, 1.0f }, &error),
             "Voxel test points were rejected: " + error);
    require (detector.Obstacle_Point_Count ( ) == 1,
             "Collision point cloud was not voxel-downsampled");
    const std::vector<robot_model::Matrix4> pose = { identity_matrix ( ) };

    const auto collision = detector.Check_Pose (pose, 10.0);
    require (collision.collided, "Point within clearance was not detected");
    require (collision.robot_part_index == 0,
             "Wrong colliding robot part was reported");
    require (std::abs (collision.minimum_distance_mm - 5.0) < 1.0e-6,
             "Unexpected closest distance");

    const auto separated = detector.Check_Pose (pose, 4.0);
    require (!separated.collided,
             "Point outside clearance was reported as collision");

    require (detector.Set_Obstacle_Points ({ 0.0f, 0.0f, 0.0f }, &error),
             "Interior obstacle point was rejected: " + error);
    const auto enclosed = detector.Check_Pose (pose, 0.0);
    require (enclosed.collided,
             "Point enclosed by robot volume was not detected");

    require (detector.Set_Obstacle_Points (
               { 0.0f, 0.0f, 0.0f, 70.0f, 0.0f, 0.0f }, &error),
             "Mixed obstacle points were rejected: " + error);
    const auto mixed = detector.Check_Pose (pose, 10.0);
    require (mixed.collided,
             "A safe point incorrectly hid an enclosed collision point");

    auto moved_pose = identity_matrix ( );
    moved_pose[0][3] = 200.0;
    const auto moved = detector.Check_Pose ({ moved_pose }, 10.0);
    require (!moved.collided, "Part transform was not applied to collision query");

    require (!detector.Set_Obstacle_Points ({ 1.0f, 2.0f }, &error),
             "Malformed obstacle data was accepted");
    detector.Clear_Obstacle_Points ( );
    require (!detector.Has_Obstacle_Points ( ),
             "Obstacle points were not cleared");

    robot_model::Robot_Collision_Rebuild_Request rebuild_request;
    rebuild_request.source_xyz =
      std::make_shared<const std::vector<float>> (
        std::vector<float> { 0.0f, 0.0f, 0.0f });
    rebuild_request.settings.point_cloud.exclude_robot_points = false;
    rebuild_request.robot_parts = { cube_part ( ) };
    rebuild_request.reference_world_from_parts = { identity_matrix ( ) };
    rebuild_request.scene_options.check_self_collision = false;
    rebuild_request.scene_options.check_ground_collision = false;
    auto rebuilt =
      robot_model::Robot_Render_Controller::Build_Collision_Obstacle (
        std::move (rebuild_request));
    require (rebuilt.success && rebuilt.detector &&
             rebuilt.detector->Has_Obstacle_Points ( ),
             "Background collision obstacle build failed");
    require (rebuilt.detector->Check_Pose (
               { identity_matrix ( ) }, 0.0).collided,
             "Background-built obstacle index was not queryable");

    robot_model::Robot_Collision_Rebuild_Request cancelled_request;
    cancelled_request.source_xyz =
      std::make_shared<const std::vector<float>> (
        std::vector<float> { 0.0f, 0.0f, 0.0f });
    cancelled_request.robot_parts = { cube_part ( ) };
    cancelled_request.reference_world_from_parts = { identity_matrix ( ) };
    cancelled_request.cancel_requested =
      std::make_shared<std::atomic_bool> (true);
    const auto cancelled =
      robot_model::Robot_Render_Controller::Build_Collision_Obstacle (
        std::move (cancelled_request));
    require (cancelled.cancelled && !cancelled.success,
             "Cancelled collision obstacle build continued running");

    robot_model::Robot_Collision_Detector performance_detector;
    robot_model::Robot_Scene_Collision_Options performance_options;
    performance_options.check_self_collision = false;
    performance_options.check_ground_collision = false;
    performance_detector.Set_Scene_Collision_Options (performance_options);
    performance_detector.Set_Robot_Parts ({ cube_part ( ) });
    require (performance_detector.Set_Obstacle_Points (
               { 60.0f, 0.0f, 0.0f }, &error),
             "Performance obstacle point was rejected: " + error);
    constexpr std::size_t performance_query_count = 250;
    const auto performance_started_at = std::chrono::steady_clock::now ( );
    for( std::size_t query = 0; query < performance_query_count; ++query )
    {
      require (performance_detector.Check_Pose (
                 { identity_matrix ( ) }, 10.0).collided,
               "Fixed-pose performance query missed collision");
    }
    const double performance_time_ms =
      std::chrono::duration<double, std::milli> (
        std::chrono::steady_clock::now ( ) -
        performance_started_at).count ( );
    require (performance_time_ms < 2000.0,
             "Fixed-pose collision performance regressed: " +
               std::to_string (performance_time_ms) + " ms");
    require (performance_detector.Last_Query_Stats ( ).
               obstacle_candidate_points > 0,
             "Collision query performance counters were not populated");

    robot_model::Robot_Collision_Detector scene_detector;
    scene_detector.Set_Robot_Parts (
      { cube_part ( ), cube_part ( ), cube_part ( ) });
    robot_model::Robot_Scene_Collision_Options scene_options;
    scene_options.check_ground_collision = false;
    scene_detector.Set_Scene_Collision_Options (scene_options);

    const auto self_collision = scene_detector.Check_Pose ({
      identity_matrix ( ), translated_matrix (500.0, 0.0, 0.0),
      translated_matrix (25.0, 0.0, 0.0)
    }, 10.0);
    require (self_collision.collided &&
             self_collision.type ==
               robot_model::Robot_Collision_Type::Self_Collision,
             "Non-adjacent robot self-collision was not detected");
    require (self_collision.robot_part_index == 0 &&
             self_collision.other_robot_part_index == 2,
             "Wrong self-collision part pair was reported");

    const auto self_clearance_collision = scene_detector.Check_Pose ({
      identity_matrix ( ), translated_matrix (500.0, 0.0, 0.0),
      translated_matrix (102.0, 0.0, 0.0)
    }, 10.0);
    require (self_clearance_collision.collided &&
             self_clearance_collision.type ==
               robot_model::Robot_Collision_Type::Self_Collision,
             "Robot self-collision clearance was not enforced");
    require (self_clearance_collision.clearance_margin_mm < 0.0,
             "Self-collision clearance margin was not negative");

    const auto outside_self_clearance = scene_detector.Check_Pose ({
      identity_matrix ( ), translated_matrix (500.0, 0.0, 0.0),
      translated_matrix (104.0, 0.0, 0.0)
    }, 10.0);
    require (!outside_self_clearance.collided,
             "Robot outside self-collision clearance was rejected");

    const auto adjacent_contact = scene_detector.Check_Pose ({
      identity_matrix ( ), identity_matrix ( ),
      translated_matrix (500.0, 0.0, 0.0)
    }, 10.0);
    require (!adjacent_contact.collided,
             "Adjacent robot parts were incorrectly checked for self-collision");

    scene_options.self_collision_pairs = { { 0, 1 } };
    scene_detector.Set_Scene_Collision_Options (scene_options);
    const auto configured_adjacent_collision = scene_detector.Check_Pose ({
      identity_matrix ( ), identity_matrix ( ),
      translated_matrix (500.0, 0.0, 0.0)
    }, 10.0);
    require (configured_adjacent_collision.collided &&
             configured_adjacent_collision.robot_part_index == 0 &&
             configured_adjacent_collision.other_robot_part_index == 1,
             "Configured self-collision pair was not checked");

    scene_options.check_self_collision = false;
    scene_options.check_ground_collision = true;
    scene_detector.Set_Scene_Collision_Options (scene_options);
    const auto ground_collision = scene_detector.Check_Pose ({
      translated_matrix (0.0, 0.0, 500.0),
      translated_matrix (0.0, 0.0, 500.0),
      translated_matrix (0.0, 0.0, 55.0)
    }, 10.0);
    require (ground_collision.collided &&
             ground_collision.type == robot_model::Robot_Collision_Type::Ground,
             "Robot ground clearance collision was not detected");
    require (std::abs (ground_collision.minimum_distance_mm - 5.0) < 1.0e-6,
             "Wrong robot ground clearance distance was reported");

    const auto above_ground = scene_detector.Check_Pose ({
      translated_matrix (0.0, 0.0, 500.0),
      translated_matrix (0.0, 0.0, 500.0),
      translated_matrix (0.0, 0.0, 70.0)
    }, 10.0);
    require (!above_ground.collided,
             "Robot above the ground clearance was rejected");

    robot_model::Robot_Collision_Result original_collision;
    original_collision.collided = true;
    original_collision.type =
      robot_model::Robot_Collision_Type::Obstacle_Point;
    original_collision.robot_part_index = 3;
    original_collision.clearance_margin_mm = -12.0;
    auto recovering_collision = original_collision;
    recovering_collision.clearance_margin_mm = -7.0;
    require (robot_model::Is_Robot_Collision_Recovery_Improvement (
               original_collision, recovering_collision,
               original_collision.clearance_margin_mm),
             "Motion away from a collision was not accepted as recovery");
    recovering_collision.clearance_margin_mm = -15.0;
    require (!robot_model::Is_Robot_Collision_Recovery_Improvement (
               original_collision, recovering_collision,
               original_collision.clearance_margin_mm),
             "Motion deeper into a collision was accepted as recovery");
    recovering_collision = original_collision;
    recovering_collision.type = robot_model::Robot_Collision_Type::Ground;
    recovering_collision.clearance_margin_mm = -1.0;
    require (!robot_model::Is_Robot_Collision_Recovery_Improvement (
               original_collision, recovering_collision,
               original_collision.clearance_margin_mm),
             "Motion into a different collision was accepted as recovery");

    const auto models = robot_model::Scan_Models_In_Directory (
      robot_model::Find_Robot_Root ( ));
    const std::array<double, 6> home_angles = {
      0.0, -90.0, 90.0, 0.0, 90.0, 0.0
    };
    for( const auto& model : models )
    {
      std::vector<robot_model::Robot_Visual_Part> parts;
      for( std::size_t index = 0; index < model.stl_files.size ( ); ++index )
      {
        parts.push_back (robot_model::Create_STL_Part (
          model.stl_files[index], static_cast<int> (index)));
      }
      const auto params = robot_model::Load_Robot_Kinematic_Params (
        model.xml_path, model.display_name);
      const auto calibration = robot_model::Build_Assembly_Calibration (
        params, model.display_name, parts);
      const auto forward_model = robot_model::Build_Forward_Kinematics_Model (
        parts.size ( ), calibration, params);
      const auto state = robot_model::Build_Joint_State_From_Input_Angles (
        params, home_angles);
      const auto transforms = robot_model::Compute_Forward_Kinematics (
        forward_model, state);
      robot_model::Robot_Collision_Detector model_detector;
      model_detector.Set_Robot_Parts (parts);
      const auto home_collision = model_detector.Check_Pose (
        transforms.world_from_parts, 10.0);
      const auto& query_stats = model_detector.Last_Query_Stats ( );
      if( query_stats.self_exact_pair_queries > 0 )
      {
        model_detector.Check_Pose (transforms.world_from_parts, 10.0);
        require (
          model_detector.Last_Query_Stats ( ).self_exact_pair_queries == 0,
          "Unchanged safe self-collision pair repeated exact mesh detection");
      }
      require (!home_collision.collided,
               "Robot home pose is initially colliding: " +
                 model.display_name);
    }

    std::cout << "Robot collision detector tests passed.\n";
    return 0;
  }
  catch( const std::exception& error )
  {
    std::cerr << error.what ( ) << '\n';
    return 1;
  }
}

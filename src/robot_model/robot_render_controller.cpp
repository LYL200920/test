#include "robot_render_controller.h"

#include "robot_calibration_builder.h"
#include "robot_kinematic_params.h"
#include "robot_joint_sweep.h"
#include "robot_model_config_loader.h"
#include "vtk_scene.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace robot_model
{
namespace
{

constexpr double kMaximumCollisionJointStepDeg = 2.0;
constexpr double kMaximumCollisionSpatialStepMm = 5.0;
constexpr std::size_t kCollisionBoundaryRefinementIterations = 4;
constexpr std::size_t kInteractiveBoundaryRefinementIterations = 2;
constexpr std::size_t kMaximumInteractiveSweepPoseCount = 8;

bool valid_collision_settings (
  const Robot_Collision_Settings& settings,
  std::string* error_message)
{
  if( !std::isfinite (settings.clearance_mm) ||
      settings.clearance_mm < 0.0 )
  {
    if( error_message ) *error_message = "Collision clearance is invalid";
    return false;
  }
  if( !std::isfinite (settings.point_cloud.voxel_size_mm) ||
      settings.point_cloud.voxel_size_mm <= 0.0 ||
      !std::isfinite (
        settings.point_cloud.robot_exclusion_distance_mm) ||
      settings.point_cloud.robot_exclusion_distance_mm < 0.0 )
  {
    if( error_message )
      *error_message = "Collision point-cloud options are invalid";
    return false;
  }
  return true;
}

void accumulate_query_stats (
  const Robot_Collision_Query_Stats& source,
  Robot_Collision_Query_Stats* target)
{
  if( !target ) return;
  target->ground_sample_queries += source.ground_sample_queries;
  target->self_broad_phase_pairs += source.self_broad_phase_pairs;
  target->self_obb_phase_pairs += source.self_obb_phase_pairs;
  target->self_distance_sample_queries +=
    source.self_distance_sample_queries;
  target->self_exact_pair_queries += source.self_exact_pair_queries;
  target->obstacle_candidate_points += source.obstacle_candidate_points;
  target->obstacle_distance_queries += source.obstacle_distance_queries;
  if( source.minimum_self_sample_distance_mm > 0.0 &&
      std::isfinite (source.minimum_self_sample_distance_mm) )
  {
    if( target->minimum_self_sample_distance_mm <= 0.0 )
      target->minimum_self_sample_distance_mm =
        source.minimum_self_sample_distance_mm;
    else
      target->minimum_self_sample_distance_mm = std::min (
        target->minimum_self_sample_distance_mm,
        source.minimum_self_sample_distance_mm);
  }
}

double position_distance (const Point3& lhs, const Point3& rhs)
{
  const double dx = lhs[0] - rhs[0];
  const double dy = lhs[1] - rhs[1];
  const double dz = lhs[2] - rhs[2];
  return std::sqrt (dx * dx + dy * dy + dz * dz);
}

double orientation_distance_degrees (
  const Matrix4& actual,
  const Matrix4& target)
{
  double trace = 0.0;
  for( int row = 0; row < 3; ++row )
  {
    for( int column = 0; column < 3; ++column )
    {
      trace += target[column][row] * actual[column][row];
    }
  }
  const double cosine = std::clamp (( trace - 1.0 ) * 0.5, -1.0, 1.0);
  return std::acos (cosine) * 180.0 / 3.14159265358979323846;
}

std::array<double, 6> collision_influence_radii (
  const std::vector<Robot_Visual_Part>& parts,
  const Robot_Forward_Kinematics_Result& start,
  const Robot_Forward_Kinematics_Result& target)
{
  std::array<double, 6> radii = { };
  const auto accumulate_pose = [&] (
    const Robot_Forward_Kinematics_Result& pose)
  {
    for( std::size_t joint = 0;
         joint < pose.joint_positions_world.size ( ); ++joint )
    {
      const auto& pivot = pose.joint_positions_world[joint];
      for( std::size_t part_index = joint + 1;
           part_index < parts.size ( ) &&
             part_index < pose.world_from_parts.size ( ); ++part_index )
      {
        const auto& part = parts[part_index];
        if( !part.has_raw_bounds ) continue;
        for( int x_side = 0; x_side < 2; ++x_side )
        {
          for( int y_side = 0; y_side < 2; ++y_side )
          {
            for( int z_side = 0; z_side < 2; ++z_side )
            {
              const auto corner = Transform_Position (
                pose.world_from_parts[part_index],
                { part.raw_bounds[x_side],
                  part.raw_bounds[2 + y_side],
                  part.raw_bounds[4 + z_side] });
              const double dx = corner[0] - pivot[0];
              const double dy = corner[1] - pivot[1];
              const double dz = corner[2] - pivot[2];
              radii[joint] = std::max (
                radii[joint], std::sqrt (dx * dx + dy * dy + dz * dz));
            }
          }
        }
      }
    }
  };
  accumulate_pose (start);
  accumulate_pose (target);
  return radii;
}

} // namespace

void Robot_Render_Controller::Attach_Scene (Vtk_Scene* scene)
{
  m_scene = scene;
  if( m_state.Robot_Model ( ).Has_Current_Model ( ) )
  {
    Rebuild_Current_Model ( );
  }
}

void Robot_Render_Controller::Detach_Scene ( )
{
  m_assembly.Clear ( );
  m_collision_detector.Clear ( );
  m_forward_model = { };
  m_has_forward_model = false;
  m_current_pose_collision = { };
  m_scene = nullptr;
}

void Robot_Render_Controller::Load_Model (const Robot_Model_Info& model)
{
  const auto params = Load_Robot_Kinematic_Params (
    model.xml_path, model.display_name);
  m_state.Robot_Model ( ).Set_Current_Model (model, params);
  Rebuild_Current_Model ( );
}

void Robot_Render_Controller::Set_Joint_State (
  const Robot_Joint_State& joint_state)
{
  m_state.Robot_Model ( ).Set_Joint_State (joint_state);
  Apply_Joint_Pose ( );
  Update_Current_Pose_Collision ( );
  const bool scene_changed = m_current_pose_collision.collided
    ? m_assembly.Show_Collision (m_current_pose_collision)
    : m_assembly.Clear_Collision ( );
  m_last_joint_apply_result = { };
  m_last_joint_apply_result.accepted = true;
  m_last_joint_apply_result.state_changed = true;
  m_last_joint_apply_result.scene_changed = scene_changed;
}

Robot_Joint_State_Apply_Result
Robot_Render_Controller::Try_Set_Joint_State (
  const Robot_Joint_State& joint_state)
{
  return Try_Set_Joint_State_With_Refinement (
    joint_state, kCollisionBoundaryRefinementIterations);
}

Robot_Joint_State_Apply_Result
Robot_Render_Controller::Try_Set_Joint_State_With_Refinement (
  const Robot_Joint_State& joint_state,
  std::size_t boundary_refinement_iterations,
  std::size_t maximum_sweep_pose_count)
{
  m_last_joint_apply_result = { };
  auto& result = m_last_joint_apply_result;
  auto& model_state = m_state.Robot_Model ( );
  if( !model_state.Has_Current_Model ( ) ) return result;

  if( !m_has_forward_model ||
      !m_collision_detector.Has_Robot_Geometry ( ) )
  {
    Set_Joint_State (joint_state);
    result.accepted = true;
    return result;
  }

  result.collision_checked = true;
  result.clearance_mm = m_collision_settings.clearance_mm;
  const auto start_state = model_state.Joint_State ( );
  const auto start_transforms = Compute_Forward_Kinematics (
    m_forward_model, start_state);
  auto target_state = joint_state;
  auto target_transforms = Compute_Forward_Kinematics (
    m_forward_model, target_state);
  auto influence_radii = collision_influence_radii (
    m_assembly.Parts ( ), start_transforms, target_transforms);
  auto step_count = Calculate_Robot_Joint_Sweep_Step_Count (
    start_state, target_state, kMaximumCollisionJointStepDeg,
    influence_radii, kMaximumCollisionSpatialStepMm);
  for( int limiting_pass = 0;
       maximum_sweep_pose_count > 0 &&
         step_count > maximum_sweep_pose_count && limiting_pass < 3;
       ++limiting_pass )
  {
    const double fraction = static_cast<double> (
      maximum_sweep_pose_count) / static_cast<double> (step_count);
    target_state = Interpolate_Robot_Joint_State (
      start_state, target_state, fraction);
    target_transforms = Compute_Forward_Kinematics (
      m_forward_model, target_state);
    influence_radii = collision_influence_radii (
      m_assembly.Parts ( ), start_transforms, target_transforms);
    step_count = Calculate_Robot_Joint_Sweep_Step_Count (
      start_state, target_state, kMaximumCollisionJointStepDeg,
      influence_radii, kMaximumCollisionSpatialStepMm);
  }
  Robot_Joint_State last_allowed_state = start_state;
  Robot_Forward_Kinematics_Result last_allowed_transforms = start_transforms;
  Robot_Collision_Result last_allowed_collision = m_current_pose_collision;
  bool has_intermediate_allowed_state = false;
  bool recovering = m_current_pose_collision.collided;
  double recovery_margin = m_current_pose_collision.clearance_margin_mm;
  const auto check_collision = [&] (
    const std::vector<Matrix4>& world_from_parts)
  {
    const auto started_at = std::chrono::steady_clock::now ( );
    auto collision = m_collision_detector.Check_Pose (
      world_from_parts, result.clearance_mm);
    result.collision_query_time_ms +=
      std::chrono::duration<double, std::milli> (
        std::chrono::steady_clock::now ( ) - started_at).count ( );
    accumulate_query_stats (
      m_collision_detector.Last_Query_Stats ( ),
      &result.collision_query_stats);
    ++result.checked_pose_count;
    return collision;
  };
  for( std::size_t step = 1; step <= step_count; ++step )
  {
    const double t = static_cast<double> (step) /
      static_cast<double> (step_count);
    const auto sample_state = step == step_count
      ? target_state
      : Interpolate_Robot_Joint_State (start_state, target_state, t);
    const auto sample_transforms = step == step_count
      ? target_transforms
      : Compute_Forward_Kinematics (m_forward_model, sample_state);
    result.collision = check_collision (sample_transforms.world_from_parts);
    if( result.collision.collided )
    {
      const bool improves_current_collision =
        recovering &&
        Is_Robot_Collision_Recovery_Improvement (
          m_current_pose_collision, result.collision, recovery_margin);
      if( improves_current_collision )
      {
        recovery_margin = result.collision.clearance_margin_mm;
        last_allowed_state = sample_state;
        last_allowed_transforms = sample_transforms;
        last_allowed_collision = result.collision;
        has_intermediate_allowed_state = true;
        result.recovery_motion = true;
        continue;
      }

      if( recovering )
      {
        if( has_intermediate_allowed_state )
        {
          model_state.Set_Joint_State (last_allowed_state);
          m_assembly.Apply_Forward_Kinematics (last_allowed_transforms);
          m_current_pose_collision = last_allowed_collision;
          result.state_changed = true;
          result.recovery_motion = true;
        }
        result.scene_changed = m_assembly.Show_Collision (result.collision);
        return result;
      }

      if( has_intermediate_allowed_state )
      {
        auto refined_safe_state = last_allowed_state;
        auto refined_safe_transforms = last_allowed_transforms;
        auto refined_collision_state = sample_state;
        for( std::size_t refinement = 0;
             refinement < boundary_refinement_iterations;
             ++refinement )
        {
          const auto midpoint_state = Interpolate_Robot_Joint_State (
            refined_safe_state, refined_collision_state, 0.5);
          const auto midpoint_transforms = Compute_Forward_Kinematics (
            m_forward_model, midpoint_state);
          const auto midpoint_collision = check_collision (
            midpoint_transforms.world_from_parts);
          if( midpoint_collision.collided )
          {
            refined_collision_state = midpoint_state;
            result.collision = midpoint_collision;
          }
          else
          {
            refined_safe_state = midpoint_state;
            refined_safe_transforms = midpoint_transforms;
          }
        }
        model_state.Set_Joint_State (refined_safe_state);
        m_assembly.Apply_Forward_Kinematics (refined_safe_transforms);
        m_current_pose_collision = { };
        result.state_changed = true;
      }
      result.scene_changed = m_assembly.Show_Collision (result.collision);
      return result;
    }
    recovering = false;
    last_allowed_state = sample_state;
    last_allowed_transforms = sample_transforms;
    last_allowed_collision = result.collision;
    has_intermediate_allowed_state = true;
  }

  model_state.Set_Joint_State (last_allowed_state);
  m_assembly.Apply_Forward_Kinematics (last_allowed_transforms);
  m_current_pose_collision = last_allowed_collision;
  result.accepted = true;
  result.state_changed = true;
  result.scene_changed = m_current_pose_collision.collided
    ? m_assembly.Show_Collision (m_current_pose_collision)
    : m_assembly.Clear_Collision ( );
  return result;
}

bool Robot_Render_Controller::Has_Current_Model ( ) const
{
  return m_state.Robot_Model ( ).Has_Current_Model ( );
}

const Robot_Kinematic_Params& Robot_Render_Controller::Kinematic_Params ( ) const
{
  return m_state.Robot_Model ( ).Params ( );
}

const Robot_Joint_State& Robot_Render_Controller::Joint_State ( ) const
{
  return m_state.Robot_Model ( ).Joint_State ( );
}

bool Robot_Render_Controller::Has_Flange_Pose ( ) const
{
  return m_assembly.Has_Flange_Pose ( );
}

const Matrix4& Robot_Render_Controller::World_From_Flange ( ) const
{
  return m_assembly.World_From_Flange ( );
}

bool Robot_Render_Controller::Set_Collision_Obstacle_Points (
  const std::vector<float>& xyz,
  std::string* error_message)
{
  return Set_Collision_Obstacle_Points (
    std::make_shared<const std::vector<float>> (xyz), error_message);
}

bool Robot_Render_Controller::Set_Collision_Obstacle_Points (
  std::shared_ptr<const std::vector<float>> xyz,
  std::string* error_message)
{
  m_collision_source_xyz = xyz
    ? std::move (xyz)
    : std::make_shared<const std::vector<float>> ( );
  m_collision_reference_joint_state = m_state.Robot_Model ( ).Joint_State ( );
  const bool rebuilt = Rebuild_Collision_Obstacle_Points (error_message);
  if( rebuilt )
  {
    Update_Current_Pose_Collision ( );
    if( m_current_pose_collision.collided )
      m_assembly.Show_Collision (m_current_pose_collision);
    else
      m_assembly.Clear_Collision ( );
  }
  return rebuilt;
}

void Robot_Render_Controller::Clear_Collision_Obstacle_Points ( )
{
  m_collision_detector.Clear_Obstacle_Points ( );
  m_collision_source_xyz.reset ( );
  m_collision_point_cloud_stats = { };
  Update_Current_Pose_Collision ( );
  if( m_current_pose_collision.collided )
    m_assembly.Show_Collision (m_current_pose_collision);
  else
    m_assembly.Clear_Collision ( );
}

bool Robot_Render_Controller::Has_Collision_Obstacle_Points ( ) const
{
  return m_collision_detector.Has_Obstacle_Points ( );
}

std::size_t Robot_Render_Controller::Collision_Obstacle_Point_Count ( ) const
{
  return m_collision_detector.Obstacle_Point_Count ( );
}

bool Robot_Render_Controller::Set_Collision_Settings (
  const Robot_Collision_Settings& settings,
  std::string* error_message)
{
  if( !std::isfinite (settings.clearance_mm) ||
      settings.clearance_mm < 0.0 )
  {
    if( error_message ) *error_message = "Collision clearance is invalid";
    return false;
  }
  const auto previous = m_collision_settings;
  m_collision_settings = settings;
  if( !Rebuild_Collision_Obstacle_Points (error_message) )
  {
    m_collision_settings = previous;
    Rebuild_Collision_Obstacle_Points (nullptr);
    return false;
  }
  Update_Current_Pose_Collision ( );
  if( m_current_pose_collision.collided )
    m_assembly.Show_Collision (m_current_pose_collision);
  else
    m_assembly.Clear_Collision ( );
  return true;
}

bool Robot_Render_Controller::Create_Collision_Points_Rebuild_Request (
  std::shared_ptr<const std::vector<float>> xyz,
  Robot_Collision_Rebuild_Request* request,
  std::string* error_message) const
{
  if( error_message ) error_message->clear ( );
  if( !request || !xyz || xyz->empty ( ) || xyz->size ( ) % 3 != 0 )
  {
    if( error_message )
      *error_message = "Obstacle point array is empty or malformed";
    return false;
  }
  if( !valid_collision_settings (m_collision_settings, error_message) )
    return false;

  request->source_xyz = std::move (xyz);
  request->settings = m_collision_settings;
  request->reference_joint_state = m_state.Robot_Model ( ).Joint_State ( );
  request->robot_parts = m_assembly.Parts ( );
  request->scene_options = m_collision_detector.Scene_Collision_Options ( );
  if( m_has_forward_model )
  {
    request->reference_world_from_parts = Compute_Forward_Kinematics (
      m_forward_model, request->reference_joint_state).world_from_parts;
  }
  return true;
}

bool Robot_Render_Controller::Create_Collision_Settings_Rebuild_Request (
  const Robot_Collision_Settings& settings,
  Robot_Collision_Rebuild_Request* request,
  std::string* error_message) const
{
  if( error_message ) error_message->clear ( );
  if( !request || !valid_collision_settings (settings, error_message) )
    return false;

  request->source_xyz = m_collision_source_xyz;
  request->settings = settings;
  request->reference_joint_state = m_collision_reference_joint_state;
  request->robot_parts = m_assembly.Parts ( );
  request->scene_options = m_collision_detector.Scene_Collision_Options ( );
  if( m_has_forward_model )
  {
    request->reference_world_from_parts = Compute_Forward_Kinematics (
      m_forward_model, request->reference_joint_state).world_from_parts;
  }
  return true;
}

Robot_Collision_Rebuild_Result
Robot_Render_Controller::Build_Collision_Obstacle (
  Robot_Collision_Rebuild_Request request)
{
  const auto started_at = std::chrono::steady_clock::now ( );
  Robot_Collision_Rebuild_Result result;
  result.source_xyz = std::move (request.source_xyz);
  result.settings = request.settings;
  result.reference_joint_state = request.reference_joint_state;
  result.detector = std::make_unique<Robot_Collision_Detector> ( );
  result.detector->Set_Scene_Collision_Options (request.scene_options);
  if( request.cancel_requested && request.cancel_requested->load ( ) )
  {
    result.cancelled = true;
    result.stats.build_time_ms = std::chrono::duration<double, std::milli> (
      std::chrono::steady_clock::now ( ) - started_at).count ( );
    return result;
  }
  result.detector->Set_Robot_Parts (request.robot_parts);
  if( request.cancel_requested && request.cancel_requested->load ( ) )
  {
    result.cancelled = true;
    result.stats.build_time_ms = std::chrono::duration<double, std::milli> (
      std::chrono::steady_clock::now ( ) - started_at).count ( );
    return result;
  }
  if( !result.source_xyz || result.source_xyz->empty ( ) )
  {
    result.success = true;
    result.stats.build_time_ms = std::chrono::duration<double, std::milli> (
      std::chrono::steady_clock::now ( ) - started_at).count ( );
    return result;
  }
  result.success = result.detector->Set_Obstacle_Points (
    *result.source_xyz, result.settings.point_cloud,
    request.reference_world_from_parts, &result.stats,
    &result.error_message, request.cancel_requested.get ( ));
  result.cancelled = request.cancel_requested &&
    request.cancel_requested->load (std::memory_order_relaxed);
  result.stats.build_time_ms = std::chrono::duration<double, std::milli> (
    std::chrono::steady_clock::now ( ) - started_at).count ( );
  return result;
}

void Robot_Render_Controller::Apply_Collision_Rebuild_Result (
  Robot_Collision_Rebuild_Result result)
{
  if( !result.success || !result.detector ) return;

  m_collision_detector = std::move ( *result.detector );
  m_collision_source_xyz = std::move (result.source_xyz);
  m_collision_settings = result.settings;
  m_collision_reference_joint_state = result.reference_joint_state;
  m_collision_point_cloud_stats = result.stats;
  Update_Current_Pose_Collision ( );
  if( m_current_pose_collision.collided )
    m_assembly.Show_Collision (m_current_pose_collision);
  else
    m_assembly.Clear_Collision ( );
}

bool Robot_Render_Controller::Rebuild_Collision_Obstacle_Points (
  std::string* error_message)
{
  if( error_message ) error_message->clear ( );
  if( !m_collision_source_xyz || m_collision_source_xyz->empty ( ) )
  {
    m_collision_detector.Clear_Obstacle_Points ( );
    m_collision_point_cloud_stats = { };
    return true;
  }
  std::vector<Matrix4> reference_world_from_parts;
  if( m_has_forward_model )
  {
    reference_world_from_parts = Compute_Forward_Kinematics (
      m_forward_model,
      m_collision_reference_joint_state).world_from_parts;
  }
  return m_collision_detector.Set_Obstacle_Points (
    *m_collision_source_xyz,
    m_collision_settings.point_cloud,
    reference_world_from_parts,
    &m_collision_point_cloud_stats,
    error_message);
}

Robot_Position_IK_Result Robot_Render_Controller::Move_Flange_To (
  const Point3& target_world)
{
  m_last_joint_apply_result = { };
  Robot_Position_IK_Result result;
  auto& model_state = m_state.Robot_Model ( );
  if( !m_has_forward_model || !model_state.Has_Current_Model ( ) )
  {
    return result;
  }

  result = Solve_Flange_Position_IK (
    m_forward_model,
    model_state.Params ( ),
    model_state.Joint_State ( ),
    target_world);
  if( result.status != Robot_IK_Status::Invalid_Model &&
      std::isfinite (result.position_error_mm) )
  {
    Try_Set_Joint_State_With_Refinement (
      result.joint_state, kInteractiveBoundaryRefinementIterations,
      kMaximumInteractiveSweepPoseCount);
    const auto actual = Compute_Forward_Kinematics (
      m_forward_model, model_state.Joint_State ( ));
    if( actual.has_flange )
    {
      result.joint_state = model_state.Joint_State ( );
      result.achieved_position_world = {
        actual.world_from_flange[0][3],
        actual.world_from_flange[1][3],
        actual.world_from_flange[2][3]
      };
      result.position_error_mm = position_distance (
        result.achieved_position_world, target_world);
    }
  }
  return result;
}

Robot_Pose_IK_Result Robot_Render_Controller::Move_Flange_To_Pose (
  const Matrix4& target_world_from_flange,
  const Robot_Pose_IK_Options& options)
{
  m_last_joint_apply_result = { };
  Robot_Pose_IK_Result result;
  auto& model_state = m_state.Robot_Model ( );
  if( !m_has_forward_model || !model_state.Has_Current_Model ( ) )
  {
    return result;
  }
  result = Solve_Flange_Pose_IK (
    m_forward_model,
    model_state.Params ( ),
    model_state.Joint_State ( ),
    target_world_from_flange,
    options);
  if( result.status != Robot_IK_Status::Invalid_Model &&
      std::isfinite (result.position_error_mm) &&
      std::isfinite (result.orientation_error_deg) )
  {
    Try_Set_Joint_State_With_Refinement (
      result.joint_state, kInteractiveBoundaryRefinementIterations,
      kMaximumInteractiveSweepPoseCount);
    const auto actual = Compute_Forward_Kinematics (
      m_forward_model, model_state.Joint_State ( ));
    if( actual.has_flange )
    {
      result.joint_state = model_state.Joint_State ( );
      result.achieved_world_from_flange = actual.world_from_flange;
      const Point3 achieved_position = {
        actual.world_from_flange[0][3],
        actual.world_from_flange[1][3],
        actual.world_from_flange[2][3]
      };
      const Point3 target_position = {
        target_world_from_flange[0][3],
        target_world_from_flange[1][3],
        target_world_from_flange[2][3]
      };
      result.position_error_mm = position_distance (
        achieved_position, target_position);
      result.orientation_error_deg = orientation_distance_degrees (
        actual.world_from_flange, target_world_from_flange);
    }
  }
  return result;
}

void Robot_Render_Controller::Rebuild_Current_Model ( )
{
  auto& robot_model_state = m_state.Robot_Model ( );
  if( m_scene == nullptr || !m_scene->Is_Ready ( ) ||
      !robot_model_state.Has_Current_Model ( ) )
  {
    return;
  }

  const auto& model = robot_model_state.Current_Model ( );
  m_scene->Remove_All_ViewProps ( );
  m_scene->Remove_All_Lights ( );
  m_assembly.Clear ( );
  m_collision_detector.Set_Robot_Parts ({ });
  m_forward_model = { };
  m_has_forward_model = false;
  robot_model_state.Clear_Assembly_Calibration ( );

  if( model.Has_Mesh ( ) )
  {
    m_assembly.Load_Mesh (model, m_scene->Renderer ( ));
    auto scene_collision_options =
      m_collision_detector.Scene_Collision_Options ( );
    scene_collision_options.self_collision_clearance_mm =
      robot_model_state.Params ( ).self_collision_clearance_mm;
    scene_collision_options.self_collision_pairs =
      robot_model_state.Params ( ).self_collision_pairs;
    scene_collision_options.ground_checked_parts =
      robot_model_state.Params ( ).ground_collision_parts;
    m_collision_detector.Set_Scene_Collision_Options (
      scene_collision_options);
    m_collision_detector.Set_Robot_Parts (m_assembly.Parts ( ));
    const auto assembly_calibration = Build_Assembly_Calibration (
      robot_model_state.Params ( ),
      robot_model_state.Model_Name ( ),
      m_assembly.Parts ( ));
    robot_model_state.Set_Assembly_Calibration (assembly_calibration);
    m_forward_model = Build_Forward_Kinematics_Model (
      m_assembly.Parts ( ).size ( ),
      robot_model_state.Assembly_Calibration ( ),
      robot_model_state.Params ( ));
    m_has_forward_model = m_forward_model.has_flange;
    m_collision_reference_joint_state = robot_model_state.Joint_State ( );
    Rebuild_Collision_Obstacle_Points (nullptr);
    Apply_Joint_Pose ( );
    Update_Current_Pose_Collision ( );
  }
  else
  {
    m_assembly.Build_Fallback_Demo (robot_model_state.Params ( ),
                                    m_scene->Renderer ( ));
  }

  const double link_extent = robot_model_state.Params ( ).Has_Link_Lengths ( )
    ? link_length_at (robot_model_state.Params ( ), 1, 1000.0) +
      link_length_at (robot_model_state.Params ( ), 3, 1000.0)
    : 2000.0;
  const double extent = std::max (5000.0, link_extent * 2.2);
  m_scene->Add_Ground_Grid (extent);
  m_scene->Reset_Camera ( );
}

void Robot_Render_Controller::Apply_Joint_Pose ( )
{
  const auto& robot_model_state = m_state.Robot_Model ( );
  if( !robot_model_state.Has_Assembly_Calibration ( ) )
  {
    return;
  }

  if( m_has_forward_model )
  {
    m_assembly.Apply_Forward_Kinematics (Compute_Forward_Kinematics (
      m_forward_model, robot_model_state.Joint_State ( )));
  }
}

void Robot_Render_Controller::Update_Current_Pose_Collision ( )
{
  m_current_pose_collision = { };
  const auto& robot_model_state = m_state.Robot_Model ( );
  if( !m_has_forward_model ||
      !robot_model_state.Has_Current_Model ( ) ||
      !m_collision_detector.Has_Robot_Geometry ( ) )
  {
    return;
  }
  const auto transforms = Compute_Forward_Kinematics (
    m_forward_model, robot_model_state.Joint_State ( ));
  m_current_pose_collision = m_collision_detector.Check_Pose (
    transforms.world_from_parts, m_collision_settings.clearance_mm);
}

} // namespace robot_model

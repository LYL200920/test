#include "robot_forward_kinematics.h"

#include "robot_kinematic_params.h"
#include "robot_part_transform.h"

#include <algorithm>
#include <cmath>

namespace robot_model
{
namespace
{

Matrix4 identity_matrix ( )
{
  Matrix4 matrix = { };
  for( int i = 0; i < 4; ++i )
  {
    matrix[i][i] = 1.0;
  }
  return matrix;
}

Point3 transform_vector (const Matrix4& matrix, const Point3& vector)
{
  return {
    matrix[0][0] * vector[0] + matrix[0][1] * vector[1] +
      matrix[0][2] * vector[2],
    matrix[1][0] * vector[0] + matrix[1][1] * vector[1] +
      matrix[1][2] * vector[2],
    matrix[2][0] * vector[0] + matrix[2][1] * vector[1] +
      matrix[2][2] * vector[2]
  };
}

Point3 normalized_axis (Point3 axis)
{
  const double length = std::sqrt (
    axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
  if( length < 1.0e-12 )
  {
    return { 0.0, 0.0, 1.0 };
  }
  for( double& value : axis )
  {
    value /= length;
  }
  return axis;
}

Matrix4 rotation_about_point (
  double angle_deg,
  const Point3& input_axis,
  const Point3& point)
{
  const auto axis = normalized_axis (input_axis);
  const double radians = angle_deg * 3.14159265358979323846 / 180.0;
  const double c = std::cos (radians);
  const double s = std::sin (radians);
  const double t = 1.0 - c;
  const double x = axis[0];
  const double y = axis[1];
  const double z = axis[2];

  Matrix4 matrix = identity_matrix ( );
  matrix[0][0] = t * x * x + c;
  matrix[0][1] = t * x * y - s * z;
  matrix[0][2] = t * x * z + s * y;
  matrix[1][0] = t * x * y + s * z;
  matrix[1][1] = t * y * y + c;
  matrix[1][2] = t * y * z - s * x;
  matrix[2][0] = t * x * z - s * y;
  matrix[2][1] = t * y * z + s * x;
  matrix[2][2] = t * z * z + c;
  matrix[0][3] = point[0] - (
    matrix[0][0] * point[0] + matrix[0][1] * point[1] +
    matrix[0][2] * point[2]);
  matrix[1][3] = point[1] - (
    matrix[1][0] * point[0] + matrix[1][1] * point[1] +
    matrix[1][2] * point[2]);
  matrix[2][3] = point[2] - (
    matrix[2][0] * point[0] + matrix[2][1] * point[1] +
    matrix[2][2] * point[2]);
  return matrix;
}

Point3 part_origin_or (
  std::size_t part_count,
  const Robot_Assembly_Calibration& calibration,
  std::size_t index,
  const Point3& fallback)
{
  if( index >= part_count || index >= calibration.parts.size ( ) )
  {
    return fallback;
  }
  return Transform_Part_Point (
    calibration.parts[index], { 0.0, 0.0, 0.0 });
}

Point3 configured_pivot_or (
  const Robot_Kinematic_Params& params,
  std::size_t index,
  const Point3& fallback)
{
  if( index >= params.joint_frames.size ( ) ||
      !params.joint_frames[index].has_pivot )
  {
    return fallback;
  }
  return params.joint_frames[index].pivot;
}

Point3 configured_axis_or (
  const Robot_Kinematic_Params& params,
  std::size_t index,
  const Point3& fallback)
{
  if( index >= params.joint_frames.size ( ) ||
      !params.joint_frames[index].has_axis )
  {
    return fallback;
  }
  return normalized_axis (params.joint_frames[index].axis);
}

double joint_delta_at (const Robot_Joint_State& state, std::size_t index)
{
  return index < state.delta_angles_deg.size ( )
    ? state.delta_angles_deg[index]
    : 0.0;
}

} // namespace

Robot_Forward_Kinematics_Model Build_Forward_Kinematics_Model (
  std::size_t part_count,
  const Robot_Assembly_Calibration& calibration,
  const Robot_Kinematic_Params& params)
{
  Robot_Forward_Kinematics_Model model;
  const double shoulder_z = link_length_at (params, 0, 0.0);
  const double elbow_z = shoulder_z + link_length_at (params, 1, 0.0);
  const double wrist_x = link_length_at (params, 3, 0.0);
  const Point3 base_origin = { 0.0, 0.0, 0.0 };
  const Point3 shoulder_fallback = { 0.0, 0.0, shoulder_z };
  const Point3 elbow_fallback = { 0.0, 0.0, elbow_z };
  const Point3 wrist_fallback = { wrist_x, 0.0, elbow_z };
  const auto a2 = part_origin_or (
    part_count, calibration, 2, shoulder_fallback);
  const auto a3 = part_origin_or (
    part_count, calibration, 3, elbow_fallback);
  const auto a4 = part_origin_or (
    part_count, calibration, 4, wrist_fallback);
  const auto a5 = part_origin_or (part_count, calibration, 5, a4);
  const auto a6 = part_origin_or (part_count, calibration, 6, a5);

  model.joint_pivots = {
    configured_pivot_or (params, 0, base_origin),
    configured_pivot_or (params, 1, a2),
    configured_pivot_or (params, 2, a3),
    configured_pivot_or (params, 3, a4),
    configured_pivot_or (params, 4, a5),
    configured_pivot_or (params, 5, a6)
  };
  model.joint_axes = {
    configured_axis_or (params, 0, { 0.0, 0.0, 1.0 }),
    configured_axis_or (params, 1, { 0.0, 1.0, 0.0 }),
    configured_axis_or (params, 2, { 0.0, 1.0, 0.0 }),
    configured_axis_or (params, 3, { 1.0, 0.0, 0.0 }),
    configured_axis_or (params, 4, { 0.0, 1.0, 0.0 }),
    configured_axis_or (params, 5, { 1.0, 0.0, 0.0 })
  };

  const auto count = std::min (part_count, calibration.parts.size ( ));
  model.neutral_world_from_parts.reserve (count);
  for( std::size_t i = 0; i < count; ++i )
  {
    model.neutral_world_from_parts.push_back (
      Build_Part_Calibration_Matrix (calibration.parts[i]));
  }

  model.flange_from_last_part = identity_matrix ( );
  model.has_flange = model.neutral_world_from_parts.size ( ) >= 7;
  if( model.has_flange && params.has_neutral_flange_pose )
  {
    const auto neutral_world_from_flange =
      Build_Zyx_Pose_Matrix (params.neutral_flange_pose);
    model.flange_from_last_part = Multiply_Matrices (
      Invert_Rigid_Matrix (model.neutral_world_from_parts.back ( )),
      neutral_world_from_flange);
  }
  return model;
}

Robot_Forward_Kinematics_Result Compute_Forward_Kinematics (
  const Robot_Forward_Kinematics_Model& model,
  const Robot_Joint_State& joint_state)
{
  Robot_Forward_Kinematics_Result result;
  std::array<Matrix4, 6> motion_after_joint = { };
  Matrix4 chain = identity_matrix ( );
  for( std::size_t i = 0; i < motion_after_joint.size ( ); ++i )
  {
    const auto pivot_world = Transform_Position (chain, model.joint_pivots[i]);
    const auto axis_world = normalized_axis (
      transform_vector (chain, model.joint_axes[i]));
    result.joint_positions_world[i] = pivot_world;
    result.joint_axes_world[i] = axis_world;
    const auto joint_motion = rotation_about_point (
      joint_delta_at (joint_state, i), axis_world, pivot_world);
    chain = Multiply_Matrices (joint_motion, chain);
    motion_after_joint[i] = chain;
  }

  result.world_from_parts.reserve (model.neutral_world_from_parts.size ( ));
  for( std::size_t i = 0; i < model.neutral_world_from_parts.size ( ); ++i )
  {
    const Matrix4 motion = i == 0
      ? identity_matrix ( )
      : motion_after_joint[std::min<std::size_t> (i, 6) - 1];
    result.world_from_parts.push_back (
      Multiply_Matrices (motion, model.neutral_world_from_parts[i]));
  }

  result.has_flange = model.has_flange && !result.world_from_parts.empty ( );
  if( result.has_flange )
  {
    result.world_from_flange = Multiply_Matrices (
      result.world_from_parts.back ( ), model.flange_from_last_part);
  }
  return result;
}

} // namespace robot_model

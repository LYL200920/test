#include "robot_kinematics.h"

#include "robot_calibration_builder.h"
#include "transform_utils.h"

#include <algorithm>
#include <array>

namespace robot_model
{
namespace
{
double delta_joint_angle_at (const Robot_Joint_State& joint_state,
                             size_t index)
{
  return index < joint_state.delta_angles_deg.size ( )
    ? joint_state.delta_angles_deg[index]
    : 0.0;
}

std::array<double, 3> part_origin_or (
  const std::vector<Robot_Visual_Part>& parts,
  const Robot_Assembly_Calibration& calibration,
  size_t index,
  const std::array<double, 3>& fallback)
{
  if( index >= parts.size ( ) || index >= calibration.parts.size ( ) )
  {
    return fallback;
  }

  return Transform_Part_Point (calibration.parts[index], { 0.0, 0.0, 0.0 });
}

std::array<double, 3> configured_pivot_or (
  const Robot_Kinematic_Params& params,
  size_t index,
  const std::array<double, 3>& fallback)
{
  if( index >= params.joint_frames.size ( ) ||
      !params.joint_frames[index].has_pivot )
  {
    return fallback;
  }

  return params.joint_frames[index].pivot;
}

std::array<double, 3> configured_axis_or (
  const Robot_Kinematic_Params& params,
  size_t index,
  const std::array<double, 3>& fallback)
{
  if( index >= params.joint_frames.size ( ) ||
      !params.joint_frames[index].has_axis )
  {
    return fallback;
  }

  return params.joint_frames[index].axis;
}
} // namespace

Robot_Part_Transform_Result Compute_Part_Transforms (
  const std::vector<Robot_Visual_Part>& parts,
  const Robot_Assembly_Calibration& calibration,
  const Robot_Kinematic_Params& params,
  const Robot_Joint_State& joint_state)
{
  Robot_Part_Transform_Result result;
  const auto count = std::min (parts.size ( ), calibration.parts.size ( ));
  result.part_matrices.reserve (count);

  const double shoulder_z = link_length_at (params, 0, 0.0);
  const double elbow_z = shoulder_z + link_length_at (params, 1, 0.0);
  const double wrist_x = link_length_at (params, 3, 0.0);
  const std::array<double, 3> base_origin = { 0.0, 0.0, 0.0 };
  const std::array<double, 3> shoulder_fallback = { 0.0, 0.0, shoulder_z };
  const std::array<double, 3> elbow_fallback = { 0.0, 0.0, elbow_z };
  const std::array<double, 3> wrist = { wrist_x, 0.0, elbow_z };
  const auto a2_origin = part_origin_or (parts, calibration, 2, shoulder_fallback);
  const auto a3_origin = part_origin_or (parts, calibration, 3, elbow_fallback);
  const auto a4_origin = part_origin_or (parts, calibration, 4, wrist);
  const auto a5_origin = part_origin_or (parts, calibration, 5, a4_origin);
  const auto a6_origin = part_origin_or (parts, calibration, 6, a5_origin);
  const std::array<std::array<double, 3>, 6> joint_pivots =
  {
    configured_pivot_or (params, 0, base_origin),
    configured_pivot_or (params, 1, a2_origin),
    configured_pivot_or (params, 2, a3_origin),
    configured_pivot_or (params, 3, a4_origin),
    configured_pivot_or (params, 4, a5_origin),
    configured_pivot_or (params, 5, a6_origin)
  };
  const std::array<std::array<double, 3>, 6> joint_axes =
  {
    configured_axis_or (params, 0, { 0.0, 0.0, 1.0 }),
    configured_axis_or (params, 1, { 0.0, 1.0, 0.0 }),
    configured_axis_or (params, 2, { 0.0, 1.0, 0.0 }),
    configured_axis_or (params, 3, { 1.0, 0.0, 0.0 }),
    configured_axis_or (params, 4, { 0.0, 1.0, 0.0 }),
    configured_axis_or (params, 5, { 1.0, 0.0, 0.0 })
  };

  for( size_t i = 0; i < count; ++i )
  {
    auto chain = vtkSmartPointer<vtkMatrix4x4>::New ( );
    chain->Identity ( );

    const size_t joint_count = std::min (i, joint_axes.size ( ));
    for( size_t joint_index = 0; joint_index < joint_count; ++joint_index )
    {
      const auto pivot = Transform_Point (chain, joint_pivots[joint_index]);
      const auto axis = Transform_Vector (chain, joint_axes[joint_index]);
      auto joint_matrix = Build_Rotation_About_Point_Matrix (
        delta_joint_angle_at (joint_state, joint_index),
        axis,
        pivot);
      Prepend_Matrix (chain, joint_matrix);
    }

    auto static_matrix = Build_Static_Part_Matrix (calibration.parts[i]);
    auto final_matrix = vtkSmartPointer<vtkMatrix4x4>::New ( );
    vtkMatrix4x4::Multiply4x4 (chain, static_matrix, final_matrix);
    result.part_matrices.push_back (final_matrix);
  }

  return result;
}

} // namespace robot_model

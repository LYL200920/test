#include "robot_kinematics.h"

#include "robot_forward_kinematics.h"

namespace robot_model
{

Robot_Part_Transform_Result Compute_Part_Transforms (
  const std::vector<Robot_Visual_Part>& parts,
  const Robot_Assembly_Calibration& calibration,
  const Robot_Kinematic_Params& params,
  const Robot_Joint_State& joint_state)
{
  const auto model = Build_Forward_Kinematics_Model (
    parts, calibration, params);
  const auto forward = Compute_Forward_Kinematics (model, joint_state);

  Robot_Part_Transform_Result result;
  result.part_matrices.reserve (forward.world_from_parts.size ( ));
  for( const auto& source : forward.world_from_parts )
  {
    auto matrix = vtkSmartPointer<vtkMatrix4x4>::New ( );
    for( int row = 0; row < 4; ++row )
    {
      for( int column = 0; column < 4; ++column )
      {
        matrix->SetElement (row, column, source[row][column]);
      }
    }
    result.part_matrices.push_back (matrix);
  }
  result.world_from_flange = forward.world_from_flange;
  result.has_flange = forward.has_flange;
  return result;
}

} // namespace robot_model

#ifndef includeguard_robot_kinematics_h_includeguard
#define includeguard_robot_kinematics_h_includeguard

#include "robot_model_data.h"
#include "pose_transform.h"

#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

#include <vector>

namespace robot_model
{

struct Robot_Part_Transform_Result
{
  std::vector<vtkSmartPointer<vtkMatrix4x4>> part_matrices;
  Matrix4 world_from_flange = { };
  bool has_flange = false;
};

Robot_Part_Transform_Result Compute_Part_Transforms (
  const std::vector<Robot_Visual_Part>& parts,
  const Robot_Assembly_Calibration& calibration,
  const Robot_Kinematic_Params& params,
  const Robot_Joint_State& joint_state);

} // namespace robot_model

#endif

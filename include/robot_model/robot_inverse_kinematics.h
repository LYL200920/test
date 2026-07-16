#ifndef includeguard_robot_inverse_kinematics_h_includeguard
#define includeguard_robot_inverse_kinematics_h_includeguard

#include "robot_forward_kinematics.h"

#include <cstddef>

namespace robot_model
{

enum class Robot_IK_Status
{
  Converged,
  Maximum_Iterations,
  Invalid_Model
};

struct Robot_Position_IK_Options
{
  double position_tolerance_mm = 1.0;
  double damping_mm = 20.0;
  double max_joint_step_deg = 5.0;
  std::size_t max_iterations = 120;
};

struct Robot_Position_IK_Result
{
  Robot_IK_Status status = Robot_IK_Status::Invalid_Model;
  Robot_Joint_State joint_state;
  Point3 achieved_position_world = { };
  double position_error_mm = 0.0;
  std::size_t iterations = 0;

  bool Converged ( ) const { return status == Robot_IK_Status::Converged; }
};

struct Robot_Pose_IK_Options
{
  double position_tolerance_mm = 1.0;
  double orientation_tolerance_deg = 0.5;
  double orientation_weight_mm = 300.0;
  double damping_mm = 20.0;
  double max_joint_step_deg = 5.0;
  std::size_t max_iterations = 160;
};

struct Robot_Pose_IK_Result
{
  Robot_IK_Status status = Robot_IK_Status::Invalid_Model;
  Robot_Joint_State joint_state;
  Matrix4 achieved_world_from_flange = { };
  double position_error_mm = 0.0;
  double orientation_error_deg = 0.0;
  std::size_t iterations = 0;

  bool Converged ( ) const { return status == Robot_IK_Status::Converged; }
};

// Solves flange position only. Orientation remains free and is handled by a
// future pose-IK layer without coupling the solver to rendering or UI code.
Robot_Position_IK_Result Solve_Flange_Position_IK (
  const Robot_Forward_Kinematics_Model& model,
  const Robot_Kinematic_Params& params,
  const Robot_Joint_State& initial_state,
  const Point3& target_position_world,
  const Robot_Position_IK_Options& options = { });

Robot_Pose_IK_Result Solve_Flange_Pose_IK (
  const Robot_Forward_Kinematics_Model& model,
  const Robot_Kinematic_Params& params,
  const Robot_Joint_State& initial_state,
  const Matrix4& target_world_from_flange,
  const Robot_Pose_IK_Options& options = { });

} // namespace robot_model

#endif

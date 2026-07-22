#ifndef includeguard_robot_joint_sweep_h_includeguard
#define includeguard_robot_joint_sweep_h_includeguard

#include "robot_model_data.h"

#include <cstddef>
#include <array>

namespace robot_model
{

Robot_Joint_State Interpolate_Robot_Joint_State (
  const Robot_Joint_State& start,
  const Robot_Joint_State& target,
  double t);

std::size_t Calculate_Robot_Joint_Sweep_Step_Count (
  const Robot_Joint_State& start,
  const Robot_Joint_State& target,
  double maximum_joint_step_deg);

std::size_t Calculate_Robot_Joint_Sweep_Step_Count (
  const Robot_Joint_State& start,
  const Robot_Joint_State& target,
  double maximum_joint_step_deg,
  const std::array<double, 6>& joint_influence_radii_mm,
  double maximum_spatial_step_mm);

} // namespace robot_model

#endif

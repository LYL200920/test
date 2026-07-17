#include "robot_forward_kinematics.h"
#include "robot_inverse_kinematics.h"
#include "robot_kinematic_params.h"
#include "robot_part_transform.h"

#include <cmath>

int main ( )
{
  robot_model::Robot_Part_Calibration calibration;
  calibration.translate[0] = 25.0;
  const auto matrix = robot_model::Build_Part_Calibration_Matrix (calibration);
  const auto point = robot_model::Transform_Position (
    matrix, { 5.0, 0.0, 0.0 });
  if( std::abs (point[0] - 30.0) > 1.0e-9 ) return 1;

  robot_model::Robot_Kinematic_Params params;
  params.link_lengths = { 100.0 };
  if( robot_model::link_length_at (params, 0, 0.0) != 100.0 ) return 2;

  robot_model::Robot_Forward_Kinematics_Model model;
  robot_model::Robot_Joint_State state;
  const auto result = robot_model::Compute_Forward_Kinematics (model, state);
  return result.has_flange ? 3 : 0;
}

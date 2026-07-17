#ifndef includeguard_robot_calibration_builder_h_includeguard
#define includeguard_robot_calibration_builder_h_includeguard

#include "robot_model_data.h"
#include "robot_visual_data.h"
#include <string>

namespace robot_model
{

Robot_Assembly_Calibration Build_Assembly_Calibration (
  const Robot_Kinematic_Params& params, const std::string& model_name,
  const std::vector<Robot_Visual_Part>& parts);

} // namespace robot_model

#endif

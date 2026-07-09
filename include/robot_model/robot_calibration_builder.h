#ifndef includeguard_robot_calibration_builder_h_includeguard
#define includeguard_robot_calibration_builder_h_includeguard

#include "robot_model_data.h"
#include <string>

namespace robot_model
{

double link_length_at (const Robot_Kinematic_Params& params, size_t index,
                       double fallback);

Robot_Assembly_Calibration Build_Assembly_Calibration (
  const Robot_Kinematic_Params& params, const std::string& model_name,
  const std::vector<Robot_Visual_Part>& parts);

} // namespace robot_model

#endif

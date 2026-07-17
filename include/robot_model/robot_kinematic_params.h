#ifndef includeguard_robot_kinematic_params_h_includeguard
#define includeguard_robot_kinematic_params_h_includeguard

#include "robot_model_data.h"

#include <cstddef>

namespace robot_model
{

double link_length_at (const Robot_Kinematic_Params& params,
                       std::size_t index,
                       double fallback);

} // namespace robot_model

#endif

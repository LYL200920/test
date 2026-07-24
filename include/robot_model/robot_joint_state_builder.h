#ifndef includeguard_robot_joint_state_builder_h_includeguard
#define includeguard_robot_joint_state_builder_h_includeguard

#include "robot_model_data.h"

#include <array>
#include <cstddef>

namespace robot_model
{

  double Joint_Offset_At(const Robot_Kinematic_Params &params, size_t index);
  double Joint_Direction_At(const Robot_Kinematic_Params &params, size_t index);
  double Neutral_Joint_Input_At(const Robot_Kinematic_Params &params,
                                size_t index);

  Robot_Joint_State Build_Joint_State_From_Input_Angles(const Robot_Kinematic_Params &params, const std::array<double, 6> &input_angles_deg);

} // namespace robot_model

#endif

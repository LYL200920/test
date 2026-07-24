#include "robot_joint_state_builder.h"

#include <cmath>

namespace robot_model
{

  double Joint_Offset_At(const Robot_Kinematic_Params &params, size_t index)
  {
    return index < params.joint_offsets.size() ? params.joint_offsets[index]
                                               : 0.0;
  }

  double Joint_Direction_At(const Robot_Kinematic_Params &params, size_t index)
  {
    if (index >= params.joint_directions.size() ||
        params.joint_directions[index] == 0)
    {
      return 1.0;
    }

    return static_cast<double>(params.joint_directions[index]);
  }

  double Neutral_Joint_Input_At(const Robot_Kinematic_Params &params, size_t index)
  {
    const double direction = Joint_Direction_At(params, index);
    if (std::abs(direction) < 1.0e-9)
    {
      return 0.0;
    }

    return -Joint_Offset_At(params, index) / direction;
  }

  Robot_Joint_State Build_Joint_State_From_Input_Angles(const Robot_Kinematic_Params &params,
                                                        const std::array<double, 6> &input_angles_deg)
  {
    Robot_Joint_State joint_state;

    for (size_t i = 0; i < input_angles_deg.size(); ++i)
    {
      const double input_angle = input_angles_deg[i];
      const double direction = Joint_Direction_At(params, i);
      const double effective_angle = direction * input_angle + Joint_Offset_At(params, i);
      const double delta_angle = direction * (input_angle - Neutral_Joint_Input_At(params, i));

      joint_state.input_angles_deg[i] = input_angle;
      joint_state.effective_angles_deg[i] = effective_angle;
      joint_state.delta_angles_deg[i] = delta_angle;
    }

    return joint_state;
  }

} // namespace robot_model

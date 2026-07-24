#include "robot_joint_sweep.h"

#include <algorithm>
#include <cmath>

namespace robot_model
{

  Robot_Joint_State Interpolate_Robot_Joint_State(
      const Robot_Joint_State &start,
      const Robot_Joint_State &target,
      double t)
  {
    t = std::clamp(t, 0.0, 1.0);
    Robot_Joint_State state;
    for (std::size_t joint = 0; joint < state.input_angles_deg.size(); ++joint)
    {
      state.input_angles_deg[joint] = start.input_angles_deg[joint] +
                                      (target.input_angles_deg[joint] - start.input_angles_deg[joint]) * t;
      state.effective_angles_deg[joint] = start.effective_angles_deg[joint] +
                                          (target.effective_angles_deg[joint] -
                                           start.effective_angles_deg[joint]) *
                                              t;
      state.delta_angles_deg[joint] = start.delta_angles_deg[joint] +
                                      (target.delta_angles_deg[joint] - start.delta_angles_deg[joint]) * t;
    }
    return state;
  }

  std::size_t Calculate_Robot_Joint_Sweep_Step_Count(const Robot_Joint_State &start,
                                                     const Robot_Joint_State &target,
                                                     double maximum_joint_step_deg)
  {
    if (maximum_joint_step_deg <= 0.0 || !std::isfinite(maximum_joint_step_deg))
    {
      return 1;
    }
    double maximum_delta_deg = 0.0;
    for (std::size_t joint = 0; joint < start.input_angles_deg.size(); ++joint)
    {
      maximum_delta_deg = std::max(maximum_delta_deg,
                                   std::abs(target.input_angles_deg[joint] -
                                            start.input_angles_deg[joint]));
    }
    return std::max<std::size_t>(1, static_cast<std::size_t>(
                                        std::ceil(maximum_delta_deg / maximum_joint_step_deg)));
  }

  std::size_t Calculate_Robot_Joint_Sweep_Step_Count(const Robot_Joint_State &start,
                                                     const Robot_Joint_State &target,
                                                     double maximum_joint_step_deg,
                                                     const std::array<double, 6> &joint_influence_radii_mm,
                                                     double maximum_spatial_step_mm)
  {
    const auto angular_steps = Calculate_Robot_Joint_Sweep_Step_Count(start, target, maximum_joint_step_deg);
    if (maximum_spatial_step_mm <= 0.0 || !std::isfinite(maximum_spatial_step_mm))
    {
      return angular_steps;
    }

    constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
    double motion_bound_mm = 0.0;
    for (std::size_t joint = 0; joint < start.delta_angles_deg.size(); ++joint)
    {
      const double delta_radians = std::abs(target.delta_angles_deg[joint] - start.delta_angles_deg[joint]) * degrees_to_radians;
      const double radius = std::max(0.0, joint_influence_radii_mm[joint]);
      motion_bound_mm += delta_radians * radius;
    }
    const auto spatial_steps = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(motion_bound_mm / maximum_spatial_step_mm)));
    return std::max(angular_steps, spatial_steps);
  }

} // namespace robot_model

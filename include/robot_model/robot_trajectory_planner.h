#ifndef includeguard_robot_trajectory_planner_h_includeguard
#define includeguard_robot_trajectory_planner_h_includeguard

#include <array>
#include <cstddef>
#include <vector>

namespace robot_model
{

  using Robot_Joint_Trajectory_Point = std::array<double, 6>;
  using Robot_Joint_Trajectory = std::vector<Robot_Joint_Trajectory_Point>;

  Robot_Joint_Trajectory Build_Joint_Ptp_Trajectory(const Robot_Joint_Trajectory_Point &start_angles_deg,
                                                    const Robot_Joint_Trajectory_Point &target_angles_deg,
                                                    size_t frame_count);

  Robot_Joint_Trajectory Build_Multi_Point_Joint_Ptp_Trajectory(const Robot_Joint_Trajectory &points, size_t frame_count_per_segment);

} // namespace robot_model

#endif

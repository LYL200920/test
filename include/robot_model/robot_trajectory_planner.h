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
  Robot_Joint_Trajectory Build_Multi_Point_Joint_Ptp_Trajectory(
    const Robot_Joint_Trajectory &points,
    const std::vector<size_t> &frame_counts_per_segment);
  size_t Find_Matching_Joint_Point_Index(
    const Robot_Joint_Trajectory &points,
    const Robot_Joint_Trajectory_Point &current_angles_deg,
    double joint_tolerance_deg = 0.05);
  std::vector<size_t> Build_Ordered_Go_To_Point_Indices(
    const Robot_Joint_Trajectory &points,
    const Robot_Joint_Trajectory_Point &current_angles_deg,
    size_t target_index,
    double joint_tolerance_deg = 0.05);

} // namespace robot_model

#endif

#include "robot_trajectory_planner.h"

#include <algorithm>
#include <cmath>

namespace robot_model
{

  Robot_Joint_Trajectory Build_Joint_Ptp_Trajectory(const Robot_Joint_Trajectory_Point &start_angles_deg,
                                                    const Robot_Joint_Trajectory_Point &target_angles_deg,
                                                    size_t frame_count)
  {
    frame_count = std::max<size_t>(frame_count, 2);

    Robot_Joint_Trajectory frames;
    frames.reserve(frame_count);

    for (size_t frame_index = 0; frame_index < frame_count; ++frame_index)
    {
      const double t = static_cast<double>(frame_index) /
                       static_cast<double>(frame_count - 1);
      Robot_Joint_Trajectory_Point frame = {};
      for (size_t joint_index = 0; joint_index < frame.size(); ++joint_index)
      {
        frame[joint_index] = start_angles_deg[joint_index] +
                             (target_angles_deg[joint_index] - start_angles_deg[joint_index]) * t;
      }
      frames.push_back(frame);
    }

    return frames;
  }

  Robot_Joint_Trajectory Build_Multi_Point_Joint_Ptp_Trajectory(const Robot_Joint_Trajectory &points,
                                                                size_t frame_count_per_segment)
  {
    Robot_Joint_Trajectory frames;
    if (points.size() < 2)
    {
      return frames;
    }

    const size_t segment_count = points.size() - 1;
    frames.reserve(segment_count * frame_count_per_segment);

    for (size_t point_index = 0; point_index < segment_count; ++point_index)
    {
      auto segment_frames = Build_Joint_Ptp_Trajectory(points[point_index],
                                                       points[point_index + 1],
                                                       frame_count_per_segment);
      const size_t first_frame_index = point_index == 0 ? 0 : 1;
      for (size_t frame_index = first_frame_index;
           frame_index < segment_frames.size();
           ++frame_index)
      {
        frames.push_back(segment_frames[frame_index]);
      }
    }

    return frames;
  }

  Robot_Joint_Trajectory Build_Multi_Point_Joint_Ptp_Trajectory(
      const Robot_Joint_Trajectory &points,
      const std::vector<size_t> &frame_counts_per_segment)
  {
    Robot_Joint_Trajectory frames;
    if (points.size() < 2 ||
        frame_counts_per_segment.size() != points.size() - 1)
    {
      return frames;
    }
    for (size_t point_index = 0;
         point_index + 1 < points.size();
         ++point_index)
    {
      auto segment_frames = Build_Joint_Ptp_Trajectory(
        points[point_index],
        points[point_index + 1],
        frame_counts_per_segment[point_index]);
      const size_t first_frame_index = point_index == 0 ? 0 : 1;
      frames.insert(
        frames.end(),
        segment_frames.begin() +
          static_cast<std::ptrdiff_t>(first_frame_index),
        segment_frames.end());
    }
    return frames;
  }

  std::vector<size_t> Build_Ordered_Go_To_Point_Indices(
    const Robot_Joint_Trajectory &points,
    const Robot_Joint_Trajectory_Point &current_angles_deg,
    size_t target_index,
    double joint_tolerance_deg)
  {
    std::vector<size_t> indices;
    if (target_index >= points.size() ||
        !std::isfinite(joint_tolerance_deg) ||
        joint_tolerance_deg < 0.0)
    {
      return indices;
    }

    size_t current_index = points.size();
    for (size_t point_index = 0; point_index < points.size(); ++point_index)
    {
      bool matches = true;
      for (size_t joint_index = 0;
           joint_index < current_angles_deg.size();
           ++joint_index)
      {
        if (!std::isfinite(current_angles_deg[joint_index]) ||
            !std::isfinite(points[point_index][joint_index]) ||
            std::abs(current_angles_deg[joint_index] -
                     points[point_index][joint_index]) >
              joint_tolerance_deg)
        {
          matches = false;
          break;
        }
      }
      if (matches)
      {
        current_index = point_index;
        break;
      }
    }

    const size_t first_index =
      current_index <= target_index ? current_index + 1 : 0;
    for (size_t point_index = first_index;
         point_index <= target_index;
         ++point_index)
    {
      indices.push_back(point_index);
    }
    return indices;
  }

} // namespace robot_model

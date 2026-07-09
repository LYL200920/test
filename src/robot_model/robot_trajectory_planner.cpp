#include "robot_trajectory_planner.h"

#include <algorithm>

namespace robot_model
{

Robot_Joint_Trajectory Build_Joint_Ptp_Trajectory (
  const Robot_Joint_Trajectory_Point& start_angles_deg,
  const Robot_Joint_Trajectory_Point& target_angles_deg,
  size_t frame_count)
{
  frame_count = std::max<size_t> (frame_count, 2);

  Robot_Joint_Trajectory frames;
  frames.reserve (frame_count);

  for( size_t frame_index = 0; frame_index < frame_count; ++frame_index )
  {
    const double t = static_cast<double> (frame_index) /
                     static_cast<double> (frame_count - 1);
    Robot_Joint_Trajectory_Point frame = {};
    for( size_t joint_index = 0; joint_index < frame.size ( ); ++joint_index )
    {
      frame[joint_index] =
        start_angles_deg[joint_index] +
        ( target_angles_deg[joint_index] - start_angles_deg[joint_index] ) * t;
    }
    frames.push_back (frame);
  }

  return frames;
}

Robot_Joint_Trajectory Build_Multi_Point_Joint_Ptp_Trajectory (
  const Robot_Joint_Trajectory& points,
  size_t frame_count_per_segment)
{
  Robot_Joint_Trajectory frames;
  if( points.size ( ) < 2 )
  {
    return frames;
  }

  const size_t segment_count = points.size ( ) - 1;
  frames.reserve (segment_count * frame_count_per_segment);

  for( size_t point_index = 0; point_index < segment_count; ++point_index )
  {
    auto segment_frames = Build_Joint_Ptp_Trajectory (
      points[point_index],
      points[point_index + 1],
      frame_count_per_segment);
    const size_t first_frame_index = point_index == 0 ? 0 : 1;
    for( size_t frame_index = first_frame_index;
         frame_index < segment_frames.size ( );
         ++frame_index )
    {
      frames.push_back (segment_frames[frame_index]);
    }
  }

  return frames;
}

} // namespace robot_model

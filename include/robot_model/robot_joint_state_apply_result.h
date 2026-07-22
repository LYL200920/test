#ifndef includeguard_robot_joint_state_apply_result_h_includeguard
#define includeguard_robot_joint_state_apply_result_h_includeguard

#include "robot_collision_detector.h"

#include <cstddef>

namespace robot_model
{

struct Robot_Joint_State_Apply_Result
{
  bool accepted = false;
  bool state_changed = false;
  bool scene_changed = false;
  bool collision_checked = false;
  bool recovery_motion = false;
  double clearance_mm = 0.0;
  std::size_t checked_pose_count = 0;
  double collision_query_time_ms = 0.0;
  Robot_Collision_Query_Stats collision_query_stats;
  Robot_Collision_Result collision;
};

} // namespace robot_model

#endif

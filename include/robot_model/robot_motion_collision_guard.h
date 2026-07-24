#ifndef includeguard_robot_motion_collision_guard_h_includeguard
#define includeguard_robot_motion_collision_guard_h_includeguard

#include "robot_collision_detector.h"
#include "robot_forward_kinematics.h"
#include "robot_model_data.h"
#include "robot_visual_data.h"

#include <cstddef>
#include <vector>

namespace robot_model
{

  struct Robot_Motion_Collision_Options
  {
    double clearance_mm = 10.0;
    double maximum_joint_step_deg = 2.0;
    double maximum_spatial_step_mm = 5.0;
    std::size_t boundary_refinement_iterations = 4;
    // Zero means no per-update budget. Interactive motion uses a finite budget
    // while trajectory and slider motion retain a complete sweep.
    std::size_t maximum_sweep_pose_count = 0;
  };

  struct Robot_Motion_Collision_Request
  {
    Robot_Joint_State current_state;
    Robot_Joint_State requested_state;
    Robot_Collision_Result current_collision;
    Robot_Motion_Collision_Options options;
  };

  struct Robot_Motion_Collision_Result
  {
    bool accepted = false;
    bool state_changed = false;
    bool recovery_motion = false;
    Robot_Joint_State applied_state;
    Robot_Forward_Kinematics_Result applied_transforms;
    Robot_Collision_Result resulting_collision;
    Robot_Collision_Result blocking_collision;
    std::size_t checked_pose_count = 0;
    double collision_query_time_ms = 0.0;
    Robot_Collision_Query_Stats collision_query_stats;
  };

  // Pure motion-safety policy. It evaluates a requested joint transition but
  // never mutates robot state, VTK actors, or collision visualization.
  class Robot_Motion_Collision_Guard
  {
  public:
    Robot_Motion_Collision_Result Evaluate(const Robot_Forward_Kinematics_Model &forward_model,
                                           const std::vector<Robot_Visual_Part> &robot_parts,
                                           Robot_Collision_Detector &collision_detector,
                                           const Robot_Motion_Collision_Request &request) const;
  };

} // namespace robot_model

#endif

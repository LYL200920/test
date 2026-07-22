#ifndef includeguard_flange_drag_motion_executor_h_includeguard
#define includeguard_flange_drag_motion_executor_h_includeguard

#include "flange_interaction_controller.h"
#include "robot_inverse_kinematics.h"
#include "robot_joint_state_apply_result.h"

#include <functional>

namespace robot_model
{

struct Flange_Position_Motion_Outcome
{
  Robot_Position_IK_Result ik_result;
  Robot_Joint_State_Apply_Result apply_result;
};

struct Flange_Pose_Motion_Outcome
{
  Robot_Pose_IK_Result ik_result;
  Robot_Joint_State_Apply_Result apply_result;
};

struct Flange_Drag_Motion_Execution
{
  Flange_Drag_Update::Target_Type target_type =
    Flange_Drag_Update::Target_Type::None;
  Robot_Position_IK_Result position_result;
  Robot_Pose_IK_Result pose_result;
  Robot_Joint_State_Apply_Result apply_result;
  double ik_time_ms = 0.0;
  bool handled = false;
  bool pose_changed = false;
  bool notify_result = false;
};

// Converts a drag target into one motion request and normalizes the result for
// the view. Injected movers keep IK and collision implementation out of this
// interaction policy and make both branches independently testable.
class Flange_Drag_Motion_Executor
{
public:
  using Position_Mover = std::function<Flange_Position_Motion_Outcome (
    const Point3&)>;
  using Pose_Mover = std::function<Flange_Pose_Motion_Outcome (
    const Matrix4&)>;

  Flange_Drag_Motion_Execution Execute (
    const Flange_Drag_Update& update,
    const Position_Mover& move_position,
    const Pose_Mover& move_pose) const;
};

} // namespace robot_model

#endif

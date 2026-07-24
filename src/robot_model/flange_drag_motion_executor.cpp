#include "flange_drag_motion_executor.h"

#include <utility>

namespace robot_model
{

  Flange_Drag_Motion_Execution Flange_Drag_Motion_Executor::Execute(const Flange_Drag_Update &update,
                                                                    const Position_Mover &move_position,
                                                                    const Pose_Mover &move_pose) const
  {
    Flange_Drag_Motion_Execution execution;
    execution.target_type = update.target_type;

    if (update.target_type == Flange_Drag_Update::Target_Type::Position)
    {
      if (!move_position)
        return execution;
      auto outcome = move_position(update.target_position_world);
      execution.position_result = std::move(outcome.ik_result);
      execution.apply_result = std::move(outcome.apply_result);
      execution.ik_time_ms = execution.position_result.solve_time_ms;
      execution.handled = true;
    }
    else if (update.target_type == Flange_Drag_Update::Target_Type::Pose)
    {
      if (!move_pose)
        return execution;
      auto outcome = move_pose(update.target_world_from_flange);
      execution.pose_result = std::move(outcome.ik_result);
      execution.apply_result = std::move(outcome.apply_result);
      execution.ik_time_ms = execution.pose_result.solve_time_ms;
      execution.handled = true;
    }
    else
    {
      return execution;
    }

    execution.pose_changed = execution.apply_result.state_changed || execution.apply_result.scene_changed;
    execution.notify_result = execution.pose_changed ||
                              execution.apply_result.accepted ||
                              !execution.apply_result.collision.collided;
    return execution;
  }

} // namespace robot_model

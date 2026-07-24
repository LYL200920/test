#include "robot_motion_collision_guard.h"

#include "pose_transform.h"
#include "robot_joint_sweep.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace robot_model
{
  namespace
  {

    std::array<double, 6> collision_influence_radii(const std::vector<Robot_Visual_Part> &parts,
                                                    const Robot_Forward_Kinematics_Result &start,
                                                    const Robot_Forward_Kinematics_Result &target)
    {
      std::array<double, 6> radii = {};
      const auto accumulate_pose = [&](const Robot_Forward_Kinematics_Result &pose)
      {
        for (std::size_t joint = 0; joint < pose.joint_positions_world.size(); ++joint)
        {
          const auto &pivot = pose.joint_positions_world[joint];
          for (std::size_t part_index = joint + 1;
               part_index < parts.size() && part_index < pose.world_from_parts.size();
               ++part_index)
          {
            const auto &part = parts[part_index];
            if (!part.has_raw_bounds)
              continue;
            for (int x_side = 0; x_side < 2; ++x_side)
            {
              for (int y_side = 0; y_side < 2; ++y_side)
              {
                for (int z_side = 0; z_side < 2; ++z_side)
                {
                  const auto corner = Transform_Position(pose.world_from_parts[part_index],
                                                         {part.raw_bounds[x_side],
                                                          part.raw_bounds[2 + y_side],
                                                          part.raw_bounds[4 + z_side]});
                  const double dx = corner[0] - pivot[0];
                  const double dy = corner[1] - pivot[1];
                  const double dz = corner[2] - pivot[2];
                  radii[joint] = std::max(radii[joint], std::sqrt(dx * dx + dy * dy + dz * dz));
                }
              }
            }
          }
        }
      };
      accumulate_pose(start);
      accumulate_pose(target);
      return radii;
    }

    void accumulate_query_stats(const Robot_Collision_Query_Stats &source, Robot_Collision_Query_Stats *target)
    {
      if (!target)
        return;
      target->ground_sample_queries += source.ground_sample_queries;
      target->self_broad_phase_pairs += source.self_broad_phase_pairs;
      target->self_obb_phase_pairs += source.self_obb_phase_pairs;
      target->self_distance_sample_queries += source.self_distance_sample_queries;
      target->self_exact_pair_queries += source.self_exact_pair_queries;
      target->obstacle_candidate_points += source.obstacle_candidate_points;
      target->obstacle_distance_queries += source.obstacle_distance_queries;
      if (source.minimum_self_sample_distance_mm > 0.0 && std::isfinite(source.minimum_self_sample_distance_mm))
      {
        if (target->minimum_self_sample_distance_mm <= 0.0)
          target->minimum_self_sample_distance_mm = source.minimum_self_sample_distance_mm;
        else
          target->minimum_self_sample_distance_mm = std::min(target->minimum_self_sample_distance_mm,
                                                             source.minimum_self_sample_distance_mm);
      }
    }

  } // namespace

  Robot_Motion_Collision_Result Robot_Motion_Collision_Guard::Evaluate(const Robot_Forward_Kinematics_Model &forward_model,
                                                                       const std::vector<Robot_Visual_Part> &robot_parts,
                                                                       Robot_Collision_Detector &collision_detector,
                                                                       const Robot_Motion_Collision_Request &request) const
  {
    Robot_Motion_Collision_Result result;
    result.applied_state = request.current_state;
    result.resulting_collision = request.current_collision;
    if (!forward_model.has_flange ||
        !collision_detector.Has_Robot_Geometry() ||
        !std::isfinite(request.options.clearance_mm) ||
        request.options.clearance_mm < 0.0)
    {
      return result;
    }
    const auto start_transforms = Compute_Forward_Kinematics(forward_model, request.current_state);
    result.applied_transforms = start_transforms;

    auto target_state = request.requested_state;
    auto target_transforms = Compute_Forward_Kinematics(forward_model, target_state);
    auto influence_radii = collision_influence_radii(robot_parts, start_transforms, target_transforms);
    auto step_count = Calculate_Robot_Joint_Sweep_Step_Count(request.current_state, target_state,
                                                             request.options.maximum_joint_step_deg,
                                                             influence_radii,
                                                             request.options.maximum_spatial_step_mm);
    for (int limiting_pass = 0;
         request.options.maximum_sweep_pose_count > 0 && step_count > request.options.maximum_sweep_pose_count && limiting_pass < 8;
         ++limiting_pass)
    {
      const double fraction = static_cast<double>(request.options.maximum_sweep_pose_count) /
                              static_cast<double>(step_count);
      target_state = Interpolate_Robot_Joint_State(request.current_state, target_state, fraction);
      target_transforms = Compute_Forward_Kinematics(forward_model, target_state);
      influence_radii = collision_influence_radii(robot_parts, start_transforms, target_transforms);
      step_count = Calculate_Robot_Joint_Sweep_Step_Count(request.current_state, target_state,
                                                          request.options.maximum_joint_step_deg,
                                                          influence_radii, request.options.maximum_spatial_step_mm);
    }

    const auto check_collision = [&](const std::vector<Matrix4> &world_from_parts)
    {
      const auto started_at = std::chrono::steady_clock::now();
      auto collision = collision_detector.Check_Pose(world_from_parts, request.options.clearance_mm);
      result.collision_query_time_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started_at).count();
      accumulate_query_stats(collision_detector.Last_Query_Stats(),
                             &result.collision_query_stats);
      ++result.checked_pose_count;
      return collision;
    };

    Robot_Joint_State last_allowed_state = request.current_state;
    Robot_Forward_Kinematics_Result last_allowed_transforms = start_transforms;
    Robot_Collision_Result last_allowed_collision = request.current_collision;
    bool has_intermediate_allowed_state = false;
    bool recovering = request.current_collision.collided;
    double recovery_margin = request.current_collision.clearance_margin_mm;
    for (std::size_t step = 1; step <= step_count; ++step)
    {
      const double t = static_cast<double>(step) / static_cast<double>(step_count);
      const auto sample_state = step == step_count ? target_state
                                                   : Interpolate_Robot_Joint_State(request.current_state, target_state, t);
      const auto sample_transforms = step == step_count ? target_transforms
                                                        : Compute_Forward_Kinematics(forward_model, sample_state);
      result.blocking_collision = check_collision(sample_transforms.world_from_parts);
      if (result.blocking_collision.collided)
      {
        const bool improves_current_collision = recovering &&
                                                Is_Robot_Collision_Recovery_Improvement(request.current_collision,
                                                                                        result.blocking_collision,
                                                                                        recovery_margin);
        if (improves_current_collision)
        {
          recovery_margin = result.blocking_collision.clearance_margin_mm;
          last_allowed_state = sample_state;
          last_allowed_transforms = sample_transforms;
          last_allowed_collision = result.blocking_collision;
          has_intermediate_allowed_state = true;
          result.recovery_motion = true;
          continue;
        }

        if (recovering)
        {
          if (has_intermediate_allowed_state)
          {
            result.applied_state = last_allowed_state;
            result.applied_transforms = last_allowed_transforms;
            result.resulting_collision = last_allowed_collision;
            result.state_changed = true;
            result.recovery_motion = true;
          }
          return result;
        }

        if (has_intermediate_allowed_state)
        {
          auto refined_safe_state = last_allowed_state;
          auto refined_safe_transforms = last_allowed_transforms;
          auto refined_collision_state = sample_state;
          for (std::size_t refinement = 0;
               refinement < request.options.boundary_refinement_iterations;
               ++refinement)
          {
            const auto midpoint_state = Interpolate_Robot_Joint_State(refined_safe_state,
                                                                      refined_collision_state,
                                                                      0.5);
            const auto midpoint_transforms = Compute_Forward_Kinematics(forward_model, midpoint_state);
            const auto midpoint_collision = check_collision(midpoint_transforms.world_from_parts);
            if (midpoint_collision.collided)
            {
              refined_collision_state = midpoint_state;
              result.blocking_collision = midpoint_collision;
            }
            else
            {
              refined_safe_state = midpoint_state;
              refined_safe_transforms = midpoint_transforms;
            }
          }
          result.applied_state = refined_safe_state;
          result.applied_transforms = refined_safe_transforms;
          result.resulting_collision = {};
          result.state_changed = true;
        }
        return result;
      }

      recovering = false;
      last_allowed_state = sample_state;
      last_allowed_transforms = sample_transforms;
      last_allowed_collision = result.blocking_collision;
      has_intermediate_allowed_state = true;
    }

    result.applied_state = last_allowed_state;
    result.applied_transforms = last_allowed_transforms;
    result.resulting_collision = last_allowed_collision;
    result.accepted = true;
    result.state_changed = true;
    return result;
  }

} // namespace robot_model

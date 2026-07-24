#ifndef includeguard_robot_forward_kinematics_h_includeguard
#define includeguard_robot_forward_kinematics_h_includeguard

#include "pose_transform.h"
#include "robot_model_data.h"

#include <array>
#include <cstddef>
#include <vector>

namespace robot_model
{

  // Immutable geometry required by forward and future inverse kinematics.
  // It deliberately contains no VTK actors or UI state.
  struct Robot_Forward_Kinematics_Model
  {
    std::array<Point3, 6> joint_pivots = {};
    std::array<Point3, 6> joint_axes = {};
    std::vector<Matrix4> neutral_world_from_parts;
    Matrix4 flange_from_last_part = {};
    bool has_flange = false;
  };

  struct Robot_Forward_Kinematics_Result
  {
    std::array<Point3, 6> joint_positions_world = {};
    std::array<Point3, 6> joint_axes_world = {};
    std::vector<Matrix4> world_from_parts;
    Matrix4 world_from_flange = {};
    bool has_flange = false;
  };

  Robot_Forward_Kinematics_Model Build_Forward_Kinematics_Model(std::size_t part_count,
                                                                const Robot_Assembly_Calibration &calibration,
                                                                const Robot_Kinematic_Params &params);

  Robot_Forward_Kinematics_Result Compute_Forward_Kinematics(const Robot_Forward_Kinematics_Model &model,
                                                             const Robot_Joint_State &joint_state);

} // namespace robot_model

#endif

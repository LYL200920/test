#ifndef includeguard_robot_teach_point_h_includeguard
#define includeguard_robot_teach_point_h_includeguard

#include "pose_transform.h"

#include <array>
#include <cstddef>
#include <string>

namespace robot_model
{

struct Robot_Teach_Point
{
  std::size_t id = 0;
  std::string robot_model_id;
  std::array<double, 6> joint_angles_deg = { };
  XyzabcPose world_pose = { };
  bool has_world_pose = false;
};

std::string Format_Teach_Point_Name (std::size_t id);

} // namespace robot_model

#endif

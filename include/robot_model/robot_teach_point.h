#ifndef includeguard_robot_teach_point_h_includeguard
#define includeguard_robot_teach_point_h_includeguard

#include "pose_transform.h"

#include <array>
#include <cstddef>
#include <string>

namespace robot_model
{

enum class Robot_Teach_Point_Type
{
  Motion,
  Transition,
  Wait
};

struct Robot_Teach_Point
{
  std::size_t id = 0;
  std::string robot_model_id;
  std::array<double, 6> joint_angles_deg = { };
  XyzabcPose world_pose = { };
  bool has_world_pose = false;
  std::string point_cloud_path;
  std::string point_cloud_name;
  std::string coordinate_frame_id;
  std::string coordinate_frame_name;
  XyzabcPose flange_from_coordinate_pose = { };
  bool has_coordinate_frame = false;
  Robot_Teach_Point_Type type = Robot_Teach_Point_Type::Motion;
};

std::string Format_Teach_Point_Name (std::size_t id);
const char *Robot_Teach_Point_Type_Id(Robot_Teach_Point_Type type);
bool Parse_Robot_Teach_Point_Type(
  const std::string &text,
  Robot_Teach_Point_Type *type);

} // namespace robot_model

#endif

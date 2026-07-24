#ifndef includeguard_tool_coordinate_h_includeguard
#define includeguard_tool_coordinate_h_includeguard

#include "pose_transform.h"

#include <string>
#include <vector>

namespace robot_model
{

struct Tool_Coordinate_Profile
{
  std::string id;
  std::string name;
  XyzabcPose flange_from_tool_pose = {};
};

struct Tool_Coordinate_Configuration
{
  std::vector<Tool_Coordinate_Profile> tools;
  std::string active_tool_id;
};

Tool_Coordinate_Configuration Default_Tool_Coordinate_Configuration();
void Normalize_Tool_Coordinate_Configuration(
  Tool_Coordinate_Configuration *configuration);

const Tool_Coordinate_Profile *Find_Tool_Coordinate(
  const Tool_Coordinate_Configuration &configuration,
  const std::string &tool_id);
Tool_Coordinate_Profile *Find_Tool_Coordinate(
  Tool_Coordinate_Configuration *configuration,
  const std::string &tool_id);

Matrix4 Build_World_From_Tool(
  const Matrix4 &world_from_flange,
  const Tool_Coordinate_Profile &tool);
Matrix4 Build_World_From_Flange_Target(
  const Matrix4 &world_from_tool,
  const Tool_Coordinate_Profile &tool);

} // namespace robot_model

#endif

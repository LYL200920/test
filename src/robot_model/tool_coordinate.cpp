#include "tool_coordinate.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace robot_model
{
namespace
{

constexpr const char *kFlangeToolId = "flange";

Tool_Coordinate_Profile Make_Flange_Tool()
{
  Tool_Coordinate_Profile tool;
  tool.id = kFlangeToolId;
  tool.name = "Flange";
  return tool;
}

bool Is_Valid_Tool(const Tool_Coordinate_Profile &tool)
{
  if (tool.id.empty() || tool.name.empty())
  {
    return false;
  }
  return std::all_of(
    tool.flange_from_tool_pose.begin(),
    tool.flange_from_tool_pose.end(),
    [](double value) { return std::isfinite(value); });
}

}

Tool_Coordinate_Configuration Default_Tool_Coordinate_Configuration()
{
  Tool_Coordinate_Configuration configuration;
  configuration.tools.push_back(Make_Flange_Tool());
  configuration.active_tool_id = kFlangeToolId;
  return configuration;
}

void Normalize_Tool_Coordinate_Configuration(
  Tool_Coordinate_Configuration *configuration)
{
  if (!configuration)
  {
    return;
  }

  std::vector<Tool_Coordinate_Profile> normalized;
  normalized.push_back(Make_Flange_Tool());
  std::unordered_set<std::string> ids = {kFlangeToolId};
  for (Tool_Coordinate_Profile &tool : configuration->tools)
  {
    if (!Is_Valid_Tool(tool) || ids.count(tool.id) != 0)
    {
      continue;
    }
    ids.insert(tool.id);
    normalized.push_back(std::move(tool));
  }
  configuration->tools = std::move(normalized);
  if (ids.count(configuration->active_tool_id) == 0)
  {
    configuration->active_tool_id = kFlangeToolId;
  }
}

const Tool_Coordinate_Profile *Find_Tool_Coordinate(
  const Tool_Coordinate_Configuration &configuration,
  const std::string &tool_id)
{
  const auto found = std::find_if(
    configuration.tools.begin(),
    configuration.tools.end(),
    [&tool_id](const Tool_Coordinate_Profile &tool)
    {
      return tool.id == tool_id;
    });
  return found == configuration.tools.end() ? nullptr : &*found;
}

Tool_Coordinate_Profile *Find_Tool_Coordinate(
  Tool_Coordinate_Configuration *configuration,
  const std::string &tool_id)
{
  if (!configuration)
  {
    return nullptr;
  }
  const auto found = std::find_if(
    configuration->tools.begin(),
    configuration->tools.end(),
    [&tool_id](const Tool_Coordinate_Profile &tool)
    {
      return tool.id == tool_id;
    });
  return found == configuration->tools.end() ? nullptr : &*found;
}

Matrix4 Build_World_From_Tool(
  const Matrix4 &world_from_flange,
  const Tool_Coordinate_Profile &tool)
{
  return Multiply_Matrices(
    world_from_flange,
    Build_Zyx_Pose_Matrix(tool.flange_from_tool_pose));
}

Matrix4 Build_World_From_Flange_Target(
  const Matrix4 &world_from_tool,
  const Tool_Coordinate_Profile &tool)
{
  return Multiply_Matrices(
    world_from_tool,
    Invert_Rigid_Matrix(
      Build_Zyx_Pose_Matrix(tool.flange_from_tool_pose)));
}

} // namespace robot_model

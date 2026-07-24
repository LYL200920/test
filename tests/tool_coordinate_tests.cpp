#include "tool_coordinate.h"
#include "tool_coordinate_repository.h"

#include <cassert>
#include <cmath>
#include <filesystem>

namespace
{

void Require_Near(double actual, double expected)
{
  assert(std::abs(actual - expected) < 1.0e-8);
}

void Test_Tool_Transform_Round_Trip()
{
  robot_model::Tool_Coordinate_Profile tool;
  tool.id = "welder";
  tool.name = "Welder";
  tool.flange_from_tool_pose = {10.0, 20.0, 100.0, 5.0, 10.0, 15.0};
  const robot_model::Matrix4 world_from_flange =
    robot_model::Build_Zyx_Pose_Matrix(
      {500.0, -100.0, 800.0, 20.0, -30.0, 45.0});
  const robot_model::Matrix4 world_from_tool =
    robot_model::Build_World_From_Tool(world_from_flange, tool);
  const robot_model::Matrix4 restored =
    robot_model::Build_World_From_Flange_Target(world_from_tool, tool);
  for (std::size_t row = 0; row < 4; ++row)
  {
    for (std::size_t column = 0; column < 4; ++column)
    {
      Require_Near(restored[row][column], world_from_flange[row][column]);
    }
  }
}

void Test_Repository_Round_Trip()
{
  robot_model::Tool_Coordinate_Configuration configuration =
    robot_model::Default_Tool_Coordinate_Configuration();
  robot_model::Tool_Coordinate_Profile tool;
  tool.id = "camera";
  tool.name = "Camera";
  tool.flange_from_tool_pose = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  configuration.tools.push_back(tool);
  configuration.active_tool_id = tool.id;

  const std::filesystem::path path =
    std::filesystem::temp_directory_path() /
    "codex_tool_coordinate_tests.xml";
  std::string error;
  assert(robot_model::Save_Tool_Coordinate_Configuration(
    path, configuration, &error));

  robot_model::Tool_Coordinate_Configuration loaded;
  assert(robot_model::Load_Tool_Coordinate_Configuration(
    path, &loaded, &error));
  assert(loaded.active_tool_id == "camera");
  const auto *loaded_tool =
    robot_model::Find_Tool_Coordinate(loaded, "camera");
  assert(loaded_tool != nullptr);
  assert(loaded_tool->name == "Camera");
  assert(loaded_tool->flange_from_tool_pose == tool.flange_from_tool_pose);
  std::filesystem::remove(path);
}

}

int main()
{
  Test_Tool_Transform_Round_Trip();
  Test_Repository_Round_Trip();
  return 0;
}

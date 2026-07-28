#include "tool_coordinate.h"
#include "tool_coordinate_repository.h"
#include "tool_visualization.h"
#include "tool_visualization_repository.h"

#include <cassert>
#include <cmath>
#include <filesystem>

namespace
{

void Require_Near(double actual, double expected)
{
  assert(std::abs(actual - expected) < 1.0e-8);
}

void Require_Matrix_Near(const robot_model::Matrix4 &actual,
                         const robot_model::Matrix4 &expected)
{
  for (std::size_t row = 0; row < 4; ++row)
  {
    for (std::size_t column = 0; column < 4; ++column)
    {
      Require_Near(actual[row][column], expected[row][column]);
    }
  }
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
  Require_Matrix_Near(restored, world_from_flange);
}

void Test_Tool_Drag_Target_Conversion()
{
  robot_model::Tool_Coordinate_Profile tool;
  tool.id = "gripper";
  tool.name = "Gripper";
  tool.flange_from_tool_pose = {35.0, -12.0, 180.0, 8.0, -4.0, 22.0};

  const robot_model::Matrix4 target_world_from_tool =
    robot_model::Build_Zyx_Pose_Matrix(
      {720.0, 85.0, 640.0, -35.0, 18.0, 70.0});
  const robot_model::Matrix4 target_world_from_flange =
    robot_model::Build_World_From_Flange_Target(
      target_world_from_tool,
      tool);
  const robot_model::Matrix4 achieved_world_from_tool =
    robot_model::Build_World_From_Tool(
      target_world_from_flange,
      tool);

  Require_Matrix_Near(achieved_world_from_tool, target_world_from_tool);
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

void Test_Fov_Axis_Mappings()
{
  const std::array<robot_model::Coordinate_Axis, 2> axes = {
    robot_model::Coordinate_Axis::X,
    robot_model::Coordinate_Axis::Y};
  for (const auto width_axis : axes)
  {
    for (const auto length_axis : axes)
    {
      if (width_axis == length_axis)
      {
        continue;
      }
      robot_model::Fov_Visualization_Configuration fov;
      fov.width_mm = 100.0;
      fov.length_mm = 240.0;
      fov.width_axis = width_axis;
      fov.length_axis = length_axis;
      assert(robot_model::Is_Valid_Fov_Configuration(fov));
      const auto corners =
        robot_model::Build_Fov_Local_Corners(fov);
      const std::size_t width_index =
        robot_model::Coordinate_Axis_Index(width_axis);
      const std::size_t length_index =
        robot_model::Coordinate_Axis_Index(length_axis);
      const std::size_t normal_index =
        3 - width_index - length_index;
      assert(normal_index ==
             robot_model::Coordinate_Axis_Index(
               robot_model::Coordinate_Axis::Z));
      for (const auto &corner : corners)
      {
        Require_Near(std::abs(corner[width_index]), 50.0);
        Require_Near(std::abs(corner[length_index]), 120.0);
        Require_Near(corner[normal_index], 0.0);
      }
      const robot_model::Point3 first_edge = {
        corners[1][0] - corners[0][0],
        corners[1][1] - corners[0][1],
        corners[1][2] - corners[0][2]};
      const robot_model::Point3 second_edge = {
        corners[2][0] - corners[1][0],
        corners[2][1] - corners[1][1],
        corners[2][2] - corners[1][2]};
      const double normal_z =
        first_edge[0] * second_edge[1] -
        first_edge[1] * second_edge[0];
      assert(normal_z > 0.0);
    }
  }

  robot_model::Fov_Visualization_Configuration invalid;
  invalid.width_axis = robot_model::Coordinate_Axis::Y;
  invalid.length_axis = robot_model::Coordinate_Axis::Y;
  assert(!robot_model::Is_Valid_Fov_Configuration(invalid));

  invalid.width_axis = robot_model::Coordinate_Axis::Y;
  invalid.length_axis = robot_model::Coordinate_Axis::Z;
  assert(!robot_model::Is_Valid_Fov_Configuration(invalid));
}

void Test_Fov_Bound_Tool_Center()
{
  robot_model::Tool_Coordinate_Profile tool;
  tool.id = "camera";
  tool.name = "Camera";
  tool.flange_from_tool_pose =
    {120.0, -35.0, 210.0, 15.0, -20.0, 40.0};
  const robot_model::Matrix4 world_from_flange =
    robot_model::Build_Zyx_Pose_Matrix(
      {600.0, 120.0, 900.0, -25.0, 10.0, 35.0});
  const robot_model::Matrix4 world_from_fov =
    robot_model::Build_World_From_Tool(world_from_flange, tool);

  robot_model::Fov_Visualization_Configuration fov;
  fov.width_mm = 640.0;
  fov.length_mm = 480.0;
  fov.width_axis = robot_model::Coordinate_Axis::X;
  fov.length_axis = robot_model::Coordinate_Axis::Z;
  const auto corners = robot_model::Build_Fov_Local_Corners(fov);
  robot_model::Point3 center_world = {};
  for (const auto &corner : corners)
  {
    const auto corner_world =
      robot_model::Transform_Position(world_from_fov, corner);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      center_world[axis] += corner_world[axis] / 4.0;
    }
  }
  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    Require_Near(center_world[axis], world_from_fov[axis][3]);
  }
}

void Test_Camera_Tool_Right_Handed_Convention()
{
  const auto flange_from_camera =
    robot_model::Build_Zyx_Pose_Matrix(
      {164.46, -1.95, 191.62, 7.19, -89.99, 172.81});
  const std::array<double, 3> camera_x = {
    flange_from_camera[0][0],
    flange_from_camera[1][0],
    flange_from_camera[2][0]};
  const std::array<double, 3> camera_y = {
    flange_from_camera[0][1],
    flange_from_camera[1][1],
    flange_from_camera[2][1]};
  const std::array<double, 3> camera_z = {
    flange_from_camera[0][2],
    flange_from_camera[1][2],
    flange_from_camera[2][2]};
  const std::array<double, 3> x_cross_y = {
    camera_x[1] * camera_y[2] - camera_x[2] * camera_y[1],
    camera_x[2] * camera_y[0] - camera_x[0] * camera_y[2],
    camera_x[0] * camera_y[1] - camera_x[1] * camera_y[0]};
  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    Require_Near(x_cross_y[axis], camera_z[axis]);
  }
}

void Test_Fov_Configuration_Round_Trip()
{
  robot_model::Tool_Visualization_Configuration configuration;
  configuration.fov.visible = true;
  configuration.fov.tool_coordinate_id = "camera";
  configuration.fov.width_mm = 1280.5;
  configuration.fov.length_mm = 960.25;
  configuration.fov.width_axis =
    robot_model::Coordinate_Axis::Z;
  configuration.fov.length_axis =
    robot_model::Coordinate_Axis::X;
  configuration.tool_frame.visible = true;
  configuration.tool_frame.size_scale = 0.35;

  const std::filesystem::path path =
    std::filesystem::temp_directory_path() /
    "codex_tool_visualization_tests.xml";
  std::string error;
  assert(robot_model::Save_Tool_Visualization_Configuration(
    path, configuration, &error));

  robot_model::Tool_Visualization_Configuration loaded;
  assert(robot_model::Load_Tool_Visualization_Configuration(
    path, &loaded, &error));
  assert(loaded.fov.visible);
  assert(loaded.fov.tool_coordinate_id == "camera");
  Require_Near(loaded.fov.width_mm, 1280.5);
  Require_Near(loaded.fov.length_mm, 960.25);
  assert(
    loaded.fov.width_axis == robot_model::Coordinate_Axis::X);
  assert(
    loaded.fov.length_axis == robot_model::Coordinate_Axis::Y);
  assert(loaded.tool_frame.visible);
  Require_Near(loaded.tool_frame.size_scale, 0.35);
  std::filesystem::remove(path);
}

}

int main()
{
  Test_Tool_Transform_Round_Trip();
  Test_Tool_Drag_Target_Conversion();
  Test_Repository_Round_Trip();
  Test_Fov_Axis_Mappings();
  Test_Fov_Bound_Tool_Center();
  Test_Camera_Tool_Right_Handed_Convention();
  Test_Fov_Configuration_Round_Trip();
  return 0;
}

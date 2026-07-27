#ifndef includeguard_tool_visualization_h_includeguard
#define includeguard_tool_visualization_h_includeguard

#include "pose_transform.h"

#include <array>
#include <cstddef>
#include <string>

namespace robot_model
{

enum class Coordinate_Axis
{
  X = 0,
  Y = 1,
  Z = 2
};

struct Fov_Visualization_Configuration
{
  bool visible = false;
  std::string tool_coordinate_id;
  double width_mm = 500.0;
  double length_mm = 300.0;
  Coordinate_Axis width_axis = Coordinate_Axis::X;
  Coordinate_Axis length_axis = Coordinate_Axis::Y;
};

struct Tool_Frame_Visualization_Configuration
{
  bool visible = false;
  double size_scale = 0.5;
};

struct Tool_Visualization_Configuration
{
  Fov_Visualization_Configuration fov;
  Tool_Frame_Visualization_Configuration tool_frame;
};

std::size_t Coordinate_Axis_Index(Coordinate_Axis axis);
const char *Coordinate_Axis_Name(Coordinate_Axis axis);
bool Parse_Coordinate_Axis(
  const std::string &value,
  Coordinate_Axis *axis);
bool Is_Valid_Fov_Configuration(
  const Fov_Visualization_Configuration &configuration);
void Normalize_Tool_Visualization_Configuration(
  Tool_Visualization_Configuration *configuration);
std::array<Point3, 4> Build_Fov_Local_Corners(
  const Fov_Visualization_Configuration &configuration);

} // namespace robot_model

#endif

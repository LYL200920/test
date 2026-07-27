#include "tool_visualization.h"

#include <algorithm>
#include <cmath>

namespace robot_model
{

std::size_t Coordinate_Axis_Index(Coordinate_Axis axis)
{
  return static_cast<std::size_t>(axis);
}

const char *Coordinate_Axis_Name(Coordinate_Axis axis)
{
  switch (axis)
  {
    case Coordinate_Axis::X:
      return "X";
    case Coordinate_Axis::Y:
      return "Y";
    case Coordinate_Axis::Z:
      return "Z";
  }
  return "X";
}

bool Parse_Coordinate_Axis(
  const std::string &value,
  Coordinate_Axis *axis)
{
  if (!axis)
  {
    return false;
  }
  if (value == "X")
  {
    *axis = Coordinate_Axis::X;
    return true;
  }
  if (value == "Y")
  {
    *axis = Coordinate_Axis::Y;
    return true;
  }
  if (value == "Z")
  {
    *axis = Coordinate_Axis::Z;
    return true;
  }
  return false;
}

bool Is_Valid_Fov_Configuration(
  const Fov_Visualization_Configuration &configuration)
{
  return std::isfinite(configuration.width_mm) &&
         configuration.width_mm > 0.0 &&
         std::isfinite(configuration.length_mm) &&
         configuration.length_mm > 0.0 &&
         configuration.width_axis != configuration.length_axis;
}

void Normalize_Tool_Visualization_Configuration(
  Tool_Visualization_Configuration *configuration)
{
  if (!configuration)
  {
    return;
  }
  auto &fov = configuration->fov;
  if (!std::isfinite(fov.width_mm) || fov.width_mm <= 0.0)
  {
    fov.width_mm = 500.0;
  }
  if (!std::isfinite(fov.length_mm) || fov.length_mm <= 0.0)
  {
    fov.length_mm = 300.0;
  }
  if (fov.width_axis == fov.length_axis)
  {
    const auto next =
      (Coordinate_Axis_Index(fov.width_axis) + 1) % 3;
    fov.length_axis = static_cast<Coordinate_Axis>(next);
  }
  auto &tool_frame = configuration->tool_frame;
  if (!std::isfinite(tool_frame.size_scale))
  {
    tool_frame.size_scale = 0.5;
  }
  tool_frame.size_scale =
    std::clamp(tool_frame.size_scale, 0.1, 1.0);
}

std::array<Point3, 4> Build_Fov_Local_Corners(
  const Fov_Visualization_Configuration &configuration)
{
  Fov_Visualization_Configuration normalized = configuration;
  Tool_Visualization_Configuration wrapper;
  wrapper.fov = normalized;
  Normalize_Tool_Visualization_Configuration(&wrapper);
  normalized = wrapper.fov;

  const std::size_t width_axis =
    Coordinate_Axis_Index(normalized.width_axis);
  const std::size_t length_axis =
    Coordinate_Axis_Index(normalized.length_axis);
  const double half_width = normalized.width_mm * 0.5;
  const double half_length = normalized.length_mm * 0.5;

  std::array<Point3, 4> corners = {};
  corners[0][width_axis] = -half_width;
  corners[0][length_axis] = -half_length;
  corners[1][width_axis] = half_width;
  corners[1][length_axis] = -half_length;
  corners[2][width_axis] = half_width;
  corners[2][length_axis] = half_length;
  corners[3][width_axis] = -half_width;
  corners[3][length_axis] = half_length;
  return corners;
}

} // namespace robot_model

#ifndef includeguard_mosaic_configuration_h_includeguard
#define includeguard_mosaic_configuration_h_includeguard

#include "robot_teach_point.h"
#include "tool_visualization.h"
#include <string>
#include <vector>

namespace robot_model
{
enum class Mosaic_Reference { First_Camera, Robot_Base };

struct Mosaic_Group
{
  std::size_t id = 1;
  std::string name;
  std::vector<std::size_t> point_ids;
  Mosaic_Reference reference = Mosaic_Reference::First_Camera;
  Coordinate_Axis horizontal_axis = Coordinate_Axis::X;
  Coordinate_Axis vertical_axis = Coordinate_Axis::Y;
  bool reverse_horizontal = false;
  bool reverse_vertical = false;
};

// Empty groups preserve the legacy all-motion-points, first-camera-plane mode.
struct Mosaic_Configuration
{
  std::vector<Mosaic_Group> groups;
};

bool Validate_Mosaic_Configuration(
  const Mosaic_Configuration &configuration,
  const std::vector<Robot_Teach_Point> &points,
  std::string *error_message = nullptr);
} // namespace robot_model
#endif

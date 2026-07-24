#ifndef includeguard_tool_coordinate_repository_h_includeguard
#define includeguard_tool_coordinate_repository_h_includeguard

#include "tool_coordinate.h"

#include <filesystem>
#include <string>

namespace robot_model
{

std::filesystem::path Tool_Coordinate_Config_Path();

bool Load_Tool_Coordinate_Configuration(
  const std::filesystem::path &path,
  Tool_Coordinate_Configuration *configuration,
  std::string *error_message = nullptr);
bool Save_Tool_Coordinate_Configuration(
  const std::filesystem::path &path,
  const Tool_Coordinate_Configuration &configuration,
  std::string *error_message = nullptr);

} // namespace robot_model

#endif

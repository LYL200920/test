#ifndef includeguard_tool_visualization_repository_h_includeguard
#define includeguard_tool_visualization_repository_h_includeguard

#include "tool_visualization.h"

#include <filesystem>
#include <string>

namespace robot_model
{

std::filesystem::path Tool_Visualization_Config_Path();

bool Load_Tool_Visualization_Configuration(
  const std::filesystem::path &path,
  Tool_Visualization_Configuration *configuration,
  std::string *error_message = nullptr);
bool Save_Tool_Visualization_Configuration(
  const std::filesystem::path &path,
  const Tool_Visualization_Configuration &configuration,
  std::string *error_message = nullptr);

} // namespace robot_model

#endif

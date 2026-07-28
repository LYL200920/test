#ifndef includeguard_point_cloud_template_binding_repository_h_includeguard
#define includeguard_point_cloud_template_binding_repository_h_includeguard

#include "point_cloud_template_binding.h"

#include <filesystem>
#include <string>

namespace robot_model
{

std::filesystem::path Point_Cloud_Template_Binding_Config_Path();
bool Load_Point_Cloud_Template_Bindings(
  const std::filesystem::path &path,
  Point_Cloud_Template_Binding_Configuration *configuration,
  std::string *error_message = nullptr);
bool Save_Point_Cloud_Template_Bindings(
  const std::filesystem::path &path,
  const Point_Cloud_Template_Binding_Configuration &configuration,
  std::string *error_message = nullptr);

} // namespace robot_model

#endif

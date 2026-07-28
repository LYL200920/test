#ifndef includeguard_template_configuration_repository_h_includeguard
#define includeguard_template_configuration_repository_h_includeguard

#include "template_configuration.h"

#include <filesystem>
#include <string>

namespace robot_model
{

std::filesystem::path Template_Configuration_Path();

bool Load_Template_Configuration(
  const std::filesystem::path &path,
  Template_Configuration *configuration,
  std::string *error_message = nullptr);
bool Save_Template_Configuration(
  const std::filesystem::path &path,
  const Template_Configuration &configuration,
  std::string *error_message = nullptr);

} // namespace robot_model

#endif

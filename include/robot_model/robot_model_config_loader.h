#ifndef includeguard_robot_model_config_loader_h_includeguard
#define includeguard_robot_model_config_loader_h_includeguard

#include "robot_model_data.h"

#include <filesystem>
#include <string>

namespace robot_model
{

  Robot_Kinematic_Params Load_Robot_Kinematic_Params(const std::filesystem::path &xml_path, const std::string &default_name = "");

} // namespace robot_model

#endif

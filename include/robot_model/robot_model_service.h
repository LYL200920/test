#ifndef includeguard_robot_model_service_h_includeguard
#define includeguard_robot_model_service_h_includeguard

#include "robot_model_config_loader.h"
#include "robot_model_repository.h"

namespace robot_model
{

Robot_Kinematic_Params Parse_Kinematic_Params (
  const std::filesystem::path& xml_path,
  const std::string& default_name = "");

} // namespace robot_model

#endif

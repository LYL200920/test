#include "robot_model_service.h"

namespace robot_model
{

  Robot_Kinematic_Params Parse_Kinematic_Params(const std::filesystem::path &xml_path,
                                                const std::string &default_name)
  {
    return Load_Robot_Kinematic_Params(xml_path, default_name);
  }

} // namespace robot_model

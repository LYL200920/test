#ifndef includeguard_robot_model_repository_h_includeguard
#define includeguard_robot_model_repository_h_includeguard

#include "robot_model_data.h"

#include <filesystem>
#include <vector>

namespace robot_model
{

  std::filesystem::path Find_Robot_Root();

  std::vector<Robot_Model_Info> Scan_Models_In_Directory(const std::filesystem::path &root);

} // namespace robot_model

#endif

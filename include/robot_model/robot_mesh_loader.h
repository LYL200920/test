#ifndef includeguard_robot_mesh_loader_h_includeguard
#define includeguard_robot_mesh_loader_h_includeguard

#include "robot_model_data.h"

#include <filesystem>

namespace robot_model
{

Robot_Visual_Part Create_STL_Part (const std::filesystem::path& stl_path,
                                   int index);

} // namespace robot_model

#endif

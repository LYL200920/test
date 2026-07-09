#ifndef includeguard_robot_trajectory_io_h_includeguard
#define includeguard_robot_trajectory_io_h_includeguard

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace robot_model
{

bool Save_Joint_Trajectory_Csv (
  const std::filesystem::path& path,
  const std::vector<std::array<double, 6>>& points,
  std::string* error_message);

bool Load_Joint_Trajectory_Csv (
  const std::filesystem::path& path,
  std::vector<std::array<double, 6>>* points,
  std::string* error_message);

} // namespace robot_model

#endif

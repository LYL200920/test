#ifndef includeguard_app_paths_h_includeguard
#define includeguard_app_paths_h_includeguard

#include <filesystem>

namespace application
{

struct App_Paths
{
  std::filesystem::path executable_directory;
  std::filesystem::path resource_root;
  std::filesystem::path config_root;
  std::filesystem::path robot_model_root;
  std::filesystem::path point_cloud_root;
  std::filesystem::path camera_2d_root;
  std::filesystem::path progress_root;
  std::filesystem::path run_image_root;
  std::filesystem::path net_config_file;
  std::filesystem::path flow_config_file;
};

App_Paths Build_App_Paths(
  const std::filesystem::path &executable_directory,
  const std::filesystem::path &resource_root);

// Finds Resource beside the executable or in one of its ancestors. This
// supports both installed layouts and build/bin/<configuration> layouts.
App_Paths Discover_App_Paths(
  const std::filesystem::path &executable_path);

std::filesystem::path Current_Process_Executable_Path();

// Configure once in the composition root. Get_App_Paths lazily discovers a
// default for command-line tests that do not have an application bootstrap.
void Configure_App_Paths(App_Paths paths);
App_Paths Get_App_Paths();

} // namespace application

#endif

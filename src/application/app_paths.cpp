#include "app_paths.h"

#include <mutex>
#include <optional>
#include <system_error>

#ifdef _WIN32
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

namespace application
{
namespace
{
std::mutex g_paths_mutex;
std::optional<App_Paths> g_paths;

std::filesystem::path Absolute_Normalized(std::filesystem::path path)
{
  if( path.empty() ) return {};
  std::error_code error;
  auto absolute = std::filesystem::absolute(path, error);
  if( !error ) path = std::move(absolute);
  return path.lexically_normal();
}

bool Is_Usable_Resource_Root(const std::filesystem::path &root)
{
  std::error_code error;
  const auto config_root = root / "Config";
  const auto robot_root = root / "Robot";
  if( !std::filesystem::is_directory(config_root, error) || error )
    return false;
  error.clear();
  if( !std::filesystem::is_directory(robot_root, error) || error )
    return false;

  std::filesystem::directory_iterator iterator(robot_root, error);
  const std::filesystem::directory_iterator end;
  while( !error && iterator != end )
  {
    if( iterator->is_directory(error) && !error &&
        std::filesystem::is_regular_file(
          iterator->path() / "robot.xml", error) && !error )
      return true;
    error.clear();
    iterator.increment(error);
  }
  return false;
}
} // namespace

App_Paths Build_App_Paths(
  const std::filesystem::path &executable_directory,
  const std::filesystem::path &resource_root)
{
  App_Paths paths;
  paths.executable_directory = Absolute_Normalized(executable_directory);
  paths.resource_root = Absolute_Normalized(resource_root);
  paths.config_root = paths.resource_root / "Config";
  paths.robot_model_root = paths.resource_root / "Robot";
  paths.point_cloud_root = paths.resource_root / "PointCloud";
  paths.camera_2d_root = paths.resource_root / "Camera2D";
  paths.progress_root = paths.resource_root / "Progress";
  paths.run_image_root = paths.executable_directory / "RunImages";
  paths.net_config_file =
    paths.executable_directory / "vtk_net_config.ini";
  paths.flow_config_file =
    paths.executable_directory / "vtk_flow_config.ini";
  return paths;
}

App_Paths Discover_App_Paths(const std::filesystem::path &executable_path)
{
  const auto normalized_executable = Absolute_Normalized(executable_path);
  auto directory = normalized_executable.has_filename()
    ? normalized_executable.parent_path()
    : normalized_executable;
  if( directory.empty() ) directory = normalized_executable;

  auto candidate_directory = directory;
  for( int depth = 0; depth < 8 && !candidate_directory.empty(); ++depth )
  {
    const auto candidate = candidate_directory / "Resource";
    if( Is_Usable_Resource_Root(candidate) )
      return Build_App_Paths(directory, candidate);
    const auto parent = candidate_directory.parent_path();
    if( parent == candidate_directory ) break;
    candidate_directory = parent;
  }
  return Build_App_Paths(directory, directory / "Resource");
}

std::filesystem::path Current_Process_Executable_Path()
{
#ifdef _WIN32
  std::wstring buffer(32768, L'\0');
  const DWORD length = GetModuleFileNameW(
    nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
  if( length == 0 || length >= buffer.size() ) return {};
  buffer.resize(length);
  return std::filesystem::path(buffer);
#elif defined(__linux__)
  std::string buffer(4096, '\0');
  const auto length = readlink("/proc/self/exe", buffer.data(), buffer.size());
  if( length <= 0 ) return {};
  buffer.resize(static_cast<std::size_t>(length));
  return std::filesystem::path(buffer);
#else
  return {};
#endif
}

void Configure_App_Paths(App_Paths paths)
{
  std::lock_guard<std::mutex> lock(g_paths_mutex);
  g_paths = std::move(paths);
}

App_Paths Get_App_Paths()
{
  std::lock_guard<std::mutex> lock(g_paths_mutex);
  if( !g_paths )
    g_paths = Discover_App_Paths(Current_Process_Executable_Path());
  return *g_paths;
}

} // namespace application

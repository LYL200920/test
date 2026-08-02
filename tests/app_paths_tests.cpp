#include "app_paths.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void Require(bool condition, const char *message)
{
  if( !condition ) throw std::runtime_error(message);
}

void Test_Derived_Paths()
{
  const auto paths = application::Build_App_Paths(
    std::filesystem::path("C:/Program/Test/bin"),
    std::filesystem::path("D:/RobotData/Resource"));
  Require(paths.config_root.filename() == "Config", "config root");
  Require(paths.robot_model_root.filename() == "Robot", "robot root");
  Require(paths.point_cloud_root.filename() == "PointCloud", "cloud root");
  Require(paths.camera_2d_root.filename() == "Camera2D", "camera root");
  Require(paths.progress_root.filename() == "Progress", "progress root");
  Require(paths.run_image_root.filename() == "RunImages", "run image root");
  Require(paths.net_config_file.filename() == "vtk_net_config.ini",
          "net config file");
  Require(paths.flow_config_file.filename() == "vtk_flow_config.ini",
          "flow config file");
}

void Test_Resource_Discovery_And_Override()
{
  const auto unique = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  const auto root = std::filesystem::temp_directory_path() /
                    ("app_paths_tests_" + unique);
  const auto resource = root / "Resource";
  const auto executable = root / "build" / "bin" / "Release" / "test.exe";
  std::filesystem::create_directories(resource / "Config");
  std::filesystem::create_directories(resource / "Robot" / "TestRobot");
  {
    std::ofstream robot_file(resource / "Robot" / "TestRobot" / "robot.xml");
    robot_file << "<RobotTemplate/>";
  }
  std::filesystem::create_directories(executable.parent_path());

  const auto discovered = application::Discover_App_Paths(executable);
  Require(discovered.resource_root == resource, "resource discovery");
  Require(discovered.executable_directory == executable.parent_path(),
          "executable directory");

  application::Configure_App_Paths(discovered);
  const auto configured = application::Get_App_Paths();
  Require(configured.robot_model_root == resource / "Robot",
          "configured robot path");

  std::error_code cleanup_error;
  std::filesystem::remove_all(root, cleanup_error);
}

void Test_Current_Process_Discovery()
{
  const auto discovered = application::Discover_App_Paths(
    application::Current_Process_Executable_Path());
  Require(std::filesystem::is_directory(discovered.resource_root),
          "current process resource root");
  Require(std::filesystem::is_directory(discovered.robot_model_root),
          "current process robot root");
}
} // namespace

int main()
{
  try
  {
    Test_Derived_Paths();
    Test_Current_Process_Discovery();
    Test_Resource_Discovery_And_Override();
  }
  catch( const std::exception &error )
  {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }
  std::cout << "App paths tests passed.\n";
  return 0;
}

#include "template_configuration.h"
#include "template_configuration_repository.h"
#include "point_cloud_template_binding.h"
#include "point_cloud_template_binding_repository.h"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

namespace
{

robot_model::Template_Profile Make_Profile(
  const std::string &id,
  const std::string &name,
  const robot_model::XyzabcPose &pose = {})
{
  robot_model::Template_Profile profile;
  profile.id = id;
  profile.name = name;
  profile.reference_pose = pose;
  return profile;
}

void Test_Normalization()
{
  robot_model::Template_Configuration configuration;
  configuration.templates.push_back(Make_Profile(
    "template_1", "Top", {1.0, 2.0, 3.0, 4.0, 5.0, 6.0}));
  configuration.templates.push_back(
    Make_Profile("template_1", "Duplicate"));
  configuration.templates.push_back(Make_Profile("", "No id"));
  configuration.templates.push_back(Make_Profile("template_2", ""));
  configuration.templates.push_back(Make_Profile(
    "template_3",
    "Invalid",
    {std::numeric_limits<double>::quiet_NaN(),
     0.0, 0.0, 0.0, 0.0, 0.0}));
  robot_model::Normalize_Template_Configuration(&configuration);
  assert(configuration.templates.size() == 1);
  assert(configuration.templates.front().id == "template_1");
  assert(robot_model::Find_Template_Profile(
    configuration, "template_1") != nullptr);
  assert(robot_model::Find_Template_Profile(
    configuration, "missing") == nullptr);
}

void Test_Repository_Round_Trip()
{
  robot_model::Template_Configuration configuration;
  configuration.templates.push_back(Make_Profile(
    "template_1",
    "Top",
    {100.25, -200.5, 350.75, 10.0, -20.0, 30.0}));
  configuration.templates.push_back(Make_Profile(
    "template_2",
    u8"侧面模板",
    {-1.0, 2.0, 3.0, -4.0, 5.0, -6.0}));

  const std::filesystem::path path =
    std::filesystem::temp_directory_path() /
    "codex_template_configuration_tests.xml";
  std::string error;
  assert(robot_model::Save_Template_Configuration(
    path, configuration, &error));

  robot_model::Template_Configuration loaded;
  assert(robot_model::Load_Template_Configuration(
    path, &loaded, &error));
  assert(loaded.templates.size() == 2);
  for (std::size_t index = 0; index < configuration.templates.size(); ++index)
  {
    assert(loaded.templates[index].id ==
           configuration.templates[index].id);
    assert(loaded.templates[index].name ==
           configuration.templates[index].name);
    for (std::size_t pose_index = 0; pose_index < 6; ++pose_index)
    {
      assert(std::abs(
        loaded.templates[index].reference_pose[pose_index] -
        configuration.templates[index].reference_pose[pose_index]) <
        1.0e-10);
    }
  }
  std::filesystem::remove(path);
}

void Test_Missing_File_Is_Empty_Configuration()
{
  const std::filesystem::path path =
    std::filesystem::temp_directory_path() /
    "codex_template_configuration_missing.xml";
  std::filesystem::remove(path);

  robot_model::Template_Configuration loaded;
  std::string error;
  assert(robot_model::Load_Template_Configuration(
    path, &loaded, &error));
  assert(loaded.templates.empty());
}

void Test_Invalid_File_Is_Rejected()
{
  const std::filesystem::path path =
    std::filesystem::temp_directory_path() /
    "codex_template_configuration_invalid.xml";
  {
    std::ofstream output(path);
    output << "<WrongRoot/>";
  }
  robot_model::Template_Configuration loaded;
  std::string error;
  assert(!robot_model::Load_Template_Configuration(
    path, &loaded, &error));
  assert(!error.empty());
  std::filesystem::remove(path);
}

void Test_Point_Cloud_Binding_Round_Trip()
{
  const std::filesystem::path cloud_path =
    std::filesystem::temp_directory_path() / "binding_cloud.ply";
  robot_model::Point_Cloud_Template_Binding_Configuration configuration;
  robot_model::Set_Point_Cloud_Template_Binding(
    &configuration, cloud_path, "binding_cloud", "template_1");
  const auto *binding =
    robot_model::Find_Point_Cloud_Template_Binding(
      configuration, cloud_path);
  assert(binding != nullptr);
  assert(binding->template_id == "template_1");

  robot_model::Set_Point_Cloud_Template_Binding(
    &configuration, cloud_path, "renamed_cloud", "template_2");
  assert(configuration.bindings.size() == 1);
  binding = robot_model::Find_Point_Cloud_Template_Binding(
    configuration, cloud_path);
  assert(binding != nullptr);
  assert(binding->point_cloud_name == "renamed_cloud");
  assert(binding->template_id == "template_2");

  const std::filesystem::path config_path =
    std::filesystem::temp_directory_path() /
    "codex_point_cloud_template_bindings.xml";
  std::string error;
  assert(robot_model::Save_Point_Cloud_Template_Bindings(
    config_path, configuration, &error));
  robot_model::Point_Cloud_Template_Binding_Configuration loaded;
  assert(robot_model::Load_Point_Cloud_Template_Bindings(
    config_path, &loaded, &error));
  binding = robot_model::Find_Point_Cloud_Template_Binding(
    loaded, cloud_path);
  assert(binding != nullptr);
  assert(binding->template_id == "template_2");
  assert(robot_model::Remove_Point_Cloud_Template_Binding(
    &loaded, cloud_path));
  assert(loaded.bindings.empty());
  std::filesystem::remove(config_path);
}

} // namespace

int main()
{
  Test_Normalization();
  Test_Repository_Round_Trip();
  Test_Missing_File_Is_Empty_Configuration();
  Test_Invalid_File_Is_Rejected();
  Test_Point_Cloud_Binding_Round_Trip();
  return 0;
}

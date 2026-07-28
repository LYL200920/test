#include "template_configuration_repository.h"

#include "pugixml.hpp"
#include "robot_model_repository.h"

#include <filesystem>

namespace robot_model
{
namespace
{

void Set_Error(std::string *error_message, const std::string &message)
{
  if (error_message)
  {
    *error_message = message;
  }
}

} // namespace

std::filesystem::path Template_Configuration_Path()
{
  const std::filesystem::path robot_root = Find_Robot_Root();
  if (!robot_root.empty())
  {
    return robot_root.parent_path() / "Config" /
      "template_profiles.xml";
  }
  return std::filesystem::current_path() /
    "Resource" / "Config" / "template_profiles.xml";
}

bool Load_Template_Configuration(
  const std::filesystem::path &path,
  Template_Configuration *configuration,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (!configuration)
  {
    Set_Error(error_message, "Template configuration output is null");
    return false;
  }

  *configuration = {};
  if (path.empty() || !std::filesystem::exists(path))
  {
    return true;
  }

  pugi::xml_document document;
  if (!document.load_file(path.wstring().c_str()))
  {
    Set_Error(error_message, "Unable to parse template configuration");
    return false;
  }
  const pugi::xml_node root = document.child("TemplateProfiles");
  if (!root)
  {
    Set_Error(error_message, "Template configuration root is missing");
    return false;
  }

  Template_Configuration parsed;
  for (const pugi::xml_node node : root.children("Template"))
  {
    Template_Profile profile;
    profile.id = node.attribute("id").as_string();
    profile.name = node.attribute("name").as_string();
    profile.reference_pose = {
      node.attribute("x").as_double(),
      node.attribute("y").as_double(),
      node.attribute("z").as_double(),
      node.attribute("a").as_double(),
      node.attribute("b").as_double(),
      node.attribute("c").as_double()};
    if (!Is_Valid_Template_Profile(profile))
    {
      Set_Error(error_message, "Template configuration contains an invalid profile");
      return false;
    }
    if (Find_Template_Profile(parsed, profile.id))
    {
      Set_Error(error_message, "Template configuration contains a duplicate id");
      return false;
    }
    parsed.templates.push_back(std::move(profile));
  }
  *configuration = std::move(parsed);
  return true;
}

bool Save_Template_Configuration(
  const std::filesystem::path &path,
  const Template_Configuration &configuration,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (path.empty())
  {
    Set_Error(error_message, "Template configuration path is empty");
    return false;
  }

  Template_Configuration normalized = configuration;
  Normalize_Template_Configuration(&normalized);
  if (normalized.templates.size() != configuration.templates.size())
  {
    Set_Error(error_message, "Template configuration contains an invalid profile");
    return false;
  }

  std::error_code filesystem_error;
  std::filesystem::create_directories(
    path.parent_path(), filesystem_error);
  if (filesystem_error)
  {
    Set_Error(error_message, "Unable to create template configuration directory");
    return false;
  }

  pugi::xml_document document;
  pugi::xml_node root = document.append_child("TemplateProfiles");
  root.append_attribute("version") = 1;
  for (const Template_Profile &profile : normalized.templates)
  {
    pugi::xml_node node = root.append_child("Template");
    node.append_attribute("id") = profile.id.c_str();
    node.append_attribute("name") = profile.name.c_str();
    node.append_attribute("x") = profile.reference_pose[0];
    node.append_attribute("y") = profile.reference_pose[1];
    node.append_attribute("z") = profile.reference_pose[2];
    node.append_attribute("a") = profile.reference_pose[3];
    node.append_attribute("b") = profile.reference_pose[4];
    node.append_attribute("c") = profile.reference_pose[5];
  }
  if (!document.save_file(path.wstring().c_str(), "  "))
  {
    Set_Error(error_message, "Unable to save template configuration");
    return false;
  }
  return true;
}

} // namespace robot_model

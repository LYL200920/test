#include "point_cloud_template_binding_repository.h"

#include "pugixml.hpp"
#include "robot_model_repository.h"

#include <filesystem>
#include <unordered_set>

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

std::filesystem::path Point_Cloud_Template_Binding_Config_Path()
{
  const std::filesystem::path robot_root = Find_Robot_Root();
  if (!robot_root.empty())
  {
    return robot_root.parent_path() / "Config" /
      "point_cloud_template_bindings.xml";
  }
  return std::filesystem::current_path() /
    "Resource" / "Config" / "point_cloud_template_bindings.xml";
}

bool Load_Point_Cloud_Template_Bindings(
  const std::filesystem::path &path,
  Point_Cloud_Template_Binding_Configuration *configuration,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (!configuration)
  {
    Set_Error(error_message, "Point cloud template binding output is null");
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
    Set_Error(error_message, "Unable to parse point cloud template bindings");
    return false;
  }
  const pugi::xml_node root =
    document.child("PointCloudTemplateBindings");
  if (!root)
  {
    Set_Error(error_message, "Point cloud template binding root is missing");
    return false;
  }

  std::unordered_set<std::string> keys;
  for (const pugi::xml_node node : root.children("Binding"))
  {
    Point_Cloud_Template_Binding binding;
    binding.point_cloud_key =
      node.attribute("pointCloud").as_string();
    binding.point_cloud_name =
      node.attribute("pointCloudName").as_string();
    binding.template_id =
      node.attribute("templateId").as_string();
    if (binding.point_cloud_key.empty() ||
        binding.template_id.empty() ||
        keys.count(binding.point_cloud_key) != 0)
    {
      Set_Error(error_message, "Point cloud template binding is invalid");
      return false;
    }
    keys.insert(binding.point_cloud_key);
    configuration->bindings.push_back(std::move(binding));
  }
  return true;
}

bool Save_Point_Cloud_Template_Bindings(
  const std::filesystem::path &path,
  const Point_Cloud_Template_Binding_Configuration &configuration,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (path.empty())
  {
    Set_Error(error_message, "Point cloud template binding path is empty");
    return false;
  }
  std::error_code filesystem_error;
  std::filesystem::create_directories(
    path.parent_path(), filesystem_error);
  if (filesystem_error)
  {
    Set_Error(error_message, "Unable to create binding configuration directory");
    return false;
  }

  pugi::xml_document document;
  pugi::xml_node root =
    document.append_child("PointCloudTemplateBindings");
  root.append_attribute("version") = 1;
  std::unordered_set<std::string> keys;
  for (const auto &binding : configuration.bindings)
  {
    if (binding.point_cloud_key.empty() ||
        binding.template_id.empty() ||
        keys.count(binding.point_cloud_key) != 0)
    {
      Set_Error(error_message, "Point cloud template binding is invalid");
      return false;
    }
    keys.insert(binding.point_cloud_key);
    pugi::xml_node node = root.append_child("Binding");
    node.append_attribute("pointCloud") =
      binding.point_cloud_key.c_str();
    node.append_attribute("pointCloudName") =
      binding.point_cloud_name.c_str();
    node.append_attribute("templateId") =
      binding.template_id.c_str();
  }
  if (!document.save_file(path.wstring().c_str(), "  "))
  {
    Set_Error(error_message, "Unable to save point cloud template bindings");
    return false;
  }
  return true;
}

} // namespace robot_model

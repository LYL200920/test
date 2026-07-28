#include "point_cloud_template_binding.h"

#include <algorithm>

namespace robot_model
{

std::string Point_Cloud_Binding_Key(
  const std::filesystem::path &path)
{
  if (path.empty())
  {
    return {};
  }
  std::error_code error;
  std::filesystem::path normalized =
    std::filesystem::weakly_canonical(path, error);
  if (error)
  {
    error.clear();
    normalized = std::filesystem::absolute(path, error);
  }
  if (error)
  {
    normalized = path;
  }
  return normalized.lexically_normal().generic_u8string();
}

const Point_Cloud_Template_Binding *Find_Point_Cloud_Template_Binding(
  const Point_Cloud_Template_Binding_Configuration &configuration,
  const std::filesystem::path &path)
{
  const std::string key = Point_Cloud_Binding_Key(path);
  const auto found = std::find_if(
    configuration.bindings.begin(),
    configuration.bindings.end(),
    [&key](const Point_Cloud_Template_Binding &binding)
    {
      return binding.point_cloud_key == key;
    });
  return found == configuration.bindings.end() ? nullptr : &*found;
}

void Set_Point_Cloud_Template_Binding(
  Point_Cloud_Template_Binding_Configuration *configuration,
  const std::filesystem::path &path,
  const std::string &point_cloud_name,
  const std::string &template_id)
{
  if (!configuration || path.empty() || template_id.empty())
  {
    return;
  }
  const std::string key = Point_Cloud_Binding_Key(path);
  auto found = std::find_if(
    configuration->bindings.begin(),
    configuration->bindings.end(),
    [&key](const Point_Cloud_Template_Binding &binding)
    {
      return binding.point_cloud_key == key;
    });
  if (found == configuration->bindings.end())
  {
    configuration->bindings.push_back(
      {key, point_cloud_name, template_id});
  }
  else
  {
    found->point_cloud_name = point_cloud_name;
    found->template_id = template_id;
  }
}

bool Remove_Point_Cloud_Template_Binding(
  Point_Cloud_Template_Binding_Configuration *configuration,
  const std::filesystem::path &path)
{
  if (!configuration)
  {
    return false;
  }
  const std::string key = Point_Cloud_Binding_Key(path);
  const auto old_size = configuration->bindings.size();
  configuration->bindings.erase(
    std::remove_if(
      configuration->bindings.begin(),
      configuration->bindings.end(),
      [&key](const Point_Cloud_Template_Binding &binding)
      {
        return binding.point_cloud_key == key;
      }),
    configuration->bindings.end());
  return configuration->bindings.size() != old_size;
}

} // namespace robot_model

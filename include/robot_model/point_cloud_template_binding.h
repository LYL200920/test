#ifndef includeguard_point_cloud_template_binding_h_includeguard
#define includeguard_point_cloud_template_binding_h_includeguard

#include <filesystem>
#include <string>
#include <vector>

namespace robot_model
{

struct Point_Cloud_Template_Binding
{
  std::string point_cloud_key;
  std::string point_cloud_name;
  std::string template_id;
};

struct Point_Cloud_Template_Binding_Configuration
{
  std::vector<Point_Cloud_Template_Binding> bindings;
};

std::string Point_Cloud_Binding_Key(
  const std::filesystem::path &path);
const Point_Cloud_Template_Binding *Find_Point_Cloud_Template_Binding(
  const Point_Cloud_Template_Binding_Configuration &configuration,
  const std::filesystem::path &path);
void Set_Point_Cloud_Template_Binding(
  Point_Cloud_Template_Binding_Configuration *configuration,
  const std::filesystem::path &path,
  const std::string &point_cloud_name,
  const std::string &template_id);
bool Remove_Point_Cloud_Template_Binding(
  Point_Cloud_Template_Binding_Configuration *configuration,
  const std::filesystem::path &path);

} // namespace robot_model

#endif

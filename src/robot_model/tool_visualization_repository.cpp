#include "tool_visualization_repository.h"

#include "app_paths.h"
#include "pugixml.hpp"

#include <filesystem>

namespace robot_model
{

std::filesystem::path Tool_Visualization_Config_Path()
{
  return application::Get_App_Paths().config_root /
         "tool_visualizations.xml";
}

bool Load_Tool_Visualization_Configuration(
  const std::filesystem::path &path,
  Tool_Visualization_Configuration *configuration,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (!configuration)
  {
    if (error_message)
    {
      *error_message = "Tool visualization output is null";
    }
    return false;
  }

  *configuration = {};
  if (path.empty() || !std::filesystem::exists(path))
  {
    return true;
  }

  pugi::xml_document document;
  const auto loaded = document.load_file(path.wstring().c_str());
  if (!loaded)
  {
    if (error_message)
    {
      *error_message = "Unable to parse tool visualization configuration";
    }
    return false;
  }

  const pugi::xml_node root = document.child("ToolVisualizations");
  const pugi::xml_node fov_node = root.child("Fov");
  if (!root || !fov_node)
  {
    if (error_message)
    {
      *error_message = "Tool visualization configuration is incomplete";
    }
    return false;
  }

  Tool_Visualization_Configuration parsed;
  parsed.fov.visible = fov_node.attribute("visible").as_bool(false);
  parsed.fov.tool_coordinate_id =
    fov_node.attribute("coordinate").as_string();
  parsed.fov.width_mm =
    fov_node.attribute("width").as_double(500.0);
  parsed.fov.length_mm =
    fov_node.attribute("length").as_double(300.0);
  Parse_Coordinate_Axis(
    fov_node.attribute("widthAxis").as_string("X"),
    &parsed.fov.width_axis);
  Parse_Coordinate_Axis(
    fov_node.attribute("lengthAxis").as_string("Y"),
    &parsed.fov.length_axis);
  const pugi::xml_node tool_frame_node = root.child("ToolFrame");
  if (tool_frame_node)
  {
    parsed.tool_frame.visible =
      tool_frame_node.attribute("visible").as_bool(false);
    parsed.tool_frame.size_scale =
      tool_frame_node.attribute("scale").as_double(0.5);
  }
  Normalize_Tool_Visualization_Configuration(&parsed);
  *configuration = std::move(parsed);
  return true;
}

bool Save_Tool_Visualization_Configuration(
  const std::filesystem::path &path,
  const Tool_Visualization_Configuration &configuration,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (path.empty())
  {
    if (error_message)
    {
      *error_message = "Tool visualization path is empty";
    }
    return false;
  }

  Tool_Visualization_Configuration normalized = configuration;
  Normalize_Tool_Visualization_Configuration(&normalized);

  std::error_code filesystem_error;
  std::filesystem::create_directories(
    path.parent_path(),
    filesystem_error);
  if (filesystem_error)
  {
    if (error_message)
    {
      *error_message = "Unable to create tool visualization directory";
    }
    return false;
  }

  pugi::xml_document document;
  pugi::xml_node root =
    document.append_child("ToolVisualizations");
  root.append_attribute("version") = 1;
  pugi::xml_node fov_node = root.append_child("Fov");
  fov_node.append_attribute("visible") = normalized.fov.visible;
  fov_node.append_attribute("coordinate") =
    normalized.fov.tool_coordinate_id.c_str();
  fov_node.append_attribute("width") = normalized.fov.width_mm;
  fov_node.append_attribute("length") = normalized.fov.length_mm;
  fov_node.append_attribute("widthAxis") =
    Coordinate_Axis_Name(normalized.fov.width_axis);
  fov_node.append_attribute("lengthAxis") =
    Coordinate_Axis_Name(normalized.fov.length_axis);
  pugi::xml_node tool_frame_node = root.append_child("ToolFrame");
  tool_frame_node.append_attribute("visible") =
    normalized.tool_frame.visible;
  tool_frame_node.append_attribute("scale") =
    normalized.tool_frame.size_scale;

  if (!document.save_file(path.wstring().c_str(), "  "))
  {
    if (error_message)
    {
      *error_message = "Unable to save tool visualization configuration";
    }
    return false;
  }
  return true;
}

} // namespace robot_model

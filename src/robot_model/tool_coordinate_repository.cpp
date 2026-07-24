#include "tool_coordinate_repository.h"

#include "robot_model_repository.h"
#include "pugixml.hpp"

#include <filesystem>

namespace robot_model
{

std::filesystem::path Tool_Coordinate_Config_Path()
{
  const std::filesystem::path robot_root = Find_Robot_Root();
  if (!robot_root.empty())
  {
    return robot_root.parent_path() / "Config" / "tool_coordinates.xml";
  }
  return std::filesystem::current_path() /
         "Resource" / "Config" / "tool_coordinates.xml";
}

bool Load_Tool_Coordinate_Configuration(
  const std::filesystem::path &path,
  Tool_Coordinate_Configuration *configuration,
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
      *error_message = "Tool coordinate output is null";
    }
    return false;
  }

  *configuration = Default_Tool_Coordinate_Configuration();
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
      *error_message = "Unable to parse tool coordinate configuration";
    }
    return false;
  }

  const pugi::xml_node root = document.child("ToolCoordinates");
  if (!root)
  {
    if (error_message)
    {
      *error_message = "Tool coordinate configuration root is missing";
    }
    return false;
  }

  Tool_Coordinate_Configuration parsed;
  parsed.active_tool_id = root.attribute("active").as_string("flange");
  for (const pugi::xml_node node : root.children("Tool"))
  {
    Tool_Coordinate_Profile tool;
    tool.id = node.attribute("id").as_string();
    tool.name = node.attribute("name").as_string();
    tool.flange_from_tool_pose = {
      node.attribute("x").as_double(),
      node.attribute("y").as_double(),
      node.attribute("z").as_double(),
      node.attribute("a").as_double(),
      node.attribute("b").as_double(),
      node.attribute("c").as_double()};
    parsed.tools.push_back(std::move(tool));
  }
  Normalize_Tool_Coordinate_Configuration(&parsed);
  *configuration = std::move(parsed);
  return true;
}

bool Save_Tool_Coordinate_Configuration(
  const std::filesystem::path &path,
  const Tool_Coordinate_Configuration &configuration,
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
      *error_message = "Tool coordinate path is empty";
    }
    return false;
  }

  Tool_Coordinate_Configuration normalized = configuration;
  Normalize_Tool_Coordinate_Configuration(&normalized);

  std::error_code filesystem_error;
  std::filesystem::create_directories(path.parent_path(), filesystem_error);
  if (filesystem_error)
  {
    if (error_message)
    {
      *error_message = "Unable to create tool coordinate directory";
    }
    return false;
  }

  pugi::xml_document document;
  pugi::xml_node root = document.append_child("ToolCoordinates");
  root.append_attribute("version") = 1;
  root.append_attribute("active") = normalized.active_tool_id.c_str();
  for (const Tool_Coordinate_Profile &tool : normalized.tools)
  {
    pugi::xml_node node = root.append_child("Tool");
    node.append_attribute("id") = tool.id.c_str();
    node.append_attribute("name") = tool.name.c_str();
    node.append_attribute("x") = tool.flange_from_tool_pose[0];
    node.append_attribute("y") = tool.flange_from_tool_pose[1];
    node.append_attribute("z") = tool.flange_from_tool_pose[2];
    node.append_attribute("a") = tool.flange_from_tool_pose[3];
    node.append_attribute("b") = tool.flange_from_tool_pose[4];
    node.append_attribute("c") = tool.flange_from_tool_pose[5];
  }
  if (!document.save_file(path.wstring().c_str(), "  "))
  {
    if (error_message)
    {
      *error_message = "Unable to save tool coordinate configuration";
    }
    return false;
  }
  return true;
}

} // namespace robot_model

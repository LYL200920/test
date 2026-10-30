#include "robot_progress_io.h"

#include "pugixml.hpp"

#include <cmath>
#include <initializer_list>
#include <limits>
#include <locale>
#include <sstream>
#include <unordered_set>
#include <utility>

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

bool Is_Finite_Point(const Robot_Teach_Point &point)
{
  for (const double value : point.joint_angles_deg)
  {
    if (!std::isfinite(value))
    {
      return false;
    }
  }
  for (const double value : point.world_pose)
  {
    if (!std::isfinite(value))
    {
      return false;
    }
  }
  for (const double value : point.flange_from_coordinate_pose)
  {
    if (!std::isfinite(value))
    {
      return false;
    }
  }
  for (const double value : point.template_reference_pose)
  {
    if (!std::isfinite(value))
    {
      return false;
    }
  }
  return true;
}

bool Parse_Mosaic_Id(const pugi::xml_attribute attribute, std::size_t *id)
{
  const std::string text = attribute.value();
  if (!attribute || text.empty())
  {
    return false;
  }
  std::size_t value = 0;
  for (const char digit : text)
  {
    if (digit < '0' || digit > '9')
    {
      return false;
    }
    const auto number = static_cast<std::size_t>(digit - '0');
    if (value > (std::numeric_limits<std::size_t>::max() - number) / 10)
    {
      return false;
    }
    value = value * 10 + number;
  }
  if (value == 0)
  {
    return false;
  }
  *id = value;
  return true;
}

bool Parse_Mosaic_Offset(const pugi::xml_attribute attribute, double *value)
{
  if (!attribute)
  {
    return false;
  }
  const std::string text = attribute.value();
  if (text.empty() || text.find_first_not_of("0123456789+-.eE") != std::string::npos)
    return false;
  std::istringstream input(text);
  input.imbue(std::locale::classic());
  double parsed = 0;
  input >> std::noskipws >> parsed;
  if (!input || !input.eof() || !std::isfinite(parsed))
  {
    return false;
  }
  *value = parsed;
  return true;
}

bool Parse_Mosaic_Bool(const pugi::xml_attribute attribute, bool *value)
{
  const std::string text = attribute.value();
  if (attribute && (text == "true" || text == "1"))
  {
    *value = true;
    return true;
  }
  if (attribute && (text == "false" || text == "0"))
  {
    *value = false;
    return true;
  }
  return false;
}

bool Has_Only_Attributes(
  const pugi::xml_node node,
  std::initializer_list<const char *> allowed)
{
  std::unordered_set<std::string> names;
  for (const auto attribute : node.attributes())
  {
    const std::string name = attribute.name();
    bool known = false;
    for (const auto candidate : allowed)
    {
      known = known || name == candidate;
    }
    if (!known || !names.insert(name).second)
    {
      return false;
    }
  }
  return true;
}

bool Load_Mosaic_Groups(
  const pugi::xml_node root,
  Mosaic_Configuration *configuration,
  std::string *error_message)
{
  const auto groups = root.child("MosaicGroups");
  if (!groups)
  {
    return true;
  }
  if (groups.next_sibling("MosaicGroups") ||
      !Has_Only_Attributes(groups, {}))
  {
    Set_Error(error_message, "MosaicGroups is duplicated or has unknown attributes");
    return false;
  }
  for (const auto node : groups.children())
  {
    const std::string context =
      "Mosaic group " + std::string(node.attribute("id").value()) +
      " (" + node.attribute("name").value() + "): ";
    const auto fail = [&](const std::string &message)
    {
      Set_Error(error_message, context + message);
      return false;
    };
    if (node.type() != pugi::node_element ||
        std::string(node.name()) != "Group" ||
        !Has_Only_Attributes(node, {
          "id", "name", "reference", "horizontalAxis", "verticalAxis",
          "reverseHorizontal", "reverseVertical"}))
    {
      return fail("unexpected XML node or attributes");
    }
    Mosaic_Group group;
    if (!Parse_Mosaic_Id(node.attribute("id"), &group.id))
    {
      return fail("ID is not a nonzero unsigned integer");
    }
    group.name = node.attribute("name").value();
    const std::string reference = node.attribute("reference").value();
    if (reference == "first_camera")
    {
      group.reference = Mosaic_Reference::First_Camera;
    }
    else if (reference == "robot_base")
    {
      group.reference = Mosaic_Reference::Robot_Base;
    }
    else
    {
      return fail("reference is invalid or missing");
    }
    if (!Parse_Coordinate_Axis(
          node.attribute("horizontalAxis").value(), &group.horizontal_axis) ||
        !Parse_Coordinate_Axis(
          node.attribute("verticalAxis").value(), &group.vertical_axis))
    {
      return fail("axis is invalid or missing");
    }
    if (!Parse_Mosaic_Bool(
          node.attribute("reverseHorizontal"), &group.reverse_horizontal) ||
        !Parse_Mosaic_Bool(
          node.attribute("reverseVertical"), &group.reverse_vertical))
    {
      return fail("reverse flag is invalid or missing");
    }
    for (const auto point : node.children())
    {
      if (point.type() == pugi::node_element &&
          std::string(point.name()) == "Correction")
      {
        Mosaic_Point_Correction correction;
        if (!Has_Only_Attributes(point, {"pointId", "u", "v"}) ||
            point.first_child() ||
            !Parse_Mosaic_Id(point.attribute("pointId"), &correction.point_id) ||
            !Parse_Mosaic_Offset(point.attribute("u"), &correction.offset_u_mm) ||
            !Parse_Mosaic_Offset(point.attribute("v"), &correction.offset_v_mm))
        {
          return fail("point " + std::string(point.attribute("pointId").value()) +
            " correction XML is invalid");
        }
        group.corrections.push_back(correction);
        continue;
      }
      std::size_t id = 0;
      if (point.type() != pugi::node_element ||
          std::string(point.name()) != "PointRef" ||
          !Has_Only_Attributes(point, {"id"}) || point.first_child() ||
          !Parse_Mosaic_Id(point.attribute("id"), &id))
      {
        return fail(
          "point " + std::string(point.attribute("id").value()) +
          " reference XML is invalid");
      }
      group.point_ids.push_back(id);
    }
    configuration->groups.push_back(std::move(group));
  }
  return true;
}

} // namespace

bool Save_Robot_Progress(
  const std::filesystem::path &path,
  const Robot_Progress_File &progress,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (path.empty() || progress.robot_model_id.empty())
  {
    Set_Error(error_message, "Progress path or robot model is empty");
    return false;
  }
  if (progress.points.empty())
  {
    Set_Error(error_message, "Progress has no points");
    return false;
  }

  if (!Validate_Mosaic_Configuration(
        progress.mosaic_configuration, progress.points, error_message))
  {
    return false;
  }

  pugi::xml_document document;
  auto root = document.append_child("RobotProgress");
  root.append_attribute("version") = 5;
  root.append_attribute("robotModel") =
    progress.robot_model_id.c_str();
  std::unordered_set<std::size_t> ids;
  for (const auto &point : progress.points)
  {
    if (point.id == 0 ||
        ids.count(point.id) != 0 ||
        !Is_Finite_Point(point))
    {
      Set_Error(error_message, "Progress contains an invalid point");
      return false;
    }
    ids.insert(point.id);
    auto node = root.append_child("Point");
    node.append_attribute("id") =
      static_cast<unsigned long long>(point.id);
    node.append_attribute("name") =
      Format_Teach_Point_Name(point.id).c_str();
    node.append_attribute("type") =
      Robot_Teach_Point_Type_Id(point.type);
    for (std::size_t index = 0; index < 6; ++index)
    {
      const std::string name = "a" + std::to_string(index + 1);
      node.append_attribute(name.c_str()) =
        point.joint_angles_deg[index];
    }
    node.append_attribute("x") = point.world_pose[0];
    node.append_attribute("y") = point.world_pose[1];
    node.append_attribute("z") = point.world_pose[2];
    node.append_attribute("a") = point.world_pose[3];
    node.append_attribute("b") = point.world_pose[4];
    node.append_attribute("c") = point.world_pose[5];
    node.append_attribute("hasWorldPose") = point.has_world_pose;
    node.append_attribute("pointCloud") = point.point_cloud_path.c_str();
    node.append_attribute("pointCloudName") =
      point.point_cloud_name.c_str();
    node.append_attribute("templateId") =
      point.template_id.c_str();
    node.append_attribute("templateName") =
      point.template_name.c_str();
    node.append_attribute("hasTemplate") =
      point.has_template;
    node.append_attribute("templateX") =
      point.template_reference_pose[0];
    node.append_attribute("templateY") =
      point.template_reference_pose[1];
    node.append_attribute("templateZ") =
      point.template_reference_pose[2];
    node.append_attribute("templateA") =
      point.template_reference_pose[3];
    node.append_attribute("templateB") =
      point.template_reference_pose[4];
    node.append_attribute("templateC") =
      point.template_reference_pose[5];
    node.append_attribute("coordinateId") =
      point.coordinate_frame_id.c_str();
    node.append_attribute("coordinateName") =
      point.coordinate_frame_name.c_str();
    node.append_attribute("hasCoordinate") =
      point.has_coordinate_frame;
    node.append_attribute("coordinateX") =
      point.flange_from_coordinate_pose[0];
    node.append_attribute("coordinateY") =
      point.flange_from_coordinate_pose[1];
    node.append_attribute("coordinateZ") =
      point.flange_from_coordinate_pose[2];
    node.append_attribute("coordinateA") =
      point.flange_from_coordinate_pose[3];
    node.append_attribute("coordinateB") =
      point.flange_from_coordinate_pose[4];
    node.append_attribute("coordinateC") =
      point.flange_from_coordinate_pose[5];
  }
  auto groups = root.append_child("MosaicGroups");
  for (const auto &group : progress.mosaic_configuration.groups)
  {
    auto node = groups.append_child("Group");
    node.append_attribute("id") = static_cast<unsigned long long>(group.id);
    node.append_attribute("name") = group.name.c_str();
    node.append_attribute("reference") =
      group.reference == Mosaic_Reference::First_Camera
        ? "first_camera" : "robot_base";
    node.append_attribute("horizontalAxis") =
      Coordinate_Axis_Name(group.horizontal_axis);
    node.append_attribute("verticalAxis") =
      Coordinate_Axis_Name(group.vertical_axis);
    node.append_attribute("reverseHorizontal") = group.reverse_horizontal;
    node.append_attribute("reverseVertical") = group.reverse_vertical;
    for (const auto id : group.point_ids)
    {
      node.append_child("PointRef").append_attribute("id") =
        static_cast<unsigned long long>(id);
    }
    for (const auto &correction : group.corrections)
    {
      auto entry = node.append_child("Correction");
      entry.append_attribute("pointId") =
        static_cast<unsigned long long>(correction.point_id);
      entry.append_attribute("u").set_value(
        correction.offset_u_mm, std::numeric_limits<double>::max_digits10);
      entry.append_attribute("v").set_value(
        correction.offset_v_mm, std::numeric_limits<double>::max_digits10);
    }
  }
  if (!document.save_file(path.wstring().c_str(), "  "))
  {
    Set_Error(error_message, "Unable to save Progress file");
    return false;
  }
  return true;
}

bool Load_Robot_Progress(
  const std::filesystem::path &path,
  Robot_Progress_File *progress,
  std::string *error_message)
{
  if (error_message)
  {
    error_message->clear();
  }
  if (!progress)
  {
    Set_Error(error_message, "Progress output is null");
    return false;
  }

  pugi::xml_document document;
  const auto loaded = document.load_file(path.wstring().c_str());
  if (!loaded)
  {
    Set_Error(error_message, "Unable to parse Progress file");
    return false;
  }
  const auto root = document.child("RobotProgress");
  const std::string model_id =
    root.attribute("robotModel").as_string();
  if (!root || model_id.empty())
  {
    Set_Error(error_message, "Progress file header is invalid");
    return false;
  }

  Robot_Progress_File parsed;
  parsed.robot_model_id = model_id;
  std::unordered_set<std::size_t> ids;
  for (const auto node : root.children("Point"))
  {
    if (!node.attribute("id") ||
        !node.attribute("x") ||
        !node.attribute("y") ||
        !node.attribute("z") ||
        !node.attribute("a") ||
        !node.attribute("b") ||
        !node.attribute("c") ||
        !node.attribute("hasWorldPose"))
    {
      Set_Error(error_message, "Progress point fields are incomplete");
      return false;
    }
    Robot_Teach_Point point;
    point.id = static_cast<std::size_t>(
      node.attribute("id").as_ullong());
    point.robot_model_id = model_id;
    if (!Parse_Robot_Teach_Point_Type(
          node.attribute("type").as_string(),
          &point.type))
    {
      Set_Error(error_message, "Progress point type is invalid");
      return false;
    }
    for (std::size_t index = 0; index < 6; ++index)
    {
      const std::string name = "a" + std::to_string(index + 1);
      if (!node.attribute(name.c_str()))
      {
        Set_Error(
          error_message,
          "Progress point joint fields are incomplete");
        return false;
      }
      point.joint_angles_deg[index] =
        node.attribute(name.c_str()).as_double();
    }
    point.world_pose = {
      node.attribute("x").as_double(),
      node.attribute("y").as_double(),
      node.attribute("z").as_double(),
      node.attribute("a").as_double(),
      node.attribute("b").as_double(),
      node.attribute("c").as_double()};
    point.has_world_pose =
      node.attribute("hasWorldPose").as_bool(false);
    point.point_cloud_path =
      node.attribute("pointCloud").as_string();
    point.point_cloud_name =
      node.attribute("pointCloudName").as_string();
    point.template_id =
      node.attribute("templateId").as_string();
    point.template_name =
      node.attribute("templateName").as_string();
    point.has_template =
      node.attribute("hasTemplate").as_bool(
        !point.template_id.empty());
    point.template_reference_pose = {
      node.attribute("templateX").as_double(),
      node.attribute("templateY").as_double(),
      node.attribute("templateZ").as_double(),
      node.attribute("templateA").as_double(),
      node.attribute("templateB").as_double(),
      node.attribute("templateC").as_double()};
    point.coordinate_frame_id =
      node.attribute("coordinateId").as_string();
    point.coordinate_frame_name =
      node.attribute("coordinateName").as_string();
    point.has_coordinate_frame =
      node.attribute("hasCoordinate").as_bool(
        !point.coordinate_frame_id.empty());
    point.flange_from_coordinate_pose = {
      node.attribute("coordinateX").as_double(),
      node.attribute("coordinateY").as_double(),
      node.attribute("coordinateZ").as_double(),
      node.attribute("coordinateA").as_double(),
      node.attribute("coordinateB").as_double(),
      node.attribute("coordinateC").as_double()};
    if (point.id == 0 || ids.count(point.id) != 0)
    {
      Set_Error(error_message, "Progress contains an invalid point ID " +
        std::to_string(point.id));
      return false;
    }
    ids.insert(point.id);
    parsed.points.push_back(std::move(point));
  }
  if (parsed.points.empty())
  {
    Set_Error(error_message, "Progress file has no points");
    return false;
  }
  if (!Load_Mosaic_Groups(root, &parsed.mosaic_configuration, error_message) ||
      !Validate_Mosaic_Configuration(
        parsed.mosaic_configuration, parsed.points, error_message))
  {
    return false;
  }
  for (const auto &point : parsed.points)
  {
    if (!Is_Finite_Point(point))
    {
      Set_Error(error_message, "Progress contains an invalid point " +
        std::to_string(point.id));
      return false;
    }
  }
  *progress = std::move(parsed);
  return true;
}

} // namespace robot_model

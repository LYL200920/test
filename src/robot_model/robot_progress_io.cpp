#include "robot_progress_io.h"

#include "pugixml.hpp"

#include <cmath>
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

  pugi::xml_document document;
  auto root = document.append_child("RobotProgress");
  root.append_attribute("version") = 3;
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
    if (point.id == 0 ||
        ids.count(point.id) != 0 ||
        !Is_Finite_Point(point))
    {
      Set_Error(error_message, "Progress contains an invalid point");
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
  *progress = std::move(parsed);
  return true;
}

} // namespace robot_model

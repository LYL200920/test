#include "progress_flow_transform.h"

#include <unordered_map>

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

bool Collect_Progress_Template_References(
  const std::vector<Robot_Teach_Point> &points,
  std::vector<Progress_Template_Reference> *references,
  std::string *error_message)
{
  if (!references)
  {
    Set_Error(error_message, "Template reference output is null");
    return false;
  }
  references->clear();
  std::unordered_map<std::string, std::size_t> indices;
  for (const auto &point : points)
  {
    if (!point.has_template || point.template_id.empty())
    {
      Set_Error(error_message, "Progress contains an unbound template");
      return false;
    }
    const auto found = indices.find(point.template_id);
    if (found == indices.end())
    {
      indices.emplace(point.template_id, references->size());
      references->push_back({
        point.template_id,
        point.template_name,
        point.template_reference_pose});
    }
    else if ((*references)[found->second].reference_pose !=
             point.template_reference_pose)
    {
      Set_Error(error_message, "One template has inconsistent reference poses");
      return false;
    }
  }
  if (references->empty())
  {
    Set_Error(error_message, "Progress has no template references");
    return false;
  }
  return true;
}

bool Transform_Progress_Teach_Points(
  const std::vector<Robot_Teach_Point> &points,
  const std::vector<XyzabcPose> &recognized_poses,
  std::vector<XyzabcPose> *transformed_poses,
  std::vector<Matrix4> *template_transforms,
  std::string *error_message)
{
  return Transform_Progress_Teach_Points_In_Coordinate(
    points,
    recognized_poses,
    {},
    transformed_poses,
    nullptr,
    template_transforms,
    error_message);
}

bool Transform_Progress_Teach_Points_In_Coordinate(
  const std::vector<Robot_Teach_Point> &points,
  const std::vector<XyzabcPose> &recognized_poses,
  const XyzabcPose &flange_from_coordinate_pose,
  std::vector<XyzabcPose> *transformed_poses,
  std::vector<XyzabcPose> *source_coordinate_poses,
  std::vector<Matrix4> *template_transforms,
  std::string *error_message)
{
  if (!transformed_poses)
  {
    Set_Error(error_message, "Transformed point output is null");
    return false;
  }
  std::vector<Progress_Template_Reference> references;
  if (!Collect_Progress_Template_References(
        points, &references, error_message))
  {
    return false;
  }
  if (recognized_poses.size() != references.size())
  {
    Set_Error(error_message, "Recognized pose count does not match templates");
    return false;
  }

  std::unordered_map<std::string, Matrix4> transforms_by_id;
  std::vector<Matrix4> transforms;
  transforms.reserve(references.size());
  for (std::size_t index = 0; index < references.size(); ++index)
  {
    const Matrix4 transform = Build_Relative_Pose_Transform(
      references[index].reference_pose, recognized_poses[index]);
    transforms_by_id.emplace(references[index].id, transform);
    transforms.push_back(transform);
  }

  transformed_poses->clear();
  transformed_poses->reserve(points.size());
  if (source_coordinate_poses)
  {
    source_coordinate_poses->clear();
    source_coordinate_poses->reserve(points.size());
  }
  const Matrix4 flange_from_coordinate =
    Build_Zyx_Pose_Matrix(flange_from_coordinate_pose);
  for (const auto &point : points)
  {
    if (!point.has_world_pose)
    {
      Set_Error(error_message, "Progress contains a point without world pose");
      transformed_poses->clear();
      if (source_coordinate_poses) source_coordinate_poses->clear();
      return false;
    }
    const auto found = transforms_by_id.find(point.template_id);
    if (found == transforms_by_id.end())
    {
      Set_Error(error_message, "Progress point template transform is missing");
      transformed_poses->clear();
      if (source_coordinate_poses) source_coordinate_poses->clear();
      return false;
    }
    const Matrix4 world_from_coordinate = Multiply_Matrices(
      Build_Zyx_Pose_Matrix(point.world_pose),
      flange_from_coordinate);
    const XyzabcPose source_coordinate_pose =
      Build_Xyzabc_From_Zyx_Matrix(world_from_coordinate);
    if (source_coordinate_poses)
    {
      source_coordinate_poses->push_back(source_coordinate_pose);
    }
    transformed_poses->push_back(
      Transform_Xyzabc_Pose(found->second, source_coordinate_pose));
  }
  if (template_transforms)
  {
    *template_transforms = std::move(transforms);
  }
  return true;
}

} // namespace robot_model

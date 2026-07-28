#include "robot_teach_point_store.h"
#include "point_cloud_template_binding.h"

#include <algorithm>
#include <unordered_set>

namespace robot_model
{
  namespace
  {

    const std::vector<Robot_Teach_Point> EMPTY_POINTS;

  } // namespace

  std::string Format_Teach_Point_Name(std::size_t id)
  {
    return "P[" + std::to_string(id) + "]";
  }

  const char *Robot_Teach_Point_Type_Id(Robot_Teach_Point_Type type)
  {
    switch (type)
    {
      case Robot_Teach_Point_Type::Transition:
        return "transition";
      case Robot_Teach_Point_Type::Wait:
        return "wait";
      case Robot_Teach_Point_Type::Motion:
      default:
        return "motion";
    }
  }

  bool Parse_Robot_Teach_Point_Type(
      const std::string &text,
      Robot_Teach_Point_Type *type)
  {
    if (!type)
    {
      return false;
    }
    if (text.empty() || text == "motion")
    {
      *type = Robot_Teach_Point_Type::Motion;
      return true;
    }
    if (text == "transition")
    {
      *type = Robot_Teach_Point_Type::Transition;
      return true;
    }
    if (text == "wait")
    {
      *type = Robot_Teach_Point_Type::Wait;
      return true;
    }
    return false;
  }

  void Robot_Teach_Point_Store::Renumber_Points(
      Model_Points &model_points)
  {
    for (std::size_t index = 0; index < model_points.points.size(); ++index)
    {
      model_points.points[index].id = index + 1;
    }
  }

  const Robot_Teach_Point &Robot_Teach_Point_Store::Add_Point(const std::string &robot_model_id,
                                                              const std::array<double, 6> &joint_angles_deg,
                                                              const XyzabcPose &world_pose,
                                                              const std::string &point_cloud_path,
                                                              const std::string &coordinate_frame_id,
                                                              const std::string &coordinate_frame_name,
                                                              const XyzabcPose &flange_from_coordinate_pose,
                                                              const std::string &point_cloud_name,
                                                              Robot_Teach_Point_Type type,
                                                              const std::string &template_id,
                                                              const std::string &template_name,
                                                              const XyzabcPose &template_reference_pose)
  {
    auto &model_points = m_points_by_model[robot_model_id];
    Robot_Teach_Point point;
    point.id = model_points.points.size() + 1;
    point.robot_model_id = robot_model_id;
    point.joint_angles_deg = joint_angles_deg;
    point.world_pose = world_pose;
    point.has_world_pose = true;
    point.point_cloud_path = point_cloud_path;
    point.point_cloud_name = point_cloud_name;
    point.template_id = template_id;
    point.template_name = template_name;
    point.template_reference_pose = template_reference_pose;
    point.has_template = !template_id.empty();
    point.coordinate_frame_id = coordinate_frame_id;
    point.coordinate_frame_name = coordinate_frame_name;
    point.flange_from_coordinate_pose = flange_from_coordinate_pose;
    point.has_coordinate_frame = !coordinate_frame_id.empty();
    point.type = type;
    model_points.points.push_back(point);
    return model_points.points.back();
  }

  const Robot_Teach_Point &Robot_Teach_Point_Store::Insert_Point(
      const std::string &robot_model_id,
      std::size_t index,
      const std::array<double, 6> &joint_angles_deg,
      const XyzabcPose &world_pose,
      const std::string &point_cloud_path,
      const std::string &coordinate_frame_id,
      const std::string &coordinate_frame_name,
      const XyzabcPose &flange_from_coordinate_pose,
      const std::string &point_cloud_name,
      Robot_Teach_Point_Type type,
      const std::string &template_id,
      const std::string &template_name,
      const XyzabcPose &template_reference_pose)
  {
    auto &model_points = m_points_by_model[robot_model_id];
    Robot_Teach_Point point;
    point.robot_model_id = robot_model_id;
    point.joint_angles_deg = joint_angles_deg;
    point.world_pose = world_pose;
    point.has_world_pose = true;
    point.point_cloud_path = point_cloud_path;
    point.point_cloud_name = point_cloud_name;
    point.template_id = template_id;
    point.template_name = template_name;
    point.template_reference_pose = template_reference_pose;
    point.has_template = !template_id.empty();
    point.coordinate_frame_id = coordinate_frame_id;
    point.coordinate_frame_name = coordinate_frame_name;
    point.flange_from_coordinate_pose = flange_from_coordinate_pose;
    point.has_coordinate_frame = !coordinate_frame_id.empty();
    point.type = type;
    const auto insertion = model_points.points.begin() +
      static_cast<std::ptrdiff_t>(
        std::min(index, model_points.points.size()));
    const std::size_t insertion_index =
      static_cast<std::size_t>(insertion - model_points.points.begin());
    model_points.points.insert(insertion, std::move(point));
    Renumber_Points(model_points);
    return model_points.points[insertion_index];
  }

  bool Robot_Teach_Point_Store::Update_Point(
      const std::string &robot_model_id,
      std::size_t index,
      const std::array<double, 6> &joint_angles_deg,
      const XyzabcPose &world_pose,
      const std::string &point_cloud_path,
      const std::string &coordinate_frame_id,
      const std::string &coordinate_frame_name,
      const XyzabcPose &flange_from_coordinate_pose,
      const std::string &point_cloud_name,
      Robot_Teach_Point_Type type,
      const std::string &template_id,
      const std::string &template_name,
      const XyzabcPose &template_reference_pose)
  {
    auto found = m_points_by_model.find(robot_model_id);
    if (found == m_points_by_model.end() ||
        index >= found->second.points.size())
    {
      return false;
    }
    auto &point = found->second.points[index];
    point.joint_angles_deg = joint_angles_deg;
    point.world_pose = world_pose;
    point.has_world_pose = true;
    point.point_cloud_path = point_cloud_path;
    point.point_cloud_name = point_cloud_name;
    point.template_id = template_id;
    point.template_name = template_name;
    point.template_reference_pose = template_reference_pose;
    point.has_template = !template_id.empty();
    point.coordinate_frame_id = coordinate_frame_id;
    point.coordinate_frame_name = coordinate_frame_name;
    point.flange_from_coordinate_pose = flange_from_coordinate_pose;
    point.has_coordinate_frame = !coordinate_frame_id.empty();
    point.type = type;
    return true;
  }

  void Robot_Teach_Point_Store::Replace_Joint_Points(const std::string &robot_model_id,
                                                     const std::vector<std::array<double, 6>> &joint_points)
  {
    auto &model_points = m_points_by_model[robot_model_id];
    model_points.points.clear();
    model_points.points.reserve(joint_points.size());
    for (const auto &joint_angles : joint_points)
    {
      Robot_Teach_Point point;
      point.id = model_points.points.size() + 1;
      point.robot_model_id = robot_model_id;
      point.joint_angles_deg = joint_angles;
      model_points.points.push_back(point);
    }
  }

  void Robot_Teach_Point_Store::Replace_Points(
      const std::string &robot_model_id,
      const std::vector<Robot_Teach_Point> &points)
  {
    auto &model_points = m_points_by_model[robot_model_id];
    model_points.points = points;
    for (auto &point : model_points.points)
    {
      point.robot_model_id = robot_model_id;
    }
    Renumber_Points(model_points);
  }

  bool Robot_Teach_Point_Store::Delete_Point(const std::string &robot_model_id,
                                             std::size_t index)
  {
    auto found = m_points_by_model.find(robot_model_id);
    if (found == m_points_by_model.end() || index >= found->second.points.size())
      return false;
    found->second.points.erase(found->second.points.begin() + static_cast<std::ptrdiff_t>(index));
    Renumber_Points(found->second);
    return true;
  }

  std::size_t Robot_Teach_Point_Store::Delete_Points(
      const std::string &robot_model_id,
      const std::vector<std::size_t> &indices)
  {
    auto found = m_points_by_model.find(robot_model_id);
    if (found == m_points_by_model.end())
    {
      return 0;
    }
    std::vector<std::size_t> sorted = indices;
    std::sort(sorted.begin(), sorted.end(), std::greater<std::size_t>());
    sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
    std::size_t deleted = 0;
    for (const std::size_t index : sorted)
    {
      if (index < found->second.points.size())
      {
        found->second.points.erase(
          found->second.points.begin() +
          static_cast<std::ptrdiff_t>(index));
        ++deleted;
      }
    }
    if (deleted > 0)
    {
      Renumber_Points(found->second);
    }
    return deleted;
  }

  void Robot_Teach_Point_Store::Clear_Points(const std::string &robot_model_id)
  {
    auto found = m_points_by_model.find(robot_model_id);
    if (found != m_points_by_model.end())
      found->second.points.clear();
  }

  std::size_t Robot_Teach_Point_Store::Apply_Template_To_Point_Cloud(
      const std::string &robot_model_id,
      const std::string &point_cloud_path,
      const std::string &template_id,
      const std::string &template_name,
      const XyzabcPose &template_reference_pose)
  {
    const auto found = m_points_by_model.find(robot_model_id);
    if (found == m_points_by_model.end())
    {
      return 0;
    }
    const std::string target_key = Point_Cloud_Binding_Key(
      std::filesystem::u8path(point_cloud_path));
    std::size_t updated = 0;
    for (auto &point : found->second.points)
    {
      if (Point_Cloud_Binding_Key(
            std::filesystem::u8path(point.point_cloud_path)) != target_key)
      {
        continue;
      }
      point.template_id = template_id;
      point.template_name = template_name;
      point.template_reference_pose = template_reference_pose;
      point.has_template = !template_id.empty();
      ++updated;
    }
    return updated;
  }

  const std::vector<Robot_Teach_Point> &Robot_Teach_Point_Store::Points(const std::string &robot_model_id) const
  {
    const auto found = m_points_by_model.find(robot_model_id);
    return found == m_points_by_model.end() ? EMPTY_POINTS
                                            : found->second.points;
  }

  std::size_t Robot_Teach_Point_Store::Point_Count(const std::string &robot_model_id) const
  {
    return Points(robot_model_id).size();
  }

} // namespace robot_model

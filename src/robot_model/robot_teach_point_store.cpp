#include "robot_teach_point_store.h"

namespace robot_model
{
namespace
{

const std::vector<Robot_Teach_Point> EMPTY_POINTS;

} // namespace

std::string Format_Teach_Point_Name (std::size_t id)
{
  return "P[" + std::to_string (id) + "]";
}

const Robot_Teach_Point& Robot_Teach_Point_Store::Add_Point (
  const std::string& robot_model_id,
  const std::array<double, 6>& joint_angles_deg,
  const XyzabcPose& world_pose)
{
  auto& model_points = m_points_by_model[robot_model_id];
  Robot_Teach_Point point;
  point.id = model_points.next_id++;
  point.robot_model_id = robot_model_id;
  point.joint_angles_deg = joint_angles_deg;
  point.world_pose = world_pose;
  point.has_world_pose = true;
  model_points.points.push_back (point);
  return model_points.points.back ( );
}

void Robot_Teach_Point_Store::Replace_Joint_Points (
  const std::string& robot_model_id,
  const std::vector<std::array<double, 6>>& joint_points)
{
  auto& model_points = m_points_by_model[robot_model_id];
  model_points.points.clear ( );
  model_points.next_id = 1;
  model_points.points.reserve (joint_points.size ( ));
  for( const auto& joint_angles : joint_points )
  {
    Robot_Teach_Point point;
    point.id = model_points.next_id++;
    point.robot_model_id = robot_model_id;
    point.joint_angles_deg = joint_angles;
    model_points.points.push_back (point);
  }
}

bool Robot_Teach_Point_Store::Delete_Point (
  const std::string& robot_model_id,
  std::size_t index)
{
  auto found = m_points_by_model.find (robot_model_id);
  if( found == m_points_by_model.end ( ) ||
      index >= found->second.points.size ( ) ) return false;
  found->second.points.erase (
    found->second.points.begin ( ) + static_cast<std::ptrdiff_t> (index));
  return true;
}

void Robot_Teach_Point_Store::Clear_Points (
  const std::string& robot_model_id)
{
  auto found = m_points_by_model.find (robot_model_id);
  if( found != m_points_by_model.end ( ) ) found->second.points.clear ( );
}

const std::vector<Robot_Teach_Point>& Robot_Teach_Point_Store::Points (
  const std::string& robot_model_id) const
{
  const auto found = m_points_by_model.find (robot_model_id);
  return found == m_points_by_model.end ( )
    ? EMPTY_POINTS : found->second.points;
}

std::size_t Robot_Teach_Point_Store::Point_Count (
  const std::string& robot_model_id) const
{
  return Points (robot_model_id).size ( );
}

} // namespace robot_model

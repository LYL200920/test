#include "robot_drag_performance_collector.h"

#include <algorithm>

namespace robot_model
{

void Robot_Drag_Performance_Collector::Begin ( )
{
  m_stats = { };
  m_active = true;
}

bool Robot_Drag_Performance_Collector::Active ( ) const
{
  return m_active;
}

void Robot_Drag_Performance_Collector::Record (
  const Robot_Drag_Performance_Sample& sample)
{
  if( !m_active ) return;

  ++m_stats.update_count;
  if( !sample.accepted && sample.collision.collided )
  {
    ++m_stats.blocked_update_count;
    switch( sample.collision.type )
    {
      case Robot_Collision_Type::Obstacle_Point:
        ++m_stats.obstacle_blocked_update_count;
        break;
      case Robot_Collision_Type::Self_Collision:
        ++m_stats.self_blocked_update_count;
        break;
      case Robot_Collision_Type::Ground:
        ++m_stats.ground_blocked_update_count;
        break;
      default:
        break;
    }
  }
  m_stats.checked_pose_count += sample.checked_pose_count;
  m_stats.obstacle_candidate_points +=
    sample.collision_query_stats.obstacle_candidate_points;
  m_stats.obstacle_distance_queries +=
    sample.collision_query_stats.obstacle_distance_queries;
  m_stats.self_exact_pair_queries +=
    sample.collision_query_stats.self_exact_pair_queries;
  m_stats.total_update_time_ms += sample.update_time_ms;
  m_stats.maximum_update_time_ms = std::max (
    m_stats.maximum_update_time_ms, sample.update_time_ms);
  m_stats.total_ik_time_ms += sample.ik_time_ms;
  m_stats.total_collision_time_ms += sample.collision_time_ms;
  if( sample.rendered )
    m_stats.total_render_time_ms += sample.render_time_ms;
}

bool Robot_Drag_Performance_Collector::Finish (
  Robot_Drag_Performance_Stats* stats)
{
  if( !m_active ) return false;
  m_active = false;
  if( stats ) *stats = m_stats;
  return true;
}

} // namespace robot_model

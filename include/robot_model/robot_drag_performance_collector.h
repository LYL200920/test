#ifndef includeguard_robot_drag_performance_collector_h_includeguard
#define includeguard_robot_drag_performance_collector_h_includeguard

#include "robot_collision_detector.h"

#include <cstddef>

namespace robot_model
{

struct Robot_Drag_Performance_Stats
{
  std::size_t update_count = 0;
  std::size_t blocked_update_count = 0;
  std::size_t checked_pose_count = 0;
  std::size_t obstacle_candidate_points = 0;
  std::size_t obstacle_distance_queries = 0;
  std::size_t self_exact_pair_queries = 0;
  std::size_t obstacle_blocked_update_count = 0;
  std::size_t self_blocked_update_count = 0;
  std::size_t ground_blocked_update_count = 0;
  double total_update_time_ms = 0.0;
  double maximum_update_time_ms = 0.0;
  double total_ik_time_ms = 0.0;
  double total_collision_time_ms = 0.0;
  double total_render_time_ms = 0.0;
};

struct Robot_Drag_Performance_Sample
{
  bool accepted = false;
  Robot_Collision_Result collision;
  std::size_t checked_pose_count = 0;
  Robot_Collision_Query_Stats collision_query_stats;
  double update_time_ms = 0.0;
  double ik_time_ms = 0.0;
  double collision_time_ms = 0.0;
  double render_time_ms = 0.0;
  bool rendered = false;
};

class Robot_Drag_Performance_Collector
{
public:
  void Begin ( );
  bool Active ( ) const;
  void Record (const Robot_Drag_Performance_Sample& sample);
  bool Finish (Robot_Drag_Performance_Stats* stats);

private:
  Robot_Drag_Performance_Stats m_stats;
  bool m_active = false;
};

} // namespace robot_model

#endif

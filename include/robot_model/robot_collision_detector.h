#ifndef includeguard_robot_collision_detector_h_includeguard
#define includeguard_robot_collision_detector_h_includeguard

#include "pose_transform.h"
#include "robot_visual_data.h"

#include <cstddef>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace robot_model
{

class Robot_Collision_Detector;

// Immutable point-cloud data and spatial index. A completed snapshot can be
// built off the UI thread and installed without rebuilding robot geometry.
class Point_Cloud_Collision_Snapshot
{
public:
  Point_Cloud_Collision_Snapshot ( );
  ~Point_Cloud_Collision_Snapshot ( );

  Point_Cloud_Collision_Snapshot (
    const Point_Cloud_Collision_Snapshot&) = delete;
  Point_Cloud_Collision_Snapshot& operator= (
    const Point_Cloud_Collision_Snapshot&) = delete;

  std::size_t Point_Count ( ) const;
  bool Has_Points ( ) const { return Point_Count ( ) > 0; }

private:
  class Implementation;
  std::unique_ptr<Implementation> m_implementation;
  friend class Robot_Collision_Detector;
};

enum class Robot_Collision_Type
{
  None,
  Obstacle_Point,
  Self_Collision,
  Ground
};

struct Robot_Collision_Result
{
  bool collided = false;
  Robot_Collision_Type type = Robot_Collision_Type::None;
  std::size_t robot_part_index = 0;
  std::size_t other_robot_part_index = 0;
  std::size_t obstacle_point_index = 0;
  double minimum_distance_mm = 0.0;
  // Positive means separated beyond the configured clearance; negative
  // means penetrating or inside the clearance envelope.
  double clearance_margin_mm = 0.0;
  Point3 robot_closest_point_world = { };
  Point3 obstacle_point_world = { };
};

struct Robot_Scene_Collision_Options
{
  bool check_self_collision = true;
  bool check_ground_collision = true;
  double self_collision_clearance_mm = 3.0;
  double ground_height_mm = 0.0;
  std::size_t first_ground_checked_part_index = 2;
  std::vector<std::array<std::size_t, 2>> self_collision_pairs;
  std::vector<std::size_t> ground_checked_parts;
};

struct Robot_Collision_Point_Cloud_Options
{
  double voxel_size_mm = 5.0;
  double robot_exclusion_distance_mm = 15.0;
  bool exclude_robot_points = true;
};

struct Robot_Collision_Point_Cloud_Stats
{
  std::size_t source_point_count = 0;
  std::size_t voxel_point_count = 0;
  std::size_t excluded_robot_point_count = 0;
  std::size_t collision_point_count = 0;
  double build_time_ms = 0.0;
};

struct Robot_Collision_Query_Stats
{
  std::size_t ground_sample_queries = 0;
  std::size_t self_broad_phase_pairs = 0;
  std::size_t self_obb_phase_pairs = 0;
  std::size_t self_distance_sample_queries = 0;
  std::size_t self_exact_pair_queries = 0;
  std::size_t obstacle_candidate_points = 0;
  std::size_t obstacle_distance_queries = 0;
  double minimum_self_sample_distance_mm = 0.0;
};

bool Is_Robot_Collision_Recovery_Improvement (
  const Robot_Collision_Result& original_collision,
  const Robot_Collision_Result& candidate_collision,
  double previous_clearance_margin_mm);

// VTK-backed collision query for a moving robot mesh and a static obstacle
// point cloud. Robot locators are built in each part's local coordinates so
// they remain reusable as joint transforms change.
class Robot_Collision_Detector
{
public:
  Robot_Collision_Detector ( );
  ~Robot_Collision_Detector ( );

  Robot_Collision_Detector (const Robot_Collision_Detector&) = delete;
  Robot_Collision_Detector& operator= (
    const Robot_Collision_Detector&) = delete;
  Robot_Collision_Detector (Robot_Collision_Detector&&) noexcept;
  Robot_Collision_Detector& operator= (
    Robot_Collision_Detector&&) noexcept;

  void Set_Robot_Parts (const std::vector<Robot_Visual_Part>& parts);
  void Set_Robot_Parts_For_Obstacle_Filtering (
    const std::vector<Robot_Visual_Part>& parts);
  void Set_Scene_Collision_Options (
    const Robot_Scene_Collision_Options& options);
  const Robot_Scene_Collision_Options& Scene_Collision_Options ( ) const;
  bool Set_Obstacle_Points (
    const std::vector<float>& xyz,
    std::string* error_message = nullptr);
  bool Set_Obstacle_Points (
    const std::vector<float>& xyz,
    const Robot_Collision_Point_Cloud_Options& options,
    const std::vector<Matrix4>& reference_world_from_parts,
    Robot_Collision_Point_Cloud_Stats* stats = nullptr,
    std::string* error_message = nullptr,
    const std::atomic_bool* cancel_requested = nullptr);
  void Clear_Obstacle_Points ( );
  void Set_Obstacle_Snapshot (
    std::shared_ptr<const Point_Cloud_Collision_Snapshot> snapshot);
  std::shared_ptr<const Point_Cloud_Collision_Snapshot>
  Obstacle_Snapshot ( ) const;
  void Clear ( );

  bool Has_Robot_Geometry ( ) const;
  bool Has_Obstacle_Points ( ) const;
  std::size_t Obstacle_Point_Count ( ) const;
  const Robot_Collision_Query_Stats& Last_Query_Stats ( ) const;

  Robot_Collision_Result Check_Pose (
    const std::vector<Matrix4>& world_from_parts,
    double clearance_mm) const;

private:
  void Set_Robot_Parts_Internal (
    const std::vector<Robot_Visual_Part>& parts,
    bool build_pose_query_geometry);
  class Implementation;
  std::unique_ptr<Implementation> m_implementation;
};

} // namespace robot_model

#endif

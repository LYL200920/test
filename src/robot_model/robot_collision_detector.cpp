#include "robot_collision_detector.h"

#include <vtkCollisionDetectionFilter.h>
#include <vtkIdTypeArray.h>
#include <vtkImplicitPolyDataDistance.h>
#include <vtkMatrix4x4.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkOctreePointLocator.h>
#include <vtkStaticCellLocator.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <cstdint>
#include <unordered_set>
#include <utility>

namespace robot_model
{
namespace
{

struct Voxel_Key
{
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator== (const Voxel_Key& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct Voxel_Key_Hash
{
  std::size_t operator() (const Voxel_Key& key) const
  {
    std::size_t seed = std::hash<std::int64_t> { } (key.x);
    seed ^= std::hash<std::int64_t> { } (key.y) +
      0x9e3779b9U + ( seed << 6 ) + ( seed >> 2 );
    seed ^= std::hash<std::int64_t> { } (key.z) +
      0x9e3779b9U + ( seed << 6 ) + ( seed >> 2 );
    return seed;
  }
};

Voxel_Key voxel_key (const Point3& point, double voxel_size_mm)
{
  return {
    static_cast<std::int64_t> (
      std::floor (point[0] / voxel_size_mm)),
    static_cast<std::int64_t> (
      std::floor (point[1] / voxel_size_mm)),
    static_cast<std::int64_t> (
      std::floor (point[2] / voxel_size_mm))
  };
}

bool finite_point (const Point3& point)
{
  return std::isfinite (point[0]) && std::isfinite (point[1]) &&
    std::isfinite (point[2]);
}

std::array<double, 6> transformed_bounds (
  const std::array<double, 6>& local_bounds,
  const Matrix4& world_from_part)
{
  std::array<double, 6> result = {
    std::numeric_limits<double>::infinity ( ),
    -std::numeric_limits<double>::infinity ( ),
    std::numeric_limits<double>::infinity ( ),
    -std::numeric_limits<double>::infinity ( ),
    std::numeric_limits<double>::infinity ( ),
    -std::numeric_limits<double>::infinity ( )
  };
  for( int x_side = 0; x_side < 2; ++x_side )
  {
    for( int y_side = 0; y_side < 2; ++y_side )
    {
      for( int z_side = 0; z_side < 2; ++z_side )
      {
        const auto corner = Transform_Position (
          world_from_part,
          { local_bounds[x_side], local_bounds[2 + y_side],
            local_bounds[4 + z_side] });
        for( int axis = 0; axis < 3; ++axis )
        {
          result[axis * 2] = std::min (result[axis * 2], corner[axis]);
          result[axis * 2 + 1] =
            std::max (result[axis * 2 + 1], corner[axis]);
        }
      }
    }
  }
  return result;
}

bool point_inside_inflated_bounds (
  const Point3& point,
  const std::array<double, 6>& bounds,
  double clearance)
{
  for( int axis = 0; axis < 3; ++axis )
  {
    if( point[axis] < bounds[axis * 2] - clearance ||
        point[axis] > bounds[axis * 2 + 1] + clearance )
    {
      return false;
    }
  }
  return true;
}

bool bounds_overlap (
  const std::array<double, 6>& first,
  const std::array<double, 6>& second,
  double clearance = 0.0)
{
  for( int axis = 0; axis < 3; ++axis )
  {
    if( first[axis * 2 + 1] + clearance < second[axis * 2] ||
        second[axis * 2 + 1] + clearance < first[axis * 2] )
    {
      return false;
    }
  }
  return true;
}

bool oriented_bounds_overlap (
  const std::array<double, 6>& first_bounds,
  const Matrix4& world_from_first,
  const std::array<double, 6>& second_bounds,
  const Matrix4& world_from_second)
{
  const Point3 first_center_local = {
    ( first_bounds[0] + first_bounds[1] ) * 0.5,
    ( first_bounds[2] + first_bounds[3] ) * 0.5,
    ( first_bounds[4] + first_bounds[5] ) * 0.5
  };
  const Point3 second_center_local = {
    ( second_bounds[0] + second_bounds[1] ) * 0.5,
    ( second_bounds[2] + second_bounds[3] ) * 0.5,
    ( second_bounds[4] + second_bounds[5] ) * 0.5
  };
  const auto first_center = Transform_Position (
    world_from_first, first_center_local);
  const auto second_center = Transform_Position (
    world_from_second, second_center_local);
  const std::array<double, 3> first_half = {
    ( first_bounds[1] - first_bounds[0] ) * 0.5,
    ( first_bounds[3] - first_bounds[2] ) * 0.5,
    ( first_bounds[5] - first_bounds[4] ) * 0.5
  };
  const std::array<double, 3> second_half = {
    ( second_bounds[1] - second_bounds[0] ) * 0.5,
    ( second_bounds[3] - second_bounds[2] ) * 0.5,
    ( second_bounds[5] - second_bounds[4] ) * 0.5
  };
  std::array<Point3, 3> first_axes = { };
  std::array<Point3, 3> second_axes = { };
  for( int axis = 0; axis < 3; ++axis )
  {
    first_axes[axis] = {
      world_from_first[0][axis], world_from_first[1][axis],
      world_from_first[2][axis]
    };
    second_axes[axis] = {
      world_from_second[0][axis], world_from_second[1][axis],
      world_from_second[2][axis]
    };
  }
  const auto dot = [] (const Point3& first, const Point3& second)
  {
    return first[0] * second[0] + first[1] * second[1] +
      first[2] * second[2];
  };
  double rotation[3][3] = { };
  double absolute_rotation[3][3] = { };
  for( int first_axis = 0; first_axis < 3; ++first_axis )
  {
    for( int second_axis = 0; second_axis < 3; ++second_axis )
    {
      rotation[first_axis][second_axis] = dot (
        first_axes[first_axis], second_axes[second_axis]);
      absolute_rotation[first_axis][second_axis] =
        std::abs (rotation[first_axis][second_axis]) + 1.0e-9;
    }
  }
  const Point3 center_delta = {
    second_center[0] - first_center[0],
    second_center[1] - first_center[1],
    second_center[2] - first_center[2]
  };
  const std::array<double, 3> translation = {
    dot (center_delta, first_axes[0]),
    dot (center_delta, first_axes[1]),
    dot (center_delta, first_axes[2])
  };

  for( int axis = 0; axis < 3; ++axis )
  {
    const double second_radius =
      second_half[0] * absolute_rotation[axis][0] +
      second_half[1] * absolute_rotation[axis][1] +
      second_half[2] * absolute_rotation[axis][2];
    if( std::abs (translation[axis]) >
        first_half[axis] + second_radius ) return false;
  }
  for( int axis = 0; axis < 3; ++axis )
  {
    const double first_radius =
      first_half[0] * absolute_rotation[0][axis] +
      first_half[1] * absolute_rotation[1][axis] +
      first_half[2] * absolute_rotation[2][axis];
    const double projected_translation = std::abs (
      translation[0] * rotation[0][axis] +
      translation[1] * rotation[1][axis] +
      translation[2] * rotation[2][axis]);
    if( projected_translation > first_radius + second_half[axis] )
      return false;
  }
  for( int first_axis = 0; first_axis < 3; ++first_axis )
  {
    const int first_next = ( first_axis + 1 ) % 3;
    const int first_last = ( first_axis + 2 ) % 3;
    for( int second_axis = 0; second_axis < 3; ++second_axis )
    {
      const int second_next = ( second_axis + 1 ) % 3;
      const int second_last = ( second_axis + 2 ) % 3;
      const double first_radius =
        first_half[first_next] *
          absolute_rotation[first_last][second_axis] +
        first_half[first_last] *
          absolute_rotation[first_next][second_axis];
      const double second_radius =
        second_half[second_next] *
          absolute_rotation[first_axis][second_last] +
        second_half[second_last] *
          absolute_rotation[first_axis][second_next];
      const double projected_translation = std::abs (
        translation[first_last] * rotation[first_next][second_axis] -
        translation[first_next] * rotation[first_last][second_axis]);
      if( projected_translation > first_radius + second_radius ) return false;
    }
  }
  return true;
}

void set_vtk_matrix (vtkMatrix4x4* target, const Matrix4& source)
{
  if( !target ) return;
  for( int row = 0; row < 4; ++row )
  {
    for( int column = 0; column < 4; ++column )
    {
      target->SetElement (row, column, source[row][column]);
    }
  }
  target->Modified ( );
}

} // namespace

class Robot_Collision_Detector::Implementation
{
public:
  struct Part
  {
    std::size_t source_index = 0;
    vtkSmartPointer<vtkPolyData> mesh;
    vtkSmartPointer<vtkImplicitPolyDataDistance> signed_distance;
    vtkSmartPointer<vtkStaticCellLocator> surface_locator;
    std::array<double, 6> local_bounds = { };
    std::vector<Point3> surface_samples;
  };

  struct Self_Collision_Pair
  {
    std::size_t first_part = 0;
    std::size_t second_part = 0;
    vtkSmartPointer<vtkMatrix4x4> first_matrix;
    vtkSmartPointer<vtkMatrix4x4> second_matrix;
    vtkSmartPointer<vtkCollisionDetectionFilter> detector;
    bool exact_baseline_valid = false;
    bool last_exact_collided = false;
  };

  std::vector<Part> parts;
  std::vector<Self_Collision_Pair> self_collision_pairs;
  Robot_Scene_Collision_Options scene_options;
  vtkSmartPointer<vtkPolyData> obstacle_data;
  vtkSmartPointer<vtkOctreePointLocator> obstacle_locator;
  vtkSmartPointer<vtkIdTypeArray> candidate_ids =
    vtkSmartPointer<vtkIdTypeArray>::New ( );
  Robot_Collision_Query_Stats last_query_stats;

  void Build_Self_Collision_Pairs ( )
  {
    self_collision_pairs.clear ( );
    if( !scene_options.check_self_collision ) return;
    for( std::size_t first = 0; first < parts.size ( ); ++first )
    {
      for( std::size_t second = first + 1;
           second < parts.size ( ); ++second )
      {
        const auto first_source = parts[first].source_index;
        const auto second_source = parts[second].source_index;
        bool enabled = second_source > first_source + 1;
        if( !scene_options.self_collision_pairs.empty ( ) )
        {
          enabled = std::find (
            scene_options.self_collision_pairs.begin ( ),
            scene_options.self_collision_pairs.end ( ),
            std::array<std::size_t, 2> { first_source, second_source }) !=
              scene_options.self_collision_pairs.end ( );
        }
        if( !enabled ) continue;

        Self_Collision_Pair pair;
        pair.first_part = first;
        pair.second_part = second;
        pair.first_matrix = vtkSmartPointer<vtkMatrix4x4>::New ( );
        pair.second_matrix = vtkSmartPointer<vtkMatrix4x4>::New ( );
        pair.detector =
          vtkSmartPointer<vtkCollisionDetectionFilter>::New ( );
        pair.detector->SetInputData (0, parts[first].mesh);
        pair.detector->SetInputData (1, parts[second].mesh);
        pair.detector->SetMatrix (0, pair.first_matrix);
        pair.detector->SetMatrix (1, pair.second_matrix);
        pair.detector->SetCollisionModeToFirstContact ( );
        pair.detector->SetBoxTolerance (0.01f);
        pair.detector->SetCellTolerance (0.0);
        self_collision_pairs.push_back (std::move (pair));
      }
    }
  }
};

Robot_Collision_Detector::Robot_Collision_Detector ( )
  : m_implementation (std::make_unique<Implementation> ( ))
{
}

Robot_Collision_Detector::~Robot_Collision_Detector ( ) = default;
Robot_Collision_Detector::Robot_Collision_Detector (
  Robot_Collision_Detector&&) noexcept = default;
Robot_Collision_Detector& Robot_Collision_Detector::operator= (
  Robot_Collision_Detector&&) noexcept = default;

bool Is_Robot_Collision_Recovery_Improvement (
  const Robot_Collision_Result& original_collision,
  const Robot_Collision_Result& candidate_collision,
  double previous_clearance_margin_mm)
{
  if( !original_collision.collided || !candidate_collision.collided ||
      original_collision.type != candidate_collision.type ||
      original_collision.robot_part_index !=
        candidate_collision.robot_part_index )
  {
    return false;
  }
  if( original_collision.type == Robot_Collision_Type::Self_Collision &&
      original_collision.other_robot_part_index !=
        candidate_collision.other_robot_part_index )
  {
    return false;
  }
  return candidate_collision.clearance_margin_mm >
    previous_clearance_margin_mm + 1.0e-6;
}

void Robot_Collision_Detector::Set_Robot_Parts (
  const std::vector<Robot_Visual_Part>& parts)
{
  m_implementation->parts.clear ( );
  m_implementation->self_collision_pairs.clear ( );
  m_implementation->parts.reserve (parts.size ( ));
  for( std::size_t source_index = 0;
       source_index < parts.size ( ); ++source_index )
  {
    const auto& source = parts[source_index];
    if( !source.mesh_data || source.mesh_data->GetNumberOfCells ( ) <= 0 )
    {
      continue;
    }

    Implementation::Part part;
    part.source_index = source_index;
    part.mesh = source.mesh_data;
    part.mesh->GetBounds (part.local_bounds.data ( ));
    part.signed_distance =
      vtkSmartPointer<vtkImplicitPolyDataDistance>::New ( );
    part.signed_distance->SetInput (part.mesh);
    part.surface_locator = vtkSmartPointer<vtkStaticCellLocator>::New ( );
    part.surface_locator->SetDataSet (part.mesh);
    part.surface_locator->BuildLocator ( );
    std::unordered_set<Voxel_Key, Voxel_Key_Hash> sample_voxels;
    if( auto* mesh_points = part.mesh->GetPoints ( ) )
    {
      sample_voxels.reserve (static_cast<std::size_t> (
        mesh_points->GetNumberOfPoints ( )));
      for( vtkIdType point_id = 0;
           point_id < mesh_points->GetNumberOfPoints ( ); ++point_id )
      {
        double point[3] = { };
        mesh_points->GetPoint (point_id, point);
        const Point3 sample = { point[0], point[1], point[2] };
        if( sample_voxels.insert (voxel_key (sample, 2.0)).second )
        {
          part.surface_samples.push_back (sample);
        }
      }
    }
    m_implementation->parts.push_back (std::move (part));
  }
  m_implementation->Build_Self_Collision_Pairs ( );
}

void Robot_Collision_Detector::Set_Scene_Collision_Options (
  const Robot_Scene_Collision_Options& options)
{
  m_implementation->scene_options = options;
  m_implementation->Build_Self_Collision_Pairs ( );
}

const Robot_Scene_Collision_Options&
Robot_Collision_Detector::Scene_Collision_Options ( ) const
{
  return m_implementation->scene_options;
}

bool Robot_Collision_Detector::Set_Obstacle_Points (
  const std::vector<float>& xyz,
  std::string* error_message)
{
  return Set_Obstacle_Points (
    xyz, Robot_Collision_Point_Cloud_Options { }, { }, nullptr,
    error_message);
}

bool Robot_Collision_Detector::Set_Obstacle_Points (
  const std::vector<float>& xyz,
  const Robot_Collision_Point_Cloud_Options& options,
  const std::vector<Matrix4>& reference_world_from_parts,
  Robot_Collision_Point_Cloud_Stats* stats,
  std::string* error_message,
  const std::atomic_bool* cancel_requested)
{
  if( error_message ) error_message->clear ( );
  if( stats ) *stats = { };
  if( xyz.empty ( ) || xyz.size ( ) % 3 != 0 )
  {
    if( error_message )
      *error_message = "Obstacle point array is empty or malformed";
    return false;
  }

  if( !std::isfinite (options.voxel_size_mm) ||
      options.voxel_size_mm <= 0.0 ||
      !std::isfinite (options.robot_exclusion_distance_mm) ||
      options.robot_exclusion_distance_mm < 0.0 )
  {
    if( error_message ) *error_message = "Collision point-cloud options are invalid";
    return false;
  }

  struct Reference_Part
  {
    Implementation::Part* part = nullptr;
    Matrix4 part_from_world = { };
    std::array<double, 6> world_bounds = { };
  };
  std::vector<Reference_Part> reference_parts;
  if( options.exclude_robot_points )
  {
    reference_parts.reserve (m_implementation->parts.size ( ));
    for( auto& part : m_implementation->parts )
    {
      if( part.source_index >= reference_world_from_parts.size ( ) ) continue;
      const auto& world_from_part =
        reference_world_from_parts[part.source_index];
      reference_parts.push_back ({
        &part,
        Invert_Rigid_Matrix (world_from_part),
        transformed_bounds (part.local_bounds, world_from_part)
      });
    }
  }

  auto points = vtkSmartPointer<vtkPoints>::New ( );
  points->SetDataTypeToFloat ( );
  points->Allocate (static_cast<vtkIdType> (xyz.size ( ) / 3));
  std::unordered_set<Voxel_Key, Voxel_Key_Hash> occupied_voxels;
  occupied_voxels.reserve (xyz.size ( ) / 3);
  Robot_Collision_Point_Cloud_Stats local_stats;
  local_stats.source_point_count = xyz.size ( ) / 3;
  for( std::size_t index = 0; index < xyz.size ( ) / 3; ++index )
  {
    if( ( index & 2047U ) == 0U && cancel_requested &&
        cancel_requested->load (std::memory_order_relaxed) )
    {
      if( error_message ) *error_message = "Collision obstacle build cancelled";
      if( stats ) *stats = local_stats;
      return false;
    }
    const Point3 point = {
      xyz[index * 3], xyz[index * 3 + 1], xyz[index * 3 + 2]
    };
    if( !finite_point (point) || !occupied_voxels.insert (
          voxel_key (point, options.voxel_size_mm)).second )
    {
      continue;
    }
    ++local_stats.voxel_point_count;

    bool belongs_to_robot = false;
    for( auto& reference : reference_parts )
    {
      if( !point_inside_inflated_bounds (
            point, reference.world_bounds,
            options.robot_exclusion_distance_mm) ) continue;
      const auto local_point = Transform_Position (
        reference.part_from_world, point);
      double local_query[3] = {
        local_point[0], local_point[1], local_point[2]
      };
      const double signed_distance =
        reference.part->signed_distance->EvaluateFunction (local_query);
      if( signed_distance < 0.0 ||
          std::abs (signed_distance) <=
            options.robot_exclusion_distance_mm )
      {
        belongs_to_robot = true;
        break;
      }
    }
    if( belongs_to_robot )
    {
      ++local_stats.excluded_robot_point_count;
      continue;
    }
    points->InsertNextPoint (point.data ( ));
  }
  if( cancel_requested &&
      cancel_requested->load (std::memory_order_relaxed) )
  {
    if( error_message ) *error_message = "Collision obstacle build cancelled";
    if( stats ) *stats = local_stats;
    return false;
  }
  if( points->GetNumberOfPoints ( ) <= 0 )
  {
    if( local_stats.voxel_point_count > 0 &&
        local_stats.excluded_robot_point_count ==
          local_stats.voxel_point_count )
    {
      Clear_Obstacle_Points ( );
      if( stats ) *stats = local_stats;
      return true;
    }
    if( error_message ) *error_message = "Obstacle point array has no finite points";
    return false;
  }

  auto data = vtkSmartPointer<vtkPolyData>::New ( );
  data->SetPoints (points);
  auto locator = vtkSmartPointer<vtkOctreePointLocator>::New ( );
  locator->SetDataSet (data);
  locator->BuildLocator ( );
  m_implementation->obstacle_data = data;
  m_implementation->obstacle_locator = locator;
  local_stats.collision_point_count = static_cast<std::size_t> (
    points->GetNumberOfPoints ( ));
  if( stats ) *stats = local_stats;
  return true;
}

void Robot_Collision_Detector::Clear_Obstacle_Points ( )
{
  m_implementation->obstacle_locator = nullptr;
  m_implementation->obstacle_data = nullptr;
}

void Robot_Collision_Detector::Clear ( )
{
  Clear_Obstacle_Points ( );
  m_implementation->parts.clear ( );
  m_implementation->self_collision_pairs.clear ( );
}

bool Robot_Collision_Detector::Has_Robot_Geometry ( ) const
{
  return !m_implementation->parts.empty ( );
}

bool Robot_Collision_Detector::Has_Obstacle_Points ( ) const
{
  return m_implementation->obstacle_locator != nullptr;
}

std::size_t Robot_Collision_Detector::Obstacle_Point_Count ( ) const
{
  return m_implementation->obstacle_data
    ? static_cast<std::size_t> (
        m_implementation->obstacle_data->GetNumberOfPoints ( ))
    : 0;
}

const Robot_Collision_Query_Stats&
Robot_Collision_Detector::Last_Query_Stats ( ) const
{
  return m_implementation->last_query_stats;
}

Robot_Collision_Result Robot_Collision_Detector::Check_Pose (
  const std::vector<Matrix4>& world_from_parts,
  double clearance_mm) const
{
  m_implementation->last_query_stats = { };
  m_implementation->last_query_stats.minimum_self_sample_distance_mm =
    std::numeric_limits<double>::infinity ( );
  Robot_Collision_Result result;
  result.minimum_distance_mm = std::numeric_limits<double>::infinity ( );
  if( clearance_mm < 0.0 || !std::isfinite (clearance_mm) ||
      !Has_Robot_Geometry ( ) )
  {
    return result;
  }

  struct Pose_Part_Cache
  {
    Matrix4 part_from_world = { };
    std::array<double, 6> world_bounds = { };
    bool valid = false;
  };
  std::vector<Pose_Part_Cache> pose_cache (
    m_implementation->parts.size ( ));
  for( std::size_t part_index = 0;
       part_index < m_implementation->parts.size ( ); ++part_index )
  {
    const auto& part = m_implementation->parts[part_index];
    if( part.source_index >= world_from_parts.size ( ) ) continue;
    auto& cache = pose_cache[part_index];
    cache.part_from_world = Invert_Rigid_Matrix (
      world_from_parts[part.source_index]);
    cache.world_bounds = transformed_bounds (
      part.local_bounds, world_from_parts[part.source_index]);
    cache.valid = true;
  }

  const auto& scene_options = m_implementation->scene_options;
  if( scene_options.check_ground_collision &&
      std::isfinite (scene_options.ground_height_mm) )
  {
    for( std::size_t part_index = 0;
         part_index < m_implementation->parts.size ( ); ++part_index )
    {
      const auto& part = m_implementation->parts[part_index];
      const auto& cache = pose_cache[part_index];
      const bool ground_checked = scene_options.ground_checked_parts.empty ( )
        ? part.source_index >=
            scene_options.first_ground_checked_part_index
        : std::find (scene_options.ground_checked_parts.begin ( ),
                     scene_options.ground_checked_parts.end ( ),
                     part.source_index) !=
            scene_options.ground_checked_parts.end ( );
      if( !ground_checked || !cache.valid ) continue;
      const auto& world_from_part = world_from_parts[part.source_index];
      if( cache.world_bounds[4] >
            scene_options.ground_height_mm + clearance_mm ) continue;

      Point3 lowest_world = { 0.0, 0.0,
        std::numeric_limits<double>::infinity ( ) };
      for( const auto& local : part.surface_samples )
      {
        ++m_implementation->last_query_stats.ground_sample_queries;
        const auto world = Transform_Position (
          world_from_part, local);
        if( world[2] < lowest_world[2] ) lowest_world = world;
      }
      const double distance =
        lowest_world[2] - scene_options.ground_height_mm;
      if( distance <= clearance_mm )
      {
        result.collided = true;
        result.type = Robot_Collision_Type::Ground;
        result.robot_part_index = part.source_index;
        result.minimum_distance_mm = distance;
        result.clearance_margin_mm = distance - clearance_mm;
        result.robot_closest_point_world = lowest_world;
        result.obstacle_point_world = {
          lowest_world[0], lowest_world[1], scene_options.ground_height_mm
        };
        return result;
      }
    }
  }

  if( scene_options.check_self_collision )
  {
    const double self_clearance = std::max (
      0.0, scene_options.self_collision_clearance_mm);
    for( auto& pair : m_implementation->self_collision_pairs )
    {
      const auto& first = m_implementation->parts[pair.first_part];
      const auto& second = m_implementation->parts[pair.second_part];
      const auto& first_cache = pose_cache[pair.first_part];
      const auto& second_cache = pose_cache[pair.second_part];
      if( !first_cache.valid || !second_cache.valid ) continue;
      const auto& first_transform = world_from_parts[first.source_index];
      const auto& second_transform = world_from_parts[second.source_index];
      if( !bounds_overlap (
            first_cache.world_bounds, second_cache.world_bounds,
            self_clearance) )
      {
        pair.exact_baseline_valid = false;
        pair.last_exact_collided = false;
        continue;
      }
      ++m_implementation->last_query_stats.self_broad_phase_pairs;

      double minimum_distance = std::numeric_limits<double>::infinity ( );
      double most_negative_distance = 0.0;
      bool has_negative_distance = false;
      Point3 closest_first_world = { };
      Point3 closest_second_world = { };
      const auto sample_distance = [&] (
        const Implementation::Part& source_part,
        const Matrix4& world_from_source,
        const Implementation::Part& target_part,
        const Matrix4& world_from_target,
        const Pose_Part_Cache& target_cache,
        bool source_is_first)
      {
        for( const auto& source_local : source_part.surface_samples )
        {
          const auto source_world = Transform_Position (
            world_from_source, source_local);
          if( !point_inside_inflated_bounds (
                source_world, target_cache.world_bounds,
                self_clearance) ) continue;
          ++m_implementation->last_query_stats.self_distance_sample_queries;
          const auto target_local = Transform_Position (
            target_cache.part_from_world, source_world);
          double query[3] = {
            target_local[0], target_local[1], target_local[2]
          };
          double closest_target_local[3] = { };
          vtkIdType closest_cell_id = -1;
          int closest_sub_id = -1;
          double distance_squared = 0.0;
          const double search_radius = self_clearance + 1.0;
          if( !target_part.surface_locator->FindClosestPointWithinRadius (
                query, search_radius, closest_target_local,
                closest_cell_id, closest_sub_id, distance_squared) )
          {
            continue;
          }
          const double distance = std::sqrt (distance_squared);
          double signed_distance = distance;
          if( distance <= self_clearance )
          {
            double signed_closest[3] = { };
            signed_distance =
              target_part.signed_distance->EvaluateFunctionAndGetClosestPoint (
                query, signed_closest);
          }
          if( signed_distance < most_negative_distance )
          {
            most_negative_distance = signed_distance;
            has_negative_distance = true;
          }
          m_implementation->last_query_stats.minimum_self_sample_distance_mm =
            std::min (
              m_implementation->last_query_stats.
                minimum_self_sample_distance_mm,
              distance);
          if( distance >= minimum_distance ) continue;
          minimum_distance = distance;
          const auto target_world = Transform_Position (
            world_from_target,
            { closest_target_local[0], closest_target_local[1],
              closest_target_local[2] });
          if( source_is_first )
          {
            closest_first_world = source_world;
            closest_second_world = target_world;
          }
          else
          {
            closest_first_world = target_world;
            closest_second_world = source_world;
          }
        }
      };
      sample_distance (
        first, first_transform, second, second_transform,
        second_cache, true);
      sample_distance (
        second, second_transform, first, first_transform,
        first_cache, false);

      const bool proximity_collision = has_negative_distance ||
        minimum_distance <= self_clearance;
      bool surfaces_intersect = false;
      if( !proximity_collision && bounds_overlap (
            first_cache.world_bounds, second_cache.world_bounds) )
      {
        if( oriented_bounds_overlap (
              first.local_bounds, first_transform,
              second.local_bounds, second_transform) )
        {
          ++m_implementation->last_query_stats.self_obb_phase_pairs;
          if( !pair.exact_baseline_valid || pair.last_exact_collided )
          {
            ++m_implementation->last_query_stats.self_exact_pair_queries;
            set_vtk_matrix (pair.first_matrix, first_transform);
            set_vtk_matrix (pair.second_matrix, second_transform);
            pair.detector->Update ( );
            surfaces_intersect = pair.detector->GetNumberOfContacts ( ) > 0;
            pair.exact_baseline_valid = true;
            pair.last_exact_collided = surfaces_intersect;
          }
        }
        else
        {
          pair.exact_baseline_valid = false;
          pair.last_exact_collided = false;
        }
      }

      if( !surfaces_intersect && !proximity_collision ) continue;

      result.collided = true;
      result.type = Robot_Collision_Type::Self_Collision;
      result.robot_part_index = first.source_index;
      result.other_robot_part_index = second.source_index;
      result.minimum_distance_mm = std::isfinite (minimum_distance)
        ? minimum_distance : 0.0;
      result.clearance_margin_mm = has_negative_distance
        ? most_negative_distance - self_clearance
        : result.minimum_distance_mm - self_clearance;
      result.robot_closest_point_world = closest_first_world;
      result.obstacle_point_world = closest_second_world;
      if( surfaces_intersect &&
          !has_negative_distance && result.clearance_margin_mm >= 0.0 )
      {
        result.clearance_margin_mm = -std::max (self_clearance, 0.001);
      }
      if( surfaces_intersect )
      {
        if( auto* contacts = pair.detector->GetContactsOutput ( );
            contacts && contacts->GetNumberOfPoints ( ) > 0 )
        {
          double contact[3] = { };
          contacts->GetPoint (0, contact);
          result.robot_closest_point_world = {
            contact[0], contact[1], contact[2]
          };
          result.obstacle_point_world = result.robot_closest_point_world;
        }
      }
      return result;
    }
  }

  if( !Has_Obstacle_Points ( ) ) return result;

  auto* candidate_ids = m_implementation->candidate_ids.GetPointer ( );
  for( std::size_t part_index = 0;
       part_index < m_implementation->parts.size ( ); ++part_index )
  {
    const auto& part = m_implementation->parts[part_index];
    const auto& cache = pose_cache[part_index];
    if( !cache.valid ) continue;
    const auto& world_from_part = world_from_parts[part.source_index];
    double query_bounds[6] = {
      cache.world_bounds[0] - clearance_mm,
      cache.world_bounds[1] + clearance_mm,
      cache.world_bounds[2] - clearance_mm,
      cache.world_bounds[3] + clearance_mm,
      cache.world_bounds[4] - clearance_mm,
      cache.world_bounds[5] + clearance_mm
    };
    candidate_ids->Reset ( );
    m_implementation->obstacle_locator->FindPointsInArea (
      query_bounds, candidate_ids, true);
    m_implementation->last_query_stats.obstacle_candidate_points +=
      static_cast<std::size_t> (candidate_ids->GetNumberOfValues ( ));

    for( vtkIdType candidate = 0;
         candidate < candidate_ids->GetNumberOfValues ( ); ++candidate )
    {
      const vtkIdType point_id = candidate_ids->GetValue (candidate);
      double raw_world_point[3] = { };
      m_implementation->obstacle_data->GetPoint (point_id, raw_world_point);
      const Point3 world_point = {
        raw_world_point[0], raw_world_point[1], raw_world_point[2]
      };
      const auto local_point = Transform_Position (
        cache.part_from_world, world_point);
      if( !point_inside_inflated_bounds (
            local_point, part.local_bounds, clearance_mm) ) continue;
      double closest_local[3] = { };
      double local_query[3] = {
        local_point[0], local_point[1], local_point[2]
      };
      ++m_implementation->last_query_stats.obstacle_distance_queries;
      const double signed_distance =
        part.signed_distance->EvaluateFunctionAndGetClosestPoint (
          local_query, closest_local);
      const double distance = std::abs (signed_distance);
      const bool candidate_collided =
        signed_distance < 0.0 || distance <= clearance_mm;
      if( candidate_collided )
      {
        result.collided = true;
        result.type = Robot_Collision_Type::Obstacle_Point;
        result.minimum_distance_mm = distance;
        result.clearance_margin_mm = signed_distance - clearance_mm;
        result.robot_part_index = part.source_index;
        result.obstacle_point_index = static_cast<std::size_t> (point_id);
        result.obstacle_point_world = world_point;
        result.robot_closest_point_world = Transform_Position (
          world_from_part,
          { closest_local[0], closest_local[1], closest_local[2] });
        return result;
      }
    }
  }
  return result;
}

} // namespace robot_model

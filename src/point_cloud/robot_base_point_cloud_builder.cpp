#include "robot_base_point_cloud_builder.h"

#include "camera_point_cloud_converter.h"
#include "pose_transform.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace point_cloud
{

bool Calculate_Point_Cloud_Bounds (
  const Point_Cloud_Data& cloud,
  std::array<double, 6>* bounds,
  std::string* error_message)
{
  if( error_message ) error_message->clear ( );
  if( bounds == nullptr || cloud.xyz.empty ( ) ||
      ( cloud.xyz.size ( ) % 3 ) != 0 )
  {
    if( error_message ) *error_message = "Point cloud is empty or malformed";
    return false;
  }
  *bounds = {
    std::numeric_limits<double>::infinity ( ),
    -std::numeric_limits<double>::infinity ( ),
    std::numeric_limits<double>::infinity ( ),
    -std::numeric_limits<double>::infinity ( ),
    std::numeric_limits<double>::infinity ( ),
    -std::numeric_limits<double>::infinity ( ) };
  for( std::size_t index = 0; index < cloud.Point_Count ( ); ++index )
  {
    for( int axis = 0; axis < 3; ++axis )
    {
      const double value = cloud.xyz[index * 3 + axis];
      if( !std::isfinite (value) )
      {
        if( error_message )
          *error_message = "Point cloud contains a non-finite coordinate";
        return false;
      }
      ( *bounds )[axis * 2] = std::min (( *bounds )[axis * 2], value);
      ( *bounds )[axis * 2 + 1] =
        std::max (( *bounds )[axis * 2 + 1], value);
    }
  }
  return true;
}

bool Are_Robot_Base_Bounds_Reasonable (
  const std::array<double, 6>& bounds)
{
  constexpr double kMaximumWorldMagnitudeMm = 50000.0;
  for( double value : bounds )
  {
    if( !std::isfinite (value) ||
        std::abs (value) > kMaximumWorldMagnitudeMm )
    {
      return false;
    }
  }
  for( int axis = 0; axis < 3; ++axis )
  {
    if( bounds[axis * 2 + 1] < bounds[axis * 2] ||
        bounds[axis * 2 + 1] - bounds[axis * 2] >
          kMaximumWorldMagnitudeMm )
    {
      return false;
    }
  }
  return true;
}

bool Build_Robot_Base_Point_Cloud (
  const Camera_Frame& frame,
  const robot_model::Eye_To_Hand_Calibration& calibration,
  Robot_Base_Point_Cloud* output,
  std::string* error_message)
{
  if( error_message ) error_message->clear ( );
  if( output == nullptr )
  {
    if( error_message ) *error_message = "Robot-base point-cloud output is null";
    return false;
  }
  *output = {};
  if( calibration.camera_coordinate_type == 0 )
  {
    if( error_message )
      *error_message = "CameraCoordinateType is missing from hand-eye calibration";
    return false;
  }

  Camera_Point_Cloud camera_cloud;
  if( !Convert_Camera_Frame_To_Point_Cloud (
        frame, &camera_cloud, error_message, 500000,
        calibration.camera_coordinate_type) )
  {
    return false;
  }
  if( camera_cloud.source_coordinate_type !=
      calibration.camera_coordinate_type )
  {
    if( error_message )
      *error_message = "Point-cloud and hand-eye calibration coordinate systems differ";
    return false;
  }

  Camera_Point_Cloud_Filter_Options filter_options;
  filter_options.coordinate_to_mm = calibration.point_cloud_unit_to_mm;
  Camera_Point_Cloud_Filter_Stats filter_stats;
  if( !Prepare_Camera_Point_Cloud_For_Overlay (
        &camera_cloud, filter_options, &filter_stats, error_message) )
  {
    return false;
  }

  const auto world_from_camera = calibration.World_From_Camera ( );
  for( std::size_t index = 0; index < camera_cloud.Point_Count ( ); ++index )
  {
    const auto world = robot_model::Transform_Position (
      world_from_camera,
      { camera_cloud.xyz[index * 3], camera_cloud.xyz[index * 3 + 1],
        camera_cloud.xyz[index * 3 + 2] });
    camera_cloud.xyz[index * 3] = static_cast<float> (world[0]);
    camera_cloud.xyz[index * 3 + 1] = static_cast<float> (world[1]);
    camera_cloud.xyz[index * 3 + 2] = static_cast<float> (world[2]);
  }

  output->data.xyz = std::move (camera_cloud.xyz);
  output->data.rgb = std::move (camera_cloud.rgb);
  output->source_frame_number = camera_cloud.source_frame_number;
  output->source_point_count = camera_cloud.source_point_count;
  output->filtered_point_count = filter_stats.valid_points;
  output->median_depth_mm = filter_stats.median_depth_mm;
  if( !Calculate_Point_Cloud_Bounds (
        output->data, &output->world_bounds_mm, error_message) ||
      !Are_Robot_Base_Bounds_Reasonable (output->world_bounds_mm) )
  {
    if( error_message && error_message->empty ( ) )
      *error_message = "Robot-base point-cloud bounds are invalid or exceed 50 metres";
    *output = {};
    return false;
  }
  return true;
}

} // namespace point_cloud

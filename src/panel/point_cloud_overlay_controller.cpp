#include "point_cloud_overlay_controller.h"

#include "camera_service.h"
#include "eye_to_hand_calibration.h"
#include "point_cloud_file_repository.h"
#include "point_cloud_renderer.h"
#include "robot_base_point_cloud_builder.h"
#include "robot_model_repository.h"

#include <algorithm>

namespace
{

robot_model::Matrix4 identity_matrix ( )
{
  robot_model::Matrix4 matrix = { };
  for( int index = 0; index < 4; ++index ) matrix[index][index] = 1.0;
  return matrix;
}

void copy_statistics (
  const point_cloud::Robot_Base_Point_Cloud& cloud,
  Point_Cloud_Overlay_Result* result)
{
  result->source_point_count = cloud.source_point_count;
  result->filtered_point_count = cloud.filtered_point_count;
  result->point_count = cloud.data.Point_Count ( );
  result->median_depth_mm = cloud.median_depth_mm;
  result->world_bounds_mm = cloud.world_bounds_mm;
  result->has_world_bounds = true;
}

} // namespace

class Point_Cloud_Overlay_Controller::Implementation
{
public:
  explicit Implementation (Camera_Service& camera_service)
    : camera_service (camera_service)
  {
  }

  bool Build_Latest (
    point_cloud::Robot_Base_Point_Cloud* cloud,
    std::string* error_message) const
  {
    if( !camera_service.Is_Device_Open ( ) )
    {
      *error_message = "请先连接并打开相机";
      return false;
    }
    const auto frame = camera_service.Latest_Frame ( );
    if( !frame )
    {
      *error_message = "尚未取得相机帧，请先开始采集";
      return false;
    }

    const auto robot_root = robot_model::Find_Robot_Root ( );
    robot_model::Eye_To_Hand_Calibration calibration;
    if( robot_root.empty ( ) ||
        !robot_model::Load_Eye_To_Hand_Calibration (
          robot_root / "RT_eyetohand.txt", &calibration, error_message) )
    {
      if( error_message->empty ( ) )
        *error_message = "Resource/Robot was not found";
      return false;
    }
    return point_cloud::Build_Robot_Base_Point_Cloud (
      *frame, calibration, cloud, error_message);
  }

  Point_Cloud_Overlay_Result Render (
    const point_cloud::Robot_Base_Point_Cloud& cloud,
    vtkRenderer* vtk_renderer)
  {
    Point_Cloud_Overlay_Result result;
    if( vtk_renderer == nullptr )
    {
      result.error_message = "机械臂 VTK 场景尚未初始化";
      return result;
    }
    renderer.Attach_Renderer (vtk_renderer);
    renderer.Set_World_From_Point_Cloud (identity_matrix ( ));
    if( !renderer.Set_Point_Data (
          cloud.data.xyz, cloud.data.rgb, &result.error_message) )
    {
      return result;
    }

    double vtk_bounds[6] = {};
    if( !renderer.Get_World_Bounds (vtk_bounds) )
    {
      renderer.Clear ( );
      result.error_message = "Unable to calculate point-cloud bounds";
      return result;
    }
    std::array<double, 6> rendered_bounds = {};
    std::copy (vtk_bounds, vtk_bounds + 6, rendered_bounds.begin ( ));
    if( !point_cloud::Are_Robot_Base_Bounds_Reasonable (rendered_bounds) )
    {
      renderer.Clear ( );
      result.error_message =
        "点云的机器人基坐标范围无效或超过 50 米";
      return result;
    }

    vtk_renderer->ResetCameraClippingRange ( );
    result.success = true;
    copy_statistics (cloud, &result);
    result.point_count = renderer.Point_Count ( );
    result.world_bounds_mm = rendered_bounds;
    return result;
  }

  Camera_Service& camera_service;
  point_cloud::Point_Cloud_Renderer renderer;
};

Point_Cloud_Overlay_Controller::Point_Cloud_Overlay_Controller (
  Camera_Service& camera_service)
  : m_implementation (
      std::make_unique<Implementation> (camera_service))
{
}

Point_Cloud_Overlay_Controller::~Point_Cloud_Overlay_Controller ( ) = default;

Point_Cloud_Overlay_Result Point_Cloud_Overlay_Controller::Load_Latest (
  vtkRenderer* renderer)
{
  point_cloud::Robot_Base_Point_Cloud cloud;
  Point_Cloud_Overlay_Result result;
  if( !m_implementation->Build_Latest (&cloud, &result.error_message) )
  {
    return result;
  }
  return m_implementation->Render (cloud, renderer);
}

Point_Cloud_Save_Result
Point_Cloud_Overlay_Controller::Save_Latest_To_File (
  const std::filesystem::path& path,
  const std::string& robot_model_id)
{
  Point_Cloud_Save_Result result;
  point_cloud::Robot_Base_Point_Cloud cloud;
  if( !m_implementation->Build_Latest (&cloud, &result.error_message) )
  {
    return result;
  }
  const auto saved = point_cloud::Save_Robot_Base_Point_Cloud_To_File (
    path, cloud.data, robot_model_id, cloud.source_frame_number);
  if( !saved.success )
  {
    result.error_message = saved.error_message;
    return result;
  }
  result.success = true;
  result.file_path = saved.path;
  result.source_point_count = cloud.source_point_count;
  result.filtered_point_count = cloud.filtered_point_count;
  result.point_count = cloud.data.Point_Count ( );
  result.median_depth_mm = cloud.median_depth_mm;
  result.world_bounds_mm = cloud.world_bounds_mm;
  return result;
}

Point_Cloud_Overlay_Result Point_Cloud_Overlay_Controller::Load_File (
  const std::filesystem::path& path,
  const std::string& expected_robot_model,
  vtkRenderer* renderer)
{
  Point_Cloud_Overlay_Result result;
  point_cloud::Point_Cloud_File_Metadata metadata;
  point_cloud::Robot_Base_Point_Cloud cloud;
  if( !point_cloud::Load_Robot_Base_Point_Cloud_For_Model (
        path, expected_robot_model, &cloud.data, &metadata,
        &result.error_message) )
  {
    return result;
  }
  cloud.source_frame_number = metadata.source_frame_number;
  cloud.source_point_count = cloud.data.Point_Count ( );
  cloud.filtered_point_count = cloud.data.Point_Count ( );
  if( !point_cloud::Calculate_Point_Cloud_Bounds (
        cloud.data, &cloud.world_bounds_mm, &result.error_message) ||
      !point_cloud::Are_Robot_Base_Bounds_Reasonable (
        cloud.world_bounds_mm) )
  {
    if( result.error_message.empty ( ) )
      result.error_message = "文件点云的机器人基坐标范围无效或超过 50 米";
    return result;
  }
  return m_implementation->Render (cloud, renderer);
}

void Point_Cloud_Overlay_Controller::Attach_Renderer (vtkRenderer* renderer)
{
  m_implementation->renderer.Attach_Renderer (renderer);
}

void Point_Cloud_Overlay_Controller::Clear ( )
{
  m_implementation->renderer.Clear ( );
}

bool Point_Cloud_Overlay_Controller::Has_Point_Cloud ( ) const
{
  return m_implementation->renderer.Has_Point_Cloud ( );
}

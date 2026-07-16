#include "point_cloud_overlay_controller.h"

#include "camera_point_cloud_converter.h"
#include "camera_service.h"
#include "eye_to_hand_calibration.h"
#include "robot_model_repository.h"

namespace
{
constexpr double CAMERA_POINT_COORDINATE_TO_MM = 0.001;
}

Point_Cloud_Overlay_Controller::Point_Cloud_Overlay_Controller (
  Camera_Service& camera_service)
  : m_camera_service (camera_service)
{
}

Point_Cloud_Overlay_Result Point_Cloud_Overlay_Controller::Load_Latest (
  vtkRenderer* renderer)
{
  Point_Cloud_Overlay_Result result;
  if( !m_camera_service.Is_Device_Open ( ) )
  {
    result.error_message = "请先连接并打开相机";
    return result;
  }
  const auto frame = m_camera_service.Latest_Frame ( );
  if( !frame )
  {
    result.error_message = "尚未取得相机帧，请先开始采集";
    return result;
  }

  Camera_Point_Cloud cloud;
  if( !Convert_Camera_Frame_To_Point_Cloud (
        *frame, &cloud, &result.error_message) )
  {
    return result;
  }

  const auto robot_root = robot_model::Find_Robot_Root ( );
  robot_model::Eye_To_Hand_Calibration calibration;
  if( robot_root.empty ( ) ||
      !robot_model::Load_Eye_To_Hand_Calibration (
        robot_root / "RT_eyetohand.txt", &calibration,
        &result.error_message) )
  {
    if( result.error_message.empty ( ) )
    {
      result.error_message = "Resource/Robot was not found";
    }
    return result;
  }
  if( !renderer )
  {
    result.error_message = "机械臂 VTK 场景尚未初始化";
    return result;
  }

  m_renderer.Attach_Renderer (renderer);
  m_renderer.Set_World_From_Point_Cloud (
    calibration.World_From_Camera (CAMERA_POINT_COORDINATE_TO_MM));
  if( !m_renderer.Set_Point_Data (
        cloud.xyz, cloud.rgb, &result.error_message) )
  {
    return result;
  }
  result.success = true;
  result.point_count = m_renderer.Point_Count ( );
  return result;
}

void Point_Cloud_Overlay_Controller::Attach_Renderer (vtkRenderer* renderer)
{
  m_renderer.Attach_Renderer (renderer);
}

void Point_Cloud_Overlay_Controller::Clear ( )
{
  m_renderer.Clear ( );
}

bool Point_Cloud_Overlay_Controller::Has_Point_Cloud ( ) const
{
  return m_renderer.Has_Point_Cloud ( );
}

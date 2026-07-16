#include "camera_pose_controller.h"

#include "eye_to_hand_calibration.h"
#include "robot_model_repository.h"

bool Camera_Pose_Controller::Show (
  vtkRenderer* renderer,
  std::string* error_message)
{
  if( error_message ) error_message->clear ( );
  const auto robot_root = robot_model::Find_Robot_Root ( );
  robot_model::Eye_To_Hand_Calibration calibration;
  if( robot_root.empty ( ) ||
      !robot_model::Load_Eye_To_Hand_Calibration (
        robot_root / "RT_eyetohand.txt", &calibration, error_message) )
  {
    if( error_message && error_message->empty ( ) )
    {
      *error_message = "Resource/Robot was not found";
    }
    return false;
  }
  if( !renderer )
  {
    if( error_message ) *error_message = "机械臂 VTK 场景尚未初始化";
    return false;
  }
  m_renderer.Attach_Renderer (renderer);
  m_renderer.Set_World_From_Camera (calibration.World_From_Camera ( ));
  m_renderer.Set_Visible (true);
  return true;
}

void Camera_Pose_Controller::Hide ( )
{
  m_renderer.Set_Visible (false);
}

void Camera_Pose_Controller::Attach_Renderer (vtkRenderer* renderer)
{
  m_renderer.Attach_Renderer (renderer);
}

bool Camera_Pose_Controller::Is_Visible ( ) const
{
  return m_renderer.Is_Visible ( );
}

#ifndef includeguard_camera_pose_controller_h_includeguard
#define includeguard_camera_pose_controller_h_includeguard

#include "camera_pose_renderer.h"

#include <string>

class vtkRenderer;

class Camera_Pose_Controller
{
public:
  bool Show (vtkRenderer* renderer, std::string* error_message);
  void Hide ( );
  void Attach_Renderer (vtkRenderer* renderer);
  bool Is_Visible ( ) const;

private:
  robot_model::Camera_Pose_Renderer m_renderer;
};

#endif

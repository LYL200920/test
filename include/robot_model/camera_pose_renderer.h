#ifndef includeguard_camera_pose_renderer_h_includeguard
#define includeguard_camera_pose_renderer_h_includeguard

#include "pose_transform.h"

#include <vtkActor.h>
#include <vtkMatrix4x4.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <vector>

namespace robot_model
{

class Camera_Pose_Renderer
{
public:
  void Attach_Renderer (vtkRenderer* renderer);
  void Detach_Renderer ( );
  void Set_World_From_Camera (const Matrix4& world_from_camera);
  void Set_Visible (bool visible);
  bool Is_Visible ( ) const { return m_visible; }

private:
  void Ensure_Actors ( );
  void Add_Actors ( );
  void Remove_Actors ( );

private:
  vtkRenderer* m_renderer = nullptr;
  vtkSmartPointer<vtkMatrix4x4> m_world_from_camera;
  std::vector<vtkSmartPointer<vtkActor>> m_actors;
  bool m_visible = false;
};

} // namespace robot_model

#endif

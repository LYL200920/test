#ifndef includeguard_flange_transform_gizmo_renderer_h_includeguard
#define includeguard_flange_transform_gizmo_renderer_h_includeguard

#include "pose_transform.h"

#include <vtkActor.h>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

#include <vector>

class vtkRenderer;

namespace robot_model
{

// Visual handles only. Hit testing and robot motion are owned by independent
// dragger and IK modules.
class Flange_Transform_Gizmo_Renderer
{
public:
  Flange_Transform_Gizmo_Renderer (double axis_length_mm = 220.0,
                                   double ring_radius_mm = 145.0);
  void Attach_Renderer (vtkRenderer* renderer);
  void Detach_Renderer ( );
  void Set_World_From_Flange (const Matrix4& world_from_flange);
  void Set_Visible (bool visible);
  void Set_Highlighted_Handle (int translation_axis, int rotation_axis);

private:
  void Ensure_Actors ( );
  void Add_Actors ( );
  void Remove_Actors ( );

private:
  double m_axis_length_mm = 220.0;
  double m_ring_radius_mm = 145.0;
  vtkRenderer* m_renderer = nullptr;
  vtkSmartPointer<vtkMatrix4x4> m_world_from_flange;
  std::vector<vtkSmartPointer<vtkActor>> m_actors;
  bool m_visible = false;
  int m_translation_highlight = -1;
  int m_rotation_highlight = -1;
};

} // namespace robot_model

#endif

#ifndef includeguard_coordinate_frame_renderer_h_includeguard
#define includeguard_coordinate_frame_renderer_h_includeguard

#include "pose_transform.h"

#include <vtkActor.h>
#include <vtkMatrix4x4.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <vector>

namespace robot_model
{

// Reusable world-space coordinate frame visualization. It owns only its VTK
// props; pose computation and user interaction remain separate concerns.
class Coordinate_Frame_Renderer
{
public:
  explicit Coordinate_Frame_Renderer (double axis_length_mm = 250.0,
                                      double origin_radius_mm = 16.0,
                                      bool show_axes = true);

  void Attach_Renderer (vtkRenderer* renderer);
  void Detach_Renderer ( );
  void Set_World_From_Frame (const Matrix4& world_from_frame);
  void Set_Visible (bool visible);
  void Set_Origin_Highlighted (bool highlighted);
  bool Is_Visible ( ) const { return m_visible; }

private:
  void Ensure_Actors ( );
  void Add_Actors ( );
  void Remove_Actors ( );

private:
  double m_axis_length_mm = 250.0;
  double m_origin_radius_mm = 16.0;
  bool m_show_axes = true;
  vtkRenderer* m_renderer = nullptr;
  vtkSmartPointer<vtkMatrix4x4> m_world_from_frame;
  std::vector<vtkSmartPointer<vtkActor>> m_actors;
  vtkSmartPointer<vtkActor> m_origin_actor;
  bool m_visible = false;
};

} // namespace robot_model

#endif

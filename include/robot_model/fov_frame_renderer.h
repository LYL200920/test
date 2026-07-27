#ifndef includeguard_fov_frame_renderer_h_includeguard
#define includeguard_fov_frame_renderer_h_includeguard

#include "tool_visualization.h"

#include <vtkActor.h>
#include <vtkLineSource.h>
#include <vtkMatrix4x4.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <vector>

namespace robot_model
{

class Fov_Frame_Renderer
{
public:
  void Attach_Renderer(vtkRenderer *renderer);
  void Detach_Renderer();
  void Set_Configuration(
    const Fov_Visualization_Configuration &configuration);
  void Set_World_From_Fov(const Matrix4 &world_from_fov);
  void Set_Visible(bool visible);
  bool Is_Visible() const { return m_visible; }

private:
  void Ensure_Actors();
  void Update_Geometry();
  void Add_Actors();
  void Remove_Actors();

private:
  Fov_Visualization_Configuration m_configuration;
  vtkRenderer *m_renderer = nullptr;
  vtkSmartPointer<vtkMatrix4x4> m_world_from_fov;
  std::vector<vtkSmartPointer<vtkLineSource>> m_line_sources;
  std::vector<vtkSmartPointer<vtkActor>> m_actors;
  bool m_visible = false;
};

} // namespace robot_model

#endif

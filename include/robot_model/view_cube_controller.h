#ifndef includeguard_view_cube_controller_h_includeguard
#define includeguard_view_cube_controller_h_includeguard

#include <vtkPropAssembly.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTextActor.h>

#include <array>

class vtkGenericOpenGLRenderWindow;

namespace robot_model
{

  class View_Cube_Controller
  {
  public:
    void Setup(vtkGenericOpenGLRenderWindow *render_window, vtkRenderer *scene_renderer);
    void Shutdown();

    void Update_Camera();
    bool Handle_Click(int x, int y);

  private:
    vtkGenericOpenGLRenderWindow *m_render_window = nullptr;
    vtkRenderer *m_scene_renderer = nullptr;
    vtkSmartPointer<vtkRenderer> m_view_cube_renderer;
    vtkSmartPointer<vtkPropAssembly> m_view_cube_assembly;
    std::array<vtkSmartPointer<vtkTextActor>, 3> m_axis_labels = {};
    std::array<vtkSmartPointer<vtkTextActor>, 6> m_face_labels = {};
  };

} // namespace robot_model

#endif

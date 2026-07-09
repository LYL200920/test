#ifndef includeguard_vtk_scene_h_includeguard
#define includeguard_vtk_scene_h_includeguard

#include <wx/glcanvas.h>

#include "view_cube_controller.h"

#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkGenericRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

namespace robot_model
{

class Vtk_Scene
{
public:
  Vtk_Scene ( );
  ~Vtk_Scene ( );

  void Init (wxGLCanvas* canvas, wxGLContext* context);
  void Shutdown ( );

  bool Is_Ready ( ) const { return m_ready; }

  void Render ( );
  void Resize (int width, int height);

  vtkGenericRenderWindowInteractor* Get_Interactor ( ) const;
  vtkRenderer* Renderer ( ) const;

  void Add_Actor (vtkActor* actor);
  void Remove_All_ViewProps ( );
  void Remove_All_Lights ( );

  void Reset_Camera ( );
  void Add_Ground_Grid (double extent);
  bool Handle_View_Cube_Click (int x, int y);

private:
  static void On_Make_Current (vtkObject*, unsigned long, void* client_data, void*);
  static void On_Is_Current (vtkObject*, unsigned long, void* client_data, void* call_data);
  static void On_Frame (vtkObject*, unsigned long, void* client_data, void*);
  static void On_Supports_OpenGL (vtkObject*, unsigned long, void* client_data, void* call_data);

  void Setup_View_Cube ( );
  void Update_View_Cube_Camera ( );

private:
  wxGLCanvas* m_canvas = nullptr;
  wxGLContext* m_gl_context = nullptr;

  vtkSmartPointer<vtkRenderer> m_renderer;
  vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_render_window;
  vtkSmartPointer<vtkGenericRenderWindowInteractor> m_interactor;
  View_Cube_Controller m_view_cube;

  bool m_ready = false;
};

} // namespace robot_model

#endif

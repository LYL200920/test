#ifndef includeguard_robot_model_view_h_includeguard
#define includeguard_robot_model_view_h_includeguard

#include "robot_model_data.h"
#include "robot_render_controller.h"

#include <wx/glcanvas.h>

#include <memory>

class vtkGenericRenderWindowInteractor;

namespace robot_model
{
class Vtk_Scene;
} // namespace robot_model

class Robot_Model_View : public wxGLCanvas
{
public:
  explicit Robot_Model_View (wxWindow* parent, wxWindowID id = wxID_ANY);
  ~Robot_Model_View ( ) override;

  void Load_Model (const robot_model::Robot_Model_Info& model);
  void Set_Joint_State (const robot_model::Robot_Joint_State& state);

private:
  void Ensure_VTK ( );
  void On_Size (wxSizeEvent& event);
  void On_Paint (wxPaintEvent& event);
  void On_Erase_Background (wxEraseEvent& event);
  void On_Mouse_Move (wxMouseEvent& event);
  void On_Mouse_Wheel (wxMouseEvent& event);
  void On_Left_Down (wxMouseEvent& event);
  void On_Left_Up (wxMouseEvent& event);
  void On_Right_Down (wxMouseEvent& event);
  void On_Right_Up (wxMouseEvent& event);
  void On_Middle_Down (wxMouseEvent& event);
  void On_Middle_Up (wxMouseEvent& event);

  void Render ( );
  vtkGenericRenderWindowInteractor* Interactor ( );
  void Set_Interactor_Event (const wxMouseEvent& event);

private:
  wxGLContext* m_gl_context = nullptr;
  std::unique_ptr<robot_model::Vtk_Scene> m_scene;
  robot_model::Robot_Render_Controller m_render_controller;
};

#endif

#ifndef includeguard_robot_model_view_h_includeguard
#define includeguard_robot_model_view_h_includeguard

#include "robot_model_data.h"
#include "robot_render_controller.h"
#include "coordinate_frame_renderer.h"
#include "flange_interaction_controller.h"

#include <wx/glcanvas.h>

#include <memory>
#include <functional>

class vtkGenericRenderWindowInteractor;
class vtkRenderer;

namespace robot_model
{
class Vtk_Scene;
} // namespace robot_model

class Robot_Model_View : public wxGLCanvas
{
public:
  using Flange_Interaction_Mode = robot_model::Flange_Interaction_Mode;

  explicit Robot_Model_View (wxWindow* parent, wxWindowID id = wxID_ANY);
  ~Robot_Model_View ( ) override;

  void Load_Model (const robot_model::Robot_Model_Info& model);
  void Set_Joint_State (const robot_model::Robot_Joint_State& state);
  bool Has_Current_Model ( ) const;
  const robot_model::Robot_Kinematic_Params& Kinematic_Params ( ) const;
  const robot_model::Robot_Joint_State& Joint_State ( ) const;
  vtkRenderer* Scene_Renderer ( );
  void Render_Scene ( );
  void Set_Flange_Frame_Visible (bool visible);
  bool Has_Flange_Pose ( ) const;
  bool Get_World_From_Flange (robot_model::Matrix4* pose) const;
  robot_model::Robot_Pose_IK_Result Move_Flange_To_Pose (
    const robot_model::Matrix4& target_world_from_flange,
    const robot_model::Robot_Pose_IK_Options& options = { });
  void Set_World_Frame_Visible (bool visible);
  void Set_Flange_Interaction_Mode (Flange_Interaction_Mode mode);
  void Set_On_Flange_Dragged (
    std::function<void (const robot_model::Robot_Position_IK_Result&)> callback);
  void Set_On_Flange_Pose_Dragged (
    std::function<void (const robot_model::Robot_Pose_IK_Result&)> callback);

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
  void On_Mouse_Capture_Lost (wxMouseCaptureLostEvent& event);

  void Render ( );
  vtkGenericRenderWindowInteractor* Interactor ( );
  void Set_Interactor_Event (const wxMouseEvent& event);
  void Update_Flange_Frame ( );
  void Update_Flange_Hover (const wxMouseEvent& event);

private:
  wxGLContext* m_gl_context = nullptr;
  std::unique_ptr<robot_model::Vtk_Scene> m_scene;
  robot_model::Robot_Render_Controller m_render_controller;
  robot_model::Coordinate_Frame_Renderer m_world_frame_renderer {
    500.0, 20.0, true };
  robot_model::Flange_Interaction_Controller m_flange_interaction;
  std::function<void (const robot_model::Robot_Position_IK_Result&)>
    m_on_flange_dragged;
  std::function<void (const robot_model::Robot_Pose_IK_Result&)>
    m_on_flange_pose_dragged;
};

#endif

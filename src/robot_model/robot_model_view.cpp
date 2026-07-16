#include "robot_model_view.h"

#include "vtk_scene.h"

#include <wx/dcclient.h>
#include <wx/msw/window.h>

#include <GL/gl.h>

#include <utility>

Robot_Model_View::Robot_Model_View (wxWindow* parent, wxWindowID id)
  : wxGLCanvas (parent, id, nullptr, wxDefaultPosition, wxDefaultSize,
                wxBORDER_SIMPLE | wxFULL_REPAINT_ON_RESIZE)
{
  SetBackgroundStyle (wxBG_STYLE_PAINT);
  m_gl_context = new wxGLContext (this);

  Bind (wxEVT_SIZE, &Robot_Model_View::On_Size, this);
  Bind (wxEVT_PAINT, &Robot_Model_View::On_Paint, this);
  Bind (wxEVT_ERASE_BACKGROUND, &Robot_Model_View::On_Erase_Background, this);
  Bind (wxEVT_MOTION, &Robot_Model_View::On_Mouse_Move, this);
  Bind (wxEVT_MOUSEWHEEL, &Robot_Model_View::On_Mouse_Wheel, this);
  Bind (wxEVT_LEFT_DOWN, &Robot_Model_View::On_Left_Down, this);
  Bind (wxEVT_LEFT_UP, &Robot_Model_View::On_Left_Up, this);
  Bind (wxEVT_RIGHT_DOWN, &Robot_Model_View::On_Right_Down, this);
  Bind (wxEVT_RIGHT_UP, &Robot_Model_View::On_Right_Up, this);
  Bind (wxEVT_MIDDLE_DOWN, &Robot_Model_View::On_Middle_Down, this);
  Bind (wxEVT_MIDDLE_UP, &Robot_Model_View::On_Middle_Up, this);
  Bind (wxEVT_MOUSE_CAPTURE_LOST,
        &Robot_Model_View::On_Mouse_Capture_Lost, this);
}

Robot_Model_View::~Robot_Model_View ( )
{
  m_flange_interaction.Detach_Renderer ( );
  m_world_frame_renderer.Detach_Renderer ( );
  m_render_controller.Detach_Scene ( );
  m_scene.reset ( );
  delete m_gl_context;
  m_gl_context = nullptr;
}

void Robot_Model_View::Load_Model (const robot_model::Robot_Model_Info& model)
{
  Ensure_VTK ( );
  m_render_controller.Load_Model (model);
  if( m_scene )
  {
    m_flange_interaction.Attach_Renderer (m_scene->Renderer ( ));
    m_world_frame_renderer.Attach_Renderer (m_scene->Renderer ( ));
    m_world_frame_renderer.Set_Visible (m_world_frame_renderer.Is_Visible ( ));
  }
  Update_Flange_Frame ( );
  Render ( );
}

void Robot_Model_View::Set_Joint_State (
  const robot_model::Robot_Joint_State& state)
{
  m_render_controller.Set_Joint_State (state);
  Update_Flange_Frame ( );
  Render ( );
  Refresh (false);
}

bool Robot_Model_View::Has_Current_Model ( ) const
{
  return m_render_controller.Has_Current_Model ( );
}

const robot_model::Robot_Kinematic_Params&
Robot_Model_View::Kinematic_Params ( ) const
{
  return m_render_controller.Kinematic_Params ( );
}

const robot_model::Robot_Joint_State& Robot_Model_View::Joint_State ( ) const
{
  return m_render_controller.Joint_State ( );
}

vtkRenderer* Robot_Model_View::Scene_Renderer ( )
{
  Ensure_VTK ( );
  return m_scene ? m_scene->Renderer ( ) : nullptr;
}

void Robot_Model_View::Render_Scene ( )
{
  Render ( );
  Refresh (false);
}

void Robot_Model_View::Set_Flange_Frame_Visible (bool visible)
{
  Ensure_VTK ( );
  m_flange_interaction.Set_Frame_Visible (visible);
  Update_Flange_Frame ( );
  Render_Scene ( );
}

bool Robot_Model_View::Has_Flange_Pose ( ) const
{
  return m_render_controller.Has_Flange_Pose ( );
}

bool Robot_Model_View::Get_World_From_Flange (
  robot_model::Matrix4* pose) const
{
  if( !pose || !m_render_controller.Has_Flange_Pose ( ) ) return false;
  *pose = m_render_controller.World_From_Flange ( );
  return true;
}

void Robot_Model_View::Set_World_Frame_Visible (bool visible)
{
  Ensure_VTK ( );
  robot_model::Matrix4 identity = { };
  for( int index = 0; index < 4; ++index ) identity[index][index] = 1.0;
  m_world_frame_renderer.Set_World_From_Frame (identity);
  m_world_frame_renderer.Set_Visible (visible);
  Render_Scene ( );
}

void Robot_Model_View::Set_Flange_Interaction_Mode (
  Flange_Interaction_Mode mode)
{
  Ensure_VTK ( );
  m_flange_interaction.Set_Mode (mode);
  Update_Flange_Frame ( );
  Render_Scene ( );
}

void Robot_Model_View::Set_On_Flange_Dragged (
  std::function<void (const robot_model::Robot_Position_IK_Result&)> callback)
{
  m_on_flange_dragged = std::move (callback);
}

void Robot_Model_View::Set_On_Flange_Pose_Dragged (
  std::function<void (const robot_model::Robot_Pose_IK_Result&)> callback)
{
  m_on_flange_pose_dragged = std::move (callback);
}

void Robot_Model_View::Ensure_VTK ( )
{
  if( m_scene )
  {
    return;
  }

  if( !IsShownOnScreen ( ) )
  {
    return;
  }

  SetCurrent (*m_gl_context);
  m_scene = std::make_unique<robot_model::Vtk_Scene> ( );
  m_scene->Init (this, m_gl_context);
  m_render_controller.Attach_Scene (m_scene.get ( ));
  m_flange_interaction.Attach_Renderer (m_scene->Renderer ( ));
  m_world_frame_renderer.Attach_Renderer (m_scene->Renderer ( ));
  robot_model::Matrix4 identity = { };
  for( int index = 0; index < 4; ++index ) identity[index][index] = 1.0;
  m_world_frame_renderer.Set_World_From_Frame (identity);
  Update_Flange_Frame ( );
}

void Robot_Model_View::Update_Flange_Frame ( )
{
  if( m_render_controller.Has_Flange_Pose ( ) )
  {
    m_flange_interaction.Update_Flange_Pose (
      &m_render_controller.World_From_Flange ( ));
  }
  else
  {
    m_flange_interaction.Update_Flange_Pose (nullptr);
  }
}

void Robot_Model_View::On_Size (wxSizeEvent& event)
{
  Ensure_VTK ( );
  const auto size = GetClientSize ( );
  if( m_scene && size.x > 0 && size.y > 0 )
  {
    m_scene->Resize (size.x, size.y);
    Render ( );
  }
  event.Skip ( );
}

void Robot_Model_View::On_Paint (wxPaintEvent&)
{
  wxPaintDC dc (this);
  Ensure_VTK ( );
  Render ( );
}

void Robot_Model_View::On_Erase_Background (wxEraseEvent&)
{
}

void Robot_Model_View::Render ( )
{
  if( m_scene && m_scene->Is_Ready ( ) )
  {
    SetCurrent (*m_gl_context);
    const auto size = GetClientSize ( );
    glViewport (0, 0, size.x, size.y);
    glClearColor (0.02f, 0.08f, 0.16f, 1.0f);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m_scene->Render ( );
  }
}

vtkGenericRenderWindowInteractor* Robot_Model_View::Interactor ( )
{
  Ensure_VTK ( );
  return m_scene ? m_scene->Get_Interactor ( ) : nullptr;
}

void Robot_Model_View::Set_Interactor_Event (const wxMouseEvent& event)
{
  if( !m_scene )
  {
    return;
  }

  auto* interactor = m_scene->Get_Interactor ( );
  if( !interactor )
  {
    return;
  }

  const auto pos = event.GetPosition ( );
  interactor->SetEventInformationFlipY (
    pos.x, pos.y,
    event.ControlDown ( ) ? 1 : 0,
    event.ShiftDown ( ) ? 1 : 0
  );
}

void Robot_Model_View::On_Mouse_Move (wxMouseEvent& event)
{
  if( m_flange_interaction.Is_Dragging ( ) )
  {
    const auto position = event.GetPosition ( );
    const auto size = GetClientSize ( );
    robot_model::Flange_Drag_Update update;
    if( m_flange_interaction.Update_Drag (
          position.x, size.y - 1 - position.y, &update) )
    {
      if( update.target_type ==
          robot_model::Flange_Drag_Update::Target_Type::Position )
      {
        const auto result = m_render_controller.Move_Flange_To (
          update.target_position_world);
        Update_Flange_Frame ( );
        if( m_on_flange_dragged ) m_on_flange_dragged (result);
      }
      else if( update.target_type ==
               robot_model::Flange_Drag_Update::Target_Type::Pose )
      {
        const auto result = m_render_controller.Move_Flange_To_Pose (
          update.target_world_from_flange);
        Update_Flange_Frame ( );
        if( m_on_flange_pose_dragged ) m_on_flange_pose_dragged (result);
      }
      Render ( );
      Refresh (false);
    }
    return;
  }

  if( !event.LeftIsDown ( ) && !event.RightIsDown ( ) &&
      !event.MiddleIsDown ( ) )
  {
    Update_Flange_Hover (event);
  }

  auto* interactor = Interactor ( );
  if( !interactor )
  {
    return;
  }

  Set_Interactor_Event (event);
  interactor->MouseMoveEvent ( );
  Render ( );
}

void Robot_Model_View::On_Mouse_Wheel (wxMouseEvent& event)
{
  auto* interactor = Interactor ( );
  if( !interactor )
  {
    return;
  }

  Set_Interactor_Event (event);
  if( event.GetWheelRotation ( ) > 0 )
  {
    interactor->MouseWheelForwardEvent ( );
  }
  else
  {
    interactor->MouseWheelBackwardEvent ( );
  }
  Render ( );
}

void Robot_Model_View::On_Left_Down (wxMouseEvent& event)
{
  Ensure_VTK ( );
  if( m_scene )
  {
    const auto pos = event.GetPosition ( );
    if( m_scene->Handle_View_Cube_Click (pos.x, pos.y) )
    {
      Render ( );
      return;
    }
  }

  if( m_render_controller.Has_Flange_Pose ( ) )
  {
    const auto position = event.GetPosition ( );
    const auto size = GetClientSize ( );
    const auto& pose = m_render_controller.World_From_Flange ( );
    if( m_flange_interaction.Begin_Drag (
          position.x, size.y - 1 - position.y, pose) )
    {
      SetFocus ( );
      if( !HasCapture ( ) )
      {
        CaptureMouse ( );
      }
      return;
    }
  }

  auto* interactor = m_scene ? m_scene->Get_Interactor ( ) : nullptr;
  if( !interactor )
  {
    return;
  }

  SetFocus ( );
  if( !HasCapture ( ) )
  {
    CaptureMouse ( );
  }
  Set_Interactor_Event (event);
  interactor->LeftButtonPressEvent ( );
  Render ( );
}

void Robot_Model_View::On_Left_Up (wxMouseEvent& event)
{
  if( m_flange_interaction.Is_Dragging ( ) )
  {
    m_flange_interaction.End_Drag ( );
    if( HasCapture ( ) ) ReleaseMouse ( );
    Render ( );
    return;
  }

  if( auto* interactor = Interactor ( ) )
  {
    Set_Interactor_Event (event);
    interactor->LeftButtonReleaseEvent ( );
    Render ( );
  }
  if( HasCapture ( ) )
  {
    ReleaseMouse ( );
  }
}

void Robot_Model_View::On_Right_Down (wxMouseEvent& event)
{
  auto* interactor = Interactor ( );
  if( !interactor )
  {
    return;
  }

  SetFocus ( );
  if( !HasCapture ( ) )
  {
    CaptureMouse ( );
  }
  Set_Interactor_Event (event);
  interactor->RightButtonPressEvent ( );
  Render ( );
}

void Robot_Model_View::On_Right_Up (wxMouseEvent& event)
{
  if( auto* interactor = Interactor ( ) )
  {
    Set_Interactor_Event (event);
    interactor->RightButtonReleaseEvent ( );
    Render ( );
  }
  if( HasCapture ( ) )
  {
    ReleaseMouse ( );
  }
}

void Robot_Model_View::On_Middle_Down (wxMouseEvent& event)
{
  auto* interactor = Interactor ( );
  if( !interactor )
  {
    return;
  }

  SetFocus ( );
  if( !HasCapture ( ) )
  {
    CaptureMouse ( );
  }
  Set_Interactor_Event (event);
  interactor->MiddleButtonPressEvent ( );
  Render ( );
}

void Robot_Model_View::On_Middle_Up (wxMouseEvent& event)
{
  if( auto* interactor = Interactor ( ) )
  {
    Set_Interactor_Event (event);
    interactor->MiddleButtonReleaseEvent ( );
    Render ( );
  }
  if( HasCapture ( ) )
  {
    ReleaseMouse ( );
  }
}

void Robot_Model_View::On_Mouse_Capture_Lost (wxMouseCaptureLostEvent&)
{
  m_flange_interaction.End_Drag ( );
  Render ( );
}

void Robot_Model_View::Update_Flange_Hover (const wxMouseEvent& event)
{
  const auto position = event.GetPosition ( );
  const auto size = GetClientSize ( );
  const robot_model::Matrix4* pose = m_render_controller.Has_Flange_Pose ( )
    ? &m_render_controller.World_From_Flange ( )
    : nullptr;
  bool hover_changed = false;
  const bool over_handle = m_flange_interaction.Update_Hover (
    position.x, size.y - 1 - position.y, pose, &hover_changed);
  if( !hover_changed ) return;
  SetCursor (over_handle ? wxCursor (wxCURSOR_HAND) : wxNullCursor);
  Render ( );
}

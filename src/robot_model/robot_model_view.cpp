#include "robot_model_view.h"

#include "vtk_scene.h"

#include <wx/dcclient.h>
#include <wx/msw/window.h>

#include <GL/gl.h>

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
}

Robot_Model_View::~Robot_Model_View ( )
{
  m_render_controller.Detach_Scene ( );
  m_scene.reset ( );
  delete m_gl_context;
  m_gl_context = nullptr;
}

void Robot_Model_View::Load_Model (const robot_model::Robot_Model_Info& model)
{
  Ensure_VTK ( );
  m_render_controller.Load_Model (model);
  Render ( );
}

void Robot_Model_View::Set_Joint_State (
  const robot_model::Robot_Joint_State& state)
{
  m_render_controller.Set_Joint_State (state);
  Render ( );
  Refresh (false);
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

#include "point_cloud_view.h"

#include "camera_service.h"
#include "camera_stream_config.h"
#include "point_cloud_renderer.h"
#include "vtk_scene.h"

#include <wx/button.h>
#include <wx/dcclient.h>
#include <wx/filedlg.h>
#include <wx/glcanvas.h>
#include <wx/msw/window.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

#include <vtkGenericRenderWindowInteractor.h>
#include <vtkCommand.h>

#include <GL/gl.h>

#include <filesystem>
#include <memory>
#include <utility>

namespace
{
wxString From_Utf8 (const char* text)
{
  return wxString::FromUTF8 (text);
}

wxString From_Utf8 (const std::string& text)
{
  return wxString::FromUTF8 (text.c_str ( ));
}
} // namespace

class Point_Cloud_Canvas : public wxGLCanvas
{
public:
  explicit Point_Cloud_Canvas (wxWindow* parent)
    : wxGLCanvas (
        parent, wxID_ANY, nullptr, wxDefaultPosition, wxDefaultSize,
        wxBORDER_SIMPLE | wxFULL_REPAINT_ON_RESIZE)
  {
    SetBackgroundStyle (wxBG_STYLE_PAINT);
    m_gl_context = new wxGLContext (this);
    Bind (wxEVT_SIZE, &Point_Cloud_Canvas::On_Size, this);
    Bind (wxEVT_PAINT, &Point_Cloud_Canvas::On_Paint, this);
    Bind (wxEVT_ERASE_BACKGROUND,
          &Point_Cloud_Canvas::On_Erase_Background, this);
    Bind (wxEVT_MOTION, &Point_Cloud_Canvas::On_Mouse_Move, this);
    Bind (wxEVT_MOUSEWHEEL, &Point_Cloud_Canvas::On_Mouse_Wheel, this);
    Bind (wxEVT_LEFT_DOWN, &Point_Cloud_Canvas::On_Left_Down, this);
    Bind (wxEVT_LEFT_UP, &Point_Cloud_Canvas::On_Left_Up, this);
    Bind (wxEVT_RIGHT_DOWN, &Point_Cloud_Canvas::On_Right_Down, this);
    Bind (wxEVT_RIGHT_UP, &Point_Cloud_Canvas::On_Right_Up, this);
    Bind (wxEVT_MIDDLE_DOWN, &Point_Cloud_Canvas::On_Middle_Down, this);
    Bind (wxEVT_MIDDLE_UP, &Point_Cloud_Canvas::On_Middle_Up, this);
  }

  ~Point_Cloud_Canvas ( ) override
  {
    m_renderer.Detach_Renderer ( );
    m_scene.reset ( );
    delete m_gl_context;
    m_gl_context = nullptr;
  }

  bool Load_Ply (
    const std::filesystem::path& path,
    std::string* error_message)
  {
    Ensure_VTK ( );
    if( !m_scene || !m_scene->Is_Ready ( ) )
    {
      if( error_message )
      {
        *error_message = "VTK point-cloud scene is not ready";
      }
      return false;
    }
    if( !m_renderer.Load_Ply (path, error_message) )
    {
      return false;
    }
    m_scene->Remove_All_Lights ( );
    m_scene->Reset_Camera ( );
    Render ( );
    Refresh (false);
    return true;
  }

  bool Set_Live_Point_Cloud (
    const Camera_Point_Cloud& cloud,
    std::string* error_message)
  {
    Ensure_VTK ( );
    if( !m_scene || !m_scene->Is_Ready ( ) )
    {
      if( error_message )
      {
        *error_message = "VTK point-cloud scene is not ready";
      }
      return false;
    }
    const bool had_cloud = m_renderer.Has_Point_Cloud ( );
    if( !m_renderer.Set_Point_Data (cloud.xyz, cloud.rgb, error_message) )
    {
      return false;
    }
    if( !had_cloud )
    {
      m_scene->Remove_All_Lights ( );
      m_scene->Reset_Camera ( );
    }
    Render ( );
    Refresh (false);
    return true;
  }

  void Clear ( )
  {
    m_renderer.Clear ( );
    Render ( );
    Refresh (false);
  }

  void Fit_View ( )
  {
    Ensure_VTK ( );
    if( m_scene && m_renderer.Has_Point_Cloud ( ) )
    {
      m_scene->Remove_All_Lights ( );
      m_scene->Reset_Camera ( );
      Render ( );
      Refresh (false);
    }
  }

  std::size_t Point_Count ( ) const
  {
    return m_renderer.Point_Count ( );
  }

private:
  void Ensure_VTK ( )
  {
    if( m_scene || !IsShownOnScreen ( ) )
    {
      return;
    }
    SetCurrent (*m_gl_context);
    m_scene = std::make_unique<robot_model::Vtk_Scene> ( );
    m_scene->Init (this, m_gl_context);
    m_renderer.Attach_Renderer (m_scene->Renderer ( ));
  }

  void Render ( )
  {
    if( !m_scene || !m_scene->Is_Ready ( ) )
    {
      return;
    }
    SetCurrent (*m_gl_context);
    const auto size = GetClientSize ( );
    glViewport (0, 0, size.x, size.y);
    glClearColor (0.02f, 0.04f, 0.07f, 1.0f);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m_scene->Render ( );
  }

  vtkGenericRenderWindowInteractor* Interactor ( )
  {
    Ensure_VTK ( );
    return m_scene ? m_scene->Get_Interactor ( ) : nullptr;
  }

  void Set_Interactor_Event (const wxMouseEvent& event)
  {
    auto* interactor = m_scene ? m_scene->Get_Interactor ( ) : nullptr;
    if( !interactor )
    {
      return;
    }
    const auto pos = event.GetPosition ( );
    interactor->SetEventInformationFlipY (
      pos.x, pos.y,
      event.ControlDown ( ) ? 1 : 0,
      event.ShiftDown ( ) ? 1 : 0);
  }

  void On_Size (wxSizeEvent& event)
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

  void On_Paint (wxPaintEvent&)
  {
    wxPaintDC dc (this);
    Ensure_VTK ( );
    Render ( );
  }

  void On_Erase_Background (wxEraseEvent&)
  {
  }

  void On_Mouse_Move (wxMouseEvent& event)
  {
    if( auto* interactor = Interactor ( ) )
    {
      Set_Interactor_Event (event);
      interactor->MouseMoveEvent ( );
      Render ( );
    }
  }

  void On_Mouse_Wheel (wxMouseEvent& event)
  {
    if( auto* interactor = Interactor ( ) )
    {
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
  }

  void Begin_Mouse_Action (wxMouseEvent& event, unsigned long vtk_event)
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
    interactor->InvokeEvent (vtk_event);
    Render ( );
  }

  void End_Mouse_Action (wxMouseEvent& event, unsigned long vtk_event)
  {
    if( auto* interactor = Interactor ( ) )
    {
      Set_Interactor_Event (event);
      interactor->InvokeEvent (vtk_event);
      Render ( );
    }
    if( HasCapture ( ) )
    {
      ReleaseMouse ( );
    }
  }

  void On_Left_Down (wxMouseEvent& event)
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
    Begin_Mouse_Action (event, vtkCommand::LeftButtonPressEvent);
  }

  void On_Left_Up (wxMouseEvent& event)
  {
    End_Mouse_Action (event, vtkCommand::LeftButtonReleaseEvent);
  }

  void On_Right_Down (wxMouseEvent& event)
  {
    Begin_Mouse_Action (event, vtkCommand::RightButtonPressEvent);
  }

  void On_Right_Up (wxMouseEvent& event)
  {
    End_Mouse_Action (event, vtkCommand::RightButtonReleaseEvent);
  }

  void On_Middle_Down (wxMouseEvent& event)
  {
    Begin_Mouse_Action (event, vtkCommand::MiddleButtonPressEvent);
  }

  void On_Middle_Up (wxMouseEvent& event)
  {
    End_Mouse_Action (event, vtkCommand::MiddleButtonReleaseEvent);
  }

private:
  wxGLContext* m_gl_context = nullptr;
  std::unique_ptr<robot_model::Vtk_Scene> m_scene;
  point_cloud::Point_Cloud_Renderer m_renderer;
};

Point_Cloud_View::Point_Cloud_View (
  wxWindow* parent,
  Camera_Service& camera_service)
  : wxPanel (parent, wxID_ANY),
    m_camera_service (camera_service)
{
  auto* title = new wxStaticText (
    this, wxID_ANY, From_Utf8 (u8"点云显示"));
  m_live_button = new wxButton (
    this, wxID_ANY, From_Utf8 (u8"实时相机"));
  auto* load_button = new wxButton (
    this, wxID_ANY, From_Utf8 (u8"加载 PLY"));
  auto* clear_button = new wxButton (
    this, wxID_ANY, From_Utf8 (u8"清空"));
  auto* fit_button = new wxButton (
    this, wxID_ANY, From_Utf8 (u8"适应视图"));
  m_status_text = new wxStaticText (
    this, wxID_ANY, From_Utf8 (u8"等待相机点云"));
  m_canvas = new Point_Cloud_Canvas (this);
  m_canvas->SetMinSize (wxSize (320, 240));

  m_live_button->Bind (
    wxEVT_BUTTON, &Point_Cloud_View::On_Live_Camera, this);
  load_button->Bind (wxEVT_BUTTON, &Point_Cloud_View::On_Load_Ply, this);
  clear_button->Bind (wxEVT_BUTTON, &Point_Cloud_View::On_Clear, this);
  fit_button->Bind (wxEVT_BUTTON, &Point_Cloud_View::On_Fit_View, this);

  auto* toolbar = new wxBoxSizer (wxHORIZONTAL);
  toolbar->Add (title, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 12);
  toolbar->Add (m_live_button, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
  toolbar->Add (load_button, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
  toolbar->Add (clear_button, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
  toolbar->Add (fit_button, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 12);
  toolbar->Add (m_status_text, 1, wxALIGN_CENTER_VERTICAL);

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (toolbar, 0, wxEXPAND | wxALL, 8);
  sizer->Add (m_canvas, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  SetSizer (sizer);

  m_timer.SetOwner (this);
  Bind (wxEVT_TIMER, &Point_Cloud_View::On_Timer, this, m_timer.GetId ( ));
  m_worker = std::thread (&Point_Cloud_View::Worker_Loop, this);
  m_timer.Start (50);
}

Point_Cloud_View::~Point_Cloud_View ( )
{
  m_timer.Stop ( );
  {
    std::lock_guard<std::mutex> lock (m_worker_mutex);
    m_exit_requested = true;
    m_pending_job.reset ( );
  }
  m_worker_ready.notify_all ( );
  if( m_worker.joinable ( ) )
  {
    m_worker.join ( );
  }
}

void Point_Cloud_View::Submit_Frame (
  std::shared_ptr<const Camera_Frame> frame)
{
  if( !frame || frame->generation == m_last_submitted_generation )
  {
    return;
  }
  Conversion_Job job;
  job.id = m_next_job_id++;
  job.epoch = m_live_epoch;
  job.generation = frame->generation;
  job.frame = std::move (frame);
  const auto generation = job.generation;
  {
    std::lock_guard<std::mutex> lock (m_worker_mutex);
    m_pending_job = std::move (job);
  }
  m_last_submitted_generation = generation;
  m_worker_ready.notify_one ( );
}

void Point_Cloud_View::Worker_Loop ( )
{
  while( true )
  {
    Conversion_Job job;
    {
      std::unique_lock<std::mutex> lock (m_worker_mutex);
      m_worker_ready.wait (
        lock,
        [this] { return m_exit_requested || m_pending_job.has_value ( ); });
      if( m_exit_requested )
      {
        return;
      }
      job = std::move (*m_pending_job);
      m_pending_job.reset ( );
    }

    Conversion_Result result;
    result.id = job.id;
    result.epoch = job.epoch;
    result.generation = job.generation;
    result.success = Convert_Camera_Frame_To_Point_Cloud (
      *job.frame, &result.cloud, &result.error);
    {
      std::lock_guard<std::mutex> lock (m_worker_mutex);
      m_pending_result = std::move (result);
    }
  }
}

void Point_Cloud_View::Consume_Result ( )
{
  std::optional<Conversion_Result> result;
  {
    std::lock_guard<std::mutex> lock (m_worker_mutex);
    if( m_pending_result )
    {
      result = std::move (m_pending_result);
      m_pending_result.reset ( );
    }
  }
  if( !result || result->id <= m_last_displayed_job )
  {
    return;
  }
  m_last_displayed_job = result->id;
  if( m_source_mode != Source_Mode::Live_Camera ||
      result->epoch != m_live_epoch )
  {
    return;
  }
  if( !result->success )
  {
    m_status_text->SetLabel (
      From_Utf8 (u8"无法显示：") + From_Utf8 (result->error));
    return;
  }

  std::string error;
  if( !m_canvas->Set_Live_Point_Cloud (result->cloud, &error) )
  {
    m_status_text->SetLabel (
      From_Utf8 (u8"显示失败：") + From_Utf8 (error));
    return;
  }

  m_status_text->SetLabel (wxString::Format (
    From_Utf8 (u8"%s  有效点=%zu/%zu  Frame=%u"),
    From_Utf8 (Camera_Image_Type_Name (result->cloud.source_image_type)),
    result->cloud.Point_Count ( ),
    result->cloud.source_point_count,
    result->cloud.source_frame_number));
}

void Point_Cloud_View::On_Live_Camera (wxCommandEvent&)
{
  m_source_mode = Source_Mode::Live_Camera;
  ++m_live_epoch;
  m_last_submitted_generation = 0;
  m_status_text->SetLabel (From_Utf8 (u8"等待相机点云……"));
}

void Point_Cloud_View::On_Load_Ply (wxCommandEvent&)
{
  wxFileDialog dialog (
    this,
    From_Utf8 (u8"加载点云"),
    "",
    "",
    "PLY files (*.ply)|*.ply|All files (*.*)|*.*",
    wxFD_OPEN | wxFD_FILE_MUST_EXIST);
  if( dialog.ShowModal ( ) != wxID_OK )
  {
    return;
  }

  m_status_text->SetLabel (From_Utf8 (u8"正在加载 PLY……"));
  m_status_text->Update ( );
  std::string error;
  if( !m_canvas->Load_Ply (
        std::filesystem::path (dialog.GetPath ( ).ToStdWstring ( )), &error) )
  {
    m_status_text->SetLabel (
      From_Utf8 (u8"加载失败：") + From_Utf8 (error));
    return;
  }

  m_source_mode = Source_Mode::Ply_File;
  ++m_live_epoch;
  m_status_text->SetLabel (wxString::Format (
    From_Utf8 (u8"PLY 已加载，有效点=%zu"), m_canvas->Point_Count ( )));
}

void Point_Cloud_View::On_Clear (wxCommandEvent&)
{
  m_source_mode = Source_Mode::None;
  ++m_live_epoch;
  m_canvas->Clear ( );
  m_status_text->SetLabel (
    From_Utf8 (u8"点云已清空；点击“实时相机”可继续显示"));
}

void Point_Cloud_View::On_Fit_View (wxCommandEvent&)
{
  m_canvas->Fit_View ( );
}

void Point_Cloud_View::On_Timer (wxTimerEvent&)
{
  if( !IsShownOnScreen ( ) )
  {
    return;
  }
  Consume_Result ( );
  if( m_source_mode != Source_Mode::Live_Camera )
  {
    return;
  }
  if( !m_camera_service.Is_Device_Open ( ) )
  {
    m_status_text->SetLabel (From_Utf8 (u8"请先打开相机"));
    return;
  }

  const auto frame = m_camera_service.Latest_Frame ( );
  if( !frame )
  {
    m_status_text->SetLabel (From_Utf8 (u8"等待采集点云……"));
    return;
  }
  Submit_Frame (frame);
}

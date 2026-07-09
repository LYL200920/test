#include "camera_image_view.h"

#include "camera_service.h"
#include "camera_stream_config.h"

#include <wx/choice.h>
#include <wx/dcbuffer.h>
#include <wx/dcmemory.h>
#include <wx/image.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

#include <algorithm>
#include <cstring>
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

class Camera_Bitmap_Canvas : public wxPanel
{
public:
  explicit Camera_Bitmap_Canvas (wxWindow* parent)
    : wxPanel (parent, wxID_ANY)
  {
    SetBackgroundStyle (wxBG_STYLE_PAINT);
    SetBackgroundColour (*wxBLACK);
    Bind (wxEVT_PAINT, &Camera_Bitmap_Canvas::On_Paint, this);
    Bind (wxEVT_SIZE, &Camera_Bitmap_Canvas::On_Size, this);
  }

  void Set_Bitmap (wxBitmap bitmap)
  {
    m_bitmap = std::move (bitmap);
    Refresh (false);
  }

  void Clear_Bitmap ( )
  {
    if( !m_bitmap.IsOk ( ) )
    {
      return;
    }
    m_bitmap = wxBitmap ( );
    Refresh (false);
  }

private:
  void On_Size (wxSizeEvent& event)
  {
    event.Skip ( );
    Refresh (false);
  }

  void On_Paint (wxPaintEvent&)
  {
    wxAutoBufferedPaintDC dc (this);
    dc.SetBackground (wxBrush (*wxBLACK));
    dc.Clear ( );
    if( !m_bitmap.IsOk ( ) )
    {
      return;
    }

    const auto client = GetClientSize ( );
    if( client.x <= 0 || client.y <= 0 )
    {
      return;
    }
    const double image_aspect = static_cast<double> (m_bitmap.GetWidth ( )) /
                                m_bitmap.GetHeight ( );
    const double client_aspect = static_cast<double> (client.x) / client.y;
    int target_width = client.x;
    int target_height = client.y;
    if( client_aspect > image_aspect )
    {
      target_width = static_cast<int> (client.y * image_aspect);
    }
    else
    {
      target_height = static_cast<int> (client.x / image_aspect);
    }
    target_width = std::max (target_width, 1);
    target_height = std::max (target_height, 1);
    const int x = (client.x - target_width) / 2;
    const int y = (client.y - target_height) / 2;
    wxMemoryDC source_dc;
    source_dc.SelectObject (m_bitmap);
    dc.StretchBlit (
      x, y, target_width, target_height,
      &source_dc,
      0, 0, m_bitmap.GetWidth ( ), m_bitmap.GetHeight ( ),
      wxCOPY,
      false);
    source_dc.SelectObject (wxNullBitmap);
  }

private:
  wxBitmap m_bitmap;
};

Camera_Image_View::Camera_Image_View (
  wxWindow* parent,
  Camera_Service& camera_service)
  : wxPanel (parent, wxID_ANY),
    m_camera_service (camera_service)
{
  auto* title = new wxStaticText (
    this, wxID_ANY, From_Utf8 (u8"相机图像预览"));
  m_mode_choice = new wxChoice (this, wxID_ANY);
  m_mode_choice->Append (From_Utf8 (u8"自动选择"));
  m_mode_choice->Append (From_Utf8 (u8"彩色图"));
  m_mode_choice->Append (From_Utf8 (u8"深度图"));
  m_mode_choice->SetSelection (0);
  m_mode_choice->Bind (
    wxEVT_CHOICE, &Camera_Image_View::On_Mode_Changed, this);
  m_status_text = new wxStaticText (
    this, wxID_ANY, From_Utf8 (u8"等待相机图像"));
  m_canvas = new Camera_Bitmap_Canvas (this);
  m_canvas->SetMinSize (wxSize (320, 240));

  auto* toolbar = new wxBoxSizer (wxHORIZONTAL);
  toolbar->Add (title, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 12);
  toolbar->Add (m_mode_choice, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 12);
  toolbar->Add (m_status_text, 1, wxALIGN_CENTER_VERTICAL);

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (toolbar, 0, wxEXPAND | wxALL, 8);
  sizer->Add (m_canvas, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  SetSizer (sizer);

  m_timer.SetOwner (this);
  Bind (wxEVT_TIMER, &Camera_Image_View::On_Timer, this, m_timer.GetId ( ));
  m_worker = std::thread (&Camera_Image_View::Worker_Loop, this);
  m_timer.Start (33);
}

Camera_Image_View::~Camera_Image_View ( )
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

Camera_Image_Display_Mode Camera_Image_View::Selected_Mode ( ) const
{
  switch( m_mode_choice->GetSelection ( ) )
  {
  case 1:
    return Camera_Image_Display_Mode::Color;
  case 2:
    return Camera_Image_Display_Mode::Depth;
  default:
    return Camera_Image_Display_Mode::Automatic;
  }
}

void Camera_Image_View::Submit_Frame (
  std::shared_ptr<const Camera_Frame> frame)
{
  if( !frame )
  {
    return;
  }
  const auto mode = Selected_Mode ( );
  if( frame->generation == m_last_submitted_generation &&
      mode == m_last_submitted_mode )
  {
    return;
  }

  Conversion_Job job;
  job.id = m_next_job_id++;
  job.generation = frame->generation;
  job.mode = mode;
  job.frame = std::move (frame);
  const auto submitted_generation = job.generation;
  {
    std::lock_guard<std::mutex> lock (m_worker_mutex);
    m_pending_job = std::move (job);
  }
  m_last_submitted_generation = submitted_generation;
  m_last_submitted_mode = mode;
  m_worker_ready.notify_one ( );
}

void Camera_Image_View::Worker_Loop ( )
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
    result.generation = job.generation;
    result.mode = job.mode;
    result.success = Convert_Camera_Frame_To_Display_Image (
      *job.frame, job.mode, &result.image, &result.error);
    {
      std::lock_guard<std::mutex> lock (m_worker_mutex);
      m_pending_result = std::move (result);
    }
  }
}

void Camera_Image_View::Consume_Result ( )
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
  if( result->mode != Selected_Mode ( ) )
  {
    return;
  }
  if( !result->success )
  {
    m_canvas->Clear_Bitmap ( );
    m_status_text->SetLabel (
      From_Utf8 (u8"无法显示：") + From_Utf8 (result->error));
    return;
  }

  wxImage image (
    static_cast<int> (result->image.width),
    static_cast<int> (result->image.height));
  const auto data_size = result->image.rgb.size ( );
  if( !image.IsOk ( ) || image.GetData ( ) == nullptr ||
      data_size != static_cast<std::size_t> (result->image.width) *
                   result->image.height * 3 )
  {
    m_canvas->Clear_Bitmap ( );
    m_status_text->SetLabel (From_Utf8 (u8"显示图像缓冲区无效"));
    return;
  }
  std::memcpy (image.GetData ( ), result->image.rgb.data ( ), data_size);
  m_canvas->Set_Bitmap (wxBitmap (image));
  m_status_text->SetLabel (wxString::Format (
    From_Utf8 (u8"%s  %ux%u  Frame=%u"),
    From_Utf8 (Camera_Image_Type_Name (result->image.source_image_type)),
    result->image.width,
    result->image.height,
    result->image.source_frame_number));
}

void Camera_Image_View::On_Mode_Changed (wxCommandEvent&)
{
  m_last_submitted_generation = 0;
  m_status_text->SetLabel (From_Utf8 (u8"正在切换预览图像……"));
}

void Camera_Image_View::On_Timer (wxTimerEvent&)
{
  Consume_Result ( );
  if( !IsShownOnScreen ( ) )
  {
    return;
  }

  const auto frame = m_camera_service.Latest_Frame ( );
  if( !frame )
  {
    if( !m_camera_service.Is_Device_Open ( ) )
    {
      m_canvas->Clear_Bitmap ( );
    }
    m_status_text->SetLabel (
      m_camera_service.Is_Device_Open ( )
        ? From_Utf8 (u8"等待采集图像……")
        : From_Utf8 (u8"请先打开相机"));
    return;
  }
  Submit_Frame (frame);
}

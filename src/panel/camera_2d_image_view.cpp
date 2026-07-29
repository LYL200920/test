#include "camera_2d_image_view.h"

#include "camera_2d_service.h"

#include <wx/dcbuffer.h>
#include <wx/image.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

#include <algorithm>
#include <cstring>
#include <utility>

Camera_2D_Bitmap_Canvas::Camera_2D_Bitmap_Canvas(wxWindow *parent)
  : wxPanel(parent, wxID_ANY)
{
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  Bind(wxEVT_PAINT, &Camera_2D_Bitmap_Canvas::On_Paint, this);
  Bind(wxEVT_SIZE, &Camera_2D_Bitmap_Canvas::On_Size, this);
}

void Camera_2D_Bitmap_Canvas::Set_Bitmap(wxBitmap bitmap)
{
  m_bitmap = std::move(bitmap);
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::Clear_Bitmap()
{
  m_bitmap = wxBitmap();
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::On_Paint(wxPaintEvent &)
{
  wxAutoBufferedPaintDC dc(this);
  dc.SetBackground(wxBrush(wxColour(24, 24, 24)));
  dc.Clear();
  if (!m_bitmap.IsOk())
  {
    return;
  }
  const wxSize area = GetClientSize();
  const int source_width = m_bitmap.GetWidth();
  const int source_height = m_bitmap.GetHeight();
  if (area.x <= 0 || area.y <= 0 ||
      source_width <= 0 || source_height <= 0)
  {
    return;
  }
  const double scale = std::min(
    static_cast<double>(area.x) / source_width,
    static_cast<double>(area.y) / source_height);
  const int width = std::max(
    1, static_cast<int>(source_width * scale));
  const int height = std::max(
    1, static_cast<int>(source_height * scale));
  wxBitmap rendered = m_bitmap;
  if (width != source_width || height != source_height)
  {
    rendered = wxBitmap(
      m_bitmap.ConvertToImage().Scale(
        width, height, wxIMAGE_QUALITY_HIGH));
  }
  dc.DrawBitmap(
    rendered,
    (area.x - width) / 2,
    (area.y - height) / 2,
    false);
}

void Camera_2D_Bitmap_Canvas::On_Size(wxSizeEvent &event)
{
  Refresh(false);
  event.Skip();
}

Camera_2D_Image_View::Camera_2D_Image_View(
  wxWindow *parent,
  Camera_2D_Service &camera_service)
  : wxPanel(parent, wxID_ANY),
    m_camera_service(camera_service)
{
  m_canvas = new Camera_2D_Bitmap_Canvas(this);
  m_status_text = new wxStaticText(
    this, wxID_ANY, wxString::FromUTF8(u8"请先打开2D相机"));
  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(m_canvas, 1, wxEXPAND);
  sizer->Add(m_status_text, 0, wxEXPAND | wxALL, 6);
  SetSizer(sizer);

  m_timer.SetOwner(this);
  Bind(
    wxEVT_TIMER, &Camera_2D_Image_View::On_Timer,
    this, m_timer.GetId());
  m_timer.Start(33);
  m_worker = std::thread(&Camera_2D_Image_View::Worker_Loop, this);
}

Camera_2D_Image_View::~Camera_2D_Image_View()
{
  m_timer.Stop();
  {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    m_exit_requested = true;
    m_pending_frame.reset();
  }
  m_worker_ready.notify_one();
  if (m_worker.joinable())
  {
    m_worker.join();
  }
}

void Camera_2D_Image_View::Submit_Frame(
  std::shared_ptr<const jutze_camera::camera_frame> frame)
{
  if (!frame || frame == m_last_submitted_frame)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    m_pending_frame = frame;
  }
  m_last_submitted_frame = std::move(frame);
  m_worker_ready.notify_one();
}

void Camera_2D_Image_View::Worker_Loop()
{
  while (true)
  {
    std::shared_ptr<const jutze_camera::camera_frame> frame;
    unsigned long long job_id = 0;
    {
      std::unique_lock<std::mutex> lock(m_worker_mutex);
      m_worker_ready.wait(
        lock,
        [this]
        {
          return m_exit_requested || static_cast<bool>(m_pending_frame);
        });
      if (m_exit_requested)
      {
        return;
      }
      frame = std::move(m_pending_frame);
      m_pending_frame.reset();
      job_id = m_next_job_id++;
    }

    Conversion_Result result;
    result.job_id = job_id;
    result.success = Convert_Camera_2D_Frame(
      *frame, &result.image, &result.error);
    {
      std::lock_guard<std::mutex> lock(m_worker_mutex);
      m_pending_result = std::move(result);
    }
  }
}

void Camera_2D_Image_View::Consume_Result()
{
  std::optional<Conversion_Result> result;
  {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    if (m_pending_result)
    {
      result = std::move(m_pending_result);
      m_pending_result.reset();
    }
  }
  if (!result || result->job_id <= m_last_displayed_job_id)
  {
    return;
  }
  m_last_displayed_job_id = result->job_id;
  if (!result->success)
  {
    m_canvas->Clear_Bitmap();
    m_status_text->SetLabel(
      wxString::FromUTF8(result->error.c_str()));
    return;
  }

  wxImage image(
    static_cast<int>(result->image.width),
    static_cast<int>(result->image.height));
  if (!image.IsOk() || !image.GetData())
  {
    m_status_text->SetLabel(
      wxString::FromUTF8(u8"创建2D显示图像失败"));
    return;
  }
  std::memcpy(
    image.GetData(),
    result->image.rgb.data(),
    result->image.rgb.size());
  m_canvas->Set_Bitmap(wxBitmap(image));
  const auto status = m_camera_service.Status();
  m_status_text->SetLabel(wxString::Format(
    wxString::FromUTF8(
      u8"%u × %u  Frame=%llu  %.1f FPS  %s"),
    result->image.width,
    result->image.height,
    result->image.frame_number,
    status.frames_per_second,
    wxString::FromUTF8(
      jutze_camera::pixel_format_name(status.pixel_format))));
}

void Camera_2D_Image_View::On_Timer(wxTimerEvent &)
{
  Consume_Result();
  if (!IsShownOnScreen())
  {
    return;
  }
  const auto frame = m_camera_service.Latest_Frame();
  if (!frame)
  {
    if (!m_camera_service.Is_Open())
    {
      m_canvas->Clear_Bitmap();
      m_last_submitted_frame.reset();
      m_status_text->SetLabel(
        wxString::FromUTF8(u8"请先打开2D相机"));
    }
    else
    {
      m_status_text->SetLabel(
        wxString::FromUTF8(u8"等待2D图像…"));
    }
    return;
  }
  Submit_Frame(frame);
}

#include "camera_2d_image_view.h"

#include "camera_2d_service.h"

#include <wx/dcbuffer.h>
#include <wx/dcmemory.h>
#include <wx/image.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

Camera_2D_Bitmap_Canvas::Camera_2D_Bitmap_Canvas(wxWindow *parent)
  : wxPanel(parent, wxID_ANY)
{
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  Bind(wxEVT_PAINT, &Camera_2D_Bitmap_Canvas::On_Paint, this);
  Bind(wxEVT_SIZE, &Camera_2D_Bitmap_Canvas::On_Size, this);
  Bind(wxEVT_LEFT_DOWN, &Camera_2D_Bitmap_Canvas::On_Left_Down, this);
  Bind(wxEVT_LEFT_UP, &Camera_2D_Bitmap_Canvas::On_Left_Up, this);
  Bind(wxEVT_MOTION, &Camera_2D_Bitmap_Canvas::On_Mouse_Move, this);
  Bind(
    wxEVT_MOUSEWHEEL,
    &Camera_2D_Bitmap_Canvas::On_Mouse_Wheel,
    this);
}

void Camera_2D_Bitmap_Canvas::Set_Bitmap(wxBitmap bitmap)
{
  m_bitmap = std::move(bitmap);
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::Clear_Bitmap()
{
  m_bitmap = wxBitmap();
  m_detection.reset();
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::Set_Detection(
  std::optional<Camera_2D_Cross_Detection> detection)
{
  m_detection = std::move(detection);
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::Begin_Roi_Selection(
  std::function<void(Camera_2D_Roi)> callback)
{
  m_roi_callback = std::move(callback);
  m_selecting_roi = false;
  SetCursor(wxCursor(wxCURSOR_CROSS));
}

double Camera_2D_Bitmap_Canvas::Display_Scale() const
{
  if (!m_bitmap.IsOk()) return 1.0;
  if (!m_fit_to_window) return m_zoom;
  const wxSize area = GetClientSize();
  return std::max(
    0.01,
    std::min(
      static_cast<double>(area.x) / m_bitmap.GetWidth(),
      static_cast<double>(area.y) / m_bitmap.GetHeight()));
}

void Camera_2D_Bitmap_Canvas::Zoom_In()
{
  if (m_fit_to_window)
  {
    m_zoom = Display_Scale();
    m_fit_to_window = false;
  }
  m_zoom = std::min(8.0, m_zoom * 1.25);
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::Zoom_Out()
{
  if (m_fit_to_window)
  {
    m_zoom = Display_Scale();
    m_fit_to_window = false;
  }
  m_zoom = std::max(0.05, m_zoom / 1.25);
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::Fit_To_Window()
{
  m_fit_to_window = true;
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
  const double scale = Display_Scale();
  const int width = std::max(
    1, static_cast<int>(source_width * scale));
  const int height = std::max(
    1, static_cast<int>(source_height * scale));
  wxMemoryDC source_dc;
  source_dc.SelectObject(m_bitmap);
  dc.StretchBlit(
    (area.x - width) / 2,
    (area.y - height) / 2,
    width,
    height,
    &source_dc,
    0,
    0,
    source_width,
    source_height,
    wxCOPY,
    false);
  source_dc.SelectObject(wxNullBitmap);
  const int offset_x = (area.x - width) / 2;
  const int offset_y = (area.y - height) / 2;
  const auto canvas_x = [offset_x, scale](double x)
  {
    return offset_x + static_cast<int>(std::lround(x * scale));
  };
  const auto canvas_y = [offset_y, scale](double y)
  {
    return offset_y + static_cast<int>(std::lround(y * scale));
  };
  if (m_detection)
  {
    const auto &detection = *m_detection;
    dc.SetBrush(*wxTRANSPARENT_BRUSH);
    dc.SetPen(wxPen(
      detection.found ? wxColour(40, 220, 80) : wxColour(255, 80, 80),
      2));
    dc.DrawRectangle(
      canvas_x(detection.search_roi.x),
      canvas_y(detection.search_roi.y),
      std::max(1, static_cast<int>(
        detection.search_roi.width * scale)),
      std::max(1, static_cast<int>(
        detection.search_roi.height * scale)));
    if (detection.found)
    {
      dc.SetPen(wxPen(wxColour(40, 255, 100), 2));
      for (const auto &point : detection.outline)
      {
        dc.DrawPoint(canvas_x(point[0]), canvas_y(point[1]));
      }
      const int cx = canvas_x(detection.center_x);
      const int cy = canvas_y(detection.center_y);
      const double angle = detection.angle_deg * 3.14159265358979323846 / 180.0;
      const int axis = std::max(18, static_cast<int>(
        std::min(detection.search_roi.width,
                 detection.search_roi.height) * scale * 0.25));
      const int dx = static_cast<int>(std::lround(std::cos(angle) * axis));
      const int dy = static_cast<int>(std::lround(std::sin(angle) * axis));
      dc.DrawLine(cx - dx, cy - dy, cx + dx, cy + dy);
      dc.DrawLine(cx + dy, cy - dx, cx - dy, cy + dx);
      dc.DrawCircle(cx, cy, 4);
    }
  }
  if (m_selecting_roi)
  {
    dc.SetBrush(*wxTRANSPARENT_BRUSH);
    dc.SetPen(wxPen(wxColour(255, 210, 40), 2, wxPENSTYLE_SHORT_DASH));
    const int x0 = canvas_x(std::min(m_roi_start.x, m_roi_end.x));
    const int y0 = canvas_y(std::min(m_roi_start.y, m_roi_end.y));
    const int x1 = canvas_x(std::max(m_roi_start.x, m_roi_end.x));
    const int y1 = canvas_y(std::max(m_roi_start.y, m_roi_end.y));
    dc.DrawRectangle(x0, y0, x1 - x0, y1 - y0);
  }
}

void Camera_2D_Bitmap_Canvas::On_Size(wxSizeEvent &event)
{
  Refresh(false);
  event.Skip();
}

wxPoint Camera_2D_Bitmap_Canvas::Image_Point(
  const wxPoint &canvas_point) const
{
  if (!m_bitmap.IsOk()) return wxPoint(-1, -1);
  const wxSize area = GetClientSize();
  const double scale = Display_Scale();
  if (scale <= 0.0) return wxPoint(-1, -1);
  const double offset_x = (area.x - m_bitmap.GetWidth() * scale) * 0.5;
  const double offset_y = (area.y - m_bitmap.GetHeight() * scale) * 0.5;
  return wxPoint(
    std::clamp(
      static_cast<int>((canvas_point.x - offset_x) / scale),
      0, m_bitmap.GetWidth() - 1),
    std::clamp(
      static_cast<int>((canvas_point.y - offset_y) / scale),
      0, m_bitmap.GetHeight() - 1));
}

void Camera_2D_Bitmap_Canvas::On_Left_Down(wxMouseEvent &event)
{
  if (!m_roi_callback || !m_bitmap.IsOk()) return;
  m_roi_start = Image_Point(event.GetPosition());
  m_roi_end = m_roi_start;
  m_selecting_roi = m_roi_start.x >= 0;
  if (m_selecting_roi) CaptureMouse();
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::On_Mouse_Move(wxMouseEvent &event)
{
  if (!m_selecting_roi || !event.Dragging()) return;
  m_roi_end = Image_Point(event.GetPosition());
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::On_Mouse_Wheel(wxMouseEvent &event)
{
  if (event.GetWheelRotation() > 0)
    Zoom_In();
  else if (event.GetWheelRotation() < 0)
    Zoom_Out();
}

void Camera_2D_Bitmap_Canvas::On_Left_Up(wxMouseEvent &event)
{
  if (!m_selecting_roi) return;
  m_roi_end = Image_Point(event.GetPosition());
  m_selecting_roi = false;
  if (HasCapture()) ReleaseMouse();
  SetCursor(wxNullCursor);
  const int x0 = std::min(m_roi_start.x, m_roi_end.x);
  const int y0 = std::min(m_roi_start.y, m_roi_end.y);
  Camera_2D_Roi roi{
    x0,
    y0,
    std::abs(m_roi_end.x - m_roi_start.x) + 1,
    std::abs(m_roi_end.y - m_roi_start.y) + 1};
  auto callback = std::move(m_roi_callback);
  m_roi_callback = nullptr;
  Refresh(false);
  if (callback) callback(roi);
}

Camera_2D_Image_View::Camera_2D_Image_View(
  wxWindow *parent,
  Camera_2D_Service &camera_service,
  Camera_2D_Cross_Template_Service &template_service)
  : wxPanel(parent, wxID_ANY),
    m_camera_service(camera_service),
    m_template_service(template_service)
{
  m_canvas = new Camera_2D_Bitmap_Canvas(this);
  auto *fit = new wxButton(
    this, wxID_ANY, wxString::FromUTF8(u8"适应窗口"));
  fit->Bind(wxEVT_BUTTON, &Camera_2D_Image_View::On_Fit, this);
  auto *zoom_sizer = new wxBoxSizer(wxHORIZONTAL);
  zoom_sizer->Add(fit, 0);
  m_status_text = new wxStaticText(
    this, wxID_ANY, wxString::FromUTF8(u8"请先打开2D相机"));
  auto *sizer = new wxBoxSizer(wxVERTICAL);
  sizer->Add(zoom_sizer, 0, wxEXPAND | wxALL, 6);
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

void Camera_2D_Image_View::Begin_Template_Roi_Selection(
  std::function<void(Camera_2D_Roi)> callback)
{
  m_canvas->Begin_Roi_Selection(std::move(callback));
  m_status_text->SetLabel(
    wxString::FromUTF8(u8"请在图像中拖动框选完整的平面十字"));
}

void Camera_2D_Image_View::Show_Template_Detection(
  std::optional<Camera_2D_Cross_Detection> detection)
{
  m_canvas->Set_Detection(std::move(detection));
}

void Camera_2D_Image_View::On_Fit(wxCommandEvent &)
{
  m_canvas->Fit_To_Window();
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
    if (result.success)
    {
      result.detection = m_template_service.Detect(result.image);
    }
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
  m_canvas->Set_Detection(result->detection);
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
  if (result->detection && result->detection->found)
  {
    m_status_text->SetLabel(wxString::Format(
      wxString::FromUTF8(
        u8"%u × %u  Frame=%llu  %.1f FPS  "
        u8"十字中心（图片像素坐标系）X=%.2f px，Y=%.2f px"),
      result->image.width,
      result->image.height,
      result->image.frame_number,
      status.frames_per_second,
      result->detection->center_x,
      result->detection->center_y));
  }
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

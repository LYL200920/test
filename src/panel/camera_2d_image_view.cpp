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

namespace
{
constexpr int kRoi_Left = 1;
constexpr int kRoi_Right = 2;
constexpr int kRoi_Top = 4;
constexpr int kRoi_Bottom = 8;
constexpr int kRoi_Inside = 16;
constexpr int kRoi_Handle_Radius = 5;
constexpr int kMinimum_Roi_Size = 20;
}

Camera_2D_Bitmap_Canvas::Camera_2D_Bitmap_Canvas(wxWindow *parent)
  : wxPanel(parent, wxID_ANY)
{
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  SetBackgroundColour(wxColour(24, 24, 24));
  SetDoubleBuffered(true);
  Bind(wxEVT_PAINT, &Camera_2D_Bitmap_Canvas::On_Paint, this);
  Bind(wxEVT_SIZE, &Camera_2D_Bitmap_Canvas::On_Size, this);
  Bind(wxEVT_LEFT_DOWN, &Camera_2D_Bitmap_Canvas::On_Left_Down, this);
  Bind(wxEVT_LEFT_UP, &Camera_2D_Bitmap_Canvas::On_Left_Up, this);
  Bind(wxEVT_RIGHT_DOWN, &Camera_2D_Bitmap_Canvas::On_Right_Down, this);
  Bind(wxEVT_RIGHT_UP, &Camera_2D_Bitmap_Canvas::On_Right_Up, this);
  Bind(wxEVT_MOTION, &Camera_2D_Bitmap_Canvas::On_Mouse_Move, this);
  Bind(
    wxEVT_MOUSEWHEEL,
    &Camera_2D_Bitmap_Canvas::On_Mouse_Wheel,
    this);
}

void Camera_2D_Bitmap_Canvas::Set_Bitmap(wxBitmap bitmap)
{
  if (!m_bitmap.IsOk() ||
      m_bitmap.GetWidth() != bitmap.GetWidth() ||
      m_bitmap.GetHeight() != bitmap.GetHeight())
  {
    m_pan_offset = wxPoint();
  }
  m_bitmap = std::move(bitmap);
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::Set_Frame(
  wxBitmap bitmap,
  std::vector<Camera_2D_Cross_Detection> detections)
{
  if (!m_bitmap.IsOk() ||
      m_bitmap.GetWidth() != bitmap.GetWidth() ||
      m_bitmap.GetHeight() != bitmap.GetHeight())
  {
    m_pan_offset = wxPoint();
  }
  m_bitmap = std::move(bitmap);
  m_detections = std::move(detections);
  // The image and its overlays belong to the same camera frame. Updating them
  // together avoids two back-to-back paints in continuous acquisition mode.
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::Clear_Bitmap()
{
  if (!m_bitmap.IsOk() && m_detections.empty()) return;
  m_bitmap = wxBitmap();
  m_detections.clear();
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::Set_Detection(
  std::optional<Camera_2D_Cross_Detection> detection)
{
  m_detections.clear();
  if (detection) m_detections.push_back(std::move(*detection));
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::Set_Detections(
  std::vector<Camera_2D_Cross_Detection> detections)
{
  m_detections = std::move(detections);
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::Begin_Roi_Selection(
  std::function<void(Camera_2D_Roi)> callback)
{
  m_roi_callback = std::move(callback);
  m_selecting_roi = false;
  m_has_editable_roi = false;
  m_roi_drag_mode = Roi_Drag_Mode::None;
  SetCursor(wxCursor(wxCURSOR_CROSS));
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::Begin_Roi_Editing(
  const Camera_2D_Roi &roi,
  std::function<void(Camera_2D_Roi)> callback)
{
  Begin_Roi_Selection(std::move(callback));
  if (!m_bitmap.IsOk()) return;
  m_editable_roi.x = std::clamp(
    roi.x, 0, std::max(0, m_bitmap.GetWidth() - 1));
  m_editable_roi.y = std::clamp(
    roi.y, 0, std::max(0, m_bitmap.GetHeight() - 1));
  m_editable_roi.width = std::clamp(
    roi.width, 1, m_bitmap.GetWidth() - m_editable_roi.x);
  m_editable_roi.height = std::clamp(
    roi.height, 1, m_bitmap.GetHeight() - m_editable_roi.y);
  m_has_editable_roi = true;
  Refresh(false);
}

bool Camera_2D_Bitmap_Canvas::Has_Editable_Roi() const
{
  return m_roi_callback && m_has_editable_roi &&
    m_editable_roi.width >= kMinimum_Roi_Size &&
    m_editable_roi.height >= kMinimum_Roi_Size;
}

void Camera_2D_Bitmap_Canvas::Confirm_Roi_Selection()
{
  if (!Has_Editable_Roi()) return;
  const Camera_2D_Roi roi = m_editable_roi;
  auto callback = std::move(m_roi_callback);
  m_roi_callback = nullptr;
  m_has_editable_roi = false;
  m_selecting_roi = false;
  m_roi_drag_mode = Roi_Drag_Mode::None;
  SetCursor(wxNullCursor);
  Refresh(false);
  if (callback) callback(roi);
}

void Camera_2D_Bitmap_Canvas::Cancel_Roi_Selection()
{
  m_roi_callback = nullptr;
  m_has_editable_roi = false;
  m_selecting_roi = false;
  m_roi_drag_mode = Roi_Drag_Mode::None;
  if (HasCapture()) ReleaseMouse();
  SetCursor(wxNullCursor);
  Refresh(false);
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
  m_pan_offset = wxPoint();
  Refresh(false);
}

wxPoint Camera_2D_Bitmap_Canvas::Display_Offset(double scale) const
{
  const wxSize area = GetClientSize();
  return wxPoint(
    static_cast<int>(std::lround(
      (area.x - m_bitmap.GetWidth() * scale) * 0.5)) +
      m_pan_offset.x,
    static_cast<int>(std::lround(
      (area.y - m_bitmap.GetHeight() * scale) * 0.5)) +
      m_pan_offset.y);
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
  const wxPoint offset = Display_Offset(scale);
  wxMemoryDC source_dc;
  source_dc.SelectObject(m_bitmap);
  dc.StretchBlit(
    offset.x,
    offset.y,
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
  const int offset_x = offset.x;
  const int offset_y = offset.y;
  const auto canvas_x = [offset_x, scale](double x)
  {
    return offset_x + static_cast<int>(std::lround(x * scale));
  };
  const auto canvas_y = [offset_y, scale](double y)
  {
    return offset_y + static_cast<int>(std::lround(y * scale));
  };
  {
    for (const auto &detection : m_detections)
    {
      dc.SetBrush(*wxTRANSPARENT_BRUSH);
      dc.SetPen(wxPen(
        detection.found ? wxColour(40, 220, 80) : wxColour(255, 80, 80),
        2));
      const int box_x = canvas_x(detection.search_roi.x);
      const int box_y = canvas_y(detection.search_roi.y);
      dc.DrawRectangle(
        box_x,
        box_y,
        std::max(1, static_cast<int>(
          detection.search_roi.width * scale)),
        std::max(1, static_cast<int>(
          detection.search_roi.height * scale)));
      dc.DrawText(
        wxString::FromUTF8(detection.template_name.c_str()),
        box_x + 3,
        box_y + 3);
      if (detection.found)
      {
        dc.SetPen(wxPen(wxColour(40, 255, 100), 2));
        for (const auto &point : detection.outline)
        {
          dc.DrawPoint(canvas_x(point[0]), canvas_y(point[1]));
        }
        const int cx = canvas_x(detection.center_x);
        const int cy = canvas_y(detection.center_y);
        const double angle =
          detection.angle_deg * 3.14159265358979323846 / 180.0;
        const int axis = std::max(18, static_cast<int>(
          std::min(detection.search_roi.width,
                   detection.search_roi.height) * scale * 0.25));
        const int dx =
          static_cast<int>(std::lround(std::cos(angle) * axis));
        const int dy =
          static_cast<int>(std::lround(std::sin(angle) * axis));
        dc.DrawLine(cx - dx, cy - dy, cx + dx, cy + dy);
        dc.DrawLine(cx + dy, cy - dx, cx - dy, cy + dx);
        dc.DrawCircle(cx, cy, 4);
      }
    }
  }
  if (m_roi_callback && m_has_editable_roi)
  {
    dc.SetBrush(*wxTRANSPARENT_BRUSH);
    dc.SetPen(wxPen(wxColour(255, 55, 55), 2, wxPENSTYLE_SHORT_DASH));
    const int x0 = canvas_x(m_editable_roi.x);
    const int y0 = canvas_y(m_editable_roi.y);
    const int x1 = canvas_x(
      m_editable_roi.x + m_editable_roi.width - 1);
    const int y1 = canvas_y(
      m_editable_roi.y + m_editable_roi.height - 1);
    dc.DrawRectangle(x0, y0, x1 - x0, y1 - y0);
    dc.SetPen(wxPen(wxColour(210, 20, 20), 1));
    dc.SetBrush(wxBrush(wxColour(255, 95, 95)));
    const int center_x = (x0 + x1) / 2;
    const int center_y = (y0 + y1) / 2;
    const std::array<wxPoint, 8> handles{{
      {x0, y0}, {center_x, y0}, {x1, y0},
      {x0, center_y}, {x1, center_y},
      {x0, y1}, {center_x, y1}, {x1, y1}}};
    for (const auto &handle : handles)
    {
      dc.DrawRectangle(
        handle.x - kRoi_Handle_Radius,
        handle.y - kRoi_Handle_Radius,
        kRoi_Handle_Radius * 2 + 1,
        kRoi_Handle_Radius * 2 + 1);
    }
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
  const double scale = Display_Scale();
  if (scale <= 0.0) return wxPoint(-1, -1);
  const wxPoint offset = Display_Offset(scale);
  return wxPoint(
    std::clamp(
      static_cast<int>((canvas_point.x - offset.x) / scale),
      0, m_bitmap.GetWidth() - 1),
    std::clamp(
      static_cast<int>((canvas_point.y - offset.y) / scale),
      0, m_bitmap.GetHeight() - 1));
}

int Camera_2D_Bitmap_Canvas::Hit_Test_Roi(
  const wxPoint &canvas_point) const
{
  if (!m_roi_callback || !m_has_editable_roi || !m_bitmap.IsOk())
    return 0;
  const double scale = Display_Scale();
  const wxPoint offset = Display_Offset(scale);
  const int x0 = offset.x + static_cast<int>(
    std::lround(m_editable_roi.x * scale));
  const int y0 = offset.y + static_cast<int>(
    std::lround(m_editable_roi.y * scale));
  const int x1 = offset.x + static_cast<int>(std::lround(
    (m_editable_roi.x + m_editable_roi.width - 1) * scale));
  const int y1 = offset.y + static_cast<int>(std::lround(
    (m_editable_roi.y + m_editable_roi.height - 1) * scale));
  constexpr int tolerance = kRoi_Handle_Radius + 3;
  if (canvas_point.x < x0 - tolerance ||
      canvas_point.x > x1 + tolerance ||
      canvas_point.y < y0 - tolerance ||
      canvas_point.y > y1 + tolerance)
    return 0;
  int result = 0;
  if (std::abs(canvas_point.x - x0) <= tolerance) result |= kRoi_Left;
  if (std::abs(canvas_point.x - x1) <= tolerance) result |= kRoi_Right;
  if (std::abs(canvas_point.y - y0) <= tolerance) result |= kRoi_Top;
  if (std::abs(canvas_point.y - y1) <= tolerance) result |= kRoi_Bottom;
  if (result == 0 &&
      canvas_point.x > x0 && canvas_point.x < x1 &&
      canvas_point.y > y0 && canvas_point.y < y1)
    result = kRoi_Inside;
  return result;
}

void Camera_2D_Bitmap_Canvas::Update_Roi_Cursor(
  const wxPoint &canvas_point)
{
  if (!m_roi_callback)
  {
    SetCursor(wxNullCursor);
    return;
  }
  const int hit = Hit_Test_Roi(canvas_point);
  if ((hit & (kRoi_Left | kRoi_Top)) ==
        (kRoi_Left | kRoi_Top) ||
      (hit & (kRoi_Right | kRoi_Bottom)) ==
        (kRoi_Right | kRoi_Bottom))
    SetCursor(wxCursor(wxCURSOR_SIZENWSE));
  else if ((hit & (kRoi_Right | kRoi_Top)) ==
             (kRoi_Right | kRoi_Top) ||
           (hit & (kRoi_Left | kRoi_Bottom)) ==
             (kRoi_Left | kRoi_Bottom))
    SetCursor(wxCursor(wxCURSOR_SIZENESW));
  else if (hit & (kRoi_Left | kRoi_Right))
    SetCursor(wxCursor(wxCURSOR_SIZEWE));
  else if (hit & (kRoi_Top | kRoi_Bottom))
    SetCursor(wxCursor(wxCURSOR_SIZENS));
  else if (hit == kRoi_Inside)
    SetCursor(wxCursor(wxCURSOR_HAND));
  else
    SetCursor(wxCursor(wxCURSOR_CROSS));
}

void Camera_2D_Bitmap_Canvas::Update_Drawn_Roi(
  const wxPoint &image_point)
{
  m_roi_end = image_point;
  m_editable_roi = {
    std::min(m_roi_start.x, m_roi_end.x),
    std::min(m_roi_start.y, m_roi_end.y),
    std::abs(m_roi_end.x - m_roi_start.x) + 1,
    std::abs(m_roi_end.y - m_roi_start.y) + 1};
  m_has_editable_roi = true;
}

void Camera_2D_Bitmap_Canvas::On_Left_Down(wxMouseEvent &event)
{
  if (!m_roi_callback || !m_bitmap.IsOk()) return;
  const int hit = Hit_Test_Roi(event.GetPosition());
  m_roi_drag_start = Image_Point(event.GetPosition());
  m_roi_before_drag = m_editable_roi;
  if (hit & (kRoi_Left | kRoi_Right | kRoi_Top | kRoi_Bottom))
  {
    m_roi_drag_mode = Roi_Drag_Mode::Resize;
    m_roi_resize_edges = hit;
  }
  else if (hit == kRoi_Inside)
  {
    m_roi_drag_mode = Roi_Drag_Mode::Move;
  }
  else
  {
    m_roi_drag_mode = Roi_Drag_Mode::Draw;
    m_roi_start = m_roi_drag_start;
    m_roi_end = m_roi_start;
    Update_Drawn_Roi(m_roi_start);
  }
  m_selecting_roi = m_roi_drag_start.x >= 0;
  if (m_selecting_roi) CaptureMouse();
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::On_Mouse_Move(wxMouseEvent &event)
{
  if (m_panning && event.RightIsDown())
  {
    const wxPoint delta = event.GetPosition() - m_pan_start;
    m_pan_offset = m_pan_origin + delta;
    Refresh(false);
    return;
  }
  if (m_selecting_roi && event.LeftIsDown())
  {
    const wxPoint point = Image_Point(event.GetPosition());
    if (m_roi_drag_mode == Roi_Drag_Mode::Draw)
    {
      Update_Drawn_Roi(point);
    }
    else if (m_roi_drag_mode == Roi_Drag_Mode::Move)
    {
      const wxPoint delta = point - m_roi_drag_start;
      m_editable_roi.x = std::clamp(
        m_roi_before_drag.x + delta.x,
        0,
        m_bitmap.GetWidth() - m_roi_before_drag.width);
      m_editable_roi.y = std::clamp(
        m_roi_before_drag.y + delta.y,
        0,
        m_bitmap.GetHeight() - m_roi_before_drag.height);
    }
    else if (m_roi_drag_mode == Roi_Drag_Mode::Resize)
    {
      int left = m_roi_before_drag.x;
      int top = m_roi_before_drag.y;
      int right = left + m_roi_before_drag.width - 1;
      int bottom = top + m_roi_before_drag.height - 1;
      if (m_roi_resize_edges & kRoi_Left)
        left = std::min(point.x, right - kMinimum_Roi_Size + 1);
      if (m_roi_resize_edges & kRoi_Right)
        right = std::max(point.x, left + kMinimum_Roi_Size - 1);
      if (m_roi_resize_edges & kRoi_Top)
        top = std::min(point.y, bottom - kMinimum_Roi_Size + 1);
      if (m_roi_resize_edges & kRoi_Bottom)
        bottom = std::max(point.y, top + kMinimum_Roi_Size - 1);
      left = std::clamp(left, 0, m_bitmap.GetWidth() - 1);
      right = std::clamp(right, 0, m_bitmap.GetWidth() - 1);
      top = std::clamp(top, 0, m_bitmap.GetHeight() - 1);
      bottom = std::clamp(bottom, 0, m_bitmap.GetHeight() - 1);
      m_editable_roi = {
        left, top, right - left + 1, bottom - top + 1};
    }
    Refresh(false);
    return;
  }
  if (!m_selecting_roi && !m_panning)
    Update_Roi_Cursor(event.GetPosition());
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
  const wxPoint point = Image_Point(event.GetPosition());
  if (m_roi_drag_mode == Roi_Drag_Mode::Draw)
    Update_Drawn_Roi(point);
  m_selecting_roi = false;
  m_roi_drag_mode = Roi_Drag_Mode::None;
  if (HasCapture()) ReleaseMouse();
  Update_Roi_Cursor(event.GetPosition());
  Refresh(false);
}

void Camera_2D_Bitmap_Canvas::On_Right_Down(wxMouseEvent &event)
{
  if (!m_bitmap.IsOk() || m_selecting_roi) return;
  m_panning = true;
  m_pan_start = event.GetPosition();
  m_pan_origin = m_pan_offset;
  if (!HasCapture()) CaptureMouse();
  SetCursor(wxCursor(wxCURSOR_HAND));
}

void Camera_2D_Bitmap_Canvas::On_Right_Up(wxMouseEvent &event)
{
  if (!m_panning) return;
  m_panning = false;
  if (HasCapture()) ReleaseMouse();
  Update_Roi_Cursor(event.GetPosition());
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
    this, wxID_ANY, wxString::FromUTF8(u8"请先打开2D相机"),
    wxDefaultPosition, wxDefaultSize, wxST_NO_AUTORESIZE);
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
    wxString::FromUTF8(
      u8"绘制ROI后可拖动内部移动，拖动边框或控制点调整大小"));
}

void Camera_2D_Image_View::Begin_Template_Roi_Editing(
  const Camera_2D_Roi &roi,
  std::function<void(Camera_2D_Roi)> callback)
{
  m_canvas->Begin_Roi_Editing(roi, std::move(callback));
  m_status_text->SetLabel(
    wxString::FromUTF8(
      u8"正在编辑模板ROI：拖动内部移动，拖动红色控制点调整大小"));
}

bool Camera_2D_Image_View::Has_Editable_Template_Roi() const
{
  return m_canvas->Has_Editable_Roi();
}

void Camera_2D_Image_View::Confirm_Template_Roi_Selection()
{
  m_canvas->Confirm_Roi_Selection();
}

void Camera_2D_Image_View::Cancel_Template_Roi_Selection()
{
  m_canvas->Cancel_Roi_Selection();
  m_status_text->SetLabel(wxString::FromUTF8(u8"已取消ROI编辑"));
}

void Camera_2D_Image_View::Show_Template_Detection(
  std::optional<Camera_2D_Cross_Detection> detection)
{
  m_canvas->Set_Detection(std::move(detection));
}

void Camera_2D_Image_View::Set_Matched_Template_Ids(
  std::vector<std::string> template_ids)
{
  const bool empty = template_ids.empty();
  {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    m_matching_template_ids = std::move(template_ids);
  }
  if (empty)
    m_canvas->Set_Detections({});
}

void Camera_2D_Image_View::Show_Static_Image(
  const Camera_2D_Display_Image &image_data,
  const std::vector<std::string> &template_ids,
  const std::string &source_name)
{
  m_static_image_active = true;
  {
    std::lock_guard<std::mutex> lock(m_worker_mutex);
    m_pending_frame.reset();
    m_pending_result.reset();
  }
  std::vector<Camera_2D_Cross_Detection> detections;
  {
    std::lock_guard<std::mutex> lock(m_detection_mutex);
    detections = m_template_service.Detect(image_data, template_ids);
  }
  wxImage image(
    static_cast<int>(image_data.width),
    static_cast<int>(image_data.height));
  if (!image.IsOk() || !image.GetData()) return;
  std::memcpy(
    image.GetData(), image_data.rgb.data(), image_data.rgb.size());
  m_canvas->Set_Frame(wxBitmap(image), std::move(detections));
  m_status_text->SetLabel(wxString::Format(
    wxString::FromUTF8(u8"读取图片：%s  %u × %u"),
    wxString::FromUTF8(source_name.c_str()),
    image_data.width,
    image_data.height));
}

void Camera_2D_Image_View::Resume_Live_Image()
{
  m_static_image_active = false;
  m_last_submitted_frame.reset();
  m_status_text->SetLabel(wxString::FromUTF8(u8"已恢复实时2D图像"));
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
    std::vector<std::string> template_ids;
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
      template_ids = m_matching_template_ids;
      job_id = m_next_job_id++;
    }

    Conversion_Result result;
    result.job_id = job_id;
    result.success = Convert_Camera_2D_Frame(
      *frame, &result.image, &result.error);
    if (result.success)
    {
      std::lock_guard<std::mutex> lock(m_detection_mutex);
      result.detections =
        m_template_service.Detect(result.image, template_ids);
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
    // A transient bad frame must not blank the last valid image. Otherwise a
    // continuous stream visibly alternates between the image and background.
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
  const std::size_t expected_rgb_size =
    static_cast<std::size_t>(result->image.width) *
    result->image.height * 3;
  if (result->image.rgb.size() != expected_rgb_size)
  {
    m_status_text->SetLabel(
      wxString::FromUTF8(u8"2D显示图像缓冲区无效"));
    return;
  }
  std::memcpy(
    image.GetData(),
    result->image.rgb.data(),
    result->image.rgb.size());
  m_canvas->Set_Frame(wxBitmap(image), result->detections);
  const auto status = m_camera_service.Status();
  wxString status_label = wxString::Format(
    wxString::FromUTF8(
      u8"%u × %u  Frame=%llu  %.1f FPS  %s"),
    result->image.width,
    result->image.height,
    result->image.frame_number,
    status.frames_per_second,
    wxString::FromUTF8(
      jutze_camera::pixel_format_name(status.pixel_format)));
  const auto found_detection = std::find_if(
    result->detections.begin(),
    result->detections.end(),
    [](const auto &item) { return item.found; });
  if (found_detection != result->detections.end())
  {
    status_label = wxString::Format(
      wxString::FromUTF8(
        u8"%u × %u  Frame=%llu  %.1f FPS  "
        u8"十字中心（图片像素坐标系）X=%.2f px，Y=%.2f px"),
      result->image.width,
      result->image.height,
      result->image.frame_number,
      status.frames_per_second,
      found_detection->center_x,
      found_detection->center_y);
  }
  m_status_text->SetLabel(status_label);
}

void Camera_2D_Image_View::On_Timer(wxTimerEvent &)
{
  if (m_static_image_active) return;
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

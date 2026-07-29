#ifndef includeguard_camera_2d_image_view_h_includeguard
#define includeguard_camera_2d_image_view_h_includeguard

#include "camera_2d_image_converter.h"
#include "camera_2d_cross_template.h"

#include <wx/bitmap.h>
#include <wx/panel.h>
#include <wx/timer.h>

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace jutze_camera
{
struct camera_frame;
}

class Camera_2D_Service;
class wxStaticText;

class Camera_2D_Bitmap_Canvas : public wxPanel
{
public:
  explicit Camera_2D_Bitmap_Canvas(wxWindow *parent);
  void Set_Bitmap(wxBitmap bitmap);
  void Clear_Bitmap();
  void Set_Detection(
    std::optional<Camera_2D_Cross_Detection> detection);
  void Set_Detection_Visible(bool visible);
  void Begin_Roi_Selection(
    std::function<void(Camera_2D_Roi)> callback);
  void Begin_Roi_Editing(
    const Camera_2D_Roi &roi,
    std::function<void(Camera_2D_Roi)> callback);
  bool Has_Editable_Roi() const;
  void Confirm_Roi_Selection();
  void Cancel_Roi_Selection();
  void Zoom_In();
  void Zoom_Out();
  void Fit_To_Window();

private:
  void On_Paint(wxPaintEvent &event);
  void On_Size(wxSizeEvent &event);
  void On_Left_Down(wxMouseEvent &event);
  void On_Left_Up(wxMouseEvent &event);
  void On_Right_Down(wxMouseEvent &event);
  void On_Right_Up(wxMouseEvent &event);
  void On_Mouse_Move(wxMouseEvent &event);
  void On_Mouse_Wheel(wxMouseEvent &event);
  wxPoint Image_Point(const wxPoint &canvas_point) const;
  wxPoint Display_Offset(double scale) const;
  int Hit_Test_Roi(const wxPoint &canvas_point) const;
  void Update_Roi_Cursor(const wxPoint &canvas_point);
  void Update_Drawn_Roi(const wxPoint &image_point);
  double Display_Scale() const;
  enum class Roi_Drag_Mode
  {
    None,
    Draw,
    Move,
    Resize
  };
  wxBitmap m_bitmap;
  std::optional<Camera_2D_Cross_Detection> m_detection;
  bool m_detection_visible = true;
  std::function<void(Camera_2D_Roi)> m_roi_callback;
  bool m_selecting_roi = false;
  bool m_has_editable_roi = false;
  Camera_2D_Roi m_editable_roi;
  Camera_2D_Roi m_roi_before_drag;
  Roi_Drag_Mode m_roi_drag_mode = Roi_Drag_Mode::None;
  int m_roi_resize_edges = 0;
  wxPoint m_roi_drag_start;
  wxPoint m_roi_start;
  wxPoint m_roi_end;
  bool m_panning = false;
  wxPoint m_pan_start;
  wxPoint m_pan_origin;
  wxPoint m_pan_offset;
  bool m_fit_to_window = true;
  double m_zoom = 1.0;
};

class Camera_2D_Image_View : public wxPanel
{
public:
  Camera_2D_Image_View(
    wxWindow *parent,
    Camera_2D_Service &camera_service,
    Camera_2D_Cross_Template_Service &template_service);
  ~Camera_2D_Image_View() override;
  void Begin_Template_Roi_Selection(
    std::function<void(Camera_2D_Roi)> callback);
  void Begin_Template_Roi_Editing(
    const Camera_2D_Roi &roi,
    std::function<void(Camera_2D_Roi)> callback);
  bool Has_Editable_Template_Roi() const;
  void Confirm_Template_Roi_Selection();
  void Cancel_Template_Roi_Selection();
  void Show_Template_Detection(
    std::optional<Camera_2D_Cross_Detection> detection);
  void Set_Template_Detection_Visible(bool visible);

private:
  struct Conversion_Result
  {
    unsigned long long job_id = 0;
    bool success = false;
    Camera_2D_Display_Image image;
    std::optional<Camera_2D_Cross_Detection> detection;
    std::string error;
  };

  void Submit_Frame(
    std::shared_ptr<const jutze_camera::camera_frame> frame);
  void Worker_Loop();
  void Consume_Result();
  void On_Timer(wxTimerEvent &event);
  void On_Fit(wxCommandEvent &event);

  Camera_2D_Service &m_camera_service;
  Camera_2D_Cross_Template_Service &m_template_service;
  Camera_2D_Bitmap_Canvas *m_canvas = nullptr;
  wxStaticText *m_status_text = nullptr;
  wxTimer m_timer;
  std::thread m_worker;
  std::mutex m_worker_mutex;
  std::condition_variable m_worker_ready;
  bool m_exit_requested = false;
  unsigned long long m_next_job_id = 1;
  unsigned long long m_last_displayed_job_id = 0;
  std::shared_ptr<const jutze_camera::camera_frame> m_pending_frame;
  std::shared_ptr<const jutze_camera::camera_frame> m_last_submitted_frame;
  std::optional<Conversion_Result> m_pending_result;
};

#endif

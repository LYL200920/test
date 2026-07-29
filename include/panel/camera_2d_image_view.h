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
  void Begin_Roi_Selection(
    std::function<void(Camera_2D_Roi)> callback);

private:
  void On_Paint(wxPaintEvent &event);
  void On_Size(wxSizeEvent &event);
  void On_Left_Down(wxMouseEvent &event);
  void On_Left_Up(wxMouseEvent &event);
  void On_Mouse_Move(wxMouseEvent &event);
  wxPoint Image_Point(const wxPoint &canvas_point) const;
  wxBitmap m_bitmap;
  std::optional<Camera_2D_Cross_Detection> m_detection;
  std::function<void(Camera_2D_Roi)> m_roi_callback;
  bool m_selecting_roi = false;
  wxPoint m_roi_start;
  wxPoint m_roi_end;
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

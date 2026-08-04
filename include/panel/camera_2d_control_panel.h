#ifndef includeguard_camera_2d_control_panel_h_includeguard
#define includeguard_camera_2d_control_panel_h_includeguard

#include "camera_2d_service.h"

#include <wx/scrolwin.h>
#include <wx/timer.h>

#include <functional>
#include <string>
#include <vector>

class Camera_2D_Service;
class wxButton;
class wxChoice;
class wxSpinCtrl;
class wxSpinCtrlDouble;
class wxStaticText;

class Camera_2D_Control_Panel : public wxScrolledWindow
{
public:
  Camera_2D_Control_Panel(
    wxWindow *parent,
    Camera_2D_Service &camera_service);

  void Set_On_Show_Image(std::function<void()> callback);
  void Set_On_Calibrate(std::function<void()> callback);
  void Set_On_Availability_Changed(
    std::function<void(bool)> callback);

private:
  void Refresh_Device_List();
  void Refresh_View();
  void Sync_Configuration_Editors();
  void Show_Error(const std::string &message);
  void On_Refresh(wxCommandEvent &event);
  void On_Open(wxCommandEvent &event);
  void On_Close(wxCommandEvent &event);
  void On_Start(wxCommandEvent &event);
  void On_Stop(wxCommandEvent &event);
  void On_Software_Trigger(wxCommandEvent &event);
  void On_Apply(wxCommandEvent &event);
  void On_Reload(wxCommandEvent &event);
  void On_Save(wxCommandEvent &event);
  void On_Show_Image(wxCommandEvent &event);
  void On_Calibrate(wxCommandEvent &event);
  void On_Timer(wxTimerEvent &event);

  Camera_2D_Service &m_camera_service;
  std::vector<std::string> m_device_keys;
  std::function<void()> m_on_show_image;
  std::function<void()> m_on_calibrate;
  wxChoice *m_device_choice = nullptr;
  wxButton *m_refresh_button = nullptr;
  wxButton *m_open_button = nullptr;
  wxButton *m_close_button = nullptr;
  wxStaticText *m_state_text = nullptr;
  wxStaticText *m_device_info_text = nullptr;
  wxSpinCtrl *m_width_spin = nullptr;
  wxSpinCtrl *m_height_spin = nullptr;
  wxChoice *m_trigger_choice = nullptr;
  wxSpinCtrlDouble *m_exposure_spin = nullptr;
  wxSpinCtrlDouble *m_gain_spin = nullptr;
  wxSpinCtrlDouble *m_frame_rate_spin = nullptr;
  wxChoice *m_pixel_format_choice = nullptr;
  wxButton *m_apply_button = nullptr;
  wxButton *m_reload_button = nullptr;
  wxButton *m_start_button = nullptr;
  wxButton *m_stop_button = nullptr;
  wxButton *m_trigger_button = nullptr;
  wxButton *m_save_button = nullptr;
  wxButton *m_show_button = nullptr;
  wxButton *m_calibrate_button = nullptr;
  wxStaticText *m_frame_info_text = nullptr;
  wxTimer m_timer;
  Camera_2D_Parameters m_parameters;
  std::function<void(bool)> m_on_availability_changed;
  bool m_last_available = false;
};

#endif

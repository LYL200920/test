#ifndef includeguard_camera_2d_template_panel_h_includeguard
#define includeguard_camera_2d_template_panel_h_includeguard

#include "camera_2d_cross_template.h"

#include <wx/scrolwin.h>
#include <wx/timer.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

class Camera_2D_Image_View;
class Camera_2D_Service;
class wxButton;
class wxChoice;
class wxStaticText;
class wxTextCtrl;
namespace jutze_camera
{
struct camera_frame;
}

class Camera_2D_Template_Panel : public wxScrolledWindow
{
public:
  Camera_2D_Template_Panel(
    wxWindow *parent,
    Camera_2D_Service &camera_service,
    Camera_2D_Cross_Template_Service &template_service,
    Camera_2D_Image_View &image_view);

  void Set_On_Show_Image(std::function<void()> callback);

private:
  void Refresh_Template_List();
  void Refresh_View();
  void Show_Error(const std::string &message);
  void On_Template_Selected(wxCommandEvent &event);
  void On_Create(wxCommandEvent &event);
  void On_Roi_Selected(
    const std::string &name,
    const Camera_2D_Roi &roi);
  void On_Timer(wxTimerEvent &event);

  Camera_2D_Service &m_camera_service;
  Camera_2D_Cross_Template_Service &m_template_service;
  Camera_2D_Image_View &m_image_view;
  std::function<void()> m_on_show_image;
  wxChoice *m_type_choice = nullptr;
  wxTextCtrl *m_name_text = nullptr;
  wxButton *m_create_button = nullptr;
  wxChoice *m_template_choice = nullptr;
  wxStaticText *m_result_text = nullptr;
  wxStaticText *m_instruction_text = nullptr;
  std::vector<std::string> m_template_ids;
  std::shared_ptr<const jutze_camera::camera_frame> m_creation_frame;
  wxTimer m_timer;
};

#endif

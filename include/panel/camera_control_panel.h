#ifndef includeguard_camera_control_panel_h_includeguard
#define includeguard_camera_control_panel_h_includeguard

#include <wx/panel.h>
#include <wx/timer.h>

#include "camera_parameter.h"

#include <functional>
#include <vector>

class Camera_Acquisition_Panel;
class Camera_Info_Panel;
class Camera_Parameter_Panel;
class Camera_Service;
class wxSimplebook;
class wxStaticText;
class wxToggleButton;

enum class Camera_Function_Page
{
  Info = 0,
  Acquisition = 1,
  Configuration = 2
};

class Camera_Control_Panel : public wxPanel
{
public:
  Camera_Control_Panel (wxWindow* parent, Camera_Service& camera_service);
  ~Camera_Control_Panel ( ) override;

  void Set_On_Availability_Changed (std::function<void (bool)> callback);

private:
  void Select_Page (Camera_Function_Page page);
  void Update_Menu_State ( );
  void Update_Controls ( );
  void Request_Device_Info ( );
  void Request_Stream_Configuration ( );
  void Request_Load_Parameters ( );
  void Request_Apply_Parameters (
    std::vector<Camera_Parameter_Update> updates);
  void Request_Start ( );
  void Request_Stop ( );
  void Request_Close ( );
  void On_Info_Menu (wxCommandEvent& event);
  void On_Acquisition_Menu (wxCommandEvent& event);
  void On_Configuration_Menu (wxCommandEvent& event);
  void On_Camera_Updated (wxThreadEvent& event);
  void On_Frame_Timer (wxTimerEvent& event);

private:
  Camera_Service& m_camera_service;
  std::function<void (bool)> m_on_availability_changed;
  Camera_Function_Page m_active_page = Camera_Function_Page::Info;
  wxToggleButton* m_info_menu_button = nullptr;
  wxToggleButton* m_acquisition_menu_button = nullptr;
  wxToggleButton* m_configuration_menu_button = nullptr;
  wxSimplebook* m_page_book = nullptr;
  Camera_Info_Panel* m_info_panel = nullptr;
  Camera_Acquisition_Panel* m_acquisition_panel = nullptr;
  Camera_Parameter_Panel* m_parameter_panel = nullptr;
  wxStaticText* m_status_text = nullptr;
  wxTimer m_frame_timer;
};

#endif

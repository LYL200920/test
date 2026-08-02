#ifndef includeguard_robot_model_panel_h_includeguard
#define includeguard_robot_model_panel_h_includeguard

#include <wx/panel.h>

#include <memory>

class Camera_Service;
class Camera_2D_Service;
class Camera_2D_Cross_Template_Service;
class Robot_Model_Panel_Controller;

namespace application
{
class Robot_Connection_Controller;
}

// Thin wx host. The controller owns interaction state and workflow behavior;
// this panel only provides layout and forwards its public entry points.
class Robot_Model_Panel final : public wxPanel
{
public:
  Robot_Model_Panel(
    wxWindow *parent,
    Camera_Service &camera_service,
    Camera_2D_Service &camera_2d_service,
    Camera_2D_Cross_Template_Service &camera_2d_template_service,
    std::shared_ptr<application::Robot_Connection_Controller>
      robot_connection,
    wxWindowID id = wxID_ANY);
  ~Robot_Model_Panel() override;

  void Show_Model_Configuration(wxWindow *parent = nullptr);
  void Refresh_Template_Configuration();

private:
  Robot_Model_Panel_Controller *m_controller = nullptr;
};

#endif

#ifndef includeguard_robot_model_panel_h_includeguard
#define includeguard_robot_model_panel_h_includeguard

#include "robot_model_panel_controller.h"

#include <memory>

// Keep the public panel as the native host of Robot_Model_View.  An additional
// wxPanel layer around wxGLCanvas is not reliably composited by every Windows
// OpenGL driver.
class Robot_Model_Panel final : public Robot_Model_Panel_Controller
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
  ~Robot_Model_Panel() override = default;
};

#endif

#ifndef includeguard_robot_model_panel_h_includeguard
#define includeguard_robot_model_panel_h_includeguard

#include "robot_model_panel_controller.h"

#include <memory>

// Inheritance keeps Robot_Model_Panel as the public wrapper without creating
// another native wxPanel around the wxGLCanvas.
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

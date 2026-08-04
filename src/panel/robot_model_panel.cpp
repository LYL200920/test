#include "robot_model_panel.h"

#include <utility>

Robot_Model_Panel::Robot_Model_Panel(
  wxWindow *parent,
  Camera_Service &camera_service,
  Camera_2D_Service &camera_2d_service,
  Camera_2D_Cross_Template_Service &camera_2d_template_service,
  std::shared_ptr<application::Robot_Connection_Controller>
    robot_connection,
  wxWindowID id)
  : Robot_Model_Panel_Controller(
      parent,
      camera_service,
      camera_2d_service,
      camera_2d_template_service,
      std::move(robot_connection),
      id)
{
  CallAfter(&Robot_Model_Panel::Initialize_After_Layout);
}

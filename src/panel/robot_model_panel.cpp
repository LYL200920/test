#include "robot_model_panel.h"

#include "robot_model_panel_controller.h"

#include <wx/sizer.h>

#include <utility>

Robot_Model_Panel::Robot_Model_Panel(
  wxWindow *parent,
  Camera_Service &camera_service,
  Camera_2D_Service &camera_2d_service,
  Camera_2D_Cross_Template_Service &camera_2d_template_service,
  std::shared_ptr<application::Robot_Connection_Controller>
    robot_connection,
  wxWindowID id)
  : wxPanel(parent, id)
{
  m_controller = new Robot_Model_Panel_Controller(
    this,
    camera_service,
    camera_2d_service,
    camera_2d_template_service,
    std::move(robot_connection));

  auto *layout = new wxBoxSizer(wxVERTICAL);
  layout->Add(m_controller, 1, wxEXPAND);
  SetSizer(layout);
}

Robot_Model_Panel::~Robot_Model_Panel() = default;

void Robot_Model_Panel::Show_Model_Configuration(wxWindow *parent)
{
  m_controller->Show_Model_Configuration(parent);
}

void Robot_Model_Panel::Refresh_Template_Configuration()
{
  m_controller->Refresh_Template_Configuration();
}

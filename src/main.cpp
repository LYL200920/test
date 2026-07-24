#include <wx/wx.h>

#include "camera_config_dialog.h"
#include "camera_service.h"
#include "robot_model_panel.h"

namespace
{
  enum
  {
    ID_3D_CAMERA_CONFIG = wxID_HIGHEST + 1,
    ID_ROBOT_MODEL_CONFIG
  };
}

class Test_Frame : public wxFrame
{
public:
  Test_Frame()
      : wxFrame(nullptr, wxID_ANY, "test", wxDefaultPosition, wxSize(1400, 900))
  {
    Build_Menu_Bar();

    m_model_panel = new Robot_Model_Panel(this, m_camera_service);

    auto *sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(m_model_panel, 1, wxEXPAND | wxALL, 4);
    SetSizer(sizer);

    Centre();
  }

  ~Test_Frame() override
  {
    if (m_model_panel)
    {
      if (GetSizer())
      {
        GetSizer()->Detach(m_model_panel);
      }
      delete m_model_panel;
      m_model_panel = nullptr;
    }
  }

private:
  void Build_Menu_Bar()
  {
    auto *menu_bar = new wxMenuBar();
    auto *settings_menu = new wxMenu();
    settings_menu->Append(ID_3D_CAMERA_CONFIG,
                          wxString::FromUTF8(u8"3D相机配置...\tCtrl+Shift+C"));
    settings_menu->Append(ID_ROBOT_MODEL_CONFIG,
                          wxString::FromUTF8(u8"机械臂配置...\tCtrl+Shift+R"));
    menu_bar->Append(settings_menu, wxString::FromUTF8(u8"设置"));
    SetMenuBar(menu_bar);

    Bind(wxEVT_MENU, &Test_Frame::On_3D_Camera_Config, this, ID_3D_CAMERA_CONFIG);
    Bind(wxEVT_MENU, &Test_Frame::On_Robot_Model_Config, this, ID_ROBOT_MODEL_CONFIG);
  }

  void On_3D_Camera_Config(wxCommandEvent &)
  {
    Camera_Config_Dialog dialog(this, m_camera_service);
    dialog.ShowModal();
  }

  void On_Robot_Model_Config(wxCommandEvent &)
  {
    if (m_model_panel)
    {
      m_model_panel->Show_Model_Configuration(this);
    }
  }

private:
  Camera_Service m_camera_service;
  Robot_Model_Panel *m_model_panel = nullptr;
};

class Test_App : public wxApp
{
public:
  bool OnInit() override
  {
    auto *frame = new Test_Frame();
    frame->Show();
    return true;
  }
};

wxIMPLEMENT_APP(Test_App);

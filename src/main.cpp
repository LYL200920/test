#include <wx/wx.h>

#include "camera_config_dialog.h"
#include "camera_service.h"
#include "robot_model_panel.h"
#include "template_configuration_repository.h"
#include "template_settings_dialog.h"

#include <filesystem>
#include <string>

namespace
{
  enum
  {
    ID_3D_CAMERA_CONFIG = wxID_HIGHEST + 1,
    ID_ROBOT_MODEL_CONFIG,
    ID_TEMPLATE_CONFIG
  };
}

class Test_Frame : public wxFrame
{
public:
  Test_Frame()
      : wxFrame(nullptr, wxID_ANY, "test", wxDefaultPosition, wxSize(1400, 900))
  {
    Build_Menu_Bar();

    m_model_panel = new Robot_Model_Panel(
      this, m_camera_service, m_camera_2d_service);

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
    settings_menu->Append(
      ID_TEMPLATE_CONFIG,
      wxString::FromUTF8(u8"模板配置...\tCtrl+Shift+T"));
    menu_bar->Append(settings_menu, wxString::FromUTF8(u8"设置"));
    SetMenuBar(menu_bar);

    Bind(wxEVT_MENU, &Test_Frame::On_3D_Camera_Config, this, ID_3D_CAMERA_CONFIG);
    Bind(wxEVT_MENU, &Test_Frame::On_Robot_Model_Config, this, ID_ROBOT_MODEL_CONFIG);
    Bind(wxEVT_MENU, &Test_Frame::On_Template_Config, this, ID_TEMPLATE_CONFIG);
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

  void On_Template_Config(wxCommandEvent &)
  {
    robot_model::Template_Configuration configuration;
    std::string error_message;
    const std::filesystem::path path =
      robot_model::Template_Configuration_Path();
    if (!robot_model::Load_Template_Configuration(
          path, &configuration, &error_message))
    {
      wxMessageBox(
        wxString::FromUTF8(error_message.c_str()),
        wxString::FromUTF8(u8"模板配置加载失败"),
        wxOK | wxICON_ERROR,
        this);
      return;
    }

    Template_Settings_Dialog dialog(this, configuration);
    if (dialog.ShowModal() != wxID_OK)
    {
      return;
    }
    if (!robot_model::Save_Template_Configuration(
          path, dialog.Configuration(), &error_message))
    {
      wxMessageBox(
        wxString::FromUTF8(error_message.c_str()),
        wxString::FromUTF8(u8"模板配置保存失败"),
        wxOK | wxICON_ERROR,
        this);
      return;
    }
    if (m_model_panel)
    {
      m_model_panel->Refresh_Template_Configuration();
    }
  }

private:
  Camera_Service m_camera_service;
  Camera_2D_Service m_camera_2d_service;
  Robot_Model_Panel *m_model_panel = nullptr;
};

class Test_App : public wxApp
{
public:
  bool OnInit() override
  {
    wxInitAllImageHandlers();
    auto *frame = new Test_Frame();
    frame->Show();
    return true;
  }
};

wxIMPLEMENT_APP(Test_App);

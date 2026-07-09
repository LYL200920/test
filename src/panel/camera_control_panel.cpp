#include "camera_control_panel.h"

#include "camera_acquisition_panel.h"
#include "camera_info_panel.h"
#include "camera_parameter_panel.h"
#include "camera_service.h"

#include <wx/simplebook.h>
#include <wx/sizer.h>
#include <wx/statline.h>
#include <wx/stattext.h>
#include <wx/tglbtn.h>

#include <utility>

namespace
{
wxString From_Utf8 (const char* text)
{
  return wxString::FromUTF8 (text);
}

wxString From_Utf8 (const std::string& text)
{
  return wxString::FromUTF8 (text.c_str ( ));
}

int Page_Index (Camera_Function_Page page)
{
  return static_cast<int> (page);
}
} // namespace

Camera_Control_Panel::Camera_Control_Panel (
  wxWindow* parent,
  Camera_Service& camera_service)
  : wxPanel (parent, wxID_ANY),
    m_camera_service (camera_service)
{
  auto* title = new wxStaticText (
    this, wxID_ANY, From_Utf8 (u8"3D相机"));
  m_info_menu_button = new wxToggleButton (
    this, wxID_ANY, From_Utf8 (u8"信息"));
  m_acquisition_menu_button = new wxToggleButton (
    this, wxID_ANY, From_Utf8 (u8"采集"));
  m_configuration_menu_button = new wxToggleButton (
    this, wxID_ANY, From_Utf8 (u8"配置"));
  m_info_menu_button->Bind (
    wxEVT_TOGGLEBUTTON, &Camera_Control_Panel::On_Info_Menu, this);
  m_acquisition_menu_button->Bind (
    wxEVT_TOGGLEBUTTON, &Camera_Control_Panel::On_Acquisition_Menu, this);
  m_configuration_menu_button->Bind (
    wxEVT_TOGGLEBUTTON, &Camera_Control_Panel::On_Configuration_Menu, this);

  auto* menu_sizer = new wxBoxSizer (wxHORIZONTAL);
  menu_sizer->Add (m_info_menu_button, 1, wxEXPAND | wxRIGHT, 4);
  menu_sizer->Add (m_acquisition_menu_button, 1, wxEXPAND | wxRIGHT, 4);
  menu_sizer->Add (m_configuration_menu_button, 1, wxEXPAND);

  m_page_book = new wxSimplebook (this, wxID_ANY);
  m_info_panel = new Camera_Info_Panel (m_page_book);
  m_acquisition_panel = new Camera_Acquisition_Panel (m_page_book);
  m_parameter_panel = new Camera_Parameter_Panel (m_page_book);
  m_page_book->AddPage (m_info_panel, wxEmptyString, true);
  m_page_book->AddPage (m_acquisition_panel, wxEmptyString, false);
  m_page_book->AddPage (m_parameter_panel, wxEmptyString, false);

  m_info_panel->Set_On_Refresh (
    [this] { Request_Device_Info ( ); });
  Camera_Acquisition_Panel::Callbacks acquisition_callbacks;
  acquisition_callbacks.start = [this] { Request_Start ( ); };
  acquisition_callbacks.stop = [this] { Request_Stop ( ); };
  acquisition_callbacks.close = [this] { Request_Close ( ); };
  acquisition_callbacks.refresh_streams =
    [this] { Request_Stream_Configuration ( ); };
  m_acquisition_panel->Set_Callbacks (std::move (acquisition_callbacks));
  Camera_Parameter_Panel::Callbacks parameter_callbacks;
  parameter_callbacks.refresh = [this] { Request_Load_Parameters ( ); };
  parameter_callbacks.apply =
    [this] (std::vector<Camera_Parameter_Update> updates)
    {
      Request_Apply_Parameters (std::move (updates));
    };
  m_parameter_panel->Set_Callbacks (std::move (parameter_callbacks));

  m_status_text = new wxStaticText (
    this, wxID_ANY, From_Utf8 (u8"等待相机操作"));
  m_status_text->Wrap (360);

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (title, 0, wxEXPAND | wxALL, 8);
  sizer->Add (menu_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (new wxStaticLine (this), 0, wxEXPAND | wxLEFT | wxRIGHT, 8);
  sizer->Add (m_page_book, 1, wxEXPAND | wxTOP, 4);
  sizer->Add (new wxStaticLine (this), 0, wxEXPAND | wxLEFT | wxRIGHT, 8);
  sizer->Add (m_status_text, 0, wxEXPAND | wxALL, 8);
  SetSizer (sizer);

  m_camera_service.Bind (
    wxEVT_CAMERA_UPDATED,
    &Camera_Control_Panel::On_Camera_Updated,
    this);
  m_frame_timer.SetOwner (this);
  Bind (
    wxEVT_TIMER,
    &Camera_Control_Panel::On_Frame_Timer,
    this,
    m_frame_timer.GetId ( ));

  Select_Page (Camera_Function_Page::Info);
  Update_Controls ( );
}

Camera_Control_Panel::~Camera_Control_Panel ( )
{
  m_frame_timer.Stop ( );
  m_camera_service.Unbind (
    wxEVT_CAMERA_UPDATED,
    &Camera_Control_Panel::On_Camera_Updated,
    this);
}

void Camera_Control_Panel::Set_On_Availability_Changed (
  std::function<void (bool)> callback)
{
  m_on_availability_changed = std::move (callback);
  if( m_on_availability_changed )
  {
    m_on_availability_changed (m_camera_service.Is_Device_Open ( ));
  }
}

void Camera_Control_Panel::Select_Page (Camera_Function_Page page)
{
  m_active_page = page;
  if( m_page_book )
  {
    m_page_book->SetSelection (Page_Index (page));
  }
  Update_Menu_State ( );
  if( page == Camera_Function_Page::Configuration &&
      m_camera_service.Is_Device_Open ( ) &&
      !m_camera_service.Parameters_Loaded ( ) &&
      !m_camera_service.Is_Busy ( ) )
  {
    Request_Load_Parameters ( );
  }
}

void Camera_Control_Panel::Update_Menu_State ( )
{
  if( m_info_menu_button )
  {
    m_info_menu_button->SetValue (
      m_active_page == Camera_Function_Page::Info);
  }
  if( m_acquisition_menu_button )
  {
    m_acquisition_menu_button->SetValue (
      m_active_page == Camera_Function_Page::Acquisition);
  }
  if( m_configuration_menu_button )
  {
    m_configuration_menu_button->SetValue (
      m_active_page == Camera_Function_Page::Configuration);
  }
}

void Camera_Control_Panel::Update_Controls ( )
{
  const auto state = m_camera_service.State ( );
  const bool busy = m_camera_service.Is_Busy ( );
  const bool device_open = m_camera_service.Is_Device_Open ( );

  m_info_panel->Update_View (
    m_camera_service.Device_Detail ( ), device_open, busy);
  m_acquisition_panel->Update_View (
    state,
    device_open,
    busy,
    m_camera_service.Selected_Serial_Number ( ),
    m_camera_service.Stream_Configuration ( ),
    m_camera_service.Stream_Configuration_Loaded ( ));
  m_parameter_panel->Update_View (
    m_camera_service.Parameters ( ),
    m_camera_service.Parameters_Loaded ( ),
    m_camera_service.Parameter_Status ( ),
    device_open,
    state == Camera_State::Grabbing,
    busy);
  if( device_open )
  {
    m_acquisition_panel->Update_Frame (m_camera_service.Latest_Frame ( ));
  }

  if( state == Camera_State::Grabbing )
  {
    if( !m_frame_timer.IsRunning ( ) )
    {
      m_frame_timer.Start (100);
    }
  }
  else
  {
    m_frame_timer.Stop ( );
  }

  if( m_on_availability_changed )
  {
    m_on_availability_changed (device_open);
  }
}

void Camera_Control_Panel::Request_Device_Info ( )
{
  m_status_text->SetLabel (From_Utf8 (u8"正在获取设备信息……"));
  if( m_camera_service.Request_Get_Device_Info ( ) == 0 )
  {
    m_status_text->SetLabel (From_Utf8 (u8"无法提交设备信息请求"));
  }
  Update_Controls ( );
}

void Camera_Control_Panel::Request_Stream_Configuration ( )
{
  m_status_text->SetLabel (From_Utf8 (u8"正在读取输出流配置……"));
  if( m_camera_service.Request_Get_Stream_Configuration ( ) == 0 )
  {
    m_status_text->SetLabel (From_Utf8 (u8"无法提交流配置读取请求"));
  }
  Update_Controls ( );
}

void Camera_Control_Panel::Request_Load_Parameters ( )
{
  m_status_text->SetLabel (From_Utf8 (u8"正在读取相机参数……"));
  if( m_camera_service.Request_Load_Parameters ( ) == 0 )
  {
    m_status_text->SetLabel (From_Utf8 (u8"无法提交参数读取请求"));
  }
  Update_Controls ( );
}

void Camera_Control_Panel::Request_Apply_Parameters (
  std::vector<Camera_Parameter_Update> updates)
{
  m_status_text->SetLabel (From_Utf8 (u8"正在应用相机参数……"));
  if( m_camera_service.Request_Apply_Parameters (std::move (updates)) == 0 )
  {
    m_status_text->SetLabel (From_Utf8 (u8"无法提交参数应用请求"));
  }
  Update_Controls ( );
}

void Camera_Control_Panel::Request_Start ( )
{
  m_status_text->SetLabel (From_Utf8 (u8"正在启动采集……"));
  if( m_camera_service.Request_Start_Acquisition ( ) == 0 )
  {
    m_status_text->SetLabel (From_Utf8 (u8"无法提交启动采集请求"));
  }
  Update_Controls ( );
}

void Camera_Control_Panel::Request_Stop ( )
{
  m_status_text->SetLabel (From_Utf8 (u8"正在停止采集……"));
  if( m_camera_service.Request_Stop_Acquisition ( ) == 0 )
  {
    m_status_text->SetLabel (From_Utf8 (u8"无法提交停止采集请求"));
  }
  Update_Controls ( );
}

void Camera_Control_Panel::Request_Close ( )
{
  m_status_text->SetLabel (From_Utf8 (u8"正在关闭相机……"));
  if( m_camera_service.Request_Close_Device ( ) == 0 )
  {
    m_status_text->SetLabel (From_Utf8 (u8"无法提交关闭相机请求"));
  }
  Update_Controls ( );
}

void Camera_Control_Panel::On_Info_Menu (wxCommandEvent&)
{
  Select_Page (Camera_Function_Page::Info);
}

void Camera_Control_Panel::On_Acquisition_Menu (wxCommandEvent&)
{
  Select_Page (Camera_Function_Page::Acquisition);
}

void Camera_Control_Panel::On_Configuration_Menu (wxCommandEvent&)
{
  Select_Page (Camera_Function_Page::Configuration);
}

void Camera_Control_Panel::On_Camera_Updated (wxThreadEvent& event)
{
  event.Skip ( );
  const auto operation = m_camera_service.Last_Operation ( );
  const bool success = m_camera_service.Last_Operation_Succeeded ( );

  if( !success )
  {
    if( operation == Camera_Operation::Get_Device_Info )
    {
      m_status_text->SetLabel (
        From_Utf8 (u8"获取设备信息失败：") +
        From_Utf8 (m_camera_service.Last_Error ( )));
    }
    else if( operation == Camera_Operation::Get_Stream_Configuration )
    {
      m_status_text->SetLabel (
        From_Utf8 (u8"读取输出流配置失败：") +
        From_Utf8 (m_camera_service.Last_Error ( )));
    }
    else if( operation == Camera_Operation::Load_Parameters ||
             operation == Camera_Operation::Apply_Parameters )
    {
      m_status_text->SetLabel (
        From_Utf8 (u8"相机参数操作失败：") +
        From_Utf8 (m_camera_service.Last_Error ( )));
    }
    else if( operation == Camera_Operation::Frame_Error )
    {
      m_status_text->SetLabel (
        From_Utf8 (u8"采集失败：") +
        From_Utf8 (m_camera_service.Last_Error ( )));
    }
    else if( operation != Camera_Operation::Refresh_Devices &&
             operation != Camera_Operation::Select_Device )
    {
      m_status_text->SetLabel (
        From_Utf8 (u8"操作失败：") +
        From_Utf8 (m_camera_service.Last_Error ( )));
    }
  }
  else
  {
    switch( operation )
    {
    case Camera_Operation::Open_Device:
      m_status_text->SetLabel (
        From_Utf8 (u8"相机已打开，正在读取设备信息……"));
      Select_Page (Camera_Function_Page::Info);
      break;
    case Camera_Operation::Get_Device_Info:
      m_status_text->SetLabel (From_Utf8 (u8"设备信息已更新"));
      break;
    case Camera_Operation::Get_Stream_Configuration:
      m_status_text->SetLabel (From_Utf8 (u8"输出流配置已更新"));
      break;
    case Camera_Operation::Load_Parameters:
      m_status_text->SetLabel (From_Utf8 (u8"相机参数已读取"));
      break;
    case Camera_Operation::Apply_Parameters:
      m_status_text->SetLabel (From_Utf8 (u8"相机参数已应用"));
      break;
    case Camera_Operation::Start_Acquisition:
      m_status_text->SetLabel (From_Utf8 (u8"采集已启动"));
      break;
    case Camera_Operation::Stop_Acquisition:
      m_status_text->SetLabel (From_Utf8 (u8"采集已停止"));
      break;
    case Camera_Operation::Close_Device:
      m_status_text->SetLabel (From_Utf8 (u8"相机已关闭"));
      break;
    default:
      break;
    }
  }

  Update_Controls ( );
}

void Camera_Control_Panel::On_Frame_Timer (wxTimerEvent&)
{
  m_acquisition_panel->Update_Frame (m_camera_service.Latest_Frame ( ));
}

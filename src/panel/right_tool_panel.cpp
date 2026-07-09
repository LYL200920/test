#include "right_tool_panel.h"

#include <wx/button.h>
#include <wx/simplebook.h>
#include <wx/sizer.h>

namespace
{
constexpr int TOOL_RAIL_WIDTH = 72;
constexpr int TOOL_CONTENT_WIDTH = 400;

int Page_Index (Right_Tool_Page page)
{
  return static_cast<int> (page);
}
} // namespace

Right_Tool_Panel::Right_Tool_Panel (wxWindow* parent)
  : wxPanel (parent, wxID_ANY)
{
  auto* rail_panel = new wxPanel (this, wxID_ANY);
  rail_panel->SetMinSize (wxSize (TOOL_RAIL_WIDTH, -1));

  m_tcp_button = new wxButton (
    rail_panel, wxID_ANY, "TCP", wxDefaultPosition, wxSize (68, 40));
  m_flow_button = new wxButton (
    rail_panel, wxID_ANY, wxString::FromUTF8 (u8"流程"),
    wxDefaultPosition, wxSize (68, 40));
  m_camera_button = new wxButton (
    rail_panel, wxID_ANY, wxString::FromUTF8 (u8"相机"),
    wxDefaultPosition, wxSize (68, 40));
  m_robot_button = new wxButton (
    rail_panel, wxID_ANY, wxString::FromUTF8 (u8"机械臂"),
    wxDefaultPosition, wxSize (68, 40));
  m_camera_button->Enable (false);
  m_camera_button->SetToolTip (
    wxString::FromUTF8 (u8"请先在设置中选择并打开3D相机"));
  m_robot_button->Enable (false);
  m_robot_button->SetToolTip (
    wxString::FromUTF8 (u8"请先在设置中加载机械臂模型"));

  m_tcp_button->Bind (
    wxEVT_BUTTON, &Right_Tool_Panel::On_Tcp_Click, this);
  m_flow_button->Bind (
    wxEVT_BUTTON, &Right_Tool_Panel::On_Flow_Click, this);
  m_camera_button->Bind (
    wxEVT_BUTTON, &Right_Tool_Panel::On_Camera_Click, this);
  m_robot_button->Bind (
    wxEVT_BUTTON, &Right_Tool_Panel::On_Robot_Click, this);

  auto* rail_sizer = new wxBoxSizer (wxVERTICAL);
  rail_sizer->Add (m_tcp_button, 0, wxEXPAND | wxBOTTOM, 4);
  rail_sizer->Add (m_flow_button, 0, wxEXPAND | wxBOTTOM, 4);
  rail_sizer->Add (m_camera_button, 0, wxEXPAND | wxBOTTOM, 4);
  rail_sizer->Add (m_robot_button, 0, wxEXPAND);
  rail_sizer->AddStretchSpacer (1);
  rail_panel->SetSizer (rail_sizer);

  m_page_book = new wxSimplebook (this, wxID_ANY);
  m_page_book->SetMinSize (wxSize (TOOL_CONTENT_WIDTH, -1));

  auto* sizer = new wxBoxSizer (wxHORIZONTAL);
  sizer->Add (rail_panel, 0, wxEXPAND | wxRIGHT, 4);
  sizer->Add (m_page_book, 1, wxEXPAND);
  SetSizer (sizer);

  Set_Collapsed (true);
}

wxWindow* Right_Tool_Panel::Page_Parent ( ) const
{
  return m_page_book;
}

void Right_Tool_Panel::Add_Page (
  Right_Tool_Page page,
  wxWindow* window)
{
  if( m_page_book == nullptr || window == nullptr )
  {
    return;
  }

  const auto page_index = static_cast<size_t> (Page_Index (page));
  if( page_index <= m_page_book->GetPageCount ( ) )
  {
    m_page_book->InsertPage (page_index, window, wxEmptyString, false);
  }
}

void Right_Tool_Panel::Set_Robot_Tool_Enabled (bool enabled)
{
  m_robot_tool_enabled = enabled;
  if( m_robot_button )
  {
    m_robot_button->Enable (enabled);
    m_robot_button->SetToolTip (
      enabled ? wxString ( ) :
        wxString::FromUTF8 (u8"请先在设置中加载机械臂模型"));
  }

  if( !enabled && !m_collapsed &&
      m_active_page == Right_Tool_Page::Robot )
  {
    Set_Collapsed (true);
  }
}

void Right_Tool_Panel::Set_Camera_Tool_Enabled (bool enabled)
{
  m_camera_tool_enabled = enabled;
  if( m_camera_button )
  {
    m_camera_button->Enable (enabled);
    m_camera_button->SetToolTip (
      enabled ? wxString ( ) :
        wxString::FromUTF8 (u8"请先在设置中选择并打开3D相机"));
  }

  if( !enabled && !m_collapsed &&
      m_active_page == Right_Tool_Page::Camera )
  {
    Set_Collapsed (true);
  }
}

void Right_Tool_Panel::On_Tcp_Click (wxCommandEvent&)
{
  Toggle_Page (Right_Tool_Page::Tcp);
}

void Right_Tool_Panel::On_Flow_Click (wxCommandEvent&)
{
  Toggle_Page (Right_Tool_Page::Flow);
}

void Right_Tool_Panel::On_Camera_Click (wxCommandEvent&)
{
  Toggle_Page (Right_Tool_Page::Camera);
}

void Right_Tool_Panel::On_Robot_Click (wxCommandEvent&)
{
  Toggle_Page (Right_Tool_Page::Robot);
}

void Right_Tool_Panel::Toggle_Page (Right_Tool_Page page)
{
  if( page == Right_Tool_Page::Robot && !m_robot_tool_enabled )
  {
    return;
  }
  if( page == Right_Tool_Page::Camera && !m_camera_tool_enabled )
  {
    return;
  }

  if( !m_collapsed && m_active_page == page )
  {
    Set_Collapsed (true);
    return;
  }

  m_active_page = page;
  const auto page_index = Page_Index (page);
  if( page_index < static_cast<int> (m_page_book->GetPageCount ( )) )
  {
    m_page_book->SetSelection (page_index);
    Set_Collapsed (false);
  }
}

void Right_Tool_Panel::Set_Collapsed (bool collapsed)
{
  m_collapsed = collapsed;
  if( m_page_book )
  {
    m_page_book->Show (!collapsed);
  }

  SetMinSize (wxSize (
    collapsed ? TOOL_RAIL_WIDTH : TOOL_RAIL_WIDTH + TOOL_CONTENT_WIDTH + 4,
    -1));
  Update_Button_Labels ( );
  Layout ( );
  if( GetParent ( ) )
  {
    GetParent ( )->Layout ( );
  }
}

void Right_Tool_Panel::Update_Button_Labels ( )
{
  if( m_tcp_button )
  {
    m_tcp_button->SetLabel (
      !m_collapsed && m_active_page == Right_Tool_Page::Tcp ? "TCP <" : "TCP");
  }
  if( m_flow_button )
  {
    m_flow_button->SetLabel (
      !m_collapsed && m_active_page == Right_Tool_Page::Flow
        ? wxString::FromUTF8 (u8"流程 <")
        : wxString::FromUTF8 (u8"流程"));
  }
  if( m_robot_button )
  {
    m_robot_button->SetLabel (
      !m_collapsed && m_active_page == Right_Tool_Page::Robot
        ? wxString::FromUTF8 (u8"机械臂 <")
        : wxString::FromUTF8 (u8"机械臂"));
  }
  if( m_camera_button )
  {
    m_camera_button->SetLabel (
      !m_collapsed && m_active_page == Right_Tool_Page::Camera
        ? wxString::FromUTF8 (u8"相机 <")
        : wxString::FromUTF8 (u8"相机"));
  }
}

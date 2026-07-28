#include "right_tool_panel.h"

#include <wx/button.h>
#include <wx/simplebook.h>
#include <wx/sizer.h>

#include <utility>

namespace
{
constexpr int TOOL_RAIL_WIDTH = 72;
constexpr int TOOL_CONTENT_WIDTH = 400;

int Page_Index (Right_Tool_Page page)
{
  return static_cast<int> (page) - 1;
}
} // namespace

Right_Tool_Panel::Right_Tool_Panel (wxWindow* parent)
  : wxPanel (parent, wxID_ANY)
{
  auto* rail_panel = new wxPanel (this, wxID_ANY);
  rail_panel->SetMinSize (wxSize (TOOL_RAIL_WIDTH, -1));

  m_robot_button = new wxButton (
    rail_panel, wxID_ANY, "Robot",
    wxDefaultPosition, wxSize (68, 40));
  m_teach_button = new wxButton (
    rail_panel, wxID_ANY, "Teach",
    wxDefaultPosition, wxSize (68, 40));
  m_tcp_button = new wxButton (
    rail_panel, wxID_ANY, "TCP", wxDefaultPosition, wxSize (68, 40));
  m_flow_button = new wxButton (
    rail_panel, wxID_ANY, wxString::FromUTF8 (u8"流程"),
    wxDefaultPosition, wxSize (68, 40));
  m_camera_button = new wxButton (
    rail_panel, wxID_ANY, wxString::FromUTF8 (u8"相机"),
    wxDefaultPosition, wxSize (68, 40));
  m_point_cloud_button = new wxButton (
    rail_panel, wxID_ANY, wxString::FromUTF8 (u8"点云"),
    wxDefaultPosition, wxSize (68, 40));
  m_tool_button = new wxButton (
    rail_panel, wxID_ANY, "Tool",
    wxDefaultPosition, wxSize (68, 40));
  m_camera_button->Enable (false);
  m_camera_button->SetToolTip (
    wxString::FromUTF8 (u8"请先在设置中选择并打开3D相机"));
  m_robot_button->Enable (false);
  m_robot_button->SetToolTip (
    wxString::FromUTF8 (u8"请先在设置中加载机械臂模型"));
  m_flow_button->Enable (false);
  m_flow_button->SetToolTip (
    wxString::FromUTF8 (u8"请先在 Teach 中完成 Progress 检查"));

  m_robot_button->Bind (
    wxEVT_BUTTON, &Right_Tool_Panel::On_Robot_Click, this);
  m_teach_button->Bind (
    wxEVT_BUTTON, &Right_Tool_Panel::On_Teach_Click, this);
  m_tcp_button->Bind (
    wxEVT_BUTTON, &Right_Tool_Panel::On_Tcp_Click, this);
  m_flow_button->Bind (
    wxEVT_BUTTON, &Right_Tool_Panel::On_Flow_Click, this);
  m_camera_button->Bind (
    wxEVT_BUTTON, &Right_Tool_Panel::On_Camera_Click, this);
  m_point_cloud_button->Bind (
    wxEVT_BUTTON, &Right_Tool_Panel::On_Point_Cloud_Click, this);
  m_tool_button->Bind (
    wxEVT_BUTTON, &Right_Tool_Panel::On_Tool_Click, this);

  auto* rail_sizer = new wxBoxSizer (wxVERTICAL);
  rail_sizer->Add (m_robot_button, 0, wxEXPAND | wxBOTTOM, 4);
  rail_sizer->Add (m_teach_button, 0, wxEXPAND | wxBOTTOM, 4);
  rail_sizer->Add (m_tcp_button, 0, wxEXPAND | wxBOTTOM, 4);
  rail_sizer->Add (m_flow_button, 0, wxEXPAND | wxBOTTOM, 4);
  rail_sizer->Add (m_camera_button, 0, wxEXPAND | wxBOTTOM, 4);
  rail_sizer->Add (m_point_cloud_button, 0, wxEXPAND | wxBOTTOM, 4);
  rail_sizer->Add (m_tool_button, 0, wxEXPAND);
  rail_sizer->AddStretchSpacer (1);
  rail_panel->SetSizer (rail_sizer);

  m_left_page_book = new wxSimplebook (this, wxID_ANY);
  m_left_page_book->SetMinSize (wxSize (TOOL_CONTENT_WIDTH, -1));
  m_right_page_book = new wxSimplebook (this, wxID_ANY);
  m_right_page_book->SetMinSize (wxSize (TOOL_CONTENT_WIDTH, -1));

  auto* sizer = new wxBoxSizer (wxHORIZONTAL);
  sizer->Add (m_left_page_book, 1, wxEXPAND | wxRIGHT, 4);
  sizer->Add (rail_panel, 0, wxEXPAND | wxRIGHT, 4);
  sizer->Add (m_right_page_book, 1, wxEXPAND);
  SetSizer (sizer);

  Set_Robot_Expanded (false);
  Set_Right_Expanded (false);
}

wxWindow* Right_Tool_Panel::Page_Parent (Right_Tool_Page page) const
{
  return page == Right_Tool_Page::Robot
    ? static_cast<wxWindow*> (m_left_page_book)
    : static_cast<wxWindow*> (m_right_page_book);
}

void Right_Tool_Panel::Add_Page (
  Right_Tool_Page page,
  wxWindow* window)
{
  if( window == nullptr )
  {
    return;
  }

  if( page == Right_Tool_Page::Robot )
  {
    if( m_left_page_book &&
        m_left_page_book->GetPageCount ( ) == 0 )
    {
      m_left_page_book->AddPage (
        window, wxEmptyString, true);
    }
    return;
  }

  const auto page_index = static_cast<size_t> (Page_Index (page));
  if( m_right_page_book &&
      page_index <= m_right_page_book->GetPageCount ( ) )
  {
    m_right_page_book->InsertPage (
      page_index, window, wxEmptyString, false);
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

  if( !enabled && m_robot_expanded )
  {
    Set_Robot_Expanded (false);
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

  if( !enabled && m_right_expanded &&
      m_active_page == Right_Tool_Page::Camera )
  {
    Set_Right_Expanded (false);
  }
}

void Right_Tool_Panel::Set_Flow_Tool_Enabled (bool enabled)
{
  m_flow_tool_enabled = enabled;
  if( m_flow_button )
  {
    m_flow_button->Enable (enabled);
    m_flow_button->SetToolTip (
      enabled ? wxString ( ) :
        wxString::FromUTF8 (u8"请先在 Teach 中完成 Progress 检查"));
  }
  if( !enabled && m_right_expanded &&
      m_active_page == Right_Tool_Page::Flow )
  {
    Set_Right_Expanded (false);
  }
}

void Right_Tool_Panel::Set_On_Width_Changed (
  std::function<void (int)> callback)
{
  m_on_width_changed = std::move (callback);
}

void Right_Tool_Panel::On_Tcp_Click (wxCommandEvent&)
{
  Toggle_Right_Page (Right_Tool_Page::Tcp);
}

void Right_Tool_Panel::On_Flow_Click (wxCommandEvent&)
{
  if( !m_flow_tool_enabled ) return;
  Toggle_Right_Page (Right_Tool_Page::Flow);
}

void Right_Tool_Panel::On_Camera_Click (wxCommandEvent&)
{
  Toggle_Right_Page (Right_Tool_Page::Camera);
}

void Right_Tool_Panel::On_Teach_Click (wxCommandEvent&)
{
  Toggle_Right_Page (Right_Tool_Page::Teach);
}

void Right_Tool_Panel::On_Point_Cloud_Click (wxCommandEvent&)
{
  Toggle_Right_Page (Right_Tool_Page::PointCloud);
}

void Right_Tool_Panel::On_Tool_Click (wxCommandEvent&)
{
  Toggle_Right_Page (Right_Tool_Page::Tool);
}

void Right_Tool_Panel::On_Robot_Click (wxCommandEvent&)
{
  if( !m_robot_tool_enabled )
  {
    return;
  }
  Set_Robot_Expanded (!m_robot_expanded);
}

void Right_Tool_Panel::Toggle_Right_Page (Right_Tool_Page page)
{
  if( page == Right_Tool_Page::Camera && !m_camera_tool_enabled )
  {
    return;
  }

  if( m_right_expanded && m_active_page == page )
  {
    Set_Right_Expanded (false);
    return;
  }

  m_active_page = page;
  const auto page_index = Page_Index (page);
  if( m_right_page_book &&
      page_index < static_cast<int> (
        m_right_page_book->GetPageCount ( )) )
  {
    m_right_page_book->SetSelection (page_index);
    Set_Right_Expanded (true);
  }
}

void Right_Tool_Panel::Set_Robot_Expanded (bool expanded)
{
  m_robot_expanded = expanded;
  if( m_left_page_book )
  {
    m_left_page_book->Show (expanded);
  }
  Notify_Width_Changed ( );
}

void Right_Tool_Panel::Set_Right_Expanded (bool expanded)
{
  m_right_expanded = expanded;
  if( m_right_page_book )
  {
    m_right_page_book->Show (expanded);
  }
  Notify_Width_Changed ( );
}

int Right_Tool_Panel::Desired_Width ( ) const
{
  return TOOL_RAIL_WIDTH +
    (m_robot_expanded ? TOOL_CONTENT_WIDTH + 4 : 0) +
    (m_right_expanded ? TOOL_CONTENT_WIDTH + 4 : 0);
}

void Right_Tool_Panel::Notify_Width_Changed ( )
{
  SetMinSize (wxSize (Desired_Width ( ), -1));
  Update_Button_Labels ( );
  Layout ( );
  if( GetParent ( ) )
  {
    GetParent ( )->Layout ( );
  }
  if( m_on_width_changed )
  {
    m_on_width_changed (Desired_Width ( ));
  }
}

void Right_Tool_Panel::Update_Button_Labels ( )
{
  if( m_tcp_button )
  {
    m_tcp_button->SetLabel (
      m_right_expanded && m_active_page == Right_Tool_Page::Tcp
        ? "TCP >"
        : "TCP");
  }
  if( m_flow_button )
  {
    m_flow_button->SetLabel (
      m_right_expanded && m_active_page == Right_Tool_Page::Flow
        ? wxString::FromUTF8 (u8"流程 >")
        : wxString::FromUTF8 (u8"流程"));
  }
  if( m_robot_button )
  {
    m_robot_button->SetLabel (
      m_robot_expanded ? "< Robot" : "Robot");
  }
  if( m_teach_button )
  {
    m_teach_button->SetLabel (
      m_right_expanded && m_active_page == Right_Tool_Page::Teach
        ? "Teach >"
        : "Teach");
  }
  if( m_point_cloud_button )
  {
    m_point_cloud_button->SetLabel (
      m_right_expanded && m_active_page == Right_Tool_Page::PointCloud
        ? wxString::FromUTF8 (u8"点云 >")
        : wxString::FromUTF8 (u8"点云"));
  }
  if( m_tool_button )
  {
    m_tool_button->SetLabel (
      m_right_expanded && m_active_page == Right_Tool_Page::Tool
        ? "Tool >"
        : "Tool");
  }
  if( m_camera_button )
  {
    m_camera_button->SetLabel (
      m_right_expanded && m_active_page == Right_Tool_Page::Camera
        ? wxString::FromUTF8 (u8"相机 >")
        : wxString::FromUTF8 (u8"相机"));
  }
}

#include "point_cloud_overlay_toolbar.h"

#include "camera_service.h"
#include "point_cloud_file_repository.h"

#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/filedlg.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/spinctrl.h>

#include <filesystem>
#include <utility>

Point_Cloud_Overlay_Toolbar::Point_Cloud_Overlay_Toolbar (
  wxWindow* parent,
  Camera_Service& camera_service,
  Callbacks callbacks)
  : wxPanel (parent, wxID_ANY),
    m_controller (camera_service),
    m_callbacks (std::move (callbacks))
{
  SetBackgroundColour (parent->GetBackgroundColour ( ));
  auto* title = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (u8"点云工具"));
  m_load_latest_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"叠加当前点云"));
  m_save_latest_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"保存当前点云"));
  auto* load_file = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"加载点云文件"));
  auto* clear = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"清除点云"));
  m_camera_pose_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"显示相机位姿"));

  auto* collision_title = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (u8"碰撞参数"));
  m_collision_enabled_checkbox = new wxCheckBox (
    this, wxID_ANY, wxString::FromUTF8 (u8"启用体积碰撞"));
  m_collision_enabled_checkbox->SetValue (true);
  m_clearance_ctrl = new wxSpinCtrlDouble (
    this, wxID_ANY, "10", wxDefaultPosition, wxDefaultSize,
    wxSP_ARROW_KEYS, 0.0, 100.0, 10.0, 1.0);
  m_voxel_size_ctrl = new wxSpinCtrlDouble (
    this, wxID_ANY, "5", wxDefaultPosition, wxDefaultSize,
    wxSP_ARROW_KEYS, 1.0, 50.0, 5.0, 1.0);
  m_robot_exclusion_ctrl = new wxSpinCtrlDouble (
    this, wxID_ANY, "15", wxDefaultPosition, wxDefaultSize,
    wxSP_ARROW_KEYS, 0.0, 100.0, 15.0, 1.0);
  m_exclude_robot_points = new wxCheckBox (
    this, wxID_ANY, wxString::FromUTF8 (u8"剔除机器人本体点"));
  m_exclude_robot_points->SetValue (true);
  auto* apply_collision_settings = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"应用碰撞参数"));

  m_load_latest_button->Bind (
    wxEVT_BUTTON, &Point_Cloud_Overlay_Toolbar::On_Load_Latest, this);
  m_save_latest_button->Bind (
    wxEVT_BUTTON, &Point_Cloud_Overlay_Toolbar::On_Save_Latest, this);
  load_file->Bind (
    wxEVT_BUTTON, &Point_Cloud_Overlay_Toolbar::On_Load_File, this);
  clear->Bind (
    wxEVT_BUTTON, &Point_Cloud_Overlay_Toolbar::On_Clear, this);
  m_camera_pose_button->Bind (
    wxEVT_BUTTON, &Point_Cloud_Overlay_Toolbar::On_Toggle_Camera_Pose, this);
  m_collision_enabled_checkbox->Bind (
    wxEVT_CHECKBOX,
    &Point_Cloud_Overlay_Toolbar::On_Toggle_Collision,
    this);
  apply_collision_settings->Bind (
    wxEVT_BUTTON,
    &Point_Cloud_Overlay_Toolbar::On_Apply_Collision_Settings, this);

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (title, 0, wxEXPAND | wxALL, 8);
  sizer->Add (m_load_latest_button, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (m_save_latest_button, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (load_file, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (clear, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (m_camera_pose_button, 0,
              wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (collision_title, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
  sizer->Add (m_collision_enabled_checkbox, 0,
              wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
  auto add_parameter = [this, sizer] (
    const wxString& label, wxSpinCtrlDouble* control)
  {
    auto* row = new wxBoxSizer (wxHORIZONTAL);
    row->Add (new wxStaticText (this, wxID_ANY, label),
              1, wxALIGN_CENTER_VERTICAL | wxRIGHT, 6);
    row->Add (control, 0, wxALIGN_CENTER_VERTICAL);
    sizer->Add (row, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
  };
  add_parameter (wxString::FromUTF8 (u8"安全距离 (mm)"), m_clearance_ctrl);
  add_parameter (wxString::FromUTF8 (u8"碰撞体素 (mm)"), m_voxel_size_ctrl);
  add_parameter (wxString::FromUTF8 (u8"本体剔除距离 (mm)"),
                 m_robot_exclusion_ctrl);
  sizer->Add (m_exclude_robot_points, 0,
              wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
  sizer->Add (apply_collision_settings, 0,
              wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
  sizer->AddStretchSpacer (1);
  SetSizer (sizer);
  Set_Camera_Connected (false);
}

void Point_Cloud_Overlay_Toolbar::Attach_Renderer (vtkRenderer* renderer)
{
  m_controller.Attach_Renderer (renderer);
}

bool Point_Cloud_Overlay_Toolbar::Has_Point_Cloud ( ) const
{
  return m_controller.Has_Point_Cloud ( );
}

void Point_Cloud_Overlay_Toolbar::Set_Camera_Connected (bool connected)
{
  if( m_load_latest_button ) m_load_latest_button->Enable (connected);
  if( m_save_latest_button ) m_save_latest_button->Enable (connected);
}

void Point_Cloud_Overlay_Toolbar::Set_Interactive_LOD (bool enabled)
{
  m_controller.Set_Interactive_LOD (enabled);
  if( m_callbacks.render_scene ) m_callbacks.render_scene ( );
}

void Point_Cloud_Overlay_Toolbar::On_Load_Latest (wxCommandEvent&)
{
  const auto result = m_controller.Load_Latest (Renderer ( ));
  if( !result.success )
  {
    Report_Error (wxString::FromUTF8 (u8"点云叠加失败"),
                  result.error_message);
    return;
  }
  Set_Status (wxString::Format (
    wxString::FromUTF8 (
      u8"点云已叠加：显示 %zu / 有效 %zu / 原始 %zu 点；Depth 中值 %.1f mm；基坐标 X[%.0f, %.0f] Y[%.0f, %.0f] Z[%.0f, %.0f] mm"),
    result.point_count,
    result.filtered_point_count,
    result.source_point_count,
    result.median_depth_mm,
    result.world_bounds_mm[0], result.world_bounds_mm[1],
    result.world_bounds_mm[2], result.world_bounds_mm[3],
    result.world_bounds_mm[4], result.world_bounds_mm[5]));
  Show_And_Render ( );
}

void Point_Cloud_Overlay_Toolbar::On_Save_Latest (wxCommandEvent&)
{
  const auto resource_root = point_cloud::Point_Cloud_Resource_Root ( );
  wxFileDialog dialog (
    this,
    wxString::FromUTF8 (u8"保存当前点云"),
    wxString (resource_root.wstring ( )),
    "point_cloud.ply",
    "PLY point cloud (*.ply)|*.ply",
    wxFD_SAVE);
  if( dialog.ShowModal ( ) != wxID_OK ) return;

  std::filesystem::path selected_path (dialog.GetPath ( ).ToStdWstring ( ));
  selected_path.replace_extension (".ply");
  const auto result = m_controller.Save_Latest_To_File (
    selected_path, Robot_Model_Id ( ));
  if( !result.success )
  {
    Report_Error (wxString::FromUTF8 (u8"保存点云失败"),
                  result.error_message);
    return;
  }
  Set_Status (wxString::Format (
    wxString::FromUTF8 (u8"点云已保存：%s，%zu 点，Depth 中值 %.1f mm"),
    wxString (result.file_path.filename ( ).wstring ( )).c_str ( ),
    result.point_count,
    result.median_depth_mm));
}

void Point_Cloud_Overlay_Toolbar::On_Load_File (wxCommandEvent&)
{
  const auto resource_root = point_cloud::Point_Cloud_Resource_Root ( );
  wxFileDialog dialog (
    this,
    wxString::FromUTF8 (u8"加载机器人基坐标点云叠加"),
    wxString (resource_root.wstring ( )),
    "",
    "Robot-base PLY files (*.ply)|*.ply",
    wxFD_OPEN | wxFD_FILE_MUST_EXIST);
  if( dialog.ShowModal ( ) != wxID_OK ) return;

  const std::filesystem::path selected_path (
    dialog.GetPath ( ).ToStdWstring ( ));
  const auto result = m_controller.Load_File (
    selected_path, Robot_Model_Id ( ), Renderer ( ));
  if( !result.success )
  {
    Report_Error (wxString::FromUTF8 (u8"加载点云文件叠加失败"),
                  result.error_message);
    return;
  }
  Set_Status (wxString::Format (
    wxString::FromUTF8 (
      u8"文件点云已叠加：%s，%zu 点；基坐标 X[%.0f, %.0f] Y[%.0f, %.0f] Z[%.0f, %.0f] mm"),
    wxString (selected_path.filename ( ).wstring ( )).c_str ( ),
    result.point_count,
    result.world_bounds_mm[0], result.world_bounds_mm[1],
    result.world_bounds_mm[2], result.world_bounds_mm[3],
    result.world_bounds_mm[4], result.world_bounds_mm[5]));
  Show_And_Render ( );
}

void Point_Cloud_Overlay_Toolbar::On_Clear (wxCommandEvent&)
{
  m_controller.Clear ( );
  if( m_callbacks.clear_collision_obstacle_points )
  {
    m_callbacks.clear_collision_obstacle_points ( );
  }
  Set_Status (wxString::FromUTF8 (u8"点云叠加已清除"));
  if( m_callbacks.render_scene ) m_callbacks.render_scene ( );
}

void Point_Cloud_Overlay_Toolbar::On_Toggle_Camera_Pose (wxCommandEvent&)
{
  const bool show = !m_camera_pose_visible;
  const bool applied = m_callbacks.set_camera_pose_visible &&
    m_callbacks.set_camera_pose_visible (show);
  if( !applied || !m_camera_pose_button ) return;
  m_camera_pose_visible = show;
  m_camera_pose_button->SetLabel (wxString::FromUTF8 (
    show ? u8"隐藏相机位姿" : u8"显示相机位姿"));
}

void Point_Cloud_Overlay_Toolbar::On_Toggle_Collision (wxCommandEvent&)
{
  if( !m_collision_enabled_checkbox ||
      !m_callbacks.set_collision_enabled ) return;

  const bool enabled = m_collision_enabled_checkbox->GetValue ( );
  m_callbacks.set_collision_enabled (enabled);
  if( !enabled )
  {
    Set_Status (wxString::FromUTF8 (u8"体积碰撞已关闭"));
    return;
  }

  Set_Status (m_callbacks.collision_rebuild_in_progress &&
              m_callbacks.collision_rebuild_in_progress ( )
    ? wxString::FromUTF8 (u8"体积碰撞已开启，碰撞索引正在后台构建")
    : wxString::FromUTF8 (u8"体积碰撞已开启"));
}

void Point_Cloud_Overlay_Toolbar::Report_Error (
  const wxString& title,
  const std::string& message)
{
  wxMessageBox (wxString::FromUTF8 (message.c_str ( )), title,
                wxOK | wxICON_ERROR, this);
}

void Point_Cloud_Overlay_Toolbar::On_Apply_Collision_Settings (
  wxCommandEvent&)
{
  if( !m_callbacks.apply_collision_settings || !m_clearance_ctrl ||
      !m_voxel_size_ctrl || !m_robot_exclusion_ctrl ||
      !m_exclude_robot_points ) return;
  std::size_t excluded_count = 0;
  std::size_t collision_count = 0;
  std::string error;
  if( !m_callbacks.apply_collision_settings (
        m_clearance_ctrl->GetValue ( ),
        m_voxel_size_ctrl->GetValue ( ),
        m_robot_exclusion_ctrl->GetValue ( ),
        m_exclude_robot_points->GetValue ( ),
        &excluded_count, &collision_count, &error) )
  {
    Report_Error (wxString::FromUTF8 (u8"碰撞参数应用失败"), error);
    return;
  }
  if( m_callbacks.collision_rebuild_in_progress &&
      m_callbacks.collision_rebuild_in_progress ( ) )
  {
    Set_Status (wxString::FromUTF8 (
      u8"碰撞参数正在后台构建，当前碰撞数据继续生效"));
    return;
  }
  Set_Status (wxString::Format (
    wxString::FromUTF8 (
      u8"碰撞参数已应用：剔除本体点 %zu，碰撞点 %zu"),
    excluded_count, collision_count));
  if( m_callbacks.render_scene ) m_callbacks.render_scene ( );
}

vtkRenderer* Point_Cloud_Overlay_Toolbar::Renderer ( ) const
{
  return m_callbacks.renderer ? m_callbacks.renderer ( ) : nullptr;
}

std::string Point_Cloud_Overlay_Toolbar::Robot_Model_Id ( ) const
{
  return m_callbacks.robot_model_id ? m_callbacks.robot_model_id ( ) : "";
}

void Point_Cloud_Overlay_Toolbar::Show_And_Render ( )
{
  std::string collision_error;
  if( !Sync_Collision_Obstacle_Points (&collision_error) )
  {
    if( m_callbacks.clear_collision_obstacle_points )
    {
      m_callbacks.clear_collision_obstacle_points ( );
    }
    Report_Error (wxString::FromUTF8 (u8"碰撞点云同步失败"),
                  collision_error);
    return;
  }
  if( m_callbacks.collision_rebuild_in_progress &&
      m_callbacks.collision_rebuild_in_progress ( ) )
  {
    Set_Status (wxString::FromUTF8 (
      u8"点云已显示，碰撞索引正在后台构建"));
  }
  if( m_callbacks.show_robot_page ) m_callbacks.show_robot_page ( );
  if( m_callbacks.render_scene ) m_callbacks.render_scene ( );
}

void Point_Cloud_Overlay_Toolbar::Set_Status (const wxString& status)
{
  if( m_callbacks.set_status ) m_callbacks.set_status (status);
}

bool Point_Cloud_Overlay_Toolbar::Sync_Collision_Obstacle_Points (
  std::string* error_message)
{
  if( error_message ) error_message->clear ( );
  if( !m_callbacks.set_collision_obstacle_points )
  {
    if( error_message )
      *error_message = "Collision obstacle callback is not configured";
    return false;
  }
  return m_callbacks.set_collision_obstacle_points (
    m_controller.Collision_Obstacle_Xyz ( ), error_message);
}

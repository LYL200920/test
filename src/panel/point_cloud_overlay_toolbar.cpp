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
#include <wx/tglbtn.h>

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
  auto* edit_title = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (u8"点云编辑"));
  m_edit_button = new wxToggleButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"矩形框选"));
  m_polygon_edit_button = new wxToggleButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"多边形框选"));
  m_delete_selected_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"删除选中"));
  m_undo_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"撤销"));
  m_redo_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"重做"));

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
  m_edit_button->Bind (
    wxEVT_TOGGLEBUTTON, &Point_Cloud_Overlay_Toolbar::On_Toggle_Edit, this);
  m_polygon_edit_button->Bind (
    wxEVT_TOGGLEBUTTON,
    &Point_Cloud_Overlay_Toolbar::On_Toggle_Polygon_Edit,
    this);
  m_delete_selected_button->Bind (
    wxEVT_BUTTON, &Point_Cloud_Overlay_Toolbar::On_Delete_Selected, this);
  m_undo_button->Bind (
    wxEVT_BUTTON, &Point_Cloud_Overlay_Toolbar::On_Undo_Edit, this);
  m_redo_button->Bind (
    wxEVT_BUTTON, &Point_Cloud_Overlay_Toolbar::On_Redo_Edit, this);

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (title, 0, wxEXPAND | wxALL, 8);
  sizer->Add (m_load_latest_button, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (m_save_latest_button, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (load_file, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (clear, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (m_camera_pose_button, 0,
              wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (edit_title, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
  sizer->Add (m_edit_button, 0,
              wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
  sizer->Add (m_polygon_edit_button, 0,
              wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
  auto* edit_row = new wxBoxSizer (wxHORIZONTAL);
  edit_row->Add (m_delete_selected_button, 1, wxRIGHT, 4);
  edit_row->Add (m_undo_button, 1, wxRIGHT, 4);
  edit_row->Add (m_redo_button, 1);
  sizer->Add (edit_row, 0,
              wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
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
  Update_Edit_Buttons ( );
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
  if( m_save_latest_button )
    m_save_latest_button->Enable (m_controller.Has_Point_Cloud ( ));
}

void Point_Cloud_Overlay_Toolbar::Set_Interactive_LOD (bool enabled)
{
  m_controller.Set_Interactive_LOD (enabled);
  if( m_callbacks.render_scene ) m_callbacks.render_scene ( );
}

void Point_Cloud_Overlay_Toolbar::Handle_Area_Selected (
  int start_x,
  int start_y,
  int end_x,
  int end_y,
  bool add,
  bool toggle)
{
  if( !m_edit_button || !m_edit_button->GetValue ( ) ||
      !m_controller.Has_Point_Cloud ( ) )
  {
    return;
  }
  const auto mode = toggle
    ? point_cloud::Point_Selection_Mode::Toggle
    : add
      ? point_cloud::Point_Selection_Mode::Add
      : point_cloud::Point_Selection_Mode::Replace;
  const std::size_t selected_count = m_controller.Select_In_Display_Rect (
    Renderer ( ), start_x, start_y, end_x, end_y, mode);
  Set_Status (wxString::Format (
    wxString::FromUTF8 (u8"已选择 %zu 个点"),
    selected_count));
  Update_Edit_Buttons ( );
  if( m_callbacks.render_scene ) m_callbacks.render_scene ( );
}

void Point_Cloud_Overlay_Toolbar::Handle_Polygon_Selected (
  const std::vector<std::array<int, 2>>& polygon,
  bool add,
  bool toggle)
{
  if( !m_polygon_edit_button ||
      !m_polygon_edit_button->GetValue ( ) ||
      !m_controller.Has_Point_Cloud ( ) )
  {
    return;
  }

  std::vector<point_cloud::Display_Point> display_polygon;
  display_polygon.reserve (polygon.size ( ));
  for( const auto& vertex : polygon )
  {
    display_polygon.push_back ({
      static_cast<double> (vertex[0]),
      static_cast<double> (vertex[1])});
  }
  const auto mode = toggle
    ? point_cloud::Point_Selection_Mode::Toggle
    : add
      ? point_cloud::Point_Selection_Mode::Add
      : point_cloud::Point_Selection_Mode::Replace;
  const std::size_t selected_count =
    m_controller.Select_In_Display_Polygon (
      Renderer ( ), display_polygon, mode);
  Set_Status (wxString::Format (
    wxString::FromUTF8 (u8"多边形已选择 %zu 个点"),
    selected_count));
  Update_Edit_Buttons ( );
  if( m_callbacks.render_scene ) m_callbacks.render_scene ( );
}

void Point_Cloud_Overlay_Toolbar::Delete_Selected ( )
{
  std::string error;
  const std::size_t deleted_count = m_controller.Delete_Selected (&error);
  if( deleted_count == 0 )
  {
    if( !error.empty ( ) )
      Report_Error (wxString::FromUTF8 (u8"删除点云失败"), error);
    else
      Set_Status (wxString::FromUTF8 (u8"请先框选需要删除的点"));
    Update_Edit_Buttons ( );
    return;
  }
  if( !Sync_Collision_Obstacle_Points (&error) )
  {
    Report_Error (wxString::FromUTF8 (u8"碰撞点云同步失败"), error);
  }
  if( m_callbacks.clear_point_cloud_selection_outline )
    m_callbacks.clear_point_cloud_selection_outline ( );
  Set_Status (wxString::Format (
    wxString::FromUTF8 (u8"已删除 %zu 个点，剩余 %zu 个点"),
    deleted_count,
    m_controller.Displayed_Point_Count ( )));
  Update_Edit_Buttons ( );
  if( m_callbacks.render_scene ) m_callbacks.render_scene ( );
}

void Point_Cloud_Overlay_Toolbar::Undo_Edit ( )
{
  std::string error;
  if( !m_controller.Undo (&error) )
  {
    if( !error.empty ( ) )
      Report_Error (wxString::FromUTF8 (u8"撤销失败"), error);
    Update_Edit_Buttons ( );
    return;
  }
  if( !Sync_Collision_Obstacle_Points (&error) )
  {
    Report_Error (wxString::FromUTF8 (u8"碰撞点云同步失败"), error);
  }
  if( m_callbacks.clear_point_cloud_selection_outline )
    m_callbacks.clear_point_cloud_selection_outline ( );
  Set_Status (wxString::Format (
    wxString::FromUTF8 (u8"已撤销删除，当前 %zu 个点"),
    m_controller.Displayed_Point_Count ( )));
  Update_Edit_Buttons ( );
  if( m_callbacks.render_scene ) m_callbacks.render_scene ( );
}

void Point_Cloud_Overlay_Toolbar::Redo_Edit ( )
{
  std::string error;
  if( !m_controller.Redo (&error) )
  {
    if( !error.empty ( ) )
      Report_Error (wxString::FromUTF8 (u8"重做失败"), error);
    Update_Edit_Buttons ( );
    return;
  }
  if( !Sync_Collision_Obstacle_Points (&error) )
  {
    Report_Error (wxString::FromUTF8 (u8"碰撞点云同步失败"), error);
  }
  Set_Status (wxString::Format (
    wxString::FromUTF8 (u8"已重做删除，当前 %zu 个点"),
    m_controller.Displayed_Point_Count ( )));
  Update_Edit_Buttons ( );
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
    "edited_point_cloud.ply",
    "PLY point cloud (*.ply)|*.ply",
    wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
  if( dialog.ShowModal ( ) != wxID_OK ) return;

  std::filesystem::path selected_path (dialog.GetPath ( ).ToStdWstring ( ));
  selected_path.replace_extension (".ply");
  const auto result = m_controller.Save_Current_To_File (
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
  if( m_edit_button ) m_edit_button->SetValue (false);
  if( m_polygon_edit_button ) m_polygon_edit_button->SetValue (false);
  if( m_callbacks.set_point_cloud_edit_mode )
    m_callbacks.set_point_cloud_edit_mode (false, false);
  m_controller.Clear ( );
  if( m_callbacks.clear_collision_obstacle_points )
  {
    m_callbacks.clear_collision_obstacle_points ( );
  }
  Set_Status (wxString::FromUTF8 (u8"点云叠加已清除"));
  if( m_callbacks.render_scene ) m_callbacks.render_scene ( );
  Update_Edit_Buttons ( );
}

void Point_Cloud_Overlay_Toolbar::On_Toggle_Edit (wxCommandEvent&)
{
  const bool enabled =
    m_edit_button && m_edit_button->GetValue ( ) &&
    (m_controller.Has_Point_Cloud ( ) ||
     m_controller.Can_Undo ( ) ||
     m_controller.Can_Redo ( ));
  if( m_edit_button ) m_edit_button->SetValue (enabled);
  if( enabled && m_polygon_edit_button )
    m_polygon_edit_button->SetValue (false);
  if( !enabled )
  {
    m_controller.Clear_Selection ( );
  }
  if( m_callbacks.set_point_cloud_edit_mode )
  {
    m_callbacks.set_point_cloud_edit_mode (enabled, false);
  }
  Set_Status (wxString::FromUTF8 (
    enabled
      ? u8"框选模式：拖动左键选择，Shift 追加，Ctrl 反选"
      : u8"已退出点云编辑"));
  Update_Edit_Buttons ( );
  if( m_callbacks.render_scene ) m_callbacks.render_scene ( );
}

void Point_Cloud_Overlay_Toolbar::On_Toggle_Polygon_Edit (
  wxCommandEvent&)
{
  const bool enabled =
    m_polygon_edit_button &&
    m_polygon_edit_button->GetValue ( ) &&
    (m_controller.Has_Point_Cloud ( ) ||
     m_controller.Can_Undo ( ) ||
     m_controller.Can_Redo ( ));
  if( m_polygon_edit_button )
    m_polygon_edit_button->SetValue (enabled);
  if( enabled && m_edit_button )
    m_edit_button->SetValue (false);
  if( !enabled )
  {
    m_controller.Clear_Selection ( );
  }
  if( m_callbacks.set_point_cloud_edit_mode )
  {
    m_callbacks.set_point_cloud_edit_mode (enabled, true);
  }
  Set_Status (wxString::FromUTF8 (
    enabled
      ? u8"多边形框选：左键依次添加顶点，右键添加末点并闭合"
      : u8"已退出点云编辑"));
  Update_Edit_Buttons ( );
  if( m_callbacks.render_scene ) m_callbacks.render_scene ( );
}

void Point_Cloud_Overlay_Toolbar::On_Delete_Selected (wxCommandEvent&)
{
  Delete_Selected ( );
}

void Point_Cloud_Overlay_Toolbar::On_Undo_Edit (wxCommandEvent&)
{
  Undo_Edit ( );
}

void Point_Cloud_Overlay_Toolbar::On_Redo_Edit (wxCommandEvent&)
{
  Redo_Edit ( );
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
  Update_Edit_Buttons ( );
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
  if( !m_controller.Has_Point_Cloud ( ) )
  {
    if( m_callbacks.clear_collision_obstacle_points )
    {
      m_callbacks.clear_collision_obstacle_points ( );
      return true;
    }
    if( error_message )
      *error_message = "Collision obstacle clear callback is not configured";
    return false;
  }
  if( !m_callbacks.set_collision_obstacle_points )
  {
    if( error_message )
      *error_message = "Collision obstacle callback is not configured";
    return false;
  }
  return m_callbacks.set_collision_obstacle_points (
    m_controller.Collision_Obstacle_Xyz ( ), error_message);
}

void Point_Cloud_Overlay_Toolbar::Update_Edit_Buttons ( )
{
  const bool has_cloud = m_controller.Has_Point_Cloud ( );
  const bool has_history =
    m_controller.Can_Undo ( ) || m_controller.Can_Redo ( );
  const bool editing =
    (m_edit_button && m_edit_button->GetValue ( )) ||
    (m_polygon_edit_button && m_polygon_edit_button->GetValue ( ));
  if( m_edit_button ) m_edit_button->Enable (has_cloud || has_history);
  if( m_polygon_edit_button )
    m_polygon_edit_button->Enable (has_cloud || has_history);
  if( m_save_latest_button )
    m_save_latest_button->Enable (has_cloud);
  if( m_delete_selected_button )
    m_delete_selected_button->Enable (
      editing && m_controller.Selection_Count ( ) > 0);
  if( m_undo_button )
    m_undo_button->Enable (editing && m_controller.Can_Undo ( ));
  if( m_redo_button )
    m_redo_button->Enable (editing && m_controller.Can_Redo ( ));
}

#include "robot_model_panel.h"

#include "camera_control_panel.h"
#include "camera_image_view.h"
#include "camera_service.h"
#include "robot_joint_state_builder.h"
#include "robot_model_config_dialog.h"
#include "robot_model_repository.h"
#include "robot_trajectory_io.h"
#include "right_tool_panel.h"
#include "net_panel.h"
#include "flow_panel.h"
#include "point_cloud_view.h"
#include "point_cloud_overlay_toolbar.h"
#include "teach_point_command_panel.h"
#include "teach_point_list_panel.h"

#include <wx/button.h>
#include <wx/filedlg.h>
#include <wx/msgdlg.h>
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/simplebook.h>
#include <wx/splitter.h>
#include <wx/tglbtn.h>
#include <wx/utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <utility>

namespace
{
constexpr int DEFAULT_JOINT_MIN = -180;
constexpr int DEFAULT_JOINT_MAX = 180;
constexpr int TRAJECTORY_FRAME_COUNT = 120;
constexpr int TRAJECTORY_TIMER_MS = 16;
constexpr int TRAJECTORY_SPEED_DEFAULT_INDEX = 2;
constexpr std::size_t ROBOT_JOINT_COUNT = 6;
constexpr int RIGHT_TOOL_COLLAPSED_WIDTH = 72;
constexpr int RIGHT_TOOL_DEFAULT_EXPANDED_WIDTH = 476;
constexpr int DISPLAY_MINIMUM_WIDTH = 300;
constexpr int TEACH_POINT_COLLAPSED_WIDTH = 38;
constexpr int TEACH_POINT_DEFAULT_EXPANDED_WIDTH = 240;
constexpr int WORKSPACE_MINIMUM_WIDTH = 500;
constexpr const char* DEFAULT_ROBOT_MODEL_ID = "KR10_R1100_2";
constexpr std::array<double, 5> TRAJECTORY_SPEED_SCALES = {
  0.25, 0.5, 1.0, 2.0, 4.0
};

std::array<double, 6> configured_home_input_angles (
  const robot_model::Robot_Kinematic_Params& params)
{
  if( params.has_home_input_angles ) return params.home_input_angles_deg;

  std::array<double, 6> fallback = { };
  for( size_t index = 0; index < fallback.size ( ); ++index )
    fallback[index] = robot_model::Neutral_Joint_Input_At (params, index);
  return fallback;
}

int slider_limit_at (const std::vector<double>& values, size_t index,
                     int fallback)
{
  if( index >= values.size ( ) )
  {
    return fallback;
  }

  return static_cast<int> (std::lround (values[index]));
}

std::string model_id (const robot_model::Robot_Model_Info& model)
{
  return model.model_dir.filename ( ).string ( );
}

bool validate_model_files (
  const robot_model::Robot_Model_Info& model,
  std::string* error_message)
{
  if( error_message )
  {
    error_message->clear ( );
  }

  if( model.stl_files.empty ( ) )
  {
    if( error_message )
    {
      *error_message = "The robot model does not contain STL files";
    }
    return false;
  }

  if( model.xml_path.empty ( ) || !std::filesystem::is_regular_file (model.xml_path) )
  {
    if( error_message )
    {
      *error_message = "The robot model configuration XML is missing";
    }
    return false;
  }

  const auto missing_mesh = std::find_if (
    model.stl_files.begin ( ), model.stl_files.end ( ),
    [] (const std::filesystem::path& path)
    {
      return !std::filesystem::is_regular_file (path);
    });
  if( missing_mesh != model.stl_files.end ( ) )
  {
    if( error_message )
    {
      *error_message = "One or more robot STL files are missing";
    }
    return false;
  }

  return true;
}

int cartesian_position_limit_mm (
  const robot_model::Robot_Kinematic_Params& params)
{
  double estimated_reach = 0.0;
  for( const double length : params.link_lengths )
  {
    estimated_reach += std::abs (length);
  }
  const double limit = std::max (1000.0, estimated_reach * 1.25);
  return static_cast<int> (std::ceil (limit / 100.0) * 100.0);
}

wxString collision_summary (
  const robot_model::Robot_Collision_Result& collision)
{
  switch( collision.type )
  {
    case robot_model::Robot_Collision_Type::Self_Collision:
      return wxString::Format (
        wxString::FromUTF8 (u8"部件 %zu 与部件 %zu 自碰撞"),
        collision.robot_part_index,
        collision.other_robot_part_index);
    case robot_model::Robot_Collision_Type::Ground:
      return wxString::Format (
        wxString::FromUTF8 (u8"部件 %zu 接近地面，距离 %.2f mm"),
        collision.robot_part_index,
        collision.minimum_distance_mm);
    case robot_model::Robot_Collision_Type::Obstacle_Point:
      return wxString::Format (
        wxString::FromUTF8 (u8"部件 %zu 与点云碰撞，距离 %.2f mm"),
        collision.robot_part_index,
        collision.minimum_distance_mm);
    default:
      return wxString::FromUTF8 (u8"检测到碰撞");
  }
}

} // namespace

Robot_Model_Panel::Robot_Model_Panel (
  wxWindow* parent,
  Camera_Service& camera_service,
  wxWindowID id)
  : wxPanel (parent, id)
{
  auto* title = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (u8"显示区域"));
  m_model_name_text = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (u8"当前机械臂：未加载"));
  m_robot_display_button = new wxToggleButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"机械臂"));
  m_camera_display_button = new wxToggleButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"相机图像"));
  m_point_cloud_display_button = new wxToggleButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"点云"));
  m_robot_display_button->Bind (
    wxEVT_TOGGLEBUTTON, &Robot_Model_Panel::On_Robot_Display, this);
  m_camera_display_button->Bind (
    wxEVT_TOGGLEBUTTON, &Robot_Model_Panel::On_Camera_Image_Display, this);
  m_point_cloud_display_button->Bind (
    wxEVT_TOGGLEBUTTON, &Robot_Model_Panel::On_Point_Cloud_Display, this);
  m_workspace_splitter = new wxSplitterWindow (
    this,
    wxID_ANY,
    wxDefaultPosition,
    wxDefaultSize,
    wxSP_LIVE_UPDATE | wxSP_3DSASH | wxBORDER_NONE);
  m_workspace_splitter->SetMinimumPaneSize (TEACH_POINT_COLLAPSED_WIDTH);
  m_workspace_splitter->SetSashGravity (0.0);
  m_workspace_splitter->SetSashSize (6);

  m_teach_point_list_panel = new Teach_Point_List_Panel (
    m_workspace_splitter);
  m_teach_point_list_panel->Set_On_Selection_Changed (
    [this] { Update_Trajectory_Status ( ); });
  m_teach_point_list_panel->Set_On_Collapsed_Changed (
    [this] (bool collapsed) { Resize_Teach_Point_List (collapsed); });

  m_content_splitter = new wxSplitterWindow (
    m_workspace_splitter,
    wxID_ANY,
    wxDefaultPosition,
    wxDefaultSize,
    wxSP_LIVE_UPDATE | wxSP_3DSASH | wxBORDER_NONE);
  m_content_splitter->SetMinSize (wxSize (WORKSPACE_MINIMUM_WIDTH, -1));
  m_content_splitter->SetMinimumPaneSize (RIGHT_TOOL_COLLAPSED_WIDTH);
  m_content_splitter->SetSashGravity (1.0);
  m_content_splitter->SetSashSize (6);

  auto* display_panel = new wxPanel (m_content_splitter, wxID_ANY);
  m_display_book = new wxSimplebook (display_panel, wxID_ANY);
  m_display_book->SetMinSize (wxSize (DISPLAY_MINIMUM_WIDTH, -1));
  auto* status_panel = new wxPanel (
    display_panel, wxID_ANY, wxDefaultPosition, wxDefaultSize,
    wxBORDER_SIMPLE);
  m_status_text = new wxStaticText (status_panel, wxID_ANY, "");
  auto* status_sizer = new wxBoxSizer (wxHORIZONTAL);
  status_sizer->Add (m_status_text, 1, wxALIGN_CENTER_VERTICAL | wxALL, 6);
  status_panel->SetSizer (status_sizer);
  status_panel->SetMinSize (wxSize (-1, 30));

  auto* display_sizer = new wxBoxSizer (wxVERTICAL);
  display_sizer->Add (m_display_book, 1, wxEXPAND);
  display_sizer->Add (status_panel, 0, wxEXPAND | wxTOP, 4);
  display_panel->SetSizer (display_sizer);
  m_view = new Robot_Model_View (m_display_book);
  m_view->Set_On_Collision_Rebuild_Completed (
    [this] (
      bool success,
      const robot_model::Robot_Collision_Point_Cloud_Stats& stats,
      const std::string& error_message)
    {
      if( !m_status_text ) return;
      if( !success )
      {
        m_status_text->SetLabel (
          wxString::FromUTF8 (u8"碰撞索引后台构建失败：") +
          wxString::FromUTF8 (error_message.c_str ( )));
        return;
      }
      m_status_text->SetLabel (wxString::Format (
        wxString::FromUTF8 (
          u8"碰撞索引已更新：剔除本体点 %zu，碰撞点 %zu，耗时 %.0f ms"),
        stats.excluded_robot_point_count,
        stats.collision_point_count,
        stats.build_time_ms));
    });
  m_view->Set_On_Flange_Dragged (
    [this] (const robot_model::Robot_Position_IK_Result& result)
    {
      Apply_Flange_Drag_Result (result);
    });
  m_view->Set_On_Flange_Pose_Dragged (
    [this] (const robot_model::Robot_Pose_IK_Result& result)
    {
      Apply_Flange_Pose_Drag_Result (result);
    });
  m_camera_image_view = new Camera_Image_View (
    m_display_book, camera_service);
  m_point_cloud_view = new Point_Cloud_View (
    m_display_book, camera_service);
  Point_Cloud_Overlay_Toolbar::Callbacks overlay_callbacks;
  overlay_callbacks.renderer = [this]
  {
    return m_view ? m_view->Scene_Renderer ( ) : nullptr;
  };
  overlay_callbacks.robot_model_id = [this]
  {
    return m_current_model_id;
  };
  overlay_callbacks.show_robot_page = [this]
  {
    Select_Display_Page (Main_Display_Page::Robot);
  };
  overlay_callbacks.render_scene = [this]
  {
    if( m_view ) m_view->Render_Scene ( );
  };
  overlay_callbacks.set_status = [this] (const wxString& status)
  {
    if( m_status_text ) m_status_text->SetLabel (status);
  };
  overlay_callbacks.set_camera_pose_visible = [this] (bool visible)
  {
    return Set_Camera_Pose_Visible (visible);
  };
  overlay_callbacks.set_collision_obstacle_points =
    [this] (std::shared_ptr<const std::vector<float>> xyz,
            std::string* error_message)
    {
      if( !m_view )
      {
        if( error_message ) *error_message = "Robot model view is unavailable";
        return false;
      }
      return m_view->Set_Collision_Obstacle_Points (xyz, error_message);
    };
  overlay_callbacks.clear_collision_obstacle_points = [this]
  {
    if( m_view ) m_view->Clear_Collision_Obstacle_Points ( );
  };
  overlay_callbacks.set_collision_enabled = [this] (bool enabled)
  {
    if( !m_view ) return;
    // Reset while collision is still disabled so an already-colliding pose
    // cannot prevent the robot from reaching the known safe home state.
    if( enabled ) Reset_Robot_To_Home ( );
    m_view->Set_Collision_Enabled (enabled);
  };
  overlay_callbacks.collision_rebuild_in_progress = [this]
  {
    return m_view && m_view->Collision_Rebuild_In_Progress ( );
  };
  overlay_callbacks.apply_collision_settings =
    [this] (double clearance_mm,
            double voxel_size_mm,
            double robot_exclusion_distance_mm,
            bool exclude_robot_points,
            std::size_t* excluded_robot_point_count,
            std::size_t* collision_point_count,
            std::string* error_message)
    {
      if( !m_view )
      {
        if( error_message ) *error_message = "Robot model view is unavailable";
        return false;
      }

      auto settings = m_view->Collision_Settings ( );
      settings.clearance_mm = clearance_mm;
      settings.point_cloud.voxel_size_mm = voxel_size_mm;
      settings.point_cloud.robot_exclusion_distance_mm =
        robot_exclusion_distance_mm;
      settings.point_cloud.exclude_robot_points = exclude_robot_points;
      if( !m_view->Set_Collision_Settings (settings, error_message) )
      {
        return false;
      }

      const auto& stats = m_view->Collision_Point_Cloud_Stats ( );
      if( excluded_robot_point_count )
      {
        *excluded_robot_point_count = stats.excluded_robot_point_count;
      }
      if( collision_point_count )
      {
        *collision_point_count = stats.collision_point_count;
      }
      return true;
    };
  overlay_callbacks.set_point_cloud_edit_mode =
    [this] (bool enabled, bool polygon)
  {
    if( m_view )
    {
      m_view->Set_Point_Cloud_Edit_Mode (
        enabled,
        polygon
          ? Robot_Model_View::Point_Cloud_Selection_Shape::Polygon
          : Robot_Model_View::Point_Cloud_Selection_Shape::Rectangle);
    }
  };
  overlay_callbacks.clear_point_cloud_selection_outline = [this]
  {
    if( m_view ) m_view->Clear_Point_Cloud_Selection_Outline ( );
  };
  m_display_book->AddPage (m_view, wxEmptyString, true);
  m_display_book->AddPage (m_camera_image_view, wxEmptyString, false);
  m_display_book->AddPage (m_point_cloud_view, wxEmptyString, false);
  m_right_tool_panel = new Right_Tool_Panel (m_content_splitter);
  m_right_tool_panel->Set_On_Collapsed_Changed (
    [this] (bool collapsed) { Resize_Right_Tool (collapsed); });

  auto* tcp_panel = new Net_Panel (m_right_tool_panel->Page_Parent ( ));
  auto* flow_panel = new Flow_Panel (m_right_tool_panel->Page_Parent ( ));
  m_camera_control_panel = new Camera_Control_Panel (
    m_right_tool_panel->Page_Parent ( ), camera_service);
  m_point_cloud_overlay_toolbar = new Point_Cloud_Overlay_Toolbar (
    m_right_tool_panel->Page_Parent ( ),
    camera_service,
    std::move (overlay_callbacks));
  m_view->Set_On_Point_Cloud_Area_Selected (
    [this] (
      int start_x,
      int start_y,
      int end_x,
      int end_y,
      bool add,
      bool toggle)
    {
      if( m_point_cloud_overlay_toolbar )
      {
        m_point_cloud_overlay_toolbar->Handle_Area_Selected (
          start_x, start_y, end_x, end_y, add, toggle);
      }
    });
  m_view->Set_On_Point_Cloud_Polygon_Selected (
    [this] (
      const std::vector<std::array<int, 2>>& polygon,
      bool add,
      bool toggle)
    {
      if( m_point_cloud_overlay_toolbar )
      {
        m_point_cloud_overlay_toolbar->Handle_Polygon_Selected (
          polygon, add, toggle);
      }
    });
  m_view->Set_On_Point_Cloud_Delete ([this]
  {
    if( m_point_cloud_overlay_toolbar )
      m_point_cloud_overlay_toolbar->Delete_Selected ( );
  });
  m_view->Set_On_Point_Cloud_Undo ([this]
  {
    if( m_point_cloud_overlay_toolbar )
      m_point_cloud_overlay_toolbar->Undo_Edit ( );
  });
  m_view->Set_On_Point_Cloud_Redo ([this]
  {
    if( m_point_cloud_overlay_toolbar )
      m_point_cloud_overlay_toolbar->Redo_Edit ( );
  });
  m_view->Set_On_Flange_Drag_State_Changed ([this] (bool dragging)
  {
    if( m_point_cloud_overlay_toolbar )
      m_point_cloud_overlay_toolbar->Set_Interactive_LOD (dragging);
  });
  m_view->Set_On_Flange_Drag_Performance (
    [this] (const robot_model::Robot_Drag_Performance_Stats& stats)
    {
      if( !m_status_text || stats.update_count == 0 ) return;
      const double count = static_cast<double> (stats.update_count);
      m_status_text->SetLabel (wxString::Format (
        wxString::FromUTF8 (
          u8"拖动性能：更新 %zu，平均 %.2f ms，最大 %.2f ms；IK %.2f，碰撞 %.2f，渲染 %.2f ms；姿态 %zu，候选/距离=%zu/%zu，精确网格 %zu；阻挡 点云/自碰/地面=%zu/%zu/%zu"),
        stats.update_count,
        stats.total_update_time_ms / count,
        stats.maximum_update_time_ms,
        stats.total_ik_time_ms / count,
        stats.total_collision_time_ms / count,
        stats.total_render_time_ms / count,
        stats.checked_pose_count,
        stats.obstacle_candidate_points,
        stats.obstacle_distance_queries,
        stats.self_exact_pair_queries,
        stats.obstacle_blocked_update_count,
        stats.self_blocked_update_count,
        stats.ground_blocked_update_count));
    });
  auto* robot_tool_page = Build_Robot_Tool_Page (
    m_right_tool_panel->Page_Parent ( ));
  m_right_tool_panel->Add_Page (Right_Tool_Page::Tcp, tcp_panel);
  m_right_tool_panel->Add_Page (Right_Tool_Page::Flow, flow_panel);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Camera, m_camera_control_panel);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::PointCloud, m_point_cloud_overlay_toolbar);
  m_right_tool_panel->Add_Page (Right_Tool_Page::Robot, robot_tool_page);
  m_camera_control_panel->Set_On_Availability_Changed (
    [this] (bool enabled)
    {
      m_right_tool_panel->Set_Camera_Tool_Enabled (enabled);
      if( m_point_cloud_overlay_toolbar )
      {
        m_point_cloud_overlay_toolbar->Set_Camera_Connected (enabled);
      }
    });
  m_right_tool_panel->Set_Robot_Tool_Enabled (false);
  m_content_splitter->SplitVertically (
    display_panel,
    m_right_tool_panel,
    -RIGHT_TOOL_COLLAPSED_WIDTH);
  m_workspace_splitter->SplitVertically (
    m_teach_point_list_panel,
    m_content_splitter,
    TEACH_POINT_DEFAULT_EXPANDED_WIDTH);

  m_trajectory_timer.SetOwner (this);
  Bind (wxEVT_TIMER, &Robot_Model_Panel::On_Trajectory_Timer, this,
        m_trajectory_timer.GetId ( ));

  auto* toolbar_sizer = new wxBoxSizer (wxHORIZONTAL);
  toolbar_sizer->Add (title, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 8);
  toolbar_sizer->Add (
    m_robot_display_button, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
  toolbar_sizer->Add (
    m_camera_display_button, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
  toolbar_sizer->Add (
    m_point_cloud_display_button, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 12);
  toolbar_sizer->AddStretchSpacer (1);
  toolbar_sizer->Add (m_model_name_text, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 12);

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (toolbar_sizer, 0, wxEXPAND | wxALL, 6);
  sizer->Add (
    m_workspace_splitter,
    1,
    wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM,
    6);
  SetSizer (sizer);

  Select_Display_Page (Main_Display_Page::Robot);

  Load_Model_List ( );
  Load_Default_Model ( );
}

void Robot_Model_Panel::On_Robot_Display (wxCommandEvent&)
{
  Select_Display_Page (Main_Display_Page::Robot);
}

void Robot_Model_Panel::On_Camera_Image_Display (wxCommandEvent&)
{
  Select_Display_Page (Main_Display_Page::Camera_Image);
}

void Robot_Model_Panel::On_Point_Cloud_Display (wxCommandEvent&)
{
  Select_Display_Page (Main_Display_Page::Point_Cloud);
}

void Robot_Model_Panel::On_Reset_Robot (wxCommandEvent&)
{
  if( Reset_Robot_To_Home ( ) && m_status_text )
    m_status_text->SetLabel (wxString::FromUTF8 (u8"机械臂已复位"));
}

bool Robot_Model_Panel::Reset_Robot_To_Home ( )
{
  if( !m_view || !m_view->Has_Current_Model ( ) ) return false;
  if( Is_Trajectory_Active ( ) ) Stop_Trajectory_Playback ( );

  Apply_Joint_Input_Angles_To_Sliders (
    configured_home_input_angles (m_view->Kinematic_Params ( )));
  Select_Display_Page (Main_Display_Page::Robot);
  return true;
}

bool Robot_Model_Panel::Set_Camera_Pose_Visible (bool visible)
{
  if( !visible )
  {
    m_camera_pose_controller.Hide ( );
    m_status_text->SetLabel (wxString::FromUTF8 (u8"相机位姿已隐藏"));
    m_view->Render_Scene ( );
    return true;
  }

  std::string error;
  if( !m_camera_pose_controller.Show (m_view->Scene_Renderer ( ), &error) )
  {
    wxMessageBox (
      wxString::FromUTF8 (error.c_str ( )),
      wxString::FromUTF8 (u8"相机位姿加载失败"),
      wxOK | wxICON_ERROR,
      this);
    return false;
  }

  Select_Display_Page (Main_Display_Page::Robot);
  m_status_text->SetLabel (wxString::FromUTF8 (u8"相机位姿已显示"));
  m_view->Render_Scene ( );
  return true;
}

void Robot_Model_Panel::On_Toggle_Flange_Frame (wxCommandEvent&)
{
  const bool visible =
    m_flange_frame_button && m_flange_frame_button->GetValue ( );
  m_view->Set_Flange_Frame_Visible (visible);
  if( visible && !m_view->Has_Flange_Pose ( ) )
  {
    m_flange_frame_button->SetValue (false);
    m_flange_frame_button->SetLabel (
      wxString::FromUTF8 (u8"显示法兰坐标系"));
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"当前模型没有可用的法兰位姿"));
    return;
  }

  m_flange_frame_button->SetLabel (
    visible
      ? wxString::FromUTF8 (u8"隐藏法兰坐标系")
      : wxString::FromUTF8 (u8"显示法兰坐标系"));
  m_status_text->SetLabel (
    visible
      ? wxString::FromUTF8 (u8"法兰坐标系已显示")
      : wxString::FromUTF8 (u8"法兰坐标系已隐藏"));

  if( !visible && m_flange_6d_button && m_flange_6d_button->GetValue ( ) )
  {
    Set_Flange_Interaction_Mode (
      Robot_Model_View::Flange_Interaction_Mode::Observe);
  }
}

void Robot_Model_Panel::On_Toggle_Flange_Free_Drag (wxCommandEvent&)
{
  const bool enabled = m_flange_free_drag_button &&
    m_flange_free_drag_button->GetValue ( );
  if( enabled && !m_view->Has_Flange_Pose ( ) )
  {
    m_flange_free_drag_button->SetValue (false);
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"当前模型没有可拖拽的法兰位姿"));
    return;
  }

  if( enabled && Is_Trajectory_Active ( ) )
  {
    Stop_Trajectory_Playback ( );
  }
  if( enabled )
  {
    Select_Display_Page (Main_Display_Page::Robot);
    Set_Flange_Interaction_Mode (
      Robot_Model_View::Flange_Interaction_Mode::Free_Translation);
  }
  else
  {
    Set_Flange_Interaction_Mode (
      Robot_Model_View::Flange_Interaction_Mode::Observe);
  }
  m_status_text->SetLabel (
    enabled
      ? wxString::FromUTF8 (u8"自由拖拽：拖动黄色手柄")
      : wxString::FromUTF8 (u8"观察模式"));
}

void Robot_Model_Panel::On_Toggle_Flange_6D (wxCommandEvent&)
{
  const bool enabled = m_flange_6d_button && m_flange_6d_button->GetValue ( );
  if( enabled && !m_view->Has_Flange_Pose ( ) )
  {
    m_flange_6d_button->SetValue (false);
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"当前模型没有可操纵的法兰位姿"));
    return;
  }
  if( enabled && Is_Trajectory_Active ( ) ) Stop_Trajectory_Playback ( );
  if( enabled ) Select_Display_Page (Main_Display_Page::Robot);
  Set_Flange_Interaction_Mode (enabled
    ? Robot_Model_View::Flange_Interaction_Mode::Transform_6D
    : Robot_Model_View::Flange_Interaction_Mode::Observe);
  m_status_text->SetLabel (enabled
    ? wxString::FromUTF8 (u8"6D 操纵：彩色轴端平移，圆环旋转")
    : wxString::FromUTF8 (u8"观察模式"));
}

void Robot_Model_Panel::Set_Flange_Interaction_Mode (
  Robot_Model_View::Flange_Interaction_Mode mode)
{
  const bool free_translation =
    mode == Robot_Model_View::Flange_Interaction_Mode::Free_Translation;
  const bool transform_6d =
    mode == Robot_Model_View::Flange_Interaction_Mode::Transform_6D;
  m_flange_free_drag_button->SetValue (free_translation);
  m_flange_6d_button->SetValue (transform_6d);
  m_flange_free_drag_button->SetLabel (wxString::FromUTF8 (
    free_translation ? u8"停止自由拖拽" : u8"自由拖拽"));
  m_flange_6d_button->SetLabel (wxString::FromUTF8 (
    transform_6d ? u8"停止 6D 操纵" : u8"6D 操纵"));
  m_view->Set_Flange_Interaction_Mode (mode);

  if( free_translation )
  {
    m_view->Set_Flange_Frame_Visible (false);
    m_flange_frame_button->SetValue (false);
    m_flange_frame_button->SetLabel (
      wxString::FromUTF8 (u8"显示法兰坐标系"));
  }
  else if( transform_6d )
  {
    m_view->Set_Flange_Frame_Visible (true);
    m_flange_frame_button->SetValue (true);
    m_flange_frame_button->SetLabel (
      wxString::FromUTF8 (u8"隐藏法兰坐标系"));
  }
}

void Robot_Model_Panel::Select_Display_Page (Main_Display_Page page)
{
  m_display_page = page;
  if( m_display_book )
  {
    m_display_book->SetSelection (static_cast<int> (page));
  }
  Update_Display_Menu ( );
}

void Robot_Model_Panel::Update_Display_Menu ( )
{
  if( m_robot_display_button )
  {
    m_robot_display_button->SetValue (
      m_display_page == Main_Display_Page::Robot);
  }
  if( m_camera_display_button )
  {
    m_camera_display_button->SetValue (
      m_display_page == Main_Display_Page::Camera_Image);
  }
  if( m_point_cloud_display_button )
  {
    m_point_cloud_display_button->SetValue (
      m_display_page == Main_Display_Page::Point_Cloud);
  }
}

void Robot_Model_Panel::Show_Model_Configuration (wxWindow* parent)
{
  Robot_Model_Config_Dialog dialog (
    parent ? parent : this,
    m_models,
    m_current_model_id);
  if( dialog.ShowModal ( ) != wxID_OK )
  {
    return;
  }

  wxBusyCursor busy_cursor;
  std::string error_message;
  if( !Load_Model (dialog.Selected_Model_Index ( ), &error_message) )
  {
    m_status_text->SetLabel (wxString::FromUTF8 (u8"机械臂模型加载失败"));
    wxMessageBox (
      wxString::FromUTF8 (error_message.c_str ( )),
      wxString::FromUTF8 (u8"机械臂加载失败"),
      wxOK | wxICON_ERROR,
      parent ? parent : this);
  }
}

void Robot_Model_Panel::On_Add_Trajectory_Point (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) || m_current_model_id.empty ( ) || !m_view )
  {
    return;
  }

  robot_model::Matrix4 world_from_flange = { };
  if( !m_view->Get_World_From_Flange (&world_from_flange) ) return;
  const auto& point = m_teach_point_store.Add_Point (
    m_current_model_id,
    Read_Joint_Input_Angles ( ),
    robot_model::Build_Xyzabc_From_Zyx_Matrix (world_from_flange));
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  Update_Trajectory_Status ( );
  m_status_text->SetLabel (wxString::FromUTF8 (u8"已记录示教点：") +
    wxString::FromUTF8 (
      robot_model::Format_Teach_Point_Name (point.id).c_str ( )));
}

void Robot_Model_Panel::On_Clear_Trajectory_Points (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) )
  {
    return;
  }

  m_teach_point_store.Clear_Points (m_current_model_id);
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  Update_Trajectory_Status ( );
}

void Robot_Model_Panel::On_Go_To_Trajectory_Point (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) )
  {
    return;
  }

  const int selection = Selected_Teach_Point_Index ( );
  if( selection == wxNOT_FOUND ||
      selection < 0 ||
      static_cast<size_t> (selection) >= m_trajectory_session.Point_Count ( ) )
  {
    return;
  }

  const auto start_angles = Read_Joint_Input_Angles ( );
  if( !m_trajectory_session.Start_Go_To (
        start_angles,
        static_cast<size_t> (selection),
        TRAJECTORY_FRAME_COUNT) )
  {
    return;
  }

  Set_Joint_Controls_Enabled (false);
  m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
  Update_Trajectory_Status ( );
}

void Robot_Model_Panel::On_Delete_Trajectory_Point (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) )
  {
    return;
  }

  const int selection = Selected_Teach_Point_Index ( );
  if( selection == wxNOT_FOUND ||
      selection < 0 ||
      static_cast<size_t> (selection) >= m_trajectory_session.Point_Count ( ) )
  {
    return;
  }

  const size_t point_index = static_cast<size_t> (selection);
  m_teach_point_store.Delete_Point (m_current_model_id, point_index);
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );

  if( m_trajectory_session.Point_Count ( ) > 0 )
  {
    const int next_selection = static_cast<int> (
      std::min (point_index, m_trajectory_session.Point_Count ( ) - 1));
    if( m_teach_point_list_panel )
    {
      m_teach_point_list_panel->Set_Point_Selection (next_selection);
    }
  }

  Update_Trajectory_Status ( );
}

void Robot_Model_Panel::On_Save_Trajectory (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) )
  {
    return;
  }

  if( m_trajectory_session.Point_Count ( ) == 0 )
  {
    if( m_status_text )
    {
      m_status_text->SetLabel ("No trajectory points to save");
    }
    return;
  }

  wxFileDialog dialog (
    this,
    "Save trajectory",
    "",
    "trajectory.csv",
    "CSV files (*.csv)|*.csv|All files (*.*)|*.*",
    wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
  if( dialog.ShowModal ( ) != wxID_OK )
  {
    return;
  }

  std::string error_message;
  const bool saved = robot_model::Save_Joint_Trajectory_Csv (
    std::filesystem::path (dialog.GetPath ( ).ToStdString ( )),
    m_trajectory_session.Points ( ),
    &error_message);
  if( m_status_text )
  {
    m_status_text->SetLabel (
      saved ? "Trajectory saved" :
              wxString::Format ("Save failed: %s", wxString (error_message)));
  }
}

void Robot_Model_Panel::On_Load_Trajectory (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) )
  {
    return;
  }

  wxFileDialog dialog (
    this,
    "Load trajectory",
    "",
    "",
    "CSV files (*.csv)|*.csv|All files (*.*)|*.*",
    wxFD_OPEN | wxFD_FILE_MUST_EXIST);
  if( dialog.ShowModal ( ) != wxID_OK )
  {
    return;
  }

  std::vector<std::array<double, 6>> loaded_points;
  std::string error_message;
  const bool loaded = robot_model::Load_Joint_Trajectory_Csv (
    std::filesystem::path (dialog.GetPath ( ).ToStdString ( )),
    &loaded_points,
    &error_message);
  if( !loaded )
  {
    if( m_status_text )
    {
      m_status_text->SetLabel (
        wxString::Format ("Load failed: %s", wxString (error_message)));
    }
    return;
  }

  m_teach_point_store.Replace_Joint_Points (
    m_current_model_id, loaded_points);
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  Update_Trajectory_Status ( );
  if( m_status_text )
  {
    m_status_text->SetLabel ("Trajectory loaded");
  }
}

void Robot_Model_Panel::On_Play_Trajectory (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) ||
      !m_trajectory_session.Has_Playable_Path ( ) )
  {
    return;
  }

  if( !m_trajectory_session.Start_Playback (TRAJECTORY_FRAME_COUNT) )
  {
    return;
  }

  const auto start_result = Apply_Joint_Input_Angles_To_Sliders (
    m_trajectory_session.Points ( ).front ( ));
  if( !start_result.accepted && start_result.collision.collided )
  {
    m_trajectory_session.Pause ( );
    Set_Joint_Controls_Enabled (true);
    Update_Trajectory_Status ( );
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"轨迹已暂停：起始段，") +
      collision_summary (start_result.collision));
    return;
  }
  Set_Joint_Controls_Enabled (false);
  m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
  Update_Trajectory_Status ( );
}

void Robot_Model_Panel::On_Pause_Resume_Trajectory (wxCommandEvent&)
{
  if( !m_trajectory_session.Is_Active ( ) )
  {
    return;
  }

  if( m_trajectory_timer.IsRunning ( ) )
  {
    m_trajectory_timer.Stop ( );
    m_trajectory_session.Pause ( );
  }
  else if( m_trajectory_session.Is_Paused ( ) )
  {
    m_trajectory_session.Resume ( );
    Set_Joint_Controls_Enabled (false);
    m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
  }

  Update_Trajectory_Status ( );
}

void Robot_Model_Panel::On_Stop_Trajectory (wxCommandEvent&)
{
  Stop_Trajectory_Playback ( );
}

void Robot_Model_Panel::On_Trajectory_Speed_Changed (wxCommandEvent&)
{
  Update_Trajectory_Speed_Label ( );
  if( m_trajectory_timer.IsRunning ( ) )
  {
    m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
  }
}

void Robot_Model_Panel::On_Trajectory_Timer (wxTimerEvent&)
{
  const auto* frame = m_trajectory_session.Step ( );
  if( frame == nullptr )
  {
    Stop_Trajectory_Playback ( );
    return;
  }

  const auto apply_result = Apply_Joint_Input_Angles_To_Sliders (*frame);
  if( !apply_result.accepted && apply_result.collision.collided )
  {
    m_trajectory_timer.Stop ( );
    m_trajectory_session.Pause ( );
    Set_Joint_Controls_Enabled (true);
    Update_Trajectory_Status ( );
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"轨迹已暂停：") +
      collision_summary (apply_result.collision) +
      wxString::FromUTF8 (u8"，已停在最后安全姿态"));
    return;
  }

  if( m_trajectory_session.Is_Finished ( ) )
  {
    Stop_Trajectory_Playback ( );
  }
}

wxPanel* Robot_Model_Panel::Build_Robot_Tool_Page (wxWindow* parent)
{
  auto* panel = new wxScrolledWindow (
    parent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
    wxVSCROLL | wxTAB_TRAVERSAL);
  panel->SetMinSize (wxSize (380, -1));
  panel->SetScrollRate (0, 12);

  auto* operation_title = new wxStaticText (
    panel, wxID_ANY, wxString::FromUTF8 (u8"机械臂操作"));
  m_flange_frame_button = new wxToggleButton (
    panel, wxID_ANY, wxString::FromUTF8 (u8"显示法兰坐标系"));
  m_flange_free_drag_button = new wxToggleButton (
    panel, wxID_ANY, wxString::FromUTF8 (u8"自由拖拽"));
  m_flange_6d_button = new wxToggleButton (
    panel, wxID_ANY, wxString::FromUTF8 (u8"6D 操纵"));
  m_reset_robot_button = new wxButton (
    panel, wxID_ANY, wxString::FromUTF8 (u8"机械臂复位"));
  m_reset_robot_button->Enable (false);
  m_flange_frame_button->Bind (
    wxEVT_TOGGLEBUTTON, &Robot_Model_Panel::On_Toggle_Flange_Frame, this);
  m_flange_free_drag_button->Bind (
    wxEVT_TOGGLEBUTTON,
    &Robot_Model_Panel::On_Toggle_Flange_Free_Drag,
    this);
  m_flange_6d_button->Bind (
    wxEVT_TOGGLEBUTTON, &Robot_Model_Panel::On_Toggle_Flange_6D, this);
  m_reset_robot_button->Bind (
    wxEVT_BUTTON, &Robot_Model_Panel::On_Reset_Robot, this);

  auto* operation_sizer = new wxGridSizer (2, 6, 6);
  operation_sizer->Add (m_flange_frame_button, 1, wxEXPAND);
  operation_sizer->Add (m_flange_free_drag_button, 1, wxEXPAND);
  operation_sizer->Add (m_flange_6d_button, 1, wxEXPAND);
  operation_sizer->Add (m_reset_robot_button, 1, wxEXPAND);

  m_joint_panel = new Joint_Control_Panel (panel);
  m_joint_panel->Set_On_Joint_Changed (
    [this] { Update_Joint_State_From_Sliders ( ); });

  m_cartesian_pose_panel = new Cartesian_Pose_Panel (panel);
  m_cartesian_pose_panel->Set_On_World_Frame_Visibility_Changed (
    [this] (bool visible)
    {
      if( m_view ) m_view->Set_World_Frame_Visible (visible);
    });
  m_cartesian_pose_panel->Set_On_Pose_Changed (
    [this] (const robot_model::XyzabcPose& target_pose)
    {
      Apply_Cartesian_Pose_Target (target_pose);
    });

  m_teach_point_command_panel = new Teach_Point_Command_Panel (panel);
  m_teach_point_command_panel->Set_On_Record (
    [this] { wxCommandEvent event; On_Add_Trajectory_Point (event); });

  m_trajectory_panel = new Trajectory_Control_Panel (
    panel,
    TRAJECTORY_SPEED_DEFAULT_INDEX,
    static_cast<int> (TRAJECTORY_SPEED_SCALES.size ( ) - 1));

  Trajectory_Control_Panel::Callbacks callbacks;
  callbacks.clear_points =
    [this] { wxCommandEvent event; On_Clear_Trajectory_Points (event); };
  callbacks.go_to_point =
    [this] { wxCommandEvent event; On_Go_To_Trajectory_Point (event); };
  callbacks.delete_point =
    [this] { wxCommandEvent event; On_Delete_Trajectory_Point (event); };
  callbacks.save =
    [this] { wxCommandEvent event; On_Save_Trajectory (event); };
  callbacks.load =
    [this] { wxCommandEvent event; On_Load_Trajectory (event); };
  callbacks.play =
    [this] { wxCommandEvent event; On_Play_Trajectory (event); };
  callbacks.pause_resume =
    [this] { wxCommandEvent event; On_Pause_Resume_Trajectory (event); };
  callbacks.stop =
    [this] { wxCommandEvent event; On_Stop_Trajectory (event); };
  callbacks.speed_changed =
    [this] { wxCommandEvent event; On_Trajectory_Speed_Changed (event); };
  m_trajectory_panel->Set_Callbacks (std::move (callbacks));

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (operation_title, 0, wxEXPAND | wxALL, 8);
  sizer->Add (operation_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (m_joint_panel, 0, wxEXPAND | wxTOP, 6);
  sizer->Add (m_cartesian_pose_panel, 0, wxEXPAND | wxTOP, 14);
  sizer->Add (m_teach_point_command_panel, 0, wxEXPAND | wxTOP, 14);
  sizer->Add (m_trajectory_panel, 0, wxEXPAND | wxTOP, 8);
  sizer->AddStretchSpacer (1);
  panel->SetSizer (sizer);
  panel->FitInside ( );
  Update_Trajectory_Speed_Label ( );
  Update_Trajectory_Point_List ( );
  Update_Trajectory_Status ( );

  return panel;
}

void Robot_Model_Panel::Apply_Joint_Limits (
  const robot_model::Robot_Kinematic_Params& params)
{
  if( m_joint_panel == nullptr )
  {
    return;
  }

  const auto home_angles = configured_home_input_angles (params);
  for( size_t i = 0; i < ROBOT_JOINT_COUNT; ++i )
  {
    int min_value = slider_limit_at (params.joint_mins, i, DEFAULT_JOINT_MIN);
    int max_value = slider_limit_at (params.joint_maxs, i, DEFAULT_JOINT_MAX);
    if( min_value > max_value )
    {
      std::swap (min_value, max_value);
    }

    const int neutral_value =
      static_cast<int> (std::lround (home_angles[i]));
    m_joint_panel->Set_Joint_Range_And_Value (
      i, min_value, max_value, neutral_value);
  }

  Update_Joint_State_From_Sliders ( );
}

void Robot_Model_Panel::Update_Joint_State_From_Sliders ( )
{
  if( !m_view || !m_view->Has_Current_Model ( ) || !m_joint_panel ) return;
  const auto joint_state = robot_model::Build_Joint_State_From_Input_Angles (
    m_view->Kinematic_Params ( ), m_joint_panel->Read_Input_Angles ( ));
  const auto apply_result = m_view->Try_Set_Joint_State (joint_state);
  Sync_Joint_Controls_From_State ( );
  if( !apply_result.accepted && apply_result.collision.collided &&
      m_status_text )
  {
    auto status =
      wxString::FromUTF8 (u8"关节运动已阻止：") +
      collision_summary (apply_result.collision);
    if( apply_result.collision.type !=
        robot_model::Robot_Collision_Type::Self_Collision )
    {
      status += wxString::Format (
        wxString::FromUTF8 (u8"，安全距离 %.1f mm"),
        apply_result.clearance_mm);
    }
    m_status_text->SetLabel (status);
  }
  else if( apply_result.accepted && apply_result.recovery_motion &&
           m_status_text )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"已允许脱离碰撞方向的恢复运动"));
  }
  else if( apply_result.accepted && apply_result.collision_checked &&
           m_status_text )
  {
    m_status_text->SetLabel (wxString::FromUTF8 (
      u8"关节运动已应用：碰撞安全检测通过"));
  }
}

void Robot_Model_Panel::Sync_Joint_Controls_From_State ( )
{
  if( !m_view || !m_view->Has_Current_Model ( ) ) return;
  const auto& joint_state = m_view->Joint_State ( );
  if( m_joint_panel )
  {
    m_joint_panel->Set_Input_Angles (joint_state.input_angles_deg);
  }
  for( size_t i = 0; i < joint_state.input_angles_deg.size ( ); ++i )
  {
    Update_Joint_Value_Label (
      i,
      joint_state.input_angles_deg[i],
      joint_state.effective_angles_deg[i]);
  }
  Update_Cartesian_Pose ( );
}

void Robot_Model_Panel::Update_Cartesian_Pose ( )
{
  if( !m_cartesian_pose_panel || !m_view ) return;
  robot_model::Matrix4 world_from_flange = { };
  if( m_view->Get_World_From_Flange (&world_from_flange) )
  {
    m_cartesian_pose_panel->Set_World_From_Flange (world_from_flange);
  }
  else
  {
    m_cartesian_pose_panel->Clear ( );
  }
}

void Robot_Model_Panel::Apply_Cartesian_Pose_Target (
  const robot_model::XyzabcPose& target_pose)
{
  if( !m_view || !m_view->Has_Current_Model ( ) ) return;
  if( Is_Trajectory_Active ( ) ) Stop_Trajectory_Playback ( );

  Select_Display_Page (Main_Display_Page::Robot);
  robot_model::Robot_Pose_IK_Options options;
  options.position_tolerance_mm = 0.001;
  options.orientation_tolerance_deg = 0.001;
  options.damping_mm = 1.0;
  options.max_iterations = 100;
  options.time_budget_ms = 8.0;
  const auto result = m_view->Move_Flange_To_Pose (
    robot_model::Build_Zyx_Pose_Matrix (target_pose),
    options);
  if( result.status == robot_model::Robot_IK_Status::Invalid_Model )
  {
    m_status_text->SetLabel (wxString::FromUTF8 (
      u8"世界坐标位姿控制失败：模型无效"));
    return;
  }

  Sync_Joint_Controls_From_State ( );
  const auto& apply_result = m_view->Last_Joint_State_Apply_Result ( );
  if( !apply_result.accepted && apply_result.collision.collided )
  {
    if( apply_result.state_changed ) Sync_Joint_Controls_From_State ( );
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"世界坐标运动已阻止：") +
      collision_summary (apply_result.collision));
    return;
  }
  m_status_text->SetLabel (wxString::Format (
    result.Converged ( )
      ? wxString::FromUTF8 (
          u8"世界坐标位姿：位置误差 %.2f mm，姿态误差 %.2f°")
      : wxString::FromUTF8 (
          u8"世界坐标目标受限：位置误差 %.2f mm，姿态误差 %.2f°"),
    result.position_error_mm,
    result.orientation_error_deg));
}

void Robot_Model_Panel::Apply_Flange_Drag_Result (
  const robot_model::Robot_Position_IK_Result& result)
{
  if( result.status == robot_model::Robot_IK_Status::Invalid_Model )
  {
    m_status_text->SetLabel (wxString::FromUTF8 (u8"法兰拖拽失败：模型无效"));
    return;
  }

  const auto& apply_result = m_view->Last_Joint_State_Apply_Result ( );
  if( !apply_result.accepted && apply_result.collision.collided )
  {
    if( apply_result.state_changed ) Sync_Joint_Controls_From_State ( );
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"法兰拖拽已阻止：") +
      collision_summary (apply_result.collision));
    return;
  }

  Sync_Joint_Controls_From_State ( );

  m_status_text->SetLabel (wxString::Format (
    result.Converged ( )
      ? wxString::FromUTF8 (u8"法兰拖拽：IK 误差 %.2f mm")
      : wxString::FromUTF8 (u8"法兰目标受限：当前误差 %.2f mm"),
    result.position_error_mm));
}

void Robot_Model_Panel::Apply_Flange_Pose_Drag_Result (
  const robot_model::Robot_Pose_IK_Result& result)
{
  if( result.status == robot_model::Robot_IK_Status::Invalid_Model )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"法兰姿态拖拽失败：模型无效"));
    return;
  }
  const auto& apply_result = m_view->Last_Joint_State_Apply_Result ( );
  if( !apply_result.accepted && apply_result.collision.collided )
  {
    if( apply_result.state_changed ) Sync_Joint_Controls_From_State ( );
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"姿态拖拽已阻止：") +
      collision_summary (apply_result.collision));
    return;
  }
  Sync_Joint_Controls_From_State ( );
  m_status_text->SetLabel (wxString::Format (
    result.Converged ( )
      ? wxString::FromUTF8 (u8"姿态拖拽：位置 %.2f mm，角度 %.2f°")
      : wxString::FromUTF8 (u8"姿态目标受限：位置 %.2f mm，角度 %.2f°"),
    result.position_error_mm,
    result.orientation_error_deg));
}

void Robot_Model_Panel::Update_Joint_Value_Label (size_t index,
                                                 double input_angle,
                                                 double effective_angle)
{
  if( m_joint_panel )
  {
    m_joint_panel->Set_Joint_Value_Label (index, input_angle, effective_angle);
  }
}

std::array<double, 6> Robot_Model_Panel::Read_Joint_Input_Angles ( ) const
{
  if( m_view && m_view->Has_Current_Model ( ) )
  {
    return m_view->Joint_State ( ).input_angles_deg;
  }
  return m_joint_panel ? m_joint_panel->Read_Input_Angles ( )
                       : std::array<double, 6> {};
}

robot_model::Robot_Joint_State_Apply_Result
Robot_Model_Panel::Apply_Joint_Input_Angles_To_Sliders (
  const std::array<double, 6>& input_angles_deg)
{
  if( m_joint_panel )
  {
    m_joint_panel->Set_Input_Angles (input_angles_deg);
  }

  if( !m_view || !m_view->Has_Current_Model ( ) ) return { };
  const auto joint_state = robot_model::Build_Joint_State_From_Input_Angles (
    m_view->Kinematic_Params ( ), input_angles_deg);
  const auto result = m_view->Try_Set_Joint_State (joint_state);
  Sync_Joint_Controls_From_State ( );
  return result;
}

void Robot_Model_Panel::Set_Joint_Controls_Enabled (bool enabled)
{
  if( m_joint_panel )
  {
    m_joint_panel->Set_Joint_Controls_Enabled (enabled);
  }
  if( m_cartesian_pose_panel )
  {
    m_cartesian_pose_panel->Set_Pose_Controls_Enabled (enabled);
  }
  if( m_teach_point_command_panel )
  {
    m_teach_point_command_panel->Set_Record_Enabled (enabled);
  }
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_List_Enabled (enabled);
  }
}

void Robot_Model_Panel::Update_Trajectory_Status ( )
{
  const bool active = Is_Trajectory_Active ( );
  if( m_trajectory_panel == nullptr )
  {
    return;
  }

  wxString suffix = "";
  if( m_trajectory_timer.IsRunning ( ) )
  {
    suffix = " / playing";
  }
  else if( m_trajectory_session.Is_Paused ( ) )
  {
    suffix = " / paused";
  }
  else if( !m_trajectory_session.Has_Playable_Path ( ) )
  {
    suffix = " / need 2";
  }

  m_trajectory_panel->Set_Status_Text (
    wxString::Format ("Points: %zu%s",
                      m_trajectory_session.Point_Count ( ),
                      suffix));
  m_trajectory_panel->Refresh_Command_State (
    active,
    m_trajectory_session.Is_Paused ( ),
    m_trajectory_timer.IsRunning ( ),
    m_trajectory_session.Point_Count ( ),
    Selected_Teach_Point_Index ( ) != wxNOT_FOUND);
}

void Robot_Model_Panel::Update_Trajectory_Speed_Label ( )
{
  if( m_trajectory_panel == nullptr )
  {
    return;
  }

  m_trajectory_panel->Set_Speed_Label (
    wxString::Format ("Speed: %.2gx", Get_Trajectory_Speed_Scale ( )));
}

void Robot_Model_Panel::Update_Trajectory_Point_List ( )
{
  if( !m_teach_point_list_panel ) return;

  std::vector<wxString> labels;
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  labels.reserve (points.size ( ));
  for( const auto& point : points )
  {
    labels.push_back (wxString::FromUTF8 (
      robot_model::Format_Teach_Point_Name (point.id).c_str ( )));
  }
  m_teach_point_list_panel->Set_Point_Names (labels);
}

void Robot_Model_Panel::Sync_Trajectory_From_Teach_Points ( )
{
  std::vector<std::array<double, 6>> joint_points;
  const auto& teach_points = m_teach_point_store.Points (m_current_model_id);
  joint_points.reserve (teach_points.size ( ));
  for( const auto& point : teach_points )
  {
    joint_points.push_back (point.joint_angles_deg);
  }
  m_trajectory_session.Set_Points (std::move (joint_points));
}

int Robot_Model_Panel::Selected_Teach_Point_Index ( ) const
{
  return m_teach_point_list_panel
    ? m_teach_point_list_panel->Selected_Point_Index ( )
    : wxNOT_FOUND;
}

double Robot_Model_Panel::Get_Trajectory_Speed_Scale ( ) const
{
  if( m_trajectory_panel == nullptr )
  {
    return 1.0;
  }

  const int index = std::clamp (
    m_trajectory_panel->Speed_Index ( ),
    0,
    static_cast<int> (TRAJECTORY_SPEED_SCALES.size ( ) - 1));
  return TRAJECTORY_SPEED_SCALES[static_cast<size_t> (index)];
}

int Robot_Model_Panel::Get_Trajectory_Timer_Interval_Ms ( ) const
{
  const double speed_scale = Get_Trajectory_Speed_Scale ( );
  const int interval = static_cast<int> (
    std::lround (static_cast<double> (TRAJECTORY_TIMER_MS) / speed_scale));
  return std::max (interval, 1);
}

bool Robot_Model_Panel::Is_Trajectory_Active ( ) const
{
  return m_trajectory_session.Is_Active ( );
}

void Robot_Model_Panel::Stop_Trajectory_Playback ( )
{
  if( m_trajectory_timer.IsRunning ( ) )
  {
    m_trajectory_timer.Stop ( );
  }

  m_trajectory_session.Stop ( );
  Set_Joint_Controls_Enabled (true);
  Update_Trajectory_Status ( );
}

void Robot_Model_Panel::Resize_Right_Tool (bool collapsed)
{
  if( !m_content_splitter || !m_right_tool_panel ||
      !m_content_splitter->IsSplit ( ) ) return;

  const int total_width = m_content_splitter->GetClientSize ( ).x;
  if( total_width <= RIGHT_TOOL_COLLAPSED_WIDTH ) return;

  if( collapsed )
  {
    const int current_width = m_right_tool_panel->GetSize ( ).x;
    if( current_width > RIGHT_TOOL_COLLAPSED_WIDTH + 40 )
    {
      m_expanded_right_tool_width = current_width;
    }
  }

  const int maximum_right_width = std::max (
    RIGHT_TOOL_COLLAPSED_WIDTH,
    total_width - DISPLAY_MINIMUM_WIDTH);
  const int requested_width = collapsed
    ? RIGHT_TOOL_COLLAPSED_WIDTH
    : std::max (m_expanded_right_tool_width,
                RIGHT_TOOL_DEFAULT_EXPANDED_WIDTH);
  const int right_width = std::clamp (
    requested_width,
    RIGHT_TOOL_COLLAPSED_WIDTH,
    maximum_right_width);
  m_content_splitter->SetSashPosition (total_width - right_width, true);
}

void Robot_Model_Panel::Resize_Teach_Point_List (bool collapsed)
{
  if( !m_workspace_splitter || !m_teach_point_list_panel ||
      !m_workspace_splitter->IsSplit ( ) ) return;

  const int total_width = m_workspace_splitter->GetClientSize ( ).x;
  if( total_width <= TEACH_POINT_COLLAPSED_WIDTH ) return;

  if( collapsed )
  {
    const int current_width = m_teach_point_list_panel->GetSize ( ).x;
    if( current_width > TEACH_POINT_COLLAPSED_WIDTH + 40 )
    {
      m_expanded_teach_point_width = current_width;
    }
  }

  const int maximum_left_width = std::max (
    TEACH_POINT_COLLAPSED_WIDTH,
    total_width - WORKSPACE_MINIMUM_WIDTH);
  const int requested_width = collapsed
    ? TEACH_POINT_COLLAPSED_WIDTH
    : std::max (m_expanded_teach_point_width,
                TEACH_POINT_DEFAULT_EXPANDED_WIDTH);
  const int left_width = std::clamp (
    requested_width,
    TEACH_POINT_COLLAPSED_WIDTH,
    maximum_left_width);
  m_workspace_splitter->SetSashPosition (left_width, true);
}

void Robot_Model_Panel::Load_Model_List ( )
{
  const auto root = robot_model::Find_Robot_Root ( );
  m_models.clear ( );

  if( root.empty ( ) || !std::filesystem::exists (root) )
  {
    m_status_text->SetLabel ("Resource/Robot not found");
    return;
  }

  m_models = robot_model::Scan_Models_In_Directory (root);
  std::sort (
    m_models.begin ( ), m_models.end ( ),
    [] (const auto& lhs, const auto& rhs)
    {
      return lhs.display_name < rhs.display_name;
    });

  if( m_models.empty ( ) )
  {
    m_status_text->SetLabel ("No robot model folders found");
  }
}

void Robot_Model_Panel::Load_Default_Model ( )
{
  const auto default_model = std::find_if (
    m_models.begin ( ), m_models.end ( ),
    [] (const robot_model::Robot_Model_Info& model)
    {
      return model_id (model) == DEFAULT_ROBOT_MODEL_ID;
    });
  if( default_model == m_models.end ( ) )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"默认机械臂 KR10_R1100_2 未找到"));
    if( m_reset_robot_button ) m_reset_robot_button->Enable (false);
    m_right_tool_panel->Set_Robot_Tool_Enabled (false);
    return;
  }

  std::string error_message;
  const auto model_index = static_cast<size_t> (
    std::distance (m_models.begin ( ), default_model));
  if( !Load_Model (model_index, &error_message) )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"默认机械臂模型加载失败"));
    m_right_tool_panel->Set_Robot_Tool_Enabled (false);
  }
}

bool Robot_Model_Panel::Load_Model (
  size_t model_index,
  std::string* error_message)
{
  if( error_message )
  {
    error_message->clear ( );
  }
  if( model_index >= m_models.size ( ) )
  {
    if( error_message )
    {
      *error_message = "Invalid robot model selection";
    }
    return false;
  }

  const auto& model = m_models[model_index];
  if( !validate_model_files (model, error_message) )
  {
    return false;
  }

  if( Is_Trajectory_Active ( ) )
  {
    Stop_Trajectory_Playback ( );
  }

  m_view->Load_Model (model);
  Apply_Joint_Limits (m_view->Kinematic_Params ( ));
  if( m_cartesian_pose_panel )
  {
    m_cartesian_pose_panel->Set_Position_Range (
      cartesian_position_limit_mm (m_view->Kinematic_Params ( )));
  }
  if( m_point_cloud_overlay_toolbar &&
      m_point_cloud_overlay_toolbar->Has_Point_Cloud ( ) )
  {
    m_point_cloud_overlay_toolbar->Attach_Renderer (
      m_view->Scene_Renderer ( ));
  }
  if( m_camera_pose_controller.Is_Visible ( ) )
  {
    m_camera_pose_controller.Attach_Renderer (m_view->Scene_Renderer ( ));
  }

  m_current_model_id = model_id (model);
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  Update_Trajectory_Status ( );
  m_model_name_text->SetLabel (
    wxString::FromUTF8 (u8"当前机械臂：") +
    wxString::FromUTF8 (model.display_name.c_str ( )));
  m_status_text->SetLabel (wxString::FromUTF8 (u8"模型加载成功"));
  if( m_reset_robot_button ) m_reset_robot_button->Enable (true);
  m_right_tool_panel->Set_Robot_Tool_Enabled (true);
  Select_Display_Page (Main_Display_Page::Robot);
  return true;
}

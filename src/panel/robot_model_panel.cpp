#include "robot_model_panel.h"

#include "camera_control_panel.h"
#include "camera_image_view.h"
#include "camera_service.h"
#include "camera_2d_control_panel.h"
#include "camera_2d_image_view.h"
#include "camera_2d_template_panel.h"
#include "robot_joint_state_builder.h"
#include "robot_model_config_dialog.h"
#include "robot_model_repository.h"
#include "robot_progress_io.h"
#include "tool_coordinate_repository.h"
#include "right_tool_panel.h"
#include "net_panel.h"
#include "flow_panel.h"
#include "point_cloud_view.h"
#include "point_cloud_overlay_toolbar.h"
#include "teach_point_command_panel.h"
#include "teach_point_list_panel.h"
#include "template_configuration_repository.h"
#include "tool_panel.h"
#include "tool_visualization_repository.h"

#include <wx/button.h>
#include <wx/choice.h>
#include <wx/choicdlg.h>
#include <wx/filedlg.h>
#include <wx/filename.h>
#include <wx/grid.h>
#include <wx/msgdlg.h>
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/simplebook.h>
#include <wx/slider.h>
#include <wx/splitter.h>
#include <wx/tglbtn.h>
#include <wx/utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <unordered_map>
#include <utility>

wxDEFINE_EVENT(wxEVT_KUKA_MODEL_STATE, wxThreadEvent);
wxDEFINE_EVENT(wxEVT_KUKA_SERVICE_STATUS, wxThreadEvent);

namespace
{
constexpr int DEFAULT_JOINT_MIN = -180;
constexpr int DEFAULT_JOINT_MAX = 180;
constexpr int TRAJECTORY_FRAME_COUNT = 120;
constexpr int TRAJECTORY_TIMER_MS = 16;
constexpr int TRAJECTORY_SPEED_DEFAULT_CM_PER_SECOND = 100;
constexpr std::size_t ROBOT_JOINT_COUNT = 6;
constexpr int RIGHT_TOOL_COLLAPSED_WIDTH = 72;
constexpr int DISPLAY_MINIMUM_WIDTH = 300;
constexpr int TEACH_POINT_COLLAPSED_WIDTH = 38;
constexpr int TEACH_POINT_DEFAULT_EXPANDED_WIDTH = 240;
constexpr int WORKSPACE_MINIMUM_WIDTH = 500;
constexpr const char* DEFAULT_ROBOT_MODEL_ID = "KR10_R1100_2";

std::size_t frame_count_for_one_meter_per_second (
  const robot_model::XyzabcPose& start,
  const robot_model::XyzabcPose& target)
{
  const double dx = target[0] - start[0];
  const double dy = target[1] - start[1];
  const double dz = target[2] - start[2];
  const double distance_mm = std::sqrt (dx * dx + dy * dy + dz * dz);
  if( distance_mm <= 0.0 ) return TRAJECTORY_FRAME_COUNT;
  const double millimeters_per_tick =
    1000.0 * static_cast<double> (TRAJECTORY_TIMER_MS) / 1000.0;
  return std::max<std::size_t> (
    2,
    static_cast<std::size_t> (
      std::ceil (distance_mm / millimeters_per_tick)) + 1);
}

wxString teach_point_type_label (
  robot_model::Robot_Teach_Point_Type type)
{
  switch( type )
  {
    case robot_model::Robot_Teach_Point_Type::Transition:
      return wxString::FromUTF8 (u8"过渡点");
    case robot_model::Robot_Teach_Point_Type::Wait:
      return wxString::FromUTF8 (u8"等待点");
    case robot_model::Robot_Teach_Point_Type::Motion:
    default:
      return wxString::FromUTF8 (u8"运动点");
  }
}

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
  Camera_2D_Service& camera_2d_service,
  Camera_2D_Cross_Template_Service& camera_2d_template_service,
  wxWindowID id)
  : wxPanel (parent, id)
{
  m_kuka_service = std::make_shared<kuka::Robot_Service> ( );
  Bind (
    wxEVT_KUKA_MODEL_STATE,
    &Robot_Model_Panel::On_Kuka_Model_State,
    this);
  Bind (
    wxEVT_KUKA_SERVICE_STATUS,
    &Robot_Model_Panel::On_Kuka_Service_Status,
    this);

  kuka::Service_Observer kuka_observer;
  kuka_observer.robot_state =
    [this] (const kuka::Robot_State& state, std::uint64_t revision)
    {
      auto* event = new wxThreadEvent (wxEVT_KUKA_MODEL_STATE);
      event->SetPayload (std::make_pair (state, revision));
      wxQueueEvent (this, event);
    };
  kuka_observer.service_status =
    [this] (const kuka::Service_Status& status)
    {
      auto* event = new wxThreadEvent (wxEVT_KUKA_SERVICE_STATUS);
      event->SetPayload (status);
      wxQueueEvent (this, event);
    };
  m_kuka_observer_token =
    m_kuka_service->Subscribe (std::move (kuka_observer));

  auto* title = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (u8"显示区域"));
  m_model_name_text = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (u8"当前机械臂：未加载"));
  m_robot_display_button = new wxToggleButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"机械臂"));
  m_camera_display_button = new wxToggleButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"相机图像"));
  m_camera_2d_display_button = new wxToggleButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"2D图像"));
  m_point_cloud_display_button = new wxToggleButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"点云"));
  m_robot_display_button->Bind (
    wxEVT_TOGGLEBUTTON, &Robot_Model_Panel::On_Robot_Display, this);
  m_camera_display_button->Bind (
    wxEVT_TOGGLEBUTTON, &Robot_Model_Panel::On_Camera_Image_Display, this);
  m_camera_2d_display_button->Bind (
    wxEVT_TOGGLEBUTTON, &Robot_Model_Panel::On_Camera_2D_Image_Display, this);
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
    [this] { On_Teach_Point_Selection_Changed ( ); });
  m_teach_point_list_panel->Set_On_Pose_Coordinate_Changed (
    [this] (int selection)
    {
      On_Teach_Pose_Coordinate_Changed (selection);
    });
  m_teach_point_list_panel->Set_On_Collapsed_Changed (
    [this] (bool collapsed) { Resize_Teach_Point_List (collapsed); });
  m_teach_point_list_panel->Set_On_Bind_Cloud_Template (
    [this] (int point_index)
    {
      if( point_index >= 0 )
      {
        Bind_Template_To_Teach_Point_Cloud (
          static_cast<std::size_t> (point_index));
      }
    });
  m_teach_point_list_panel->Set_On_Unbind_Cloud_Template (
    [this] (int point_index)
    {
      if( point_index >= 0 )
      {
        Unbind_Template_From_Teach_Point_Cloud (
          static_cast<std::size_t> (point_index));
      }
    });

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
      if( !success && m_waiting_for_playback_cloud &&
          m_trajectory_session.Is_Active ( ) )
      {
        m_trajectory_timer.Stop ( );
        m_trajectory_session.Pause ( );
        m_waiting_for_playback_cloud = false;
        m_playback_cloud_switch_blocked = true;
        Set_Joint_Controls_Enabled (true);
        Update_Trajectory_Status ( );
      }
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
  m_camera_2d_image_view = new Camera_2D_Image_View (
    m_display_book, camera_2d_service, camera_2d_template_service);
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
  m_display_book->AddPage (m_camera_2d_image_view, wxEmptyString, false);
  m_right_tool_panel = new Right_Tool_Panel (m_content_splitter);
  m_right_tool_panel->Set_On_Width_Changed (
    [this] (int width) { Resize_Right_Tool (width); });

  m_tcp_panel = new Net_Panel (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Tcp),
    m_kuka_service);
  m_flow_panel = new Flow_Panel (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Flow));
  m_camera_control_panel = new Camera_Control_Panel (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Camera),
    camera_service);
  m_camera_2d_control_panel = new Camera_2D_Control_Panel (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Camera2D),
    camera_2d_service);
  m_camera_2d_control_panel->Set_On_Show_Image (
    [this] { Select_Display_Page (Main_Display_Page::Camera_2D_Image); });
  m_camera_2d_template_panel = new Camera_2D_Template_Panel (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Template2D),
    camera_2d_service,
    camera_2d_template_service,
    *m_camera_2d_image_view);
  m_camera_2d_template_panel->Set_On_Show_Image (
    [this] { Select_Display_Page (Main_Display_Page::Camera_2D_Image); });
  m_point_cloud_overlay_toolbar = new Point_Cloud_Overlay_Toolbar (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::PointCloud),
    camera_service,
    std::move (overlay_callbacks));
  m_tool_panel = new Tool_Panel (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Tool));
  m_tool_panel->Set_On_Configuration_Changed (
    [this] (
      const robot_model::Tool_Visualization_Configuration& configuration)
    {
      std::string error_message;
      if( !robot_model::Save_Tool_Visualization_Configuration (
            robot_model::Tool_Visualization_Config_Path ( ),
            configuration,
            &error_message) )
      {
        if( m_status_text )
        {
          m_status_text->SetLabel (
            wxString::FromUTF8 (error_message.c_str ( )));
        }
        return;
      }
      m_tool_visualization_configuration = configuration;
      Apply_Tool_Visualization ( );
      if( m_status_text )
      {
        m_status_text->SetLabel (
          wxString::FromUTF8 (u8"FOV 设置已应用"));
      }
    });
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
    if( m_interaction_coordinate_choice )
      m_interaction_coordinate_choice->Enable (!dragging);
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
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Robot));
  auto* teach_tool_page = Build_Teach_Tool_Page (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Teach));
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Robot, robot_tool_page);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Teach, teach_tool_page);
  m_right_tool_panel->Add_Page (Right_Tool_Page::Tcp, m_tcp_panel);
  m_right_tool_panel->Add_Page (Right_Tool_Page::Flow, m_flow_panel);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Camera, m_camera_control_panel);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Camera2D, m_camera_2d_control_panel);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Template2D, m_camera_2d_template_panel);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::PointCloud, m_point_cloud_overlay_toolbar);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Tool, m_tool_panel);
  m_camera_control_panel->Set_On_Availability_Changed (
    [this] (bool enabled)
    {
      m_right_tool_panel->Set_Camera_Tool_Enabled (enabled);
      if( m_point_cloud_overlay_toolbar )
      {
        m_point_cloud_overlay_toolbar->Set_Camera_Connected (enabled);
      }
    });
  m_camera_2d_control_panel->Set_On_Availability_Changed (
    [this] (bool enabled)
    {
      if( m_right_tool_panel )
        m_right_tool_panel->Set_Template_2D_Tool_Enabled (enabled);
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
    m_camera_2d_display_button, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
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

  std::string tool_error;
  if( !robot_model::Load_Tool_Coordinate_Configuration (
        robot_model::Tool_Coordinate_Config_Path ( ),
        &m_tool_configuration,
        &tool_error) )
  {
    m_tool_configuration =
      robot_model::Default_Tool_Coordinate_Configuration ( );
  }
  Apply_Active_Tool ( );

  std::string tool_visualization_error;
  if( !robot_model::Load_Tool_Visualization_Configuration (
        robot_model::Tool_Visualization_Config_Path ( ),
        &m_tool_visualization_configuration,
        &tool_visualization_error) )
  {
    m_tool_visualization_configuration = { };
  }
  if( m_tool_panel )
  {
    m_tool_panel->Set_Configuration (
      m_tool_visualization_configuration);
  }
  Apply_Tool_Visualization ( );

  Load_Model_List ( );
  std::string connection_error;
  if( !kuka::Load_Connection_Config (
        kuka::Connection_Config_Path ( ),
        &m_kuka_connection_configuration,
        &connection_error) )
  {
    m_kuka_connection_configuration = { };
    if( m_status_text )
      m_status_text->SetLabel (wxString::FromUTF8 (connection_error.c_str ( )));
  }
  Load_Default_Model ( );
  Refresh_Kuka_Status_Table ( );
}

Robot_Model_Panel::~Robot_Model_Panel()
{
  if( m_kuka_service )
  {
    m_kuka_service->Unsubscribe (m_kuka_observer_token);
    m_kuka_observer_token = 0;
    m_kuka_service->Disconnect ( );
  }
  DeletePendingEvents ( );
  Unbind (
    wxEVT_KUKA_MODEL_STATE,
    &Robot_Model_Panel::On_Kuka_Model_State,
    this);
  Unbind (
    wxEVT_KUKA_SERVICE_STATUS,
    &Robot_Model_Panel::On_Kuka_Service_Status,
    this);
}

void Robot_Model_Panel::On_Kuka_Model_State(wxThreadEvent &event)
{
  const auto payload =
    event.GetPayload<std::pair<kuka::Robot_State, std::uint64_t>> ( );
  if( payload.second <= m_kuka_state_revision ) return;
  m_kuka_state_revision = payload.second;
  Apply_Kuka_Actual_State (payload.first);
}

void Robot_Model_Panel::On_Kuka_Service_Status(wxThreadEvent &event)
{
  Refresh_Kuka_Command_Controls (
    event.GetPayload<kuka::Service_Status> ( ));
}

void Robot_Model_Panel::Apply_Kuka_Actual_State(
  const kuka::Robot_State &state)
{
  m_kuka_latest_state = state;
  m_kuka_has_latest_state = true;
  Refresh_Kuka_Status_Table ( );
  if( !m_view || !m_view->Has_Current_Model ( ) ) return;

  const auto joint_state = robot_model::Build_Joint_State_From_Input_Angles (
    m_view->Kinematic_Params ( ), state.axis);
  // This is measured hardware state, not a proposed target. Always display
  // it even if the local collision guard would reject a commanded move.
  m_view->Set_Joint_State (joint_state);
  Sync_Joint_Controls_From_State ( );

  if( m_status_text )
  {
    m_status_text->SetLabel (wxString::Format (
      "KUKA actual state synchronized: "
      "A=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
      state.axis[0], state.axis[1], state.axis[2],
      state.axis[3], state.axis[4], state.axis[5]));
  }
}

void Robot_Model_Panel::Refresh_Kuka_Command_Controls(
  const kuka::Service_Status &status)
{
  m_kuka_service_status = status;
  if( status.state == kuka::Control_State::Disconnected )
    m_kuka_has_latest_state = false;
  if( status.state != kuka::Control_State::Disconnected ||
      !status.detail.empty ( ) )
    m_kuka_connecting = false;
  const bool ready = status.state == kuka::Control_State::Ready;
  const bool connected =
    status.state != kuka::Control_State::Disconnected;
  const bool busy =
    status.state == kuka::Control_State::Command_Sent ||
    status.state == kuka::Control_State::Running ||
    status.state == kuka::Control_State::Completed;

  if( m_kuka_move_joint_button ) m_kuka_move_joint_button->Enable (ready);
  if( m_kuka_move_ptp_button ) m_kuka_move_ptp_button->Enable (ready);
  if( m_kuka_move_linear_button ) m_kuka_move_linear_button->Enable (ready);
  if( m_kuka_sync_button ) m_kuka_sync_button->Enable (connected && !busy);
  if( m_kuka_stop_button ) m_kuka_stop_button->Enable (connected);
  if( m_kuka_ptp_velocity_slider )
    m_kuka_ptp_velocity_slider->Enable (!busy);
  if( m_kuka_linear_velocity_slider )
    m_kuka_linear_velocity_slider->Enable (!busy);
  if( m_kuka_acceleration_slider )
    m_kuka_acceleration_slider->Enable (!busy);
  if( m_kuka_connect_button )
  {
    m_kuka_connect_button->SetLabel (
      connected ? "Disconnect" : (m_kuka_connecting ? "Connecting..." : "Connect"));
    m_kuka_connect_button->Enable (!m_kuka_connecting || connected);
  }
  if( m_joint_panel ) m_joint_panel->Set_Joint_Controls_Enabled (!busy);
  if( m_cartesian_pose_panel )
    m_cartesian_pose_panel->Set_Pose_Controls_Enabled (!busy);

  if( m_kuka_status_text )
  {
    wxString label =
      "KUKA: " + wxString::FromUTF8 (kuka::To_String (status.state));
    if( status.active_sequence != 0 )
      label += wxString::Format ("  seq=%u", status.active_sequence);
    if( !status.detail.empty ( ) )
      label += "  " + wxString::FromUTF8 (status.detail);
    m_kuka_status_text->SetLabel (label);
  }
  Refresh_Kuka_Status_Table ( );
}

void Robot_Model_Panel::Refresh_Kuka_Status_Table ( )
{
  if( !m_kuka_status_table ) return;
  const bool connected =
    m_kuka_service && m_kuka_service->Is_Connected ( );
  const auto set_value = [this] (long row, const wxString& value)
  {
    m_kuka_status_table->SetCellValue (
      static_cast<int> (row), 1, value);
  };
  set_value (0, connected ? "Connected" :
    (m_kuka_connecting ? "Connecting" : "Disconnected"));
  set_value (
    1,
    wxString::FromUTF8 (kuka::To_String (m_kuka_service_status.state)));
}

void Robot_Model_Panel::On_Kuka_Connect (wxCommandEvent&)
{
  if( !m_kuka_service ) return;
  if( m_kuka_service->Is_Connected ( ) || m_kuka_connecting )
  {
    m_kuka_connecting = false;
    m_kuka_service->Disconnect ( );
    Refresh_Kuka_Command_Controls (m_kuka_service->Status ( ));
    return;
  }

  std::string error_message;
  if( !Load_Bound_Robot_Model (&error_message) )
  {
    m_status_text->SetLabel (
      "KUKA connect failed: " + wxString::FromUTF8 (error_message.c_str ( )));
    return;
  }

  try
  {
    m_kuka_connecting = true;
    Refresh_Kuka_Command_Controls (m_kuka_service->Status ( ));
    m_kuka_service->Connect (
      m_kuka_connection_configuration.host,
      m_kuka_connection_configuration.port);
  }
  catch( const std::exception& error )
  {
    m_kuka_connecting = false;
    m_status_text->SetLabel (
      "KUKA connect failed: " + wxString::FromUTF8 (error.what ( )));
    Refresh_Kuka_Command_Controls (m_kuka_service->Status ( ));
  }
}

void Robot_Model_Panel::On_Robot_Display (wxCommandEvent&)
{
  Select_Display_Page (Main_Display_Page::Robot);
}

void Robot_Model_Panel::On_Camera_Image_Display (wxCommandEvent&)
{
  Select_Display_Page (Main_Display_Page::Camera_Image);
}

void Robot_Model_Panel::On_Camera_2D_Image_Display (wxCommandEvent&)
{
  Select_Display_Page (Main_Display_Page::Camera_2D_Image);
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

  const auto& params = m_view->Kinematic_Params ( );
  if( params.has_home_pose )
  {
    // Seed pose IK with the configured reset joint solution so reset remains
    // deterministic and converges to the precise Cartesian target.
    Apply_Joint_Input_Angles_To_Sliders (
      configured_home_input_angles (params));
    return Apply_Cartesian_Pose_Target (params.home_pose_xyzabc);
  }

  Apply_Joint_Input_Angles_To_Sliders (
    configured_home_input_angles (params));
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
  if( m_camera_2d_display_button )
  {
    m_camera_2d_display_button->SetValue (
      m_display_page == Main_Display_Page::Camera_2D_Image);
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
    m_current_model_id,
    m_tool_configuration,
    m_kuka_connection_configuration);
  if( dialog.ShowModal ( ) != wxID_OK )
  {
    return;
  }

  if( dialog.Tool_Apply_Requested ( ) )
  {
    std::string error_message;
    if( !robot_model::Save_Tool_Coordinate_Configuration (
          robot_model::Tool_Coordinate_Config_Path ( ),
          dialog.Tool_Configuration ( ),
          &error_message) )
    {
      wxMessageBox (
        wxString::FromUTF8 (error_message.c_str ( )),
        wxString::FromUTF8 (u8"工具坐标保存失败"),
        wxOK | wxICON_ERROR,
        parent ? parent : this);
      return;
    }
    m_tool_configuration = dialog.Tool_Configuration ( );
    Invalidate_Completed_Progress ( );
    Apply_Active_Tool ( );
    Update_Cartesian_Pose ( );
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"当前工具坐标：") +
      wxString::FromUTF8 (Active_Tool ( ).name.c_str ( )));
    return;
  }

  if( dialog.Connection_Save_Requested ( ) )
  {
    std::string error_message;
    if( !kuka::Save_Connection_Config (
          kuka::Connection_Config_Path ( ),
          dialog.Connection_Configuration ( ),
          &error_message) )
    {
      wxMessageBox (
        wxString::FromUTF8 (error_message.c_str ( )),
        "Connection settings",
        wxOK | wxICON_ERROR,
        parent ? parent : this);
      return;
    }
    if( m_kuka_service && m_kuka_service->Is_Connected ( ) )
      m_kuka_service->Disconnect ( );
    m_kuka_connection_configuration =
      dialog.Connection_Configuration ( );
    if( !Load_Bound_Robot_Model (&error_message) )
    {
      wxMessageBox (
        wxString::FromUTF8 (error_message.c_str ( )),
        "Bound robot model",
        wxOK | wxICON_ERROR,
        parent ? parent : this);
      return;
    }
    Refresh_Kuka_Command_Controls (m_kuka_service->Status ( ));
    m_status_text->SetLabel ("Robot connection settings saved");
    return;
  }

  if( dialog.Model_Load_Requested ( ) )
  {
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
}

void Robot_Model_Panel::Refresh_Template_Configuration ( )
{
  if( m_point_cloud_overlay_toolbar )
  {
    m_point_cloud_overlay_toolbar->Refresh_Template_Configuration ( );
  }
}

void Robot_Model_Panel::On_Add_Trajectory_Point (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) || m_current_model_id.empty ( ) || !m_view )
  {
    return;
  }

  std::array<double, 6> joint_angles = { };
  robot_model::XyzabcPose world_pose = { };
  if( !Read_Current_Teach_Point (&joint_angles, &world_pose) ) return;
  std::string point_cloud_path;
  std::string point_cloud_name;
  robot_model::Tool_Coordinate_Profile coordinate;
  robot_model::Template_Profile template_profile;
  if( !Capture_Current_Teach_Bindings (
        &point_cloud_path, &point_cloud_name, &coordinate,
        &template_profile) ) return;
  const auto& point = m_teach_point_store.Add_Point (
    m_current_model_id,
    joint_angles,
    world_pose,
    point_cloud_path,
    coordinate.id,
    coordinate.name,
    coordinate.flange_from_tool_pose,
    point_cloud_name,
      m_teach_point_command_panel
      ? m_teach_point_command_panel->Selected_Point_Type ( )
      : robot_model::Robot_Teach_Point_Type::Motion,
    template_profile.id,
    template_profile.name,
    template_profile.reference_pose);
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Point_Selection (
      static_cast<int> (
        m_teach_point_store.Point_Count (m_current_model_id) - 1));
    Update_Teach_Point_Details ( );
  }
  Set_Progress_Dirty (true);
  Update_Trajectory_Status ( );
  m_status_text->SetLabel (wxString::FromUTF8 (u8"已记录示教点：") +
    wxString::FromUTF8 (
      robot_model::Format_Teach_Point_Name (point.id).c_str ( )));
}

void Robot_Model_Panel::On_Update_Teach_Point (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) ) return;
  const int selection = Selected_Teach_Point_Index ( );
  if( selection == wxNOT_FOUND ) return;

  std::array<double, 6> joint_angles = { };
  robot_model::XyzabcPose world_pose = { };
  if( !Read_Current_Teach_Point (&joint_angles, &world_pose) ) return;
  std::string point_cloud_path;
  std::string point_cloud_name;
  robot_model::Tool_Coordinate_Profile coordinate;
  robot_model::Template_Profile template_profile;
  if( !Capture_Current_Teach_Bindings (
        &point_cloud_path, &point_cloud_name, &coordinate,
        &template_profile) ) return;
  if( !m_teach_point_store.Update_Point (
        m_current_model_id,
        static_cast<std::size_t> (selection),
        joint_angles,
        world_pose,
        point_cloud_path,
        coordinate.id,
        coordinate.name,
        coordinate.flange_from_tool_pose,
        point_cloud_name,
        m_teach_point_command_panel
          ? m_teach_point_command_panel->Selected_Point_Type ( )
          : robot_model::Robot_Teach_Point_Type::Motion,
        template_profile.id,
        template_profile.name,
        template_profile.reference_pose) )
  {
    return;
  }
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  Set_Progress_Dirty (true);
  Update_Trajectory_Status ( );
  m_status_text->SetLabel (
    wxString::FromUTF8 (u8"当前示教点已更新"));
}

void Robot_Model_Panel::On_Insert_Teach_Point (bool before)
{
  if( Is_Trajectory_Active ( ) ) return;
  const int selection = Selected_Teach_Point_Index ( );
  if( selection == wxNOT_FOUND ) return;

  std::array<double, 6> joint_angles = { };
  robot_model::XyzabcPose world_pose = { };
  if( !Read_Current_Teach_Point (&joint_angles, &world_pose) ) return;
  std::string point_cloud_path;
  std::string point_cloud_name;
  robot_model::Tool_Coordinate_Profile coordinate;
  robot_model::Template_Profile template_profile;
  if( !Capture_Current_Teach_Bindings (
        &point_cloud_path, &point_cloud_name, &coordinate,
        &template_profile) ) return;
  const std::size_t insertion_index =
    static_cast<std::size_t> (selection) + (before ? 0 : 1);
  const auto& point = m_teach_point_store.Insert_Point (
    m_current_model_id,
    insertion_index,
    joint_angles,
    world_pose,
    point_cloud_path,
    coordinate.id,
    coordinate.name,
    coordinate.flange_from_tool_pose,
    point_cloud_name,
      m_teach_point_command_panel
      ? m_teach_point_command_panel->Selected_Point_Type ( )
      : robot_model::Robot_Teach_Point_Type::Motion,
    template_profile.id,
    template_profile.name,
    template_profile.reference_pose);
  const std::size_t point_id = point.id;
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Point_Selection (
      static_cast<int> (insertion_index));
    Update_Teach_Point_Details ( );
  }
  Set_Progress_Dirty (true);
  Update_Trajectory_Status ( );
  m_status_text->SetLabel (
    wxString::FromUTF8 (u8"已插入示教点：") +
    wxString::FromUTF8 (
      robot_model::Format_Teach_Point_Name (point_id).c_str ( )));
}

void Robot_Model_Panel::On_Clear_Trajectory_Points (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) )
  {
    return;
  }

  if( m_teach_point_store.Point_Count (m_current_model_id) == 0 ) return;
  if( wxMessageBox (
        wxString::FromUTF8 (u8"确定清空当前 Progress 吗？"),
        wxString::FromUTF8 (u8"清空 Progress"),
        wxYES_NO | wxNO_DEFAULT | wxICON_WARNING,
        this) != wxYES )
  {
    return;
  }
  m_teach_point_store.Clear_Points (m_current_model_id);
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  Set_Progress_Dirty (true);
  Update_Trajectory_Status ( );
}

void Robot_Model_Panel::On_Go_To_Trajectory_Point (wxCommandEvent&)
{
  const int selection = Selected_Teach_Point_Index ( );
  if( selection == wxNOT_FOUND ||
      selection < 0 ||
      static_cast<size_t> (selection) >= m_trajectory_session.Point_Count ( ) )
  {
    return;
  }
  Start_Go_To_Teach_Point (
    static_cast<std::size_t> (selection), true);
}

bool Robot_Model_Panel::Start_Go_To_Teach_Point (
  std::size_t selection,
  bool apply_bindings)
{
  if( Is_Trajectory_Active ( ) ||
      Get_Trajectory_Speed_Mps ( ) <= 0.0 ||
      selection >= m_trajectory_session.Point_Count ( ) )
  {
    return false;
  }

  if( apply_bindings &&
      !Apply_Teach_Point_Bindings_Keeping_Display (selection) )
  {
    return false;
  }
  const auto start_angles = Read_Joint_Input_Angles ( );
  std::array<double, 6> ignored_angles = { };
  robot_model::XyzabcPose start_pose = { };
  std::size_t frame_count = TRAJECTORY_FRAME_COUNT;
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( Read_Current_Teach_Point (&ignored_angles, &start_pose) &&
      selection < points.size ( ) &&
      points[selection].has_world_pose )
  {
    frame_count = frame_count_for_one_meter_per_second (
      start_pose,
      points[selection].world_pose);
  }
  if( !m_trajectory_session.Start_Go_To (
        start_angles,
        selection,
        frame_count) )
  {
    return false;
  }

  Set_Joint_Controls_Enabled (false);
  m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
  Update_Trajectory_Status ( );
  return true;
}

void Robot_Model_Panel::On_Step_To_Next_Teach_Point ( )
{
  if( Is_Trajectory_Active ( ) ) return;

  const int current_selection = Selected_Teach_Point_Index ( );
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( current_selection == wxNOT_FOUND || current_selection < 0 )
  {
    if( m_status_text )
      m_status_text->SetLabel (
        wxString::FromUTF8 (u8"请先选择一个示教点"));
    return;
  }

  const auto next_index =
    static_cast<std::size_t> (current_selection) + 1;
  if( next_index >= points.size ( ) )
  {
    if( m_status_text )
      m_status_text->SetLabel (
        wxString::FromUTF8 (u8"当前已是最后一个示教点"));
    return;
  }

  const bool hardware_connected =
    m_kuka_service && m_kuka_service->Is_Connected ( );
  if( hardware_connected && !m_kuka_service->Can_Move ( ) )
  {
    if( m_status_text )
      m_status_text->SetLabel (
        wxString::FromUTF8 (
          u8"KUKA 当前未就绪，请等待当前指令完成或先同步状态"));
    return;
  }

  if( !Apply_Teach_Point_Bindings_Keeping_Display (next_index) )
    return;

  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Point_Selection (
      static_cast<int> (next_index));
    Update_Teach_Point_Details ( );
  }
  Update_Trajectory_Status ( );

  if( hardware_connected )
  {
    try
    {
      kuka::Joint_Motion_Options options;
      options.velocity_percent = static_cast<double> (
        m_kuka_ptp_velocity_slider
          ? m_kuka_ptp_velocity_slider->GetValue ( )
          : 20);
      options.acceleration_percent = static_cast<double> (
        m_kuka_acceleration_slider
          ? m_kuka_acceleration_slider->GetValue ( )
          : 20);
      const auto sequence = m_kuka_service->Move_Joint (
        points[next_index].joint_angles_deg, options);
      if( m_status_text )
      {
        m_status_text->SetLabel (wxString::Format (
          wxString::FromUTF8 (
            u8"步进至 P[%zu]：KUKA MOVEJ 已发送，sequence=%u"),
          points[next_index].id,
          sequence));
      }
    }
    catch( const std::exception& error )
    {
      if( m_status_text )
      {
        m_status_text->SetLabel (
          wxString::FromUTF8 (u8"步进指令发送失败：") +
          wxString::FromUTF8 (error.what ( )));
      }
    }
    return;
  }

  Start_Go_To_Teach_Point (next_index, false);
}

void Robot_Model_Panel::On_Delete_Trajectory_Point (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) )
  {
    return;
  }

  const auto selections = Selected_Teach_Point_Indices ( );
  if( selections.empty ( ) )
  {
    return;
  }

  if( selections.size ( ) > 1 )
  {
    const wxString prompt = wxString::Format (
      wxString::FromUTF8 (u8"确定删除选中的 %zu 个示教点吗？"),
      selections.size ( ));
    if( wxMessageBox (
          prompt,
          wxString::FromUTF8 (u8"批量删除"),
          wxYES_NO | wxNO_DEFAULT | wxICON_WARNING,
          this) != wxYES )
    {
      return;
    }
  }

  std::vector<std::size_t> point_indices;
  point_indices.reserve (selections.size ( ));
  for( const int selection : selections )
  {
    if( selection >= 0 )
      point_indices.push_back (static_cast<std::size_t> (selection));
  }
  const std::size_t next_index = *std::min_element (
    point_indices.begin ( ), point_indices.end ( ));
  m_teach_point_store.Delete_Points (
    m_current_model_id, point_indices);
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  Set_Progress_Dirty (true);

  if( m_trajectory_session.Point_Count ( ) > 0 )
  {
    const int next_selection = static_cast<int> (
      std::min (next_index, m_trajectory_session.Point_Count ( ) - 1));
    if( m_teach_point_list_panel )
    {
      m_teach_point_list_panel->Set_Point_Selection (next_selection);
      Update_Teach_Point_Details ( );
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
      m_status_text->SetLabel (
        wxString::FromUTF8 (u8"没有可保存的示教点"));
    }
    return;
  }

  wxFileDialog dialog (
    this,
    wxString::FromUTF8 (u8"保存 Progress"),
    m_last_progress_directory,
    "progress.xml",
    "Progress XML (*.xml)|*.xml|All files (*.*)|*.*",
    wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
  if( dialog.ShowModal ( ) != wxID_OK )
  {
    return;
  }

  robot_model::Robot_Progress_File progress;
  progress.robot_model_id = m_current_model_id;
  progress.points = m_teach_point_store.Points (m_current_model_id);
  std::string error_message;
  const bool saved = robot_model::Save_Robot_Progress (
    std::filesystem::path (dialog.GetPath ( ).ToStdWstring ( )),
    progress,
    &error_message);
  if( saved )
  {
    m_last_progress_directory =
      wxFileName (dialog.GetPath ( )).GetPath ( );
    Set_Progress_Dirty (false);
  }
  if( m_status_text )
  {
    m_status_text->SetLabel (
      saved
        ? wxString::FromUTF8 (u8"Progress 已保存")
        : wxString::FromUTF8 (u8"保存失败：") +
          wxString::FromUTF8 (error_message.c_str ( )));
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
    wxString::FromUTF8 (u8"加载 Progress"),
    m_last_progress_directory,
    "",
    "Progress XML (*.xml)|*.xml|All files (*.*)|*.*",
    wxFD_OPEN | wxFD_FILE_MUST_EXIST);
  if( dialog.ShowModal ( ) != wxID_OK )
  {
    return;
  }

  if( m_teach_point_store.Point_Count (m_current_model_id) > 0 &&
      wxMessageBox (
        wxString::FromUTF8 (
          u8"加载会替换当前 Progress，是否继续？"),
        wxString::FromUTF8 (u8"加载 Progress"),
        wxYES_NO | wxNO_DEFAULT | wxICON_WARNING,
        this) != wxYES )
  {
    return;
  }

  robot_model::Robot_Progress_File progress;
  std::string error_message;
  const bool loaded = robot_model::Load_Robot_Progress (
    std::filesystem::path (dialog.GetPath ( ).ToStdWstring ( )),
    &progress,
    &error_message);
  if( !loaded )
  {
    if( m_status_text )
    {
      m_status_text->SetLabel (
        wxString::FromUTF8 (u8"加载失败：") +
        wxString::FromUTF8 (error_message.c_str ( )));
    }
    return;
  }

  if( progress.robot_model_id != m_current_model_id )
  {
    wxMessageBox (
      wxString::FromUTF8 (u8"Progress 对应的机械臂模型为：") +
      wxString::FromUTF8 (progress.robot_model_id.c_str ( )) +
      wxString::FromUTF8 (u8"\n与当前模型不一致，未加载。"),
      wxString::FromUTF8 (u8"模型不匹配"),
      wxOK | wxICON_ERROR,
      this);
    return;
  }

  m_teach_point_store.Replace_Points (
    m_current_model_id, progress.points);
  Invalidate_Completed_Progress ( );
  m_last_progress_directory =
    wxFileName (dialog.GetPath ( )).GetPath ( );
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Point_Selection (0);
    Update_Teach_Point_Details ( );
  }
  Apply_Teach_Point_Bindings (0);
  Set_Progress_Dirty (false);
  Update_Trajectory_Status ( );
  if( m_status_text )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"Progress 已加载"));
  }
}

void Robot_Model_Panel::On_Complete_Progress ( )
{
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( points.empty ( ) || !m_flow_panel || !m_right_tool_panel )
  {
    return;
  }

  robot_model::Template_Configuration templates;
  std::string configuration_error;
  if( !robot_model::Load_Template_Configuration (
        robot_model::Template_Configuration_Path ( ),
        &templates,
        &configuration_error) )
  {
    wxMessageBox (
      wxString::FromUTF8 (configuration_error.c_str ( )),
      wxString::FromUTF8 (u8"Progress 检查失败"),
      wxOK | wxICON_ERROR,
      this);
    return;
  }

  std::unordered_map<std::string, std::string> cloud_templates;
  auto fail = [this] (const robot_model::Robot_Teach_Point& point,
                      const wxString& reason)
  {
    wxMessageBox (
      wxString::FromUTF8 (
        robot_model::Format_Teach_Point_Name (point.id).c_str ( )) +
        wxString::FromUTF8 (u8"：") + reason,
      wxString::FromUTF8 (u8"Progress 检查失败"),
      wxOK | wxICON_WARNING,
      this);
  };

  for( const auto& point : points )
  {
    if( point.point_cloud_path.empty ( ) )
    {
      fail (point, wxString::FromUTF8 (u8"没有绑定点云"));
      return;
    }
    if( !point.has_template || point.template_id.empty ( ) )
    {
      fail (point, wxString::FromUTF8 (u8"点云没有绑定模板"));
      return;
    }
    const auto* template_profile = robot_model::Find_Template_Profile (
      templates, point.template_id);
    if( !template_profile )
    {
      fail (point, wxString::FromUTF8 (
        u8"绑定的模板已不存在，请重新绑定"));
      return;
    }
    for( std::size_t index = 0; index < 6; ++index )
    {
      if( !std::isfinite (point.template_reference_pose[index]) ||
          std::abs (
            point.template_reference_pose[index] -
            template_profile->reference_pose[index]) > 1.0e-9 )
      {
        fail (point, wxString::FromUTF8 (
          u8"模板参考点已变化，请重新绑定该点云模板"));
        return;
      }
    }
    const auto cloud_binding = cloud_templates.emplace (
      point.point_cloud_path, point.template_id);
    if( !cloud_binding.second &&
        cloud_binding.first->second != point.template_id )
    {
      fail (point, wxString::FromUTF8 (
        u8"同一点云存在不同模板，请重新绑定"));
      return;
    }
    if( !point.has_coordinate_frame ||
        point.coordinate_frame_id.empty ( ) ||
        !robot_model::Find_Tool_Coordinate (
          m_tool_configuration, point.coordinate_frame_id) )
    {
      fail (point, wxString::FromUTF8 (
        u8"坐标系无效或已被删除"));
      return;
    }
    if( !point.has_world_pose ||
        !std::all_of (
          point.world_pose.begin ( ),
          point.world_pose.end ( ),
          [] (double value) { return std::isfinite (value); }) )
    {
      fail (point, wxString::FromUTF8 (u8"示教位姿无效"));
      return;
    }
  }

  m_flow_panel->Set_Progress_Points (points);
  m_right_tool_panel->Set_Flow_Tool_Enabled (true);
  m_progress_completed = true;
  if( m_status_text )
  {
    m_status_text->SetLabel (wxString::Format (
      wxString::FromUTF8 (
        u8"Progress 检查完成：%zu 个点，流程选项卡已使能"),
      points.size ( )));
  }
}

void Robot_Model_Panel::On_Play_Trajectory (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) ||
      !m_trajectory_session.Has_Playable_Path ( ) ||
      Get_Trajectory_Speed_Mps ( ) <= 0.0 )
  {
    return;
  }

  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( !Apply_Teach_Point_Bindings (0, true) )
  {
    return;
  }
  std::vector<std::size_t> frame_counts;
  frame_counts.reserve (points.size ( ) > 0 ? points.size ( ) - 1 : 0);
  for( std::size_t index = 0; index + 1 < points.size ( ); ++index )
  {
    frame_counts.push_back (
      points[index].has_world_pose && points[index + 1].has_world_pose
        ? frame_count_for_one_meter_per_second (
            points[index].world_pose, points[index + 1].world_pose)
        : static_cast<std::size_t> (TRAJECTORY_FRAME_COUNT));
  }
  m_playback_waypoint_frame_indices.clear ( );
  m_playback_cloud_switches.clear ( );
  m_playback_waypoint_frame_indices.push_back (0);
  std::size_t waypoint_frame_index = 0;
  for( std::size_t index = 0; index < frame_counts.size ( ); ++index )
  {
    if( points[index].point_cloud_path !=
        points[index + 1].point_cloud_path )
    {
      m_playback_cloud_switches.push_back ({
        waypoint_frame_index + 1,
        index + 1});
    }
    waypoint_frame_index +=
      std::max<std::size_t> (frame_counts[index], 2) - 1;
    m_playback_waypoint_frame_indices.push_back (
      waypoint_frame_index);
  }
  if( !m_trajectory_session.Start_Playback (frame_counts) )
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
  m_next_playback_waypoint_index = 1;
  m_next_playback_cloud_switch = 0;
  m_waiting_for_playback_cloud = false;
  m_playback_cloud_switch_blocked = false;
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Point_Selection (0);
    Update_Teach_Point_Details ( );
  }
  m_speed_zero_paused_playback = false;
  Set_Joint_Controls_Enabled (false);
  if( m_view && m_view->Collision_Rebuild_In_Progress ( ) )
  {
    m_trajectory_session.Pause ( );
    m_waiting_for_playback_cloud = true;
    m_trajectory_timer.Start (50);
  }
  else
  {
    m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
  }
  Update_Trajectory_Status ( );
}

void Robot_Model_Panel::On_Pause_Resume_Trajectory (wxCommandEvent&)
{
  if( !m_trajectory_session.Is_Active ( ) )
  {
    return;
  }
  if( m_playback_cloud_switch_blocked )
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
    if( Get_Trajectory_Speed_Mps ( ) <= 0.0 )
    {
      return;
    }
    m_trajectory_session.Resume ( );
    m_speed_zero_paused_playback = false;
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
  const double speed_mps = Get_Trajectory_Speed_Mps ( );
  if( m_playback_cloud_switch_blocked )
  {
    Update_Trajectory_Status ( );
    return;
  }
  if( speed_mps <= 0.0 && m_trajectory_timer.IsRunning ( ) )
  {
    m_trajectory_timer.Stop ( );
    m_trajectory_session.Pause ( );
    m_speed_zero_paused_playback = true;
  }
  else if( speed_mps > 0.0 &&
           m_speed_zero_paused_playback &&
           m_trajectory_session.Is_Paused ( ) )
  {
    m_speed_zero_paused_playback = false;
    if( m_waiting_for_playback_cloud )
    {
      m_trajectory_timer.Start (50);
    }
    else
    {
      m_trajectory_session.Resume ( );
      m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
    }
  }
  else if( m_trajectory_timer.IsRunning ( ) )
  {
    m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
  }
  Update_Trajectory_Status ( );
}

void Robot_Model_Panel::On_Trajectory_Timer (wxTimerEvent&)
{
  if( m_playback_cloud_switch_blocked )
  {
    m_trajectory_timer.Stop ( );
    return;
  }
  if( m_waiting_for_playback_cloud )
  {
    if( m_view && m_view->Collision_Rebuild_In_Progress ( ) )
    {
      return;
    }
    m_waiting_for_playback_cloud = false;
    if( Get_Trajectory_Speed_Mps ( ) <= 0.0 )
    {
      m_trajectory_timer.Stop ( );
      m_speed_zero_paused_playback = true;
      Update_Trajectory_Status ( );
      return;
    }
    m_trajectory_session.Resume ( );
    m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
    Update_Trajectory_Status ( );
    return;
  }

  if( m_next_playback_cloud_switch <
        m_playback_cloud_switches.size ( ) )
  {
    const auto& cloud_switch =
      m_playback_cloud_switches[m_next_playback_cloud_switch];
    if( m_trajectory_session.Frame_Index ( ) >=
        cloud_switch.frame_index )
    {
      m_trajectory_session.Pause ( );
      m_trajectory_timer.Stop ( );
      if( !Apply_Teach_Point_Bindings (
            cloud_switch.point_index, true) )
      {
        m_playback_cloud_switch_blocked = true;
        Set_Joint_Controls_Enabled (true);
        Update_Trajectory_Status ( );
        return;
      }
      ++m_next_playback_cloud_switch;
      if( m_view && m_view->Collision_Rebuild_In_Progress ( ) )
      {
        m_waiting_for_playback_cloud = true;
        m_trajectory_timer.Start (50);
      }
      else
      {
        m_trajectory_session.Resume ( );
        m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
      }
      Update_Trajectory_Status ( );
      return;
    }
  }

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

  if( m_trajectory_session.Frame_Index ( ) > 0 )
  {
    const std::size_t applied_frame =
      m_trajectory_session.Frame_Index ( ) - 1;
    while( m_next_playback_waypoint_index <
             m_playback_waypoint_frame_indices.size ( ) &&
           applied_frame >=
             m_playback_waypoint_frame_indices[
               m_next_playback_waypoint_index] )
    {
      if( m_teach_point_list_panel )
      {
        m_teach_point_list_panel->Set_Point_Selection (
          static_cast<int> (m_next_playback_waypoint_index));
        Update_Teach_Point_Details ( );
      }
      ++m_next_playback_waypoint_index;
    }
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
  m_kuka_connect_button = new wxButton (panel, wxID_ANY, "Connect");
  m_kuka_connect_button->Bind (
    wxEVT_BUTTON, &Robot_Model_Panel::On_Kuka_Connect, this);
  m_flange_frame_button = new wxToggleButton (
    panel, wxID_ANY, wxString::FromUTF8 (u8"显示法兰坐标系"));
  m_flange_free_drag_button = new wxToggleButton (
    panel, wxID_ANY, wxString::FromUTF8 (u8"自由拖拽"));
  m_flange_6d_button = new wxToggleButton (
    panel, wxID_ANY, wxString::FromUTF8 (u8"6D 操纵"));
  auto* interaction_coordinate_label = new wxStaticText (
    panel, wxID_ANY, wxString::FromUTF8 (u8"操作坐标系"));
  m_interaction_coordinate_choice = new wxChoice (panel, wxID_ANY);
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
  m_interaction_coordinate_choice->Bind (
    wxEVT_CHOICE,
    &Robot_Model_Panel::On_Interaction_Coordinate_Changed,
    this);
  m_reset_robot_button->Bind (
    wxEVT_BUTTON, &Robot_Model_Panel::On_Reset_Robot, this);

  auto* operation_sizer = new wxGridSizer (2, 6, 6);
  operation_sizer->Add (
    interaction_coordinate_label, 0, wxALIGN_CENTER_VERTICAL);
  operation_sizer->Add (m_interaction_coordinate_choice, 1, wxEXPAND);
  operation_sizer->Add (m_flange_frame_button, 1, wxEXPAND);
  operation_sizer->Add (m_flange_free_drag_button, 1, wxEXPAND);
  operation_sizer->Add (m_flange_6d_button, 1, wxEXPAND);
  operation_sizer->Add (m_reset_robot_button, 1, wxEXPAND);

  m_kuka_status_text =
    new wxStaticText (panel, wxID_ANY, "KUKA: Disconnected");
  m_kuka_move_joint_button =
    new wxButton (panel, wxID_ANY, "MoveJ");
  m_kuka_move_ptp_button =
    new wxButton (panel, wxID_ANY, "MoveP");
  m_kuka_move_linear_button =
    new wxButton (panel, wxID_ANY, "MoveL");
  m_kuka_sync_button = new wxButton (panel, wxID_ANY, "State");
  m_kuka_stop_button = new wxButton (panel, wxID_ANY, "Stop");

  m_kuka_move_joint_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&)
    {
      try
      {
        if( !m_kuka_service )
          throw std::runtime_error ("KUKA service is unavailable.");
        kuka::Joint_Motion_Options options;
        options.velocity_percent =
          static_cast<double> (m_kuka_ptp_velocity_slider->GetValue ( ));
        options.acceleration_percent =
          static_cast<double> (m_kuka_acceleration_slider->GetValue ( ));
        const auto sequence =
          m_kuka_service->Move_Joint (
            Read_Joint_Input_Angles ( ), options);
        m_status_text->SetLabel (
          wxString::Format ("KUKA MOVEJ sent, sequence=%u", sequence));
      }
      catch( const std::exception& error )
      {
        m_status_text->SetLabel (
          "KUKA MOVEJ failed: " + wxString::FromUTF8 (error.what ( )));
      }
    });

  const auto send_cartesian =
    [this] (bool linear)
    {
      try
      {
        if( !m_kuka_service )
          throw std::runtime_error ("KUKA service is unavailable.");
        robot_model::Matrix4 world_from_flange = { };
        if( !m_view || !m_view->Get_World_From_Flange (&world_from_flange) )
          throw std::runtime_error ("No valid robot pose is available.");
        // Protocol defaults to KUKA TOOL 0, therefore transmit the flange
        // target. A configured app-tool pose must not be sent as a TOOL 0
        // flange pose because that would introduce the tool offset twice.
        const kuka::Pose target =
          robot_model::Build_Xyzabc_From_Zyx_Matrix (world_from_flange);
        kuka::Cartesian_Motion_Options options;
        options.velocity = static_cast<double> (
          linear
            ? m_kuka_linear_velocity_slider->GetValue ( )
            : m_kuka_ptp_velocity_slider->GetValue ( ));
        options.acceleration_percent = static_cast<double> (
          m_kuka_acceleration_slider->GetValue ( ));
        const auto sequence = linear
          ? m_kuka_service->Move_Linear (target, options)
          : m_kuka_service->Move_Pose_Ptp (
              target, Read_Joint_Input_Angles ( ), options);
        m_status_text->SetLabel (wxString::Format (
          linear
            ? "KUKA MOVEL sent, sequence=%u"
            : "KUKA MOVEPTP sent, sequence=%u",
          sequence));
      }
      catch( const std::exception& error )
      {
        m_status_text->SetLabel (
          "KUKA Cartesian move failed: " +
          wxString::FromUTF8 (error.what ( )));
      }
    };
  m_kuka_move_ptp_button->Bind (
    wxEVT_BUTTON,
    [send_cartesian] (wxCommandEvent&) { send_cartesian (false); });
  m_kuka_move_linear_button->Bind (
    wxEVT_BUTTON,
    [send_cartesian] (wxCommandEvent&) { send_cartesian (true); });
  m_kuka_sync_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&)
    {
      try
      {
        if( !m_kuka_service )
          throw std::runtime_error ("KUKA service is unavailable.");
        m_kuka_service->Synchronize ( );
      }
      catch( const std::exception& error )
      {
        m_status_text->SetLabel (
          "KUKA state query failed: " +
          wxString::FromUTF8 (error.what ( )));
      }
    });
  m_kuka_stop_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&)
    {
      try
      {
        if( !m_kuka_service )
          throw std::runtime_error ("KUKA service is unavailable.");
        m_kuka_service->Request_Stop ( );
      }
      catch( const std::exception& error )
      {
        m_status_text->SetLabel (
          "KUKA stop failed: " + wxString::FromUTF8 (error.what ( )));
      }
    });

  // Keep hardware controls readable on narrower layouts: two columns.
  auto* robot_command_sizer = new wxGridSizer (2, 6, 6);
  robot_command_sizer->Add (m_kuka_move_joint_button, 1, wxEXPAND);
  robot_command_sizer->Add (m_kuka_move_ptp_button, 1, wxEXPAND);
  robot_command_sizer->Add (m_kuka_move_linear_button, 1, wxEXPAND);
  robot_command_sizer->Add (m_kuka_sync_button, 1, wxEXPAND);
  robot_command_sizer->Add (m_kuka_stop_button, 1, wxEXPAND);

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

  auto* motion_parameter_sizer = new wxStaticBoxSizer (
    wxVERTICAL, panel, "Motion parameters");
  const auto add_motion_parameter =
    [panel, motion_parameter_sizer] (
      const wxString& name,
      int minimum,
      int maximum,
      int initial,
      const wxString& suffix,
      wxSlider** output)
    {
      auto* row = new wxBoxSizer (wxHORIZONTAL);
      auto* name_text = new wxStaticText (panel, wxID_ANY, name);
      name_text->SetMinSize (wxSize (145, -1));
      auto* slider = new wxSlider (
        panel,
        wxID_ANY,
        initial,
        minimum,
        maximum,
        wxDefaultPosition,
        wxDefaultSize,
        wxSL_HORIZONTAL);
      auto* value_text = new wxStaticText (
        panel,
        wxID_ANY,
        wxString::Format ("%d", initial) + suffix);
      value_text->SetMinSize (wxSize (85, -1));
      slider->Bind (
        wxEVT_SLIDER,
        [slider, value_text, suffix] (wxCommandEvent&)
        {
          value_text->SetLabel (
            wxString::Format ("%d", slider->GetValue ( )) + suffix);
        });
      row->Add (name_text, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 6);
      row->Add (slider, 1, wxALIGN_CENTER_VERTICAL | wxRIGHT, 6);
      row->Add (value_text, 0, wxALIGN_CENTER_VERTICAL);
      motion_parameter_sizer->Add (row, 0, wxEXPAND | wxALL, 4);
      *output = slider;
    };
  add_motion_parameter (
    "PTP speed (MoveJ / MoveP)",
    1,
    100,
    20,
    "%",
    &m_kuka_ptp_velocity_slider);
  add_motion_parameter (
    "Linear speed (MoveL)",
    1,
    2000,
    100,
    " mm/s",
    &m_kuka_linear_velocity_slider);
  add_motion_parameter (
    "Acceleration",
    1,
    100,
    50,
    "%",
    &m_kuka_acceleration_slider);

  m_kuka_status_table = new wxGrid (
    panel, wxID_ANY, wxDefaultPosition, wxSize (-1, 50));
  m_kuka_status_table->CreateGrid (2, 2);
  m_kuka_status_table->EnableEditing (false);
  m_kuka_status_table->EnableGridLines (true);
  m_kuka_status_table->SetRowLabelSize (0);
  m_kuka_status_table->SetColLabelSize (0);
  m_kuka_status_table->DisableDragGridSize ( );
  m_kuka_status_table->DisableDragColSize ( );
  m_kuka_status_table->DisableDragRowSize ( );
  m_kuka_status_table->SetDefaultCellAlignment (
    wxALIGN_CENTER, wxALIGN_CENTER);
  m_kuka_status_table->SetGridLineColour (wxColour (205, 205, 205));
  m_kuka_status_table->SetMargins (0, 0);
  m_kuka_status_table->ShowScrollbars (
    wxSHOW_SB_NEVER, wxSHOW_SB_NEVER);
  m_kuka_status_table->SetColSize (0, 105);
  m_kuka_status_table->SetColSize (1, 520);
  m_kuka_status_table->SetMinSize (wxSize (210, 50));
  const std::array<const char*, 2> status_rows = {
    "Connection", "State"
  };
  for( std::size_t row = 0; row < status_rows.size ( ); ++row )
  {
    m_kuka_status_table->SetCellValue (
      static_cast<int> (row), 0, status_rows[row]);
    m_kuka_status_table->SetCellValue (
      static_cast<int> (row), 1, "--");
    m_kuka_status_table->SetCellBackgroundColour (
      static_cast<int> (row), 0, wxColour (238, 238, 238));
    m_kuka_status_table->SetRowSize (static_cast<int> (row), 24);
  }
  m_kuka_status_table->Bind (
    wxEVT_SIZE,
    [this] (wxSizeEvent& event)
    {
      const int width =
        std::max (210, m_kuka_status_table->GetClientSize ( ).x);
      m_kuka_status_table->SetColSize (0, 105);
      m_kuka_status_table->SetColSize (1, std::max (80, width - 105));
      event.Skip ( );
    });

  auto* connection_sizer = new wxBoxSizer (wxHORIZONTAL);
  connection_sizer->Add (m_kuka_connect_button, 0, wxRIGHT, 8);
  connection_sizer->Add (
    m_kuka_status_text, 1, wxALIGN_CENTER_VERTICAL);

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (
    connection_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
  sizer->Add (operation_title, 0, wxEXPAND | wxALL, 8);
  sizer->Add (operation_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (m_joint_panel, 0, wxEXPAND | wxTOP, 6);
  sizer->Add (m_cartesian_pose_panel, 0, wxEXPAND | wxTOP, 14);
  sizer->Add (
    motion_parameter_sizer,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxTOP,
    8);
  sizer->Add (
    robot_command_sizer, 0, wxEXPAND | wxALL, 8);
  sizer->Add (
    new wxStaticText (panel, wxID_ANY, "Robot status"),
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxTOP,
    8);
  sizer->Add (
    m_kuka_status_table,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxTOP | wxBOTTOM,
    8);
  sizer->AddStretchSpacer (1);
  panel->SetSizer (sizer);
  Refresh_Kuka_Command_Controls (m_kuka_service->Status ( ));
  panel->FitInside ( );
  return panel;
}

wxPanel* Robot_Model_Panel::Build_Teach_Tool_Page (wxWindow* parent)
{
  auto* panel = new wxScrolledWindow (
    parent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
    wxVSCROLL | wxTAB_TRAVERSAL);
  panel->SetMinSize (wxSize (380, -1));
  panel->SetScrollRate (0, 12);

  auto* title = new wxStaticText (
    panel, wxID_ANY, "Teach");
  m_teach_point_command_panel =
    new Teach_Point_Command_Panel (panel);
  Teach_Point_Command_Panel::Callbacks edit_callbacks;
  edit_callbacks.add =
    [this] { wxCommandEvent event; On_Add_Trajectory_Point (event); };
  edit_callbacks.update =
    [this] { wxCommandEvent event; On_Update_Teach_Point (event); };
  edit_callbacks.insert_before =
    [this] { On_Insert_Teach_Point (true); };
  edit_callbacks.insert_after =
    [this] { On_Insert_Teach_Point (false); };
  edit_callbacks.delete_selected =
    [this] { wxCommandEvent event; On_Delete_Trajectory_Point (event); };
  edit_callbacks.clear =
    [this] { wxCommandEvent event; On_Clear_Trajectory_Points (event); };
  edit_callbacks.save =
    [this] { wxCommandEvent event; On_Save_Trajectory (event); };
  edit_callbacks.load =
    [this] { wxCommandEvent event; On_Load_Trajectory (event); };
  edit_callbacks.complete =
    [this] { On_Complete_Progress ( ); };
  edit_callbacks.step_next =
    [this] { On_Step_To_Next_Teach_Point ( ); };
  m_teach_point_command_panel->Set_Callbacks (
    std::move (edit_callbacks));

  m_trajectory_panel = new Trajectory_Control_Panel (
    panel,
    TRAJECTORY_SPEED_DEFAULT_CM_PER_SECOND);
  Trajectory_Control_Panel::Callbacks trajectory_callbacks;
  trajectory_callbacks.go_to_point =
    [this] { wxCommandEvent event; On_Go_To_Trajectory_Point (event); };
  trajectory_callbacks.play =
    [this] { wxCommandEvent event; On_Play_Trajectory (event); };
  trajectory_callbacks.pause_resume =
    [this] { wxCommandEvent event; On_Pause_Resume_Trajectory (event); };
  trajectory_callbacks.stop =
    [this] { wxCommandEvent event; On_Stop_Trajectory (event); };
  trajectory_callbacks.speed_changed =
    [this] { wxCommandEvent event; On_Trajectory_Speed_Changed (event); };
  m_trajectory_panel->Set_Callbacks (
    std::move (trajectory_callbacks));

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (title, 0, wxEXPAND | wxALL, 8);
  sizer->Add (
    m_teach_point_command_panel,
    0,
    wxEXPAND | wxLEFT | wxRIGHT,
    8);
  sizer->Add (
    m_trajectory_panel,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxTOP,
    8);
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
    m_cartesian_pose_panel->Set_World_From_Control_Frame (
      robot_model::Build_World_From_Tool (
        world_from_flange,
        Interaction_Tool ( )));
  }
  else
  {
    m_cartesian_pose_panel->Clear ( );
  }
}

bool Robot_Model_Panel::Apply_Cartesian_Pose_Target (
  const robot_model::XyzabcPose& target_pose)
{
  if( !m_view || !m_view->Has_Current_Model ( ) ) return false;
  if( Is_Trajectory_Active ( ) ) Stop_Trajectory_Playback ( );

  Select_Display_Page (Main_Display_Page::Robot);
  robot_model::Robot_Pose_IK_Options options;
  options.position_tolerance_mm = 0.001;
  options.orientation_tolerance_deg = 0.001;
  options.damping_mm = 1.0;
  options.max_iterations = 100;
  options.time_budget_ms = 8.0;
  const auto result = m_view->Move_Flange_To_Pose (
    robot_model::Build_World_From_Flange_Target (
      robot_model::Build_Zyx_Pose_Matrix (target_pose),
      Interaction_Tool ( )),
    options);
  if( result.status == robot_model::Robot_IK_Status::Invalid_Model )
  {
    m_status_text->SetLabel (wxString::FromUTF8 (
      u8"世界坐标位姿控制失败：模型无效"));
    return false;
  }

  Sync_Joint_Controls_From_State ( );
  const auto& apply_result = m_view->Last_Joint_State_Apply_Result ( );
  if( !apply_result.accepted && apply_result.collision.collided )
  {
    if( apply_result.state_changed ) Sync_Joint_Controls_From_State ( );
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"世界坐标运动已阻止：") +
      collision_summary (apply_result.collision));
    return false;
  }
  m_status_text->SetLabel (wxString::Format (
    result.Converged ( )
      ? wxString::FromUTF8 (
          u8"世界坐标位姿：位置误差 %.2f mm，姿态误差 %.2f°")
      : wxString::FromUTF8 (
          u8"世界坐标目标受限：位置误差 %.2f mm，姿态误差 %.2f°"),
    result.position_error_mm,
    result.orientation_error_deg));
  return result.Converged ( );
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
    const int selection = Selected_Teach_Point_Index ( );
    const auto point_count =
      m_teach_point_store.Point_Count (m_current_model_id);
    m_teach_point_command_panel->Refresh_Command_State (
      enabled && !m_current_model_id.empty ( ),
      Selected_Teach_Point_Indices ( ).size ( ),
      point_count,
      selection != wxNOT_FOUND && selection >= 0 &&
        static_cast<std::size_t> (selection) + 1 < point_count);
  }
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_List_Enabled (enabled);
  }
}

void Robot_Model_Panel::Update_Trajectory_Status ( )
{
  const bool active = Is_Trajectory_Active ( );
  const auto selections = Selected_Teach_Point_Indices ( );
  if( m_teach_point_command_panel )
  {
    const int selection = Selected_Teach_Point_Index ( );
    const auto point_count =
      m_teach_point_store.Point_Count (m_current_model_id);
    m_teach_point_command_panel->Refresh_Command_State (
      !active && !m_current_model_id.empty ( ),
      selections.size ( ),
      point_count,
      selection != wxNOT_FOUND && selection >= 0 &&
        static_cast<std::size_t> (selection) + 1 < point_count);
  }
  if( m_trajectory_panel == nullptr )
  {
    return;
  }

  wxString suffix = "";
  if( m_playback_cloud_switch_blocked )
  {
    suffix = " / cloud error";
  }
  else if( m_waiting_for_playback_cloud )
  {
    suffix = " / switching cloud";
  }
  else if( m_trajectory_timer.IsRunning ( ) )
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
    selections.size ( ) == 1);
}

void Robot_Model_Panel::Update_Trajectory_Speed_Label ( )
{
  if( m_trajectory_panel == nullptr )
  {
    return;
  }

  m_trajectory_panel->Set_Speed_Label (
    wxString::Format (
      "Speed: %.2f m/s", Get_Trajectory_Speed_Mps ( )));
}

void Robot_Model_Panel::Update_Trajectory_Point_List ( )
{
  if( !m_teach_point_list_panel ) return;

  std::vector<wxString> labels;
  std::vector<robot_model::Robot_Teach_Point_Type> types;
  std::vector<wxString> cloud_names;
  std::vector<std::string> cloud_keys;
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  labels.reserve (points.size ( ));
  types.reserve (points.size ( ));
  cloud_names.reserve (points.size ( ));
  cloud_keys.reserve (points.size ( ));
  for( const auto& point : points )
  {
    labels.push_back (wxString::FromUTF8 (
      robot_model::Format_Teach_Point_Name (point.id).c_str ( )));
    types.push_back (point.type);
    if( !point.point_cloud_name.empty ( ) )
    {
      cloud_names.push_back (
        wxString::FromUTF8 (point.point_cloud_name.c_str ( )));
    }
    else if( !point.point_cloud_path.empty ( ) )
    {
      cloud_names.push_back (wxString (
        std::filesystem::u8path (point.point_cloud_path)
          .stem ( ).wstring ( )));
    }
    else
    {
      cloud_names.push_back (wxString::FromUTF8 (u8"未绑定点云"));
    }
    cloud_names.back ( ) += wxString::FromUTF8 (u8" · ");
    cloud_names.back ( ) += point.has_template &&
      !point.template_name.empty ( )
        ? wxString::FromUTF8 (point.template_name.c_str ( ))
        : wxString::FromUTF8 (u8"未绑定模板");
    cloud_keys.push_back (
      point.point_cloud_path.empty ( )
        ? std::string ("__unbound__")
        : point.point_cloud_path);
  }
  m_teach_point_list_panel->Set_Point_Names (
    labels, types, cloud_names, cloud_keys);
  m_teach_point_list_panel->Set_Dirty (
    m_dirty_progress_models.count (m_current_model_id) != 0);
  Update_Teach_Point_Details ( );
}

void Robot_Model_Panel::Update_Teach_Point_Details ( )
{
  if( !m_teach_point_list_panel ) return;
  const auto selections = Selected_Teach_Point_Indices ( );
  if( selections.empty ( ) )
  {
    m_teach_point_list_panel->Set_Point_Details (
      wxString::FromUTF8 (u8"未选择"),
      wxString::FromUTF8 (u8"未选择"),
      wxString::FromUTF8 (u8"未选择"),
      wxString::FromUTF8 (u8"未选择"),
      robot_model::Robot_Teach_Point_Type::Motion,
      false);
    m_teach_point_list_panel->Set_Point_Pose ({ }, false);
    return;
  }
  if( selections.size ( ) > 1 )
  {
    m_teach_point_list_panel->Set_Point_Details (
      wxString::FromUTF8 (u8"多个"),
      wxString::FromUTF8 (u8"多个"),
      wxString::FromUTF8 (u8"多个"),
      wxString::FromUTF8 (u8"多个"),
      robot_model::Robot_Teach_Point_Type::Motion,
      false);
    m_teach_point_list_panel->Set_Point_Pose ({ }, false);
    return;
  }

  const int selection = selections.front ( );
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( selection < 0 ||
      static_cast<std::size_t> (selection) >= points.size ( ) )
  {
    m_teach_point_list_panel->Set_Point_Details (
      wxString::FromUTF8 (u8"未选择"),
      wxString::FromUTF8 (u8"未选择"),
      wxString::FromUTF8 (u8"未选择"),
      wxString::FromUTF8 (u8"未选择"),
      robot_model::Robot_Teach_Point_Type::Motion,
      false);
    m_teach_point_list_panel->Set_Point_Pose ({ }, false);
    return;
  }

  const auto& point = points[static_cast<std::size_t> (selection)];
  if( m_teach_point_command_panel )
  {
    m_teach_point_command_panel->Set_Selected_Point_Type (point.type);
  }
  wxString coordinate = wxString::FromUTF8 (u8"未绑定");
  if( point.has_coordinate_frame )
  {
    coordinate = wxString::FromUTF8 (
      point.coordinate_frame_name.empty ( )
        ? point.coordinate_frame_id.c_str ( )
        : point.coordinate_frame_name.c_str ( ));
  }

  wxString cloud = wxString::FromUTF8 (u8"未绑定");
  if( !point.point_cloud_path.empty ( ) )
  {
    const auto cloud_path =
      std::filesystem::u8path (point.point_cloud_path);
    cloud = point.point_cloud_name.empty ( )
      ? wxString (cloud_path.stem ( ).wstring ( ))
      : wxString::FromUTF8 (point.point_cloud_name.c_str ( ));
  }
  m_teach_point_list_panel->Set_Point_Details (
    teach_point_type_label (point.type),
    coordinate,
    cloud,
    point.has_template && !point.template_name.empty ( )
      ? wxString::FromUTF8 (point.template_name.c_str ( ))
      : wxString::FromUTF8 (u8"未绑定"),
    point.type,
    true);
  robot_model::XyzabcPose display_pose = point.world_pose;
  bool has_display_pose = point.has_world_pose;
  const auto* display_coordinate = robot_model::Find_Tool_Coordinate (
    m_tool_configuration,
    m_teach_pose_coordinate_id);
  if( has_display_pose && display_coordinate )
  {
    display_pose = robot_model::Build_Xyzabc_From_Zyx_Matrix (
      robot_model::Build_World_From_Tool (
        robot_model::Build_Zyx_Pose_Matrix (point.world_pose),
        *display_coordinate));
  }
  m_teach_point_list_panel->Set_Point_Pose (
    display_pose, has_display_pose);
}

void Robot_Model_Panel::On_Teach_Point_Selection_Changed ( )
{
  Update_Trajectory_Status ( );
  Update_Teach_Point_Details ( );
  const int selection = Selected_Teach_Point_Index ( );
  if( selection != wxNOT_FOUND && selection >= 0 &&
      !Is_Trajectory_Active ( ) )
  {
    Apply_Teach_Point_Bindings_Keeping_Display (
      static_cast<std::size_t> (selection));
  }
}

void Robot_Model_Panel::On_Teach_Pose_Coordinate_Changed (
  int selection)
{
  if( selection < 0 ||
      static_cast<std::size_t> (selection) >=
        m_tool_configuration.tools.size ( ) )
  {
    return;
  }
  m_teach_pose_coordinate_id =
    m_tool_configuration.tools[static_cast<std::size_t> (selection)].id;
  Update_Teach_Point_Details ( );
}

void Robot_Model_Panel::Bind_Template_To_Teach_Point_Cloud (
  std::size_t point_index)
{
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( point_index >= points.size ( ) ||
      points[point_index].point_cloud_path.empty ( ) ||
      !m_point_cloud_overlay_toolbar )
  {
    return;
  }

  robot_model::Template_Configuration configuration;
  std::string error_message;
  if( !robot_model::Load_Template_Configuration (
        robot_model::Template_Configuration_Path ( ),
        &configuration,
        &error_message) )
  {
    wxMessageBox (
      wxString::FromUTF8 (error_message.c_str ( )),
      wxString::FromUTF8 (u8"模板配置加载失败"),
      wxOK | wxICON_ERROR,
      this);
    return;
  }
  if( configuration.templates.empty ( ) )
  {
    wxMessageBox (
      wxString::FromUTF8 (u8"还没有可用模板，请先在设置中创建模板。"),
      wxString::FromUTF8 (u8"绑定模板"),
      wxOK | wxICON_INFORMATION,
      this);
    return;
  }

  wxArrayString choices;
  int current_selection = 0;
  for( std::size_t index = 0;
       index < configuration.templates.size ( );
       ++index )
  {
    choices.Add (wxString::FromUTF8 (
      configuration.templates[index].name.c_str ( )));
    if( points[point_index].has_template &&
        points[point_index].template_id ==
          configuration.templates[index].id )
    {
      current_selection = static_cast<int> (index);
    }
  }
  wxSingleChoiceDialog dialog (
    this,
    wxString::FromUTF8 (u8"请选择当前点云要绑定的模板"),
    wxString::FromUTF8 (u8"绑定模板"),
    choices);
  dialog.SetSelection (current_selection);
  if( dialog.ShowModal ( ) != wxID_OK ||
      dialog.GetSelection ( ) == wxNOT_FOUND )
  {
    return;
  }

  const auto& selected = configuration.templates[
    static_cast<std::size_t> (dialog.GetSelection ( ))];
  const std::string cloud_path = points[point_index].point_cloud_path;
  const std::string cloud_name = points[point_index].point_cloud_name;
  if( !m_point_cloud_overlay_toolbar->Bind_Point_Cloud_Template (
        std::filesystem::u8path (cloud_path),
        cloud_name,
        selected,
        &error_message) )
  {
    m_point_cloud_overlay_toolbar->Refresh_Template_Configuration ( );
    wxMessageBox (
      wxString::FromUTF8 (error_message.c_str ( )),
      wxString::FromUTF8 (u8"模板绑定失败"),
      wxOK | wxICON_ERROR,
      this);
    return;
  }

  const std::size_t updated =
    m_teach_point_store.Apply_Template_To_Point_Cloud (
      m_current_model_id,
      cloud_path,
      selected.id,
      selected.name,
      selected.reference_pose);
  if( updated > 0 )
  {
    Set_Progress_Dirty (true);
  }
  Update_Trajectory_Point_List ( );
  if( m_status_text )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"点云已绑定模板：") +
      wxString::FromUTF8 (selected.name.c_str ( )) +
      wxString::Format (
        wxString::FromUTF8 (u8"（已更新 %zu 个示教点）"),
        updated));
  }
}

void Robot_Model_Panel::Unbind_Template_From_Teach_Point_Cloud (
  std::size_t point_index)
{
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( point_index >= points.size ( ) ||
      points[point_index].point_cloud_path.empty ( ) ||
      !m_point_cloud_overlay_toolbar )
  {
    return;
  }
  const std::string cloud_path = points[point_index].point_cloud_path;
  std::string error_message;
  if( !m_point_cloud_overlay_toolbar->Unbind_Point_Cloud_Template (
        std::filesystem::u8path (cloud_path), &error_message) )
  {
    m_point_cloud_overlay_toolbar->Refresh_Template_Configuration ( );
    wxMessageBox (
      wxString::FromUTF8 (error_message.c_str ( )),
      wxString::FromUTF8 (u8"模板解绑失败"),
      wxOK | wxICON_ERROR,
      this);
    return;
  }

  const std::size_t updated =
    m_teach_point_store.Apply_Template_To_Point_Cloud (
      m_current_model_id,
      cloud_path,
      { },
      { },
      { });
  if( updated > 0 )
  {
    Set_Progress_Dirty (true);
  }
  Update_Trajectory_Point_List ( );
  if( m_status_text )
  {
    m_status_text->SetLabel (wxString::Format (
      wxString::FromUTF8 (
        u8"点云已解绑模板，已清除 %zu 个示教点的模板关系"),
      updated));
  }
}

bool Robot_Model_Panel::Apply_Teach_Point_Bindings (
  std::size_t index,
  bool require_point_cloud)
{
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( index >= points.size ( ) ) return false;
  const auto& point = points[index];

  if( point.has_coordinate_frame && !point.coordinate_frame_id.empty ( ) )
  {
    m_interaction_tool_id = point.coordinate_frame_id;
    if( m_interaction_coordinate_choice )
    {
      for( std::size_t choice_index = 0;
           choice_index < m_tool_configuration.tools.size ( );
           ++choice_index )
      {
        if( m_tool_configuration.tools[choice_index].id ==
            point.coordinate_frame_id )
        {
          m_interaction_coordinate_choice->SetSelection (
            static_cast<int> (choice_index));
          break;
        }
      }
    }
    if( m_view )
    {
      robot_model::Tool_Coordinate_Profile coordinate;
      coordinate.id = point.coordinate_frame_id;
      coordinate.name = point.coordinate_frame_name;
      coordinate.flange_from_tool_pose =
        point.flange_from_coordinate_pose;
      m_view->Set_Interaction_Coordinate (coordinate);
    }
  }

  if( require_point_cloud && point.point_cloud_path.empty ( ) )
  {
    if( m_status_text )
    {
      m_status_text->SetLabel (wxString::Format (
        wxString::FromUTF8 (u8"P[%zu] 没有绑定点云，播放已暂停"),
        point.id));
    }
    return false;
  }
  if( !point.point_cloud_path.empty ( ) &&
      m_point_cloud_overlay_toolbar )
  {
    std::string error_message;
    const bool loaded =
      m_point_cloud_overlay_toolbar->Load_Bound_Point_Cloud (
          std::filesystem::u8path (point.point_cloud_path),
          point.point_cloud_name,
          &error_message);
    if( !loaded )
    {
      if( m_status_text )
      {
        m_status_text->SetLabel (
          wxString::FromUTF8 (u8"加载绑定点云失败：") +
          wxString::FromUTF8 (error_message.c_str ( )));
      }
      return false;
    }
  }
  return true;
}

bool Robot_Model_Panel::Apply_Teach_Point_Bindings_Keeping_Display (
  std::size_t index,
  bool require_point_cloud)
{
  const auto display_page = m_display_page;
  const bool applied =
    Apply_Teach_Point_Bindings (index, require_point_cloud);
  if( m_display_page != display_page )
    Select_Display_Page (display_page);
  return applied;
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

std::vector<int> Robot_Model_Panel::Selected_Teach_Point_Indices ( ) const
{
  return m_teach_point_list_panel
    ? m_teach_point_list_panel->Selected_Point_Indices ( )
    : std::vector<int> { };
}

bool Robot_Model_Panel::Read_Current_Teach_Point (
  std::array<double, 6>* joint_angles,
  robot_model::XyzabcPose* world_pose) const
{
  if( !joint_angles || !world_pose || !m_view ||
      m_current_model_id.empty ( ) )
  {
    return false;
  }
  robot_model::Matrix4 world_from_flange = { };
  if( !m_view->Get_World_From_Flange (&world_from_flange) )
  {
    return false;
  }
  *joint_angles = Read_Joint_Input_Angles ( );
  *world_pose =
    robot_model::Build_Xyzabc_From_Zyx_Matrix (world_from_flange);
  return true;
}

bool Robot_Model_Panel::Capture_Current_Teach_Bindings (
  std::string* point_cloud_path,
  std::string* point_cloud_name,
  robot_model::Tool_Coordinate_Profile* coordinate,
  robot_model::Template_Profile* template_profile)
{
  if( !point_cloud_path || !point_cloud_name || !coordinate ||
      !template_profile ||
      !m_point_cloud_overlay_toolbar ||
      !m_point_cloud_overlay_toolbar->Has_Point_Cloud ( ) )
  {
    if( m_status_text )
    {
      m_status_text->SetLabel (wxString::FromUTF8 (
        u8"请先加载或采集点云，再记录 Progress 点"));
    }
    return false;
  }

  std::filesystem::path bound_path;
  std::string error_message;
  if( !m_point_cloud_overlay_toolbar->Current_Point_Cloud_Binding (
        &bound_path, point_cloud_name, &error_message) )
  {
    if( m_status_text )
    {
      m_status_text->SetLabel (
        wxString::FromUTF8 (u8"绑定点云失败：") +
        wxString::FromUTF8 (error_message.c_str ( )));
    }
    return false;
  }

  *point_cloud_path = bound_path.u8string ( );
  *coordinate = Interaction_Tool ( );
  *template_profile = { };
  bool has_template_binding = false;
  if( !m_point_cloud_overlay_toolbar->Current_Template_Profile (
        template_profile, &has_template_binding) &&
      has_template_binding )
  {
    if( m_status_text )
    {
      m_status_text->SetLabel (wxString::FromUTF8 (
        u8"当前点云绑定的模板已被删除，请重新绑定模板"));
    }
    return false;
  }
  return true;
}

void Robot_Model_Panel::Set_Progress_Dirty (bool dirty)
{
  if( m_current_model_id.empty ( ) ) return;
  if( dirty )
  {
    Invalidate_Completed_Progress ( );
    m_dirty_progress_models.insert (m_current_model_id);
  }
  else
  {
    m_dirty_progress_models.erase (m_current_model_id);
  }
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Dirty (dirty);
  }
}

void Robot_Model_Panel::Invalidate_Completed_Progress ( )
{
  if( !m_progress_completed ) return;
  m_progress_completed = false;
  if( m_flow_panel )
  {
    m_flow_panel->Clear_Progress_Points ( );
  }
  if( m_right_tool_panel )
  {
    m_right_tool_panel->Set_Flow_Tool_Enabled (false);
  }
}

double Robot_Model_Panel::Get_Trajectory_Speed_Mps ( ) const
{
  if( m_trajectory_panel == nullptr )
  {
    return 1.0;
  }
  return std::clamp (
    m_trajectory_panel->Speed_Meters_Per_Second ( ), 0.0, 2.0);
}

int Robot_Model_Panel::Get_Trajectory_Timer_Interval_Ms ( ) const
{
  const double speed_mps = Get_Trajectory_Speed_Mps ( );
  if( speed_mps <= 0.0 ) return TRAJECTORY_TIMER_MS;
  const int interval = static_cast<int> (
    std::lround (static_cast<double> (TRAJECTORY_TIMER_MS) / speed_mps));
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
  m_speed_zero_paused_playback = false;
  m_playback_waypoint_frame_indices.clear ( );
  m_next_playback_waypoint_index = 0;
  m_playback_cloud_switches.clear ( );
  m_next_playback_cloud_switch = 0;
  m_waiting_for_playback_cloud = false;
  m_playback_cloud_switch_blocked = false;
  Set_Joint_Controls_Enabled (true);
  Update_Trajectory_Status ( );
}

void Robot_Model_Panel::Resize_Right_Tool (int requested_width)
{
  if( !m_content_splitter || !m_right_tool_panel ||
      !m_content_splitter->IsSplit ( ) ) return;

  const int total_width = m_content_splitter->GetClientSize ( ).x;
  if( total_width <= RIGHT_TOOL_COLLAPSED_WIDTH ) return;

  const int maximum_right_width = std::max (
    RIGHT_TOOL_COLLAPSED_WIDTH,
    total_width - DISPLAY_MINIMUM_WIDTH);
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

const robot_model::Tool_Coordinate_Profile&
Robot_Model_Panel::Active_Tool ( ) const
{
  const auto* tool = robot_model::Find_Tool_Coordinate (
    m_tool_configuration,
    m_tool_configuration.active_tool_id);
  if( tool )
  {
    return *tool;
  }
  static const robot_model::Tool_Coordinate_Profile fallback = {
    "flange", "Flange", {}};
  return fallback;
}

const robot_model::Tool_Coordinate_Profile&
Robot_Model_Panel::Interaction_Tool ( ) const
{
  const auto* tool = robot_model::Find_Tool_Coordinate (
    m_tool_configuration,
    m_interaction_tool_id);
  return tool ? *tool : Active_Tool ( );
}

void Robot_Model_Panel::On_Interaction_Coordinate_Changed (wxCommandEvent&)
{
  if( !m_interaction_coordinate_choice ) return;
  const int selection = m_interaction_coordinate_choice->GetSelection ( );
  if( selection == wxNOT_FOUND ||
      static_cast<std::size_t> (selection) >=
        m_tool_configuration.tools.size ( ) )
  {
    return;
  }

  m_interaction_tool_id =
    m_tool_configuration.tools[static_cast<std::size_t> (selection)].id;
  if( m_view )
  {
    m_view->Set_Interaction_Coordinate (Interaction_Tool ( ));
  }
  if( m_cartesian_pose_panel )
  {
    m_cartesian_pose_panel->Set_Control_Frame_Name (
      Interaction_Tool ( ).name);
    Update_Cartesian_Pose ( );
  }
  if( m_status_text )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"操作坐标系：") +
      wxString::FromUTF8 (Interaction_Tool ( ).name.c_str ( )));
  }
}

void Robot_Model_Panel::Refresh_Interaction_Coordinate_Choices ( )
{
  robot_model::Normalize_Tool_Coordinate_Configuration (
    &m_tool_configuration);
  if( !robot_model::Find_Tool_Coordinate (
        m_tool_configuration,
        m_interaction_tool_id) )
  {
    m_interaction_tool_id = m_tool_configuration.active_tool_id;
  }

  if( m_interaction_coordinate_choice )
  {
    m_interaction_coordinate_choice->Clear ( );
    int selection = wxNOT_FOUND;
    for( std::size_t index = 0;
         index < m_tool_configuration.tools.size ( );
         ++index )
    {
      const auto& tool = m_tool_configuration.tools[index];
      m_interaction_coordinate_choice->Append (
        wxString::FromUTF8 (tool.name.c_str ( )));
      if( tool.id == m_interaction_tool_id )
      {
        selection = static_cast<int> (index);
      }
    }
    if( selection != wxNOT_FOUND )
    {
      m_interaction_coordinate_choice->SetSelection (selection);
    }
  }
}

void Robot_Model_Panel::Refresh_Teach_Pose_Coordinate_Choices ( )
{
  if( !robot_model::Find_Tool_Coordinate (
        m_tool_configuration,
        m_teach_pose_coordinate_id) )
  {
    m_teach_pose_coordinate_id = m_tool_configuration.active_tool_id;
  }
  if( !m_teach_point_list_panel ) return;

  std::vector<wxString> names;
  names.reserve (m_tool_configuration.tools.size ( ));
  int selection = wxNOT_FOUND;
  for( std::size_t index = 0;
       index < m_tool_configuration.tools.size ( );
       ++index )
  {
    const auto& tool = m_tool_configuration.tools[index];
    names.push_back (wxString::FromUTF8 (tool.name.c_str ( )));
    if( tool.id == m_teach_pose_coordinate_id )
    {
      selection = static_cast<int> (index);
    }
  }
  m_teach_point_list_panel->Set_Pose_Coordinate_Choices (
    names, selection);
  Update_Teach_Point_Details ( );
}

void Robot_Model_Panel::Apply_Active_Tool ( )
{
  robot_model::Normalize_Tool_Coordinate_Configuration (
    &m_tool_configuration);
  Refresh_Interaction_Coordinate_Choices ( );
  Refresh_Teach_Pose_Coordinate_Choices ( );
  const auto& tool = Active_Tool ( );
  if( m_view )
  {
    m_view->Set_Tool_Coordinate (tool);
    m_view->Set_Interaction_Coordinate (Interaction_Tool ( ));
  }
  if( m_cartesian_pose_panel )
  {
    m_cartesian_pose_panel->Set_Control_Frame_Name (
      Interaction_Tool ( ).name);
  }
  if( m_tool_panel )
  {
    m_tool_panel->Set_Tool_Coordinates (m_tool_configuration);
  }
  if( m_flow_panel )
  {
    m_flow_panel->Set_Tool_Coordinates (m_tool_configuration);
  }
  Apply_Tool_Visualization ( );
}

void Robot_Model_Panel::Apply_Tool_Visualization ( )
{
  robot_model::Normalize_Tool_Visualization_Configuration (
    &m_tool_visualization_configuration);
  const auto* bound_tool = robot_model::Find_Tool_Coordinate (
    m_tool_configuration,
    m_tool_visualization_configuration.fov.tool_coordinate_id);
  if( m_view )
  {
    m_view->Set_Tool_Frame_Configuration (
      m_tool_visualization_configuration.tool_frame);
    m_view->Set_Fov_Configuration (
      m_tool_visualization_configuration.fov,
      bound_tool);
  }
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
  if( Load_Bound_Robot_Model ( ) )
    return;

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

bool Robot_Model_Panel::Load_Bound_Robot_Model (
  std::string* error_message)
{
  const auto bound_model = std::find_if (
    m_models.begin ( ),
    m_models.end ( ),
    [this] (const robot_model::Robot_Model_Info& model)
    {
      return model_id (model) == m_kuka_connection_configuration.model_id;
    });
  if( bound_model == m_models.end ( ) )
  {
    if( error_message )
      *error_message =
        "The bound robot model was not found: " +
        m_kuka_connection_configuration.model_id;
    return false;
  }
  if( m_current_model_id == m_kuka_connection_configuration.model_id &&
      m_view && m_view->Has_Current_Model ( ) )
  {
    if( error_message ) error_message->clear ( );
    return true;
  }
  return Load_Model (
    static_cast<std::size_t> (
      std::distance (m_models.begin ( ), bound_model)),
    error_message);
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

  Invalidate_Completed_Progress ( );

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
  Refresh_Kuka_Status_Table ( );
  Select_Display_Page (Main_Display_Page::Robot);
  return true;
}

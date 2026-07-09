#include "robot_model_panel.h"

#include "camera_control_panel.h"
#include "camera_image_view.h"
#include "camera_service.h"
#include "robot_joint_state_builder.h"
#include "robot_model_config_loader.h"
#include "robot_model_config_dialog.h"
#include "robot_model_repository.h"
#include "robot_trajectory_io.h"
#include "right_tool_panel.h"
#include "net_panel.h"
#include "flow_panel.h"
#include "point_cloud_view.h"

#include <wx/filedlg.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/simplebook.h>
#include <wx/tglbtn.h>
#include <wx/utils.h>

#include <algorithm>
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
constexpr const char* DEFAULT_ROBOT_MODEL_ID = "KR210_R2700_2";
constexpr std::array<double, 5> TRAJECTORY_SPEED_SCALES = {
  0.25, 0.5, 1.0, 2.0, 4.0
};

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
  m_status_text = new wxStaticText (this, wxID_ANY, "");
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

  m_display_book = new wxSimplebook (this, wxID_ANY);
  m_view = new Robot_Model_View (m_display_book);
  m_camera_image_view = new Camera_Image_View (
    m_display_book, camera_service);
  m_point_cloud_view = new Point_Cloud_View (
    m_display_book, camera_service);
  m_display_book->AddPage (m_view, wxEmptyString, true);
  m_display_book->AddPage (m_camera_image_view, wxEmptyString, false);
  m_display_book->AddPage (m_point_cloud_view, wxEmptyString, false);
  m_right_tool_panel = new Right_Tool_Panel (this);

  auto* tcp_panel = new Net_Panel (m_right_tool_panel->Page_Parent ( ));
  auto* flow_panel = new Flow_Panel (m_right_tool_panel->Page_Parent ( ));
  m_camera_control_panel = new Camera_Control_Panel (
    m_right_tool_panel->Page_Parent ( ), camera_service);
  auto* robot_tool_page = Build_Robot_Tool_Page (
    m_right_tool_panel->Page_Parent ( ));
  m_right_tool_panel->Add_Page (Right_Tool_Page::Tcp, tcp_panel);
  m_right_tool_panel->Add_Page (Right_Tool_Page::Flow, flow_panel);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Camera, m_camera_control_panel);
  m_right_tool_panel->Add_Page (Right_Tool_Page::Robot, robot_tool_page);
  m_camera_control_panel->Set_On_Availability_Changed (
    [this] (bool enabled)
    {
      m_right_tool_panel->Set_Camera_Tool_Enabled (enabled);
    });
  m_right_tool_panel->Set_Robot_Tool_Enabled (false);

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
  toolbar_sizer->Add (m_model_name_text, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 12);
  toolbar_sizer->Add (m_status_text, 1, wxALIGN_CENTER_VERTICAL);

  auto* content_sizer = new wxBoxSizer (wxHORIZONTAL);
  content_sizer->Add (m_display_book, 1, wxEXPAND | wxRIGHT, 4);
  content_sizer->Add (m_right_tool_panel, 0, wxEXPAND);

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (toolbar_sizer, 0, wxEXPAND | wxALL, 6);
  sizer->Add (content_sizer, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);
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
  if( Is_Trajectory_Active ( ) )
  {
    return;
  }

  m_trajectory_session.Add_Point (Read_Joint_Input_Angles ( ));
  Update_Trajectory_Point_List ( );
  Update_Trajectory_Status ( );
}

void Robot_Model_Panel::On_Clear_Trajectory_Points (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) )
  {
    return;
  }

  m_trajectory_session.Clear_Points ( );
  Update_Trajectory_Point_List ( );
  Update_Trajectory_Status ( );
}

void Robot_Model_Panel::On_Go_To_Trajectory_Point (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) || m_trajectory_panel == nullptr )
  {
    return;
  }

  const int selection = m_trajectory_panel->Selected_Point_Index ( );
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
  if( Is_Trajectory_Active ( ) || m_trajectory_panel == nullptr )
  {
    return;
  }

  const int selection = m_trajectory_panel->Selected_Point_Index ( );
  if( selection == wxNOT_FOUND ||
      selection < 0 ||
      static_cast<size_t> (selection) >= m_trajectory_session.Point_Count ( ) )
  {
    return;
  }

  const size_t point_index = static_cast<size_t> (selection);
  m_trajectory_session.Delete_Point (point_index);
  Update_Trajectory_Point_List ( );

  if( m_trajectory_session.Point_Count ( ) > 0 )
  {
    const int next_selection = static_cast<int> (
      std::min (point_index, m_trajectory_session.Point_Count ( ) - 1));
    m_trajectory_panel->Set_Point_Selection (next_selection);
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

  m_trajectory_session.Set_Points (std::move (loaded_points));
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

  Apply_Joint_Input_Angles_To_Sliders (m_trajectory_session.Points ( ).front ( ));
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

  Apply_Joint_Input_Angles_To_Sliders (*frame);

  if( m_trajectory_session.Is_Finished ( ) )
  {
    Stop_Trajectory_Playback ( );
  }
}

wxPanel* Robot_Model_Panel::Build_Robot_Tool_Page (wxWindow* parent)
{
  auto* panel = new wxPanel (parent, wxID_ANY);
  panel->SetMinSize (wxSize (380, -1));

  m_joint_panel = new Joint_Control_Panel (panel);
  m_joint_panel->Set_On_Joint_Changed (
    [this] { Update_Joint_State_From_Sliders ( ); });

  m_trajectory_panel = new Trajectory_Control_Panel (
    panel,
    TRAJECTORY_SPEED_DEFAULT_INDEX,
    static_cast<int> (TRAJECTORY_SPEED_SCALES.size ( ) - 1));

  Trajectory_Control_Panel::Callbacks callbacks;
  callbacks.add_point =
    [this] { wxCommandEvent event; On_Add_Trajectory_Point (event); };
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
  callbacks.selection_changed =
    [this] { Update_Trajectory_Status ( ); };
  m_trajectory_panel->Set_Callbacks (std::move (callbacks));

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (m_joint_panel, 0, wxEXPAND);
  sizer->Add (m_trajectory_panel, 0, wxEXPAND);
  sizer->AddStretchSpacer (1);
  panel->SetSizer (sizer);
  Update_Joint_State_From_Sliders ( );
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

  for( size_t i = 0; i < m_joint_state.input_angles_deg.size ( ); ++i )
  {
    int min_value = slider_limit_at (params.joint_mins, i, DEFAULT_JOINT_MIN);
    int max_value = slider_limit_at (params.joint_maxs, i, DEFAULT_JOINT_MAX);
    if( min_value > max_value )
    {
      std::swap (min_value, max_value);
    }

    const int neutral_value = static_cast<int> (
      std::lround (robot_model::Neutral_Joint_Input_At (params, i)));
    m_joint_panel->Set_Joint_Range_And_Value (
      i, min_value, max_value, neutral_value);
  }

  Update_Joint_State_From_Sliders ( );
}

void Robot_Model_Panel::Update_Joint_State_From_Sliders ( )
{
  m_joint_state = robot_model::Build_Joint_State_From_Input_Angles (
    m_params, Read_Joint_Input_Angles ( ));

  for( size_t i = 0; i < m_joint_state.input_angles_deg.size ( ); ++i )
  {
    Update_Joint_Value_Label (i,
                              m_joint_state.input_angles_deg[i],
                              m_joint_state.effective_angles_deg[i]);
  }

  if( m_view )
  {
    m_view->Set_Joint_State (m_joint_state);
  }
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
  return m_joint_panel ? m_joint_panel->Read_Input_Angles ( )
                       : std::array<double, 6> {};
}

void Robot_Model_Panel::Apply_Joint_Input_Angles_To_Sliders (
  const std::array<double, 6>& input_angles_deg)
{
  if( m_joint_panel )
  {
    m_joint_panel->Set_Input_Angles (input_angles_deg);
  }

  Update_Joint_State_From_Sliders ( );
}

void Robot_Model_Panel::Set_Joint_Controls_Enabled (bool enabled)
{
  if( m_joint_panel )
  {
    m_joint_panel->Set_Joint_Controls_Enabled (enabled);
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
    m_trajectory_session.Point_Count ( ));
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
  if( m_trajectory_panel == nullptr )
  {
    return;
  }

  std::vector<wxString> labels;
  const auto& points = m_trajectory_session.Points ( );
  labels.reserve (points.size ( ));
  for( size_t i = 0; i < points.size ( ); ++i )
  {
    labels.push_back (
      Format_Trajectory_Point_Label (i, points[i]));
  }
  m_trajectory_panel->Set_Point_Labels (labels);
}

wxString Robot_Model_Panel::Format_Trajectory_Point_Label (
  size_t index, const std::array<double, 6>& point) const
{
  return wxString::Format (
    "#%zu  %.0f, %.0f, %.0f, %.0f, %.0f, %.0f",
    index + 1,
    point[0],
    point[1],
    point[2],
    point[3],
    point[4],
    point[5]);
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
      wxString::FromUTF8 (u8"默认机械臂 KR210_R2700_2 未找到"));
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

  m_params = robot_model::Load_Robot_Kinematic_Params (
    model.xml_path, model.display_name);
  Apply_Joint_Limits (m_params);

  m_view->Load_Model (model);
  m_view->Set_Joint_State (m_joint_state);

  m_current_model_id = model_id (model);
  m_model_name_text->SetLabel (
    wxString::FromUTF8 (u8"当前机械臂：") +
    wxString::FromUTF8 (model.display_name.c_str ( )));
  m_status_text->SetLabel (wxString::FromUTF8 (u8"模型加载成功"));
  m_right_tool_panel->Set_Robot_Tool_Enabled (true);
  Select_Display_Page (Main_Display_Page::Robot);
  return true;
}

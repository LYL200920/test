#ifndef includeguard_robot_model_panel_h_includeguard
#define includeguard_robot_model_panel_h_includeguard

#include "robot_model_data.h"
#include "robot_model_view.h"
#include "camera_pose_controller.h"
#include "robot_trajectory_session.h"
#include "robot_teach_point_store.h"
#include "joint_control_panel.h"
#include "trajectory_control_panel.h"
#include "cartesian_pose_panel.h"
#include "tool_coordinate.h"

#include <wx/panel.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/timer.h>

#include <array>
#include <string>
#include <vector>

class Right_Tool_Panel;
class Camera_Control_Panel;
class Camera_Image_View;
class Camera_Service;
class Point_Cloud_View;
class Point_Cloud_Overlay_Toolbar;
class Teach_Point_Command_Panel;
class Teach_Point_List_Panel;
class wxButton;
class wxSimplebook;
class wxSplitterWindow;
class wxToggleButton;

enum class Main_Display_Page
{
  Robot = 0,
  Camera_Image = 1,
  Point_Cloud = 2
};

class Robot_Model_Panel : public wxPanel
{
public:
  Robot_Model_Panel(
      wxWindow *parent,
      Camera_Service &camera_service,
      wxWindowID id = wxID_ANY);

  void Show_Model_Configuration(wxWindow *parent = nullptr);

private:
  void On_Add_Trajectory_Point(wxCommandEvent &event);
  void On_Clear_Trajectory_Points(wxCommandEvent &event);
  void On_Go_To_Trajectory_Point(wxCommandEvent &event);
  void On_Delete_Trajectory_Point(wxCommandEvent &event);
  void On_Save_Trajectory(wxCommandEvent &event);
  void On_Load_Trajectory(wxCommandEvent &event);
  void On_Play_Trajectory(wxCommandEvent &event);
  void On_Pause_Resume_Trajectory(wxCommandEvent &event);
  void On_Stop_Trajectory(wxCommandEvent &event);
  void On_Trajectory_Speed_Changed(wxCommandEvent &event);
  void On_Trajectory_Timer(wxTimerEvent &event);
  void On_Robot_Display(wxCommandEvent &event);
  void On_Camera_Image_Display(wxCommandEvent &event);
  void On_Point_Cloud_Display(wxCommandEvent &event);
  void On_Reset_Robot(wxCommandEvent &event);
  bool Reset_Robot_To_Home();
  bool Set_Camera_Pose_Visible(bool visible);
  void On_Toggle_Flange_Frame(wxCommandEvent &event);
  void On_Toggle_Flange_Free_Drag(wxCommandEvent &event);
  void On_Toggle_Flange_6D(wxCommandEvent &event);
  void Set_Flange_Interaction_Mode(Robot_Model_View::Flange_Interaction_Mode mode);
  void Select_Display_Page(Main_Display_Page page);
  void Update_Display_Menu();
  wxPanel *Build_Robot_Tool_Page(wxWindow *parent);
  void Apply_Joint_Limits(const robot_model::Robot_Kinematic_Params &params);
  void Update_Joint_State_From_Sliders();
  void Update_Cartesian_Pose();
  void Apply_Cartesian_Pose_Target(const robot_model::XyzabcPose &target_pose);
  void Apply_Flange_Drag_Result(const robot_model::Robot_Position_IK_Result &result);
  void Apply_Flange_Pose_Drag_Result(const robot_model::Robot_Pose_IK_Result &result);
  void Sync_Joint_Controls_From_State();
  void Update_Joint_Value_Label(size_t index, double input_angle, double effective_angle);
  std::array<double, 6> Read_Joint_Input_Angles() const;
  robot_model::Robot_Joint_State_Apply_Result
  Apply_Joint_Input_Angles_To_Sliders(const std::array<double, 6> &input_angles_deg);
  void Set_Joint_Controls_Enabled(bool enabled);
  void Update_Trajectory_Status();
  void Update_Trajectory_Speed_Label();
  void Update_Trajectory_Point_List();
  void Sync_Trajectory_From_Teach_Points();
  int Selected_Teach_Point_Index() const;
  double Get_Trajectory_Speed_Scale() const;
  int Get_Trajectory_Timer_Interval_Ms() const;
  bool Is_Trajectory_Active() const;
  void Stop_Trajectory_Playback();
  void Resize_Right_Tool(bool collapsed);
  void Resize_Teach_Point_List(bool collapsed);
  void Load_Model_List();
  void Load_Default_Model();
  bool Load_Model(size_t model_index, std::string *error_message = nullptr);
  const robot_model::Tool_Coordinate_Profile &Active_Tool() const;
  void Apply_Active_Tool();

private:
  wxStaticText *m_model_name_text = nullptr;
  wxStaticText *m_status_text = nullptr;
  Robot_Model_View *m_view = nullptr;
  Camera_Image_View *m_camera_image_view = nullptr;
  Point_Cloud_View *m_point_cloud_view = nullptr;
  Point_Cloud_Overlay_Toolbar *m_point_cloud_overlay_toolbar = nullptr;
  wxSimplebook *m_display_book = nullptr;
  wxSplitterWindow *m_workspace_splitter = nullptr;
  wxSplitterWindow *m_content_splitter = nullptr;
  wxToggleButton *m_robot_display_button = nullptr;
  wxToggleButton *m_camera_display_button = nullptr;
  wxToggleButton *m_point_cloud_display_button = nullptr;
  wxToggleButton *m_flange_frame_button = nullptr;
  wxToggleButton *m_flange_free_drag_button = nullptr;
  wxToggleButton *m_flange_6d_button = nullptr;
  wxButton *m_reset_robot_button = nullptr;
  Main_Display_Page m_display_page = Main_Display_Page::Robot;
  Right_Tool_Panel *m_right_tool_panel = nullptr;
  Camera_Control_Panel *m_camera_control_panel = nullptr;
  Joint_Control_Panel *m_joint_panel = nullptr;
  Cartesian_Pose_Panel *m_cartesian_pose_panel = nullptr;
  Teach_Point_Command_Panel *m_teach_point_command_panel = nullptr;
  Teach_Point_List_Panel *m_teach_point_list_panel = nullptr;
  Trajectory_Control_Panel *m_trajectory_panel = nullptr;
  wxTimer m_trajectory_timer;
  std::vector<robot_model::Robot_Model_Info> m_models;
  robot_model::Robot_Trajectory_Session m_trajectory_session;
  robot_model::Robot_Teach_Point_Store m_teach_point_store;
  std::string m_current_model_id;
  Camera_Pose_Controller m_camera_pose_controller;
  robot_model::Tool_Coordinate_Configuration m_tool_configuration;
  int m_expanded_right_tool_width = 476;
  int m_expanded_teach_point_width = 240;
};

#endif

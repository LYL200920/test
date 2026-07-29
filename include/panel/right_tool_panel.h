#ifndef includeguard_right_tool_panel_h_includeguard
#define includeguard_right_tool_panel_h_includeguard

#include <wx/panel.h>

#include <functional>

class wxButton;
class wxSimplebook;

enum class Right_Tool_Page
{
  Robot = 0,
  Teach = 1,
  Tcp = 2,
  Flow = 3,
  Camera = 4,
  Camera2D = 5,
  PointCloud = 6,
  Tool = 7
};

class Right_Tool_Panel : public wxPanel
{
public:
  explicit Right_Tool_Panel (wxWindow* parent);

  wxWindow* Page_Parent (Right_Tool_Page page) const;
  void Add_Page (Right_Tool_Page page, wxWindow* window);
  void Set_Camera_Tool_Enabled (bool enabled);
  void Set_Flow_Tool_Enabled (bool enabled);
  void Set_Robot_Tool_Enabled (bool enabled);
  void Set_On_Width_Changed (
    std::function<void (int)> callback);

private:
  void On_Tcp_Click (wxCommandEvent& event);
  void On_Teach_Click (wxCommandEvent& event);
  void On_Flow_Click (wxCommandEvent& event);
  void On_Camera_Click (wxCommandEvent& event);
  void On_Camera_2D_Click (wxCommandEvent& event);
  void On_Point_Cloud_Click (wxCommandEvent& event);
  void On_Tool_Click (wxCommandEvent& event);
  void On_Robot_Click (wxCommandEvent& event);
  void Toggle_Right_Page (Right_Tool_Page page);
  void Set_Robot_Expanded (bool expanded);
  void Set_Right_Expanded (bool expanded);
  int Desired_Width ( ) const;
  void Notify_Width_Changed ( );
  void Update_Button_Labels ( );

private:
  wxButton* m_tcp_button = nullptr;
  wxButton* m_teach_button = nullptr;
  wxButton* m_flow_button = nullptr;
  wxButton* m_camera_button = nullptr;
  wxButton* m_camera_2d_button = nullptr;
  wxButton* m_point_cloud_button = nullptr;
  wxButton* m_tool_button = nullptr;
  wxButton* m_robot_button = nullptr;
  wxSimplebook* m_left_page_book = nullptr;
  wxSimplebook* m_right_page_book = nullptr;
  Right_Tool_Page m_active_page = Right_Tool_Page::Tcp;
  bool m_robot_expanded = false;
  bool m_right_expanded = false;
  bool m_camera_tool_enabled = false;
  bool m_flow_tool_enabled = false;
  bool m_robot_tool_enabled = false;
  std::function<void (int)> m_on_width_changed;
};

#endif

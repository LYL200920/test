#ifndef includeguard_right_tool_panel_h_includeguard
#define includeguard_right_tool_panel_h_includeguard

#include <wx/panel.h>

#include <functional>

class wxButton;
class wxSimplebook;

enum class Right_Tool_Page
{
  Tcp = 0,
  Flow = 1,
  Camera = 2,
  Robot = 3
};

class Right_Tool_Panel : public wxPanel
{
public:
  explicit Right_Tool_Panel (wxWindow* parent);

  wxWindow* Page_Parent ( ) const;
  void Add_Page (Right_Tool_Page page, wxWindow* window);
  void Set_Camera_Tool_Enabled (bool enabled);
  void Set_Robot_Tool_Enabled (bool enabled);
  void Set_On_Collapsed_Changed (
    std::function<void (bool)> callback);

private:
  void On_Tcp_Click (wxCommandEvent& event);
  void On_Flow_Click (wxCommandEvent& event);
  void On_Camera_Click (wxCommandEvent& event);
  void On_Robot_Click (wxCommandEvent& event);
  void Toggle_Page (Right_Tool_Page page);
  void Set_Collapsed (bool collapsed);
  void Update_Button_Labels ( );

private:
  wxButton* m_tcp_button = nullptr;
  wxButton* m_flow_button = nullptr;
  wxButton* m_camera_button = nullptr;
  wxButton* m_robot_button = nullptr;
  wxSimplebook* m_page_book = nullptr;
  Right_Tool_Page m_active_page = Right_Tool_Page::Tcp;
  bool m_collapsed = true;
  bool m_camera_tool_enabled = false;
  bool m_robot_tool_enabled = false;
  std::function<void (bool)> m_on_collapsed_changed;
};

#endif

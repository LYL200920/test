#ifndef includeguard_tool_panel_h_includeguard
#define includeguard_tool_panel_h_includeguard

#include "tool_coordinate.h"
#include "tool_visualization.h"

#include <wx/panel.h>

#include <functional>
#include <string>
#include <vector>

class wxButton;
class wxCheckBox;
class wxChoice;
class wxSimplebook;
class wxSlider;
class wxSpinCtrlDouble;
class wxStaticText;

class Tool_Panel : public wxPanel
{
public:
  explicit Tool_Panel(wxWindow *parent);

  void Set_Tool_Coordinates(
    const robot_model::Tool_Coordinate_Configuration &configuration);
  void Set_Configuration(
    const robot_model::Tool_Visualization_Configuration &configuration);
  void Set_On_Configuration_Changed(
    std::function<void(
      const robot_model::Tool_Visualization_Configuration &)> callback);

private:
  void On_Open_Fov(wxCommandEvent &event);
  void On_Back(wxCommandEvent &event);
  void On_Apply_Fov(wxCommandEvent &event);
  void On_Apply_Tool_Frame(wxCommandEvent &event);
  void On_Tool_Frame_Scale_Changed(wxCommandEvent &event);
  void Refresh_Coordinate_Choices();
  void Load_Fov_Controls();
  void Load_Tool_Frame_Controls();
  void Set_Status(const wxString &text, bool error = false);

private:
  wxSimplebook *m_page_book = nullptr;
  wxCheckBox *m_tool_frame_visible_checkbox = nullptr;
  wxSlider *m_tool_frame_scale_slider = nullptr;
  wxStaticText *m_tool_frame_scale_text = nullptr;
  wxCheckBox *m_visible_checkbox = nullptr;
  wxChoice *m_coordinate_choice = nullptr;
  wxSpinCtrlDouble *m_width_control = nullptr;
  wxSpinCtrlDouble *m_length_control = nullptr;
  wxChoice *m_width_axis_choice = nullptr;
  wxChoice *m_length_axis_choice = nullptr;
  wxStaticText *m_status_text = nullptr;
  robot_model::Tool_Coordinate_Configuration m_tool_coordinates;
  robot_model::Tool_Visualization_Configuration m_configuration;
  std::vector<std::string> m_coordinate_ids;
  std::function<void(
    const robot_model::Tool_Visualization_Configuration &)>
    m_on_configuration_changed;
};

#endif

#ifndef includeguard_robot_model_config_dialog_h_includeguard
#define includeguard_robot_model_config_dialog_h_includeguard

#include "robot_model_data.h"
#include "tool_coordinate.h"

#include <wx/dialog.h>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

class wxButton;
class wxBookCtrlEvent;
class wxListBox;
class wxNotebook;
class wxSpinCtrlDouble;
class wxStaticText;
class wxTextCtrl;

class Robot_Model_Config_Dialog : public wxDialog
{
public:
  Robot_Model_Config_Dialog(
    wxWindow *parent,
    const std::vector<robot_model::Robot_Model_Info> &models,
    const std::string &current_model_id,
    const robot_model::Tool_Coordinate_Configuration &tool_configuration);

  std::size_t Selected_Model_Index() const;
  bool Model_Load_Requested() const;
  bool Tool_Apply_Requested() const;
  const robot_model::Tool_Coordinate_Configuration &
    Tool_Configuration() const;

private:
  enum class Result_Action
  {
    None,
    Load_Model,
    Apply_Tool
  };

  void On_Page_Changed(wxBookCtrlEvent &event);
  void On_Model_Activated(wxCommandEvent &event);
  void On_Action(wxCommandEvent &event);
  void On_Tool_Selection_Changed(wxCommandEvent &event);
  void On_New_Tool(wxCommandEvent &event);
  void On_Delete_Tool(wxCommandEvent &event);
  void Load_Tool_Editor(std::size_t index);
  bool Store_Tool_Editor();
  void Refresh_Tool_List(std::size_t selection);
  void Update_Action_Button();

private:
  const std::vector<robot_model::Robot_Model_Info> &m_models;
  robot_model::Tool_Coordinate_Configuration m_tool_configuration;
  wxNotebook *m_notebook = nullptr;
  wxListBox *m_model_list = nullptr;
  wxStaticText *m_model_status_text = nullptr;
  wxListBox *m_tool_list = nullptr;
  wxTextCtrl *m_tool_name = nullptr;
  std::array<wxSpinCtrlDouble *, 6> m_tool_pose_controls = {};
  wxButton *m_delete_tool_button = nullptr;
  wxButton *m_action_button = nullptr;
  std::size_t m_tool_editor_index = 0;
  Result_Action m_result_action = Result_Action::None;
};

#endif

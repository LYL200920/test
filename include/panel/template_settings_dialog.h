#ifndef includeguard_template_settings_dialog_h_includeguard
#define includeguard_template_settings_dialog_h_includeguard

#include "template_configuration.h"

#include <wx/dialog.h>

#include <array>
#include <cstddef>

class wxButton;
class wxGrid;
class wxListBox;
class wxSpinCtrlDouble;
class wxStaticText;
class wxTextCtrl;

class Template_Settings_Dialog : public wxDialog
{
public:
  Template_Settings_Dialog(
    wxWindow *parent,
    const robot_model::Template_Configuration &configuration);

  const robot_model::Template_Configuration &Configuration() const;

private:
  void On_Selection_Changed(wxCommandEvent &event);
  void On_New(wxCommandEvent &event);
  void On_Copy(wxCommandEvent &event);
  void On_Delete(wxCommandEvent &event);
  void On_Save(wxCommandEvent &event);
  void Load_Editor(std::size_t index);
  bool Store_Editor();
  void Refresh_List(std::size_t selection);
  void Update_Editor_State();
  void Update_Matrix_Preview();
  std::string Make_Unique_Id() const;
  std::string Make_Unique_Name(const std::string &base) const;

private:
  robot_model::Template_Configuration m_configuration;
  wxListBox *m_template_list = nullptr;
  wxStaticText *m_template_id = nullptr;
  wxTextCtrl *m_template_name = nullptr;
  std::array<wxSpinCtrlDouble *, 6> m_pose_controls = {};
  wxGrid *m_matrix_grid = nullptr;
  wxButton *m_copy_button = nullptr;
  wxButton *m_delete_button = nullptr;
  std::size_t m_editor_index = 0;
  bool m_has_editor = false;
  bool m_loading_editor = false;
};

#endif

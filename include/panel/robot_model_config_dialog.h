#ifndef includeguard_robot_model_config_dialog_h_includeguard
#define includeguard_robot_model_config_dialog_h_includeguard

#include "robot_model_data.h"

#include <wx/dialog.h>

#include <cstddef>
#include <string>
#include <vector>

class wxButton;
class wxListBox;
class wxStaticText;

class Robot_Model_Config_Dialog : public wxDialog
{
public:
  Robot_Model_Config_Dialog(wxWindow *parent, const std::vector<robot_model::Robot_Model_Info> &models, const std::string &current_model_id);

  std::size_t Selected_Model_Index() const;

private:
  void On_Selection_Changed(wxCommandEvent &event);
  void On_Model_Activated(wxCommandEvent &event);
  void On_Load(wxCommandEvent &event);
  void Update_Load_Button();

private:
  const std::vector<robot_model::Robot_Model_Info> &m_models;
  wxListBox *m_model_list = nullptr;
  wxStaticText *m_status_text = nullptr;
  wxButton *m_load_button = nullptr;
};

#endif

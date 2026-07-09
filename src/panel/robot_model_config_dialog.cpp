#include "robot_model_config_dialog.h"

#include <wx/button.h>
#include <wx/listbox.h>
#include <wx/sizer.h>
#include <wx/statline.h>
#include <wx/stattext.h>

#include <limits>

namespace
{
wxString From_Utf8 (const std::string& value)
{
  return wxString::FromUTF8 (value.c_str ( ));
}

std::string Model_Id (const robot_model::Robot_Model_Info& model)
{
  return model.model_dir.filename ( ).string ( );
}
} // namespace

Robot_Model_Config_Dialog::Robot_Model_Config_Dialog (
  wxWindow* parent,
  const std::vector<robot_model::Robot_Model_Info>& models,
  const std::string& current_model_id)
  : wxDialog (
      parent,
      wxID_ANY,
      wxString::FromUTF8 (u8"机械臂配置"),
      wxDefaultPosition,
      wxSize (480, 420),
      wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER),
    m_models (models)
{
  m_status_text = new wxStaticText (
    this, wxID_ANY,
    current_model_id.empty ( )
      ? wxString::FromUTF8 (u8"当前机械臂：未加载")
      : wxString::Format (
          wxString::FromUTF8 (u8"当前机械臂：%s"),
          From_Utf8 (current_model_id)));
  m_model_list = new wxListBox (this, wxID_ANY);

  int current_selection = wxNOT_FOUND;
  for( std::size_t i = 0; i < m_models.size ( ); ++i )
  {
    const auto label = m_models[i].display_name.empty ( )
      ? Model_Id (m_models[i]) : m_models[i].display_name;
    m_model_list->Append (From_Utf8 (label));
    if( Model_Id (m_models[i]) == current_model_id )
    {
      current_selection = static_cast<int> (i);
    }
  }

  if( current_selection != wxNOT_FOUND )
  {
    m_model_list->SetSelection (current_selection);
  }
  else if( !m_models.empty ( ) )
  {
    m_model_list->SetSelection (0);
  }

  m_load_button = new wxButton (
    this, wxID_OK, wxString::FromUTF8 (u8"加载"));
  auto* cancel_button = new wxButton (
    this, wxID_CANCEL, wxString::FromUTF8 (u8"取消"));
  m_load_button->SetDefault ( );

  m_model_list->Bind (
    wxEVT_LISTBOX,
    &Robot_Model_Config_Dialog::On_Selection_Changed,
    this);
  m_model_list->Bind (
    wxEVT_LISTBOX_DCLICK,
    &Robot_Model_Config_Dialog::On_Model_Activated,
    this);
  m_load_button->Bind (
    wxEVT_BUTTON,
    &Robot_Model_Config_Dialog::On_Load,
    this);

  auto* button_sizer = new wxBoxSizer (wxHORIZONTAL);
  button_sizer->AddStretchSpacer (1);
  button_sizer->Add (m_load_button, 0, wxRIGHT, 8);
  button_sizer->Add (cancel_button, 0);

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (m_status_text, 0, wxEXPAND | wxALL, 12);
  sizer->Add (new wxStaticLine (this), 0, wxEXPAND | wxLEFT | wxRIGHT, 12);
  sizer->Add (m_model_list, 1, wxEXPAND | wxALL, 12);
  sizer->Add (button_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 12);
  SetSizer (sizer);
  SetMinSize (wxSize (400, 320));
  Update_Load_Button ( );
  CentreOnParent ( );
}

std::size_t Robot_Model_Config_Dialog::Selected_Model_Index ( ) const
{
  const auto selection = m_model_list->GetSelection ( );
  if( selection == wxNOT_FOUND )
  {
    return std::numeric_limits<std::size_t>::max ( );
  }
  return static_cast<std::size_t> (selection);
}

void Robot_Model_Config_Dialog::On_Selection_Changed (wxCommandEvent&)
{
  Update_Load_Button ( );
}

void Robot_Model_Config_Dialog::On_Model_Activated (wxCommandEvent&)
{
  if( Selected_Model_Index ( ) < m_models.size ( ) )
  {
    EndModal (wxID_OK);
  }
}

void Robot_Model_Config_Dialog::On_Load (wxCommandEvent&)
{
  if( Selected_Model_Index ( ) < m_models.size ( ) )
  {
    EndModal (wxID_OK);
  }
}

void Robot_Model_Config_Dialog::Update_Load_Button ( )
{
  m_load_button->Enable (Selected_Model_Index ( ) < m_models.size ( ));
  if( m_models.empty ( ) )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"未发现可加载的机械臂模型"));
  }
}

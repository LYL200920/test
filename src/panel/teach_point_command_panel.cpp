#include "teach_point_command_panel.h"

#include <wx/button.h>
#include <wx/choice.h>
#include <wx/sizer.h>
#include <wx/slider.h>
#include <wx/stattext.h>

#include <utility>

Teach_Point_Command_Panel::Teach_Point_Command_Panel (wxWindow* parent)
  : wxPanel (parent, wxID_ANY)
{
  auto* title = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (u8"示教点编辑"));
  m_add_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"添加到末尾"));
  m_update_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"修改当前点"));
  m_insert_before_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"插入到上一点"));
  m_insert_after_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"插入到下一点"));
  m_delete_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"删除选中点"));
  m_clear_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"清空 Progress"));
  m_save_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"保存 Progress"));
  m_load_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"加载 Progress"));
  m_complete_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"Progress 完成"));
  m_step_back_button = new wxButton (this, wxID_ANY, "Back");
  m_step_next_button = new wxButton (this, wxID_ANY, "Next");
  m_step_speed_label = new wxStaticText (
    this, wxID_ANY,
    wxString::FromUTF8 (u8"Back/Next 速度：50%"));
  m_step_speed_slider = new wxSlider (
    this, wxID_ANY, 50, 10, 100,
    wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL);
  m_step_speed_slider->Bind (
    wxEVT_SLIDER,
    [this] (wxCommandEvent&)
    {
      if( m_step_speed_label && m_step_speed_slider )
      {
        m_step_speed_label->SetLabel (wxString::Format (
          wxString::FromUTF8 (u8"Back/Next 速度：%d%%"),
          m_step_speed_slider->GetValue ( )));
      }
    });

  auto* type_label = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (u8"类型"));
  m_type_choice = new wxChoice (this, wxID_ANY);
  m_type_choice->Append (wxString::FromUTF8 (u8"运动点"));
  m_type_choice->Append (wxString::FromUTF8 (u8"过渡点"));
  m_type_choice->Append (wxString::FromUTF8 (u8"等待点"));
  m_type_choice->SetSelection (0);

  auto* root = new wxBoxSizer (wxVERTICAL);
  root->Add (title, 0, wxEXPAND | wxBOTTOM, 4);
  auto* type_sizer = new wxBoxSizer (wxHORIZONTAL);
  type_sizer->Add (
    type_label, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 6);
  type_sizer->Add (m_type_choice, 1, wxEXPAND);
  root->Add (type_sizer, 0, wxEXPAND | wxBOTTOM, 6);
  auto* step_sizer = new wxBoxSizer (wxHORIZONTAL);
  step_sizer->Add (m_step_back_button, 1, wxRIGHT, 3);
  step_sizer->Add (m_step_next_button, 1, wxLEFT, 3);
  root->Add (step_sizer, 0, wxEXPAND | wxBOTTOM, 6);
  root->Add (m_step_speed_label, 0, wxEXPAND);
  root->Add (m_step_speed_slider, 0, wxEXPAND | wxTOP | wxBOTTOM, 4);
  auto* step_speed_range = new wxBoxSizer (wxHORIZONTAL);
  step_speed_range->Add (
    new wxStaticText (this, wxID_ANY, "10%"),
    0, wxALIGN_CENTER_VERTICAL);
  step_speed_range->AddStretchSpacer (1);
  step_speed_range->Add (
    new wxStaticText (this, wxID_ANY, "100%"),
    0, wxALIGN_CENTER_VERTICAL);
  root->Add (step_speed_range, 0, wxEXPAND | wxBOTTOM, 12);
  root->Add (m_add_button, 0, wxEXPAND);
  root->Add (m_update_button, 0, wxEXPAND | wxTOP, 6);
  auto* insert_sizer = new wxBoxSizer (wxHORIZONTAL);
  insert_sizer->Add (m_insert_before_button, 1, wxRIGHT, 3);
  insert_sizer->Add (m_insert_after_button, 1, wxLEFT, 3);
  root->Add (insert_sizer, 0, wxEXPAND | wxTOP, 6);
  auto* delete_sizer = new wxBoxSizer (wxHORIZONTAL);
  delete_sizer->Add (m_delete_button, 1, wxRIGHT, 3);
  delete_sizer->Add (m_clear_button, 1, wxLEFT, 3);
  root->Add (delete_sizer, 0, wxEXPAND | wxTOP, 6);
  auto* file_sizer = new wxBoxSizer (wxHORIZONTAL);
  file_sizer->Add (m_save_button, 1, wxRIGHT, 3);
  file_sizer->Add (m_load_button, 1, wxLEFT, 3);
  root->Add (file_sizer, 0, wxEXPAND | wxTOP, 12);
  root->Add (m_complete_button, 0, wxEXPAND | wxTOP, 12);
  SetSizer (root);
}

robot_model::Robot_Teach_Point_Type
Teach_Point_Command_Panel::Selected_Point_Type ( ) const
{
  if( !m_type_choice ) return robot_model::Robot_Teach_Point_Type::Motion;
  switch( m_type_choice->GetSelection ( ) )
  {
    case 1:
      return robot_model::Robot_Teach_Point_Type::Transition;
    case 2:
      return robot_model::Robot_Teach_Point_Type::Wait;
    default:
      return robot_model::Robot_Teach_Point_Type::Motion;
  }
}

void Teach_Point_Command_Panel::Set_Selected_Point_Type (
  robot_model::Robot_Teach_Point_Type type)
{
  if( !m_type_choice ) return;
  m_type_choice->SetSelection (
    type == robot_model::Robot_Teach_Point_Type::Transition
      ? 1
      : type == robot_model::Robot_Teach_Point_Type::Wait ? 2 : 0);
}

int Teach_Point_Command_Panel::Step_Speed_Percent ( ) const
{
  return m_step_speed_slider ? m_step_speed_slider->GetValue ( ) : 50;
}

void Teach_Point_Command_Panel::Set_Callbacks (Callbacks callbacks)
{
  m_callbacks = std::move (callbacks);
  Bind_Button (m_add_button, m_callbacks.add);
  Bind_Button (m_update_button, m_callbacks.update);
  Bind_Button (m_insert_before_button, m_callbacks.insert_before);
  Bind_Button (m_insert_after_button, m_callbacks.insert_after);
  Bind_Button (m_delete_button, m_callbacks.delete_selected);
  Bind_Button (m_clear_button, m_callbacks.clear);
  Bind_Button (m_save_button, m_callbacks.save);
  Bind_Button (m_load_button, m_callbacks.load);
  Bind_Button (m_complete_button, m_callbacks.complete);
  Bind_Button (m_step_back_button, m_callbacks.step_back);
  Bind_Button (m_step_next_button, m_callbacks.step_next);
}

void Teach_Point_Command_Panel::Refresh_Command_State (
  bool enabled,
  std::size_t selected_count,
  std::size_t point_count,
  bool can_step_back,
  bool can_step_next)
{
  const bool single_selection = selected_count == 1;
  if( m_type_choice ) m_type_choice->Enable (enabled);
  if( m_step_speed_slider ) m_step_speed_slider->Enable (enabled);
  if( m_add_button ) m_add_button->Enable (enabled);
  if( m_update_button )
    m_update_button->Enable (enabled && single_selection);
  if( m_insert_before_button )
    m_insert_before_button->Enable (enabled && single_selection);
  if( m_insert_after_button )
    m_insert_after_button->Enable (enabled && single_selection);
  if( m_delete_button )
    m_delete_button->Enable (enabled && selected_count > 0);
  if( m_clear_button )
    m_clear_button->Enable (enabled && point_count > 0);
  if( m_save_button )
    m_save_button->Enable (enabled && point_count > 0);
  if( m_load_button ) m_load_button->Enable (enabled);
  if( m_complete_button )
    m_complete_button->Enable (enabled && point_count > 0);
  if( m_step_back_button )
    m_step_back_button->Enable (
      enabled && can_step_back);
  if( m_step_next_button )
    m_step_next_button->Enable (
      enabled && can_step_next);
}

void Teach_Point_Command_Panel::Bind_Button (
  wxButton* button,
  const std::function<void ( )>& callback)
{
  if( !button ) return;
  button->Bind (
    wxEVT_BUTTON,
    [callback] (wxCommandEvent&)
    {
      if( callback ) callback ( );
    });
}

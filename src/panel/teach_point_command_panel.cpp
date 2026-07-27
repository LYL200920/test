#include "teach_point_command_panel.h"

#include <wx/button.h>
#include <wx/sizer.h>
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

  auto* root = new wxBoxSizer (wxVERTICAL);
  root->Add (title, 0, wxEXPAND | wxBOTTOM, 4);
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
  SetSizer (root);
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
}

void Teach_Point_Command_Panel::Refresh_Command_State (
  bool enabled,
  std::size_t selected_count,
  std::size_t point_count)
{
  const bool single_selection = selected_count == 1;
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

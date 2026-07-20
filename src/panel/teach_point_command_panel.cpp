#include "teach_point_command_panel.h"

#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

#include <utility>

Teach_Point_Command_Panel::Teach_Point_Command_Panel (wxWindow* parent)
  : wxPanel (parent, wxID_ANY)
{
  auto* title = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (u8"示教点工具"));
  m_record_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"记录当前示教点"));
  m_record_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&)
    {
      if( m_on_record ) m_on_record ( );
    });

  auto* root = new wxBoxSizer (wxVERTICAL);
  root->Add (title, 0, wxEXPAND | wxBOTTOM, 4);
  root->Add (m_record_button, 0, wxEXPAND);
  SetSizer (root);
}

void Teach_Point_Command_Panel::Set_On_Record (
  std::function<void ( )> callback)
{
  m_on_record = std::move (callback);
}

void Teach_Point_Command_Panel::Set_Record_Enabled (bool enabled)
{
  if( m_record_button ) m_record_button->Enable (enabled);
}

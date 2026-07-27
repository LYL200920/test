#include "teach_point_list_panel.h"

#include <wx/button.h>
#include <wx/listbox.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

#include <algorithm>
#include <utility>

Teach_Point_List_Panel::Teach_Point_List_Panel (wxWindow* parent)
  : wxPanel (parent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
             wxBORDER_SIMPLE)
{
  auto* header = new wxBoxSizer (wxHORIZONTAL);
  m_title = new wxStaticText (this, wxID_ANY, "Progress");
  m_toggle_button = new wxButton (
    this, wxID_ANY, "<", wxDefaultPosition, wxSize (30, -1), wxBU_EXACTFIT);
  header->Add (m_title, 1, wxALIGN_CENTER_VERTICAL | wxLEFT, 6);
  header->Add (m_toggle_button, 0, wxALIGN_CENTER_VERTICAL);

  m_point_list = new wxListBox (
    this,
    wxID_ANY,
    wxDefaultPosition,
    wxDefaultSize,
    0,
    nullptr,
    wxLB_EXTENDED);
  m_toggle_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&) { Toggle_Collapsed ( ); });
  m_point_list->Bind (
    wxEVT_LISTBOX,
    [this] (wxCommandEvent&)
    {
      if( m_on_selection_changed ) m_on_selection_changed ( );
    });

  auto* root = new wxBoxSizer (wxVERTICAL);
  root->Add (header, 0, wxEXPAND | wxALL, 4);
  root->Add (m_point_list, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 4);
  SetSizer (root);
  SetMinSize (wxSize (220, -1));
}

void Teach_Point_List_Panel::Set_Point_Names (
  const std::vector<wxString>& names)
{
  if( !m_point_list ) return;
  const std::vector<int> old_selections =
    Selected_Point_Indices ( );
  m_point_list->Clear ( );
  for( const auto& name : names ) m_point_list->Append (name);
  if( names.empty ( ) ) return;
  std::vector<int> selections;
  for( const int selection : old_selections )
  {
    if( selection >= 0 &&
        selection < static_cast<int> (names.size ( )) )
    {
      selections.push_back (selection);
    }
  }
  if( selections.empty ( ) )
  {
    selections.push_back (
      static_cast<int> (names.size ( ) - 1));
  }
  Set_Point_Selections (selections);
}

int Teach_Point_List_Panel::Selected_Point_Index ( ) const
{
  const auto selections = Selected_Point_Indices ( );
  return selections.size ( ) == 1 ? selections.front ( ) : wxNOT_FOUND;
}

std::vector<int> Teach_Point_List_Panel::Selected_Point_Indices ( ) const
{
  std::vector<int> selections;
  if( !m_point_list ) return selections;
  wxArrayInt selected;
  m_point_list->GetSelections (selected);
  selections.reserve (selected.size ( ));
  for( const int index : selected )
  {
    selections.push_back (index);
  }
  return selections;
}

void Teach_Point_List_Panel::Set_Point_Selection (int selection)
{
  Set_Point_Selections (
    selection == wxNOT_FOUND
      ? std::vector<int> { }
      : std::vector<int> { selection });
}

void Teach_Point_List_Panel::Set_Point_Selections (
  const std::vector<int>& selections)
{
  if( !m_point_list ) return;
  m_point_list->DeselectAll ( );
  for( const int selection : selections )
  {
    if( selection >= 0 &&
        selection < static_cast<int> (m_point_list->GetCount ( )) )
    {
      m_point_list->SetSelection (selection, true);
    }
  }
  if( !selections.empty ( ) )
  {
    m_point_list->EnsureVisible (selections.front ( ));
  }
}

void Teach_Point_List_Panel::Set_Dirty (bool dirty)
{
  m_dirty = dirty;
  if( m_title )
  {
    m_title->SetLabel (m_dirty ? "Progress *" : "Progress");
  }
}

void Teach_Point_List_Panel::Set_List_Enabled (bool enabled)
{
  if( m_point_list ) m_point_list->Enable (enabled);
}

void Teach_Point_List_Panel::Set_On_Selection_Changed (
  std::function<void ( )> callback)
{
  m_on_selection_changed = std::move (callback);
}

void Teach_Point_List_Panel::Set_On_Collapsed_Changed (
  std::function<void (bool)> callback)
{
  m_on_collapsed_changed = std::move (callback);
}

void Teach_Point_List_Panel::Toggle_Collapsed ( )
{
  m_collapsed = !m_collapsed;
  Update_Collapsed_State ( );
}

void Teach_Point_List_Panel::Update_Collapsed_State ( )
{
  if( m_title ) m_title->Show (!m_collapsed);
  if( m_point_list ) m_point_list->Show (!m_collapsed);
  if( m_toggle_button ) m_toggle_button->SetLabel (m_collapsed ? ">" : "<");
  SetMinSize (wxSize (m_collapsed ? 38 : 220, -1));
  Layout ( );
  if( m_on_collapsed_changed ) m_on_collapsed_changed (m_collapsed);
}

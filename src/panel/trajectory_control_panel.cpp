#include "trajectory_control_panel.h"

#include <wx/button.h>
#include <wx/listbox.h>
#include <wx/sizer.h>
#include <wx/slider.h>
#include <wx/stattext.h>

#include <algorithm>
#include <utility>

Trajectory_Control_Panel::Trajectory_Control_Panel (
  wxWindow* parent,
  int speed_default_index,
  int speed_max_index)
  : wxPanel (parent, wxID_ANY)
{
  m_add_point_button = new wxButton (this, wxID_ANY, "Add Point");
  m_clear_points_button = new wxButton (this, wxID_ANY, "Clear");
  m_go_to_point_button = new wxButton (this, wxID_ANY, "Go To");
  m_delete_point_button = new wxButton (this, wxID_ANY, "Delete");
  m_save_button = new wxButton (this, wxID_ANY, "Save");
  m_load_button = new wxButton (this, wxID_ANY, "Load");
  m_play_button = new wxButton (this, wxID_ANY, "Play");
  m_pause_resume_button = new wxButton (this, wxID_ANY, "Pause");
  m_stop_button = new wxButton (this, wxID_ANY, "Stop");
  m_speed_label = new wxStaticText (this, wxID_ANY, "Speed: 1.0x");
  m_speed_slider = new wxSlider (
    this, wxID_ANY, speed_default_index, 0, speed_max_index,
    wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL);
  m_status_text = new wxStaticText (this, wxID_ANY, "Points: 0 / need 2");
  m_point_list = new wxListBox (this, wxID_ANY);
  m_point_list->SetMinSize (wxSize (-1, 120));

  auto* sizer = new wxBoxSizer (wxVERTICAL);

  auto* point_buttons = new wxBoxSizer (wxHORIZONTAL);
  point_buttons->Add (m_add_point_button, 1, wxRIGHT, 4);
  point_buttons->Add (m_delete_point_button, 1, wxLEFT, 4);
  sizer->Add (point_buttons, 0, wxEXPAND | wxTOP, 14);

  auto* edit_buttons = new wxBoxSizer (wxHORIZONTAL);
  edit_buttons->Add (m_go_to_point_button, 1, wxRIGHT, 4);
  edit_buttons->Add (m_clear_points_button, 1, wxLEFT, 4);
  sizer->Add (edit_buttons, 0, wxEXPAND | wxTOP, 6);

  auto* file_buttons = new wxBoxSizer (wxHORIZONTAL);
  file_buttons->Add (m_save_button, 1, wxRIGHT, 4);
  file_buttons->Add (m_load_button, 1, wxLEFT, 4);
  sizer->Add (file_buttons, 0, wxEXPAND | wxTOP, 6);

  sizer->Add (m_play_button, 0, wxEXPAND | wxTOP, 6);

  auto* playback_buttons = new wxBoxSizer (wxHORIZONTAL);
  playback_buttons->Add (m_pause_resume_button, 1, wxRIGHT, 4);
  playback_buttons->Add (m_stop_button, 1, wxLEFT, 4);
  sizer->Add (playback_buttons, 0, wxEXPAND | wxTOP, 6);

  sizer->Add (m_speed_label, 0, wxEXPAND | wxTOP, 10);
  sizer->Add (m_speed_slider, 0, wxEXPAND | wxTOP, 2);
  sizer->Add (m_status_text, 0, wxEXPAND | wxTOP, 6);
  sizer->Add (m_point_list, 0, wxEXPAND | wxTOP, 4);
  SetSizer (sizer);
}

void Trajectory_Control_Panel::Set_Callbacks (Callbacks callbacks)
{
  m_callbacks = std::move (callbacks);

  Bind_Button (m_add_point_button, m_callbacks.add_point);
  Bind_Button (m_clear_points_button, m_callbacks.clear_points);
  Bind_Button (m_go_to_point_button, m_callbacks.go_to_point);
  Bind_Button (m_delete_point_button, m_callbacks.delete_point);
  Bind_Button (m_save_button, m_callbacks.save);
  Bind_Button (m_load_button, m_callbacks.load);
  Bind_Button (m_play_button, m_callbacks.play);
  Bind_Button (m_pause_resume_button, m_callbacks.pause_resume);
  Bind_Button (m_stop_button, m_callbacks.stop);

  if( m_speed_slider )
  {
    m_speed_slider->Bind (
      wxEVT_SLIDER,
      [this] (wxCommandEvent&)
      {
        if( m_callbacks.speed_changed )
        {
          m_callbacks.speed_changed ( );
        }
      });
  }

  if( m_point_list )
  {
    m_point_list->Bind (
      wxEVT_LISTBOX,
      [this] (wxCommandEvent&)
      {
        if( m_callbacks.selection_changed )
        {
          m_callbacks.selection_changed ( );
        }
      });
  }
}

int Trajectory_Control_Panel::Selected_Point_Index ( ) const
{
  return m_point_list ? m_point_list->GetSelection ( ) : wxNOT_FOUND;
}

void Trajectory_Control_Panel::Set_Point_Labels (
  const std::vector<wxString>& labels)
{
  if( !m_point_list )
  {
    return;
  }

  const int old_selection = m_point_list->GetSelection ( );
  m_point_list->Clear ( );

  for( const auto& label : labels )
  {
    m_point_list->Append (label);
  }

  if( labels.empty ( ) )
  {
    return;
  }

  const int selection = old_selection == wxNOT_FOUND ?
    static_cast<int> (labels.size ( ) - 1) :
    std::min (old_selection, static_cast<int> (labels.size ( ) - 1));
  m_point_list->SetSelection (selection);
}

void Trajectory_Control_Panel::Set_Point_Selection (int selection)
{
  if( m_point_list && selection != wxNOT_FOUND )
  {
    m_point_list->SetSelection (selection);
  }
}

int Trajectory_Control_Panel::Speed_Index ( ) const
{
  return m_speed_slider ? m_speed_slider->GetValue ( ) : 0;
}

void Trajectory_Control_Panel::Set_Speed_Label (const wxString& label)
{
  if( m_speed_label )
  {
    m_speed_label->SetLabel (label);
  }
}

void Trajectory_Control_Panel::Set_Status_Text (const wxString& label)
{
  if( m_status_text )
  {
    m_status_text->SetLabel (label);
  }
}

void Trajectory_Control_Panel::Refresh_Command_State (
  bool active,
  bool paused,
  bool playing,
  size_t point_count)
{
  const bool has_selected_point = Selected_Point_Index ( ) != wxNOT_FOUND;

  if( m_add_point_button )
  {
    m_add_point_button->Enable (!active);
  }
  if( m_clear_points_button )
  {
    m_clear_points_button->Enable (!active && point_count > 0);
  }
  if( m_go_to_point_button )
  {
    m_go_to_point_button->Enable (!active && has_selected_point);
  }
  if( m_delete_point_button )
  {
    m_delete_point_button->Enable (!active && has_selected_point);
  }
  if( m_save_button )
  {
    m_save_button->Enable (!active && point_count > 0);
  }
  if( m_load_button )
  {
    m_load_button->Enable (!active);
  }
  if( m_point_list )
  {
    m_point_list->Enable (!active);
  }
  if( m_play_button )
  {
    m_play_button->Enable (!active && point_count >= 2);
  }
  if( m_pause_resume_button )
  {
    m_pause_resume_button->SetLabel (paused ? "Resume" : "Pause");
    m_pause_resume_button->Enable (active);
  }
  if( m_stop_button )
  {
    m_stop_button->Enable (active);
  }

  (void)playing;
}

void Trajectory_Control_Panel::Bind_Button (
  wxButton* button,
  const std::function<void ( )>& callback)
{
  if( !button )
  {
    return;
  }

  button->Bind (
    wxEVT_BUTTON,
    [callback] (wxCommandEvent&)
    {
      if( callback )
      {
        callback ( );
      }
    });
}

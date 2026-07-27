#include "trajectory_control_panel.h"

#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/slider.h>
#include <wx/stattext.h>

#include <algorithm>
#include <utility>

Trajectory_Control_Panel::Trajectory_Control_Panel (
  wxWindow* parent,
  int speed_default_centimeters_per_second)
  : wxPanel (parent, wxID_ANY)
{
  m_go_to_point_button = new wxButton (this, wxID_ANY, "Go To");
  m_play_button = new wxButton (this, wxID_ANY, "Play");
  m_pause_resume_button = new wxButton (this, wxID_ANY, "Pause");
  m_stop_button = new wxButton (this, wxID_ANY, "Stop");
  m_speed_label = new wxStaticText (this, wxID_ANY, "Speed: 1.00 m/s");
  m_speed_slider = new wxSlider (
    this, wxID_ANY,
    std::clamp (speed_default_centimeters_per_second, 0, 200),
    0, 200,
    wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL);
  m_status_text = new wxStaticText (this, wxID_ANY, "Points: 0 / need 2");

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (m_go_to_point_button, 0, wxEXPAND | wxTOP, 14);

  sizer->Add (m_play_button, 0, wxEXPAND | wxTOP, 6);

  auto* playback_buttons = new wxBoxSizer (wxHORIZONTAL);
  playback_buttons->Add (m_pause_resume_button, 1, wxRIGHT, 4);
  playback_buttons->Add (m_stop_button, 1, wxLEFT, 4);
  sizer->Add (playback_buttons, 0, wxEXPAND | wxTOP, 6);

  sizer->Add (m_speed_label, 0, wxEXPAND | wxTOP, 10);
  sizer->Add (m_speed_slider, 0, wxEXPAND | wxTOP, 2);
  auto* speed_range = new wxBoxSizer (wxHORIZONTAL);
  speed_range->Add (
    new wxStaticText (this, wxID_ANY, "0.00 m/s"),
    0, wxALIGN_CENTER_VERTICAL);
  speed_range->AddStretchSpacer (1);
  speed_range->Add (
    new wxStaticText (this, wxID_ANY, "2.00 m/s"),
    0, wxALIGN_CENTER_VERTICAL);
  sizer->Add (speed_range, 0, wxEXPAND | wxTOP, 1);
  sizer->Add (m_status_text, 0, wxEXPAND | wxTOP, 6);
  SetSizer (sizer);
}

void Trajectory_Control_Panel::Set_Callbacks (Callbacks callbacks)
{
  m_callbacks = std::move (callbacks);

  Bind_Button (m_go_to_point_button, m_callbacks.go_to_point);
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

}

double Trajectory_Control_Panel::Speed_Meters_Per_Second ( ) const
{
  return m_speed_slider
    ? static_cast<double> (m_speed_slider->GetValue ( )) / 100.0
    : 0.0;
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
  size_t point_count,
  bool has_selected_point)
{
  if( m_go_to_point_button )
  {
    m_go_to_point_button->Enable (!active && has_selected_point);
  }
  if( m_play_button )
  {
    m_play_button->Enable (
      !active && point_count >= 2 &&
      Speed_Meters_Per_Second ( ) > 0.0);
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

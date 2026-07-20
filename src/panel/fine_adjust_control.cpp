#include "fine_adjust_control.h"

#include <wx/button.h>
#include <wx/choice.h>
#include <wx/sizer.h>

#include <algorithm>
#include <cmath>
#include <utility>

Fine_Adjust_Control::Fine_Adjust_Control (
  wxWindow* parent,
  const std::vector<double>& steps,
  const wxString& unit,
  double default_step)
  : wxPanel (parent, wxID_ANY),
    m_steps (steps)
{
  auto* sizer = new wxBoxSizer (wxHORIZONTAL);
  m_decrease_button = new wxButton (
    this, wxID_ANY, "-", wxDefaultPosition, wxSize (26, -1), wxBU_EXACTFIT);
  m_step_choice = new wxChoice (
    this, wxID_ANY, wxDefaultPosition, wxSize (66, -1));
  m_increase_button = new wxButton (
    this, wxID_ANY, "+", wxDefaultPosition, wxSize (26, -1), wxBU_EXACTFIT);

  m_decrease_button->SetToolTip (wxString::FromUTF8 (u8"减小一个微调步长"));
  m_step_choice->SetToolTip (wxString::FromUTF8 (u8"选择微调精度"));
  m_increase_button->SetToolTip (wxString::FromUTF8 (u8"增加一个微调步长"));

  int default_selection = 0;
  for( size_t index = 0; index < m_steps.size ( ); ++index )
  {
    const double step = m_steps[index];
    m_step_choice->Append (
      wxString::Format ("%g%s", step, unit.c_str ( )));
    if( std::abs (step - default_step) < 1.0e-12 )
    {
      default_selection = static_cast<int> (index);
    }
  }
  if( !m_steps.empty ( ) ) m_step_choice->SetSelection (default_selection);

  m_decrease_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&) { Notify_Adjust (-1.0); });
  m_increase_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&) { Notify_Adjust (1.0); });

  sizer->Add (m_decrease_button, 0, wxALIGN_CENTER_VERTICAL);
  sizer->Add (m_step_choice, 0, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, 2);
  sizer->Add (m_increase_button, 0, wxALIGN_CENTER_VERTICAL);
  SetSizer (sizer);
}

void Fine_Adjust_Control::Set_On_Adjust (
  std::function<void (double)> callback)
{
  m_on_adjust = std::move (callback);
}

void Fine_Adjust_Control::Set_Adjust_Enabled (bool enabled)
{
  if( m_decrease_button ) m_decrease_button->Enable (enabled);
  if( m_step_choice ) m_step_choice->Enable (enabled);
  if( m_increase_button ) m_increase_button->Enable (enabled);
}

double Fine_Adjust_Control::Selected_Step ( ) const
{
  if( !m_step_choice ) return 0.0;
  const int selection = m_step_choice->GetSelection ( );
  if( selection < 0 || static_cast<size_t> (selection) >= m_steps.size ( ) )
  {
    return 0.0;
  }
  return m_steps[static_cast<size_t> (selection)];
}

void Fine_Adjust_Control::Notify_Adjust (double direction)
{
  const double step = Selected_Step ( );
  if( step > 0.0 && m_on_adjust ) m_on_adjust (direction * step);
}

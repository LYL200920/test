#include "joint_control_panel.h"

#include <wx/sizer.h>
#include <wx/slider.h>
#include <wx/stattext.h>

#include <Windows.h>
#include <CommCtrl.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace
{

bool is_slider_thumb_hit (wxSlider* slider, const wxMouseEvent& event)
{
  if( slider == nullptr )
  {
    return false;
  }

  HWND hwnd = reinterpret_cast<HWND> (slider->GetHandle ( ));
  if( hwnd == nullptr )
  {
    return true;
  }

  RECT thumb_rect = {};
  SendMessage (hwnd, TBM_GETTHUMBRECT, 0,
               reinterpret_cast<LPARAM> (&thumb_rect));
  if( thumb_rect.right <= thumb_rect.left ||
      thumb_rect.bottom <= thumb_rect.top )
  {
    return true;
  }

  InflateRect (&thumb_rect, 4, 6);
  const auto pos = event.GetPosition ( );
  POINT point = { pos.x, pos.y };
  return PtInRect (&thumb_rect, point) != FALSE;
}

void bind_drag_only_slider (wxSlider* slider)
{
  if( slider == nullptr )
  {
    return;
  }

  slider->Bind (wxEVT_LEFT_DOWN,
                [slider] (wxMouseEvent& event)
                {
                  if( is_slider_thumb_hit (slider, event) )
                  {
                    event.Skip ( );
                  }
                });
  slider->Bind (wxEVT_LEFT_DCLICK,
                [slider] (wxMouseEvent& event)
                {
                  if( is_slider_thumb_hit (slider, event) )
                  {
                    event.Skip ( );
                  }
                });
}

} // namespace

Joint_Control_Panel::Joint_Control_Panel (wxWindow* parent)
  : wxPanel (parent, wxID_ANY)
{
  auto* sizer = new wxBoxSizer (wxVERTICAL);
  auto* title = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (u8"关节姿态"));
  sizer->Add (title, 0, wxEXPAND | wxBOTTOM, 8);

  for( size_t i = 0; i < m_joint_sliders.size ( ); ++i )
  {
    auto* row_header = new wxBoxSizer (wxHORIZONTAL);
    auto* name_label = new wxStaticText (
      this, wxID_ANY, wxString::Format ("A%d", static_cast<int> (i + 1)));
    auto* value_label = new wxStaticText (this, wxID_ANY, "0 / 0 deg");
    value_label->SetMinSize (wxSize (96, -1));

    row_header->Add (name_label, 0, wxALIGN_CENTER_VERTICAL);
    row_header->AddStretchSpacer (1);
    row_header->Add (value_label, 0, wxALIGN_CENTER_VERTICAL);

    auto* slider = new wxSlider (
      this, wxID_ANY, 0, -180, 180,
      wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL);
    slider->Bind (wxEVT_SLIDER,
                  [this] (wxCommandEvent&) { Notify_Joint_Changed ( ); });
    slider->Bind (wxEVT_SCROLL_THUMBTRACK,
                  [this] (wxScrollEvent&) { Notify_Joint_Changed ( ); });
    slider->Bind (wxEVT_SCROLL_CHANGED,
                  [this] (wxScrollEvent&) { Notify_Joint_Changed ( ); });
    bind_drag_only_slider (slider);

    m_joint_sliders[i] = slider;
    m_joint_value_labels[i] = value_label;

    sizer->Add (row_header, 0, wxEXPAND | wxTOP, i == 0 ? 0 : 10);
    sizer->Add (slider, 0, wxEXPAND | wxTOP, 2);
  }

  SetSizer (sizer);
}

void Joint_Control_Panel::Set_On_Joint_Changed (
  std::function<void ( )> callback)
{
  m_on_joint_changed = std::move (callback);
}

std::array<double, 6> Joint_Control_Panel::Read_Input_Angles ( ) const
{
  std::array<double, 6> input_angles_deg = {};

  for( size_t i = 0; i < m_joint_sliders.size ( ); ++i )
  {
    const int value = m_joint_sliders[i] ? m_joint_sliders[i]->GetValue ( ) : 0;
    input_angles_deg[i] = static_cast<double> (value);
  }

  return input_angles_deg;
}

void Joint_Control_Panel::Set_Input_Angles (
  const std::array<double, 6>& input_angles_deg)
{
  for( size_t i = 0; i < m_joint_sliders.size ( ); ++i )
  {
    auto* slider = m_joint_sliders[i];
    if( slider == nullptr )
    {
      continue;
    }

    const int value = std::clamp (
      static_cast<int> (std::lround (input_angles_deg[i])),
      slider->GetMin ( ),
      slider->GetMax ( ));
    slider->SetValue (value);
  }
}

void Joint_Control_Panel::Set_Joint_Range_And_Value (
  size_t index,
  int min_value,
  int max_value,
  int value)
{
  if( index >= m_joint_sliders.size ( ) || m_joint_sliders[index] == nullptr )
  {
    return;
  }

  auto* slider = m_joint_sliders[index];
  slider->SetRange (min_value, max_value);
  slider->SetValue (std::clamp (value, min_value, max_value));
}

void Joint_Control_Panel::Set_Joint_Value_Label (
  size_t index,
  double input_angle,
  double effective_angle)
{
  if( index >= m_joint_value_labels.size ( ) || !m_joint_value_labels[index] )
  {
    return;
  }

  m_joint_value_labels[index]->SetLabel (
    wxString::Format ("%.0f / %.0f deg", input_angle, effective_angle));
}

void Joint_Control_Panel::Set_Joint_Controls_Enabled (bool enabled)
{
  for( auto* slider : m_joint_sliders )
  {
    if( slider )
    {
      slider->Enable (enabled);
    }
  }
}

void Joint_Control_Panel::Notify_Joint_Changed ( )
{
  if( m_on_joint_changed )
  {
    m_on_joint_changed ( );
  }
}

#include "cartesian_pose_panel.h"

#include "fine_adjust_control.h"

#include <wx/button.h>
#include <wx/clipbrd.h>
#include <wx/dataobj.h>
#include <wx/sizer.h>
#include <wx/slider.h>
#include <wx/stattext.h>
#include <wx/tglbtn.h>

#include <Windows.h>
#include <CommCtrl.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace
{

const std::vector<double> POSITION_FINE_STEPS = {
  0.01, 0.1, 0.5, 1.0, 5.0, 10.0
};
const std::vector<double> ANGLE_FINE_STEPS = {
  0.01, 0.1, 0.5, 1.0, 5.0
};

bool is_slider_thumb_hit (wxSlider* slider, const wxMouseEvent& event)
{
  if( slider == nullptr ) return false;
  HWND hwnd = reinterpret_cast<HWND> (slider->GetHandle ( ));
  if( hwnd == nullptr ) return true;

  RECT thumb_rect = { };
  SendMessage (hwnd, TBM_GETTHUMBRECT, 0,
               reinterpret_cast<LPARAM> (&thumb_rect));
  if( thumb_rect.right <= thumb_rect.left ||
      thumb_rect.bottom <= thumb_rect.top ) return true;

  InflateRect (&thumb_rect, 4, 6);
  const auto pos = event.GetPosition ( );
  const POINT point = { pos.x, pos.y };
  return PtInRect (&thumb_rect, point) != FALSE;
}

} // namespace

Cartesian_Pose_Panel::Cartesian_Pose_Panel (wxWindow* parent)
  : wxPanel (parent, wxID_ANY)
{
  auto* root = new wxBoxSizer (wxVERTICAL);
  m_title_text = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (
      u8"工具世界坐标位姿控制（XYZABC）"));
  root->Add (m_title_text, 0, wxBOTTOM, 8);

  const std::array<const char*, 6> names = { "X", "Y", "Z", "A", "B", "C" };
  for( size_t index = 0; index < names.size ( ); ++index )
  {
    auto* row_header = new wxBoxSizer (wxHORIZONTAL);
    auto* name_label = new wxStaticText (this, wxID_ANY, names[index]);
    auto* value_label = new wxStaticText (this, wxID_ANY, "--");
    value_label->SetMinSize (wxSize (100, -1));
    row_header->Add (name_label, 0, wxALIGN_CENTER_VERTICAL);
    row_header->AddStretchSpacer (1);
    row_header->Add (value_label, 0, wxALIGN_CENTER_VERTICAL);

    const bool position = index < 3;
    auto* slider = new wxSlider (
      this, wxID_ANY, 0,
      position ? -2000 : -180,
      position ? 2000 : 180,
      wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL);
    slider->Bind (wxEVT_SLIDER,
                  [this, index] (wxCommandEvent&)
                  {
                    Notify_Pose_Changed (index);
                  });
    slider->Bind (wxEVT_LEFT_DOWN,
                  [this, slider, index] (wxMouseEvent& event)
                  {
                    if( !is_slider_thumb_hit (slider, event) ) return;
                    Begin_Pose_Drag (index);
                    event.Skip ( );
                  });
    slider->Bind (wxEVT_LEFT_UP,
                  [this, index] (wxMouseEvent& event)
                  {
                    End_Pose_Drag (index);
                    event.Skip ( );
                  });
    slider->Bind (wxEVT_LEFT_DCLICK,
                  [slider] (wxMouseEvent& event)
                  {
                    if( is_slider_thumb_hit (slider, event) ) event.Skip ( );
                  });

    auto* fine_adjust = new Fine_Adjust_Control (
      this,
      position ? POSITION_FINE_STEPS : ANGLE_FINE_STEPS,
      position ? wxString (" mm") : wxString::FromUTF8 (u8"°"),
      0.1);
    fine_adjust->Set_On_Adjust (
      [this, index] (double delta) { Adjust_Pose (index, delta); });

    m_value_labels[index] = value_label;
    m_pose_sliders[index] = slider;
    m_fine_adjust_controls[index] = fine_adjust;
    root->Add (row_header, 0, wxEXPAND | wxTOP, index == 0 ? 0 : 6);
    auto* control_row = new wxBoxSizer (wxHORIZONTAL);
    control_row->Add (slider, 1, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
    control_row->Add (fine_adjust, 0, wxALIGN_CENTER_VERTICAL);
    root->Add (control_row, 0, wxEXPAND | wxTOP, 2);
  }

  m_warning_text = new wxStaticText (this, wxID_ANY, "");
  m_warning_text->SetForegroundColour (wxColour (210, 120, 20));
  root->Add (m_warning_text, 0, wxEXPAND | wxTOP | wxBOTTOM, 6);

  auto* buttons = new wxBoxSizer (wxHORIZONTAL);
  auto* copy_button = new wxButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"复制位姿"));
  m_world_frame_button = new wxToggleButton (
    this, wxID_ANY, wxString::FromUTF8 (u8"显示世界坐标系"));
  copy_button->Bind (wxEVT_BUTTON, &Cartesian_Pose_Panel::On_Copy, this);
  m_world_frame_button->Bind (
    wxEVT_TOGGLEBUTTON, &Cartesian_Pose_Panel::On_Toggle_World_Frame, this);
  buttons->Add (copy_button, 0, wxRIGHT, 6);
  buttons->Add (m_world_frame_button, 0);
  root->Add (buttons, 0, wxEXPAND);
  SetSizer (root);
}

void Cartesian_Pose_Panel::Set_World_From_Flange (
  const robot_model::Matrix4& world_from_flange)
{
  m_actual_pose =
    robot_model::Build_Xyzabc_From_Zyx_Matrix (world_from_flange);
  m_has_actual_pose = true;
  m_has_pose = true;
  // IK feedback is an achieved pose, not a new command. During a slider drag
  // keep the command pose anchored to the pose captured on mouse-down so that
  // solver residuals on the other five axes cannot become the next target.
  if( m_pose_dragging ) return;
  m_pose = m_actual_pose;
  Update_Sliders ( );
  Update_Labels ( );
}

void Cartesian_Pose_Panel::Set_Control_Frame_Name(const std::string &name)
{
  if (!m_title_text)
  {
    return;
  }
  m_title_text->SetLabel(
    wxString::FromUTF8(u8"工具世界坐标位姿控制（") +
    wxString::FromUTF8(name.c_str()) +
    wxString::FromUTF8(u8" / XYZABC）"));
}

void Cartesian_Pose_Panel::Clear ( )
{
  m_has_pose = false;
  m_has_actual_pose = false;
  m_pose_dragging = false;
  Update_Labels ( );
}

void Cartesian_Pose_Panel::Set_On_World_Frame_Visibility_Changed (
  std::function<void (bool)> callback)
{
  m_on_world_frame_visibility_changed = std::move (callback);
}

void Cartesian_Pose_Panel::Set_On_Pose_Changed (
  std::function<void (const robot_model::XyzabcPose&)> callback)
{
  m_on_pose_changed = std::move (callback);
}

void Cartesian_Pose_Panel::Set_Position_Range (int absolute_limit_mm)
{
  absolute_limit_mm = std::max (absolute_limit_mm, 1);
  for( size_t index = 0; index < 3; ++index )
  {
    if( m_pose_sliders[index] )
    {
      m_pose_sliders[index]->SetRange (
        -absolute_limit_mm, absolute_limit_mm);
    }
  }
  Update_Sliders ( );
}

void Cartesian_Pose_Panel::Set_Pose_Controls_Enabled (bool enabled)
{
  for( auto* slider : m_pose_sliders )
  {
    if( slider ) slider->Enable (enabled);
  }
  for( auto* fine_adjust : m_fine_adjust_controls )
  {
    if( fine_adjust ) fine_adjust->Set_Adjust_Enabled (enabled);
  }
}

void Cartesian_Pose_Panel::Update_Sliders ( )
{
  if( !m_has_pose ) return;
  m_updating_controls = true;
  for( size_t index = 0; index < m_pose_sliders.size ( ); ++index )
  {
    auto* slider = m_pose_sliders[index];
    if( !slider ) continue;
    slider->SetValue (std::clamp (
      static_cast<int> (std::lround (m_pose[index])),
      slider->GetMin ( ), slider->GetMax ( )));
  }
  m_updating_controls = false;
}

void Cartesian_Pose_Panel::Update_Labels ( )
{
  if( !m_has_pose )
  {
    for( auto* label : m_value_labels ) if( label ) label->SetLabel ("--");
    m_warning_text->SetLabel ("");
    return;
  }
  for( int index = 0; index < 6; ++index )
  {
    m_value_labels[index]->SetLabel (index < 3
      ? wxString::Format ("%.3f mm", m_pose[index])
      : wxString::Format (
          wxString::FromUTF8 (u8"%.3f°"), m_pose[index]));
  }
  const double b_radians = m_pose[4] * 3.14159265358979323846 / 180.0;
  m_warning_text->SetLabel (
    std::abs (std::cos (b_radians)) < 0.02
      ? wxString::FromUTF8 (u8"提示：姿态接近 ZYX 欧拉角奇异点")
      : wxString (""));
}

void Cartesian_Pose_Panel::Notify_Pose_Changed (size_t index)
{
  if( m_updating_controls || !m_has_pose ||
      index >= m_pose_sliders.size ( ) || !m_pose_sliders[index] ) return;
  auto target = m_pose_dragging && index == m_pose_drag_index
    ? m_drag_start_pose
    : m_pose;
  target[index] = static_cast<double> (m_pose_sliders[index]->GetValue ( ));
  if( m_pose_dragging && index == m_pose_drag_index )
  {
    m_pose = target;
    Update_Labels ( );
  }
  if( m_on_pose_changed ) m_on_pose_changed (target);
}

void Cartesian_Pose_Panel::Begin_Pose_Drag (size_t index)
{
  if( !m_has_pose || index >= m_pose_sliders.size ( ) ) return;
  m_pose_dragging = true;
  m_pose_drag_index = index;
  m_drag_start_pose = m_pose;
}

void Cartesian_Pose_Panel::End_Pose_Drag (size_t index)
{
  if( !m_pose_dragging || index != m_pose_drag_index ) return;
  m_pose_dragging = false;
  if( m_has_actual_pose ) m_pose = m_actual_pose;
  Update_Sliders ( );
  Update_Labels ( );
}

void Cartesian_Pose_Panel::Adjust_Pose (size_t index, double delta)
{
  if( !m_has_pose || index >= m_pose_sliders.size ( ) ||
      !m_pose_sliders[index] ) return;
  auto* slider = m_pose_sliders[index];
  auto target = m_pose;
  target[index] = std::clamp (
    target[index] + delta,
    static_cast<double> (slider->GetMin ( )),
    static_cast<double> (slider->GetMax ( )));
  if( std::abs (target[index] - m_pose[index]) < 1.0e-12 ) return;
  if( m_on_pose_changed ) m_on_pose_changed (target);
}

void Cartesian_Pose_Panel::On_Copy (wxCommandEvent&)
{
  if( !m_has_pose || !wxTheClipboard->Open ( ) ) return;
  wxTheClipboard->SetData (new wxTextDataObject (wxString::FromUTF8 (
    robot_model::Format_Xyzabc_Pose_Csv (m_pose).c_str ( ))));
  wxTheClipboard->Close ( );
}

void Cartesian_Pose_Panel::On_Toggle_World_Frame (wxCommandEvent&)
{
  const bool visible = m_world_frame_button->GetValue ( );
  m_world_frame_button->SetLabel (wxString::FromUTF8 (
    visible ? u8"隐藏世界坐标系" : u8"显示世界坐标系"));
  if( m_on_world_frame_visibility_changed )
  {
    m_on_world_frame_visibility_changed (visible);
  }
}

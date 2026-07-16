#include "cartesian_pose_panel.h"

#include <wx/button.h>
#include <wx/clipbrd.h>
#include <wx/dataobj.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/tglbtn.h>

#include <cmath>
#include <utility>

Cartesian_Pose_Panel::Cartesian_Pose_Panel (wxWindow* parent)
  : wxPanel (parent, wxID_ANY)
{
  auto* root = new wxBoxSizer (wxVERTICAL);
  auto* title = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (u8"法兰全局位姿（机器人世界坐标）"));
  root->Add (title, 0, wxBOTTOM, 6);

  auto* grid = new wxFlexGridSizer (3, 4, 4, 10);
  const std::array<const char*, 6> names = { "X", "Y", "Z", "A", "B", "C" };
  for( int row = 0; row < 3; ++row )
  {
    for( int group = 0; group < 2; ++group )
    {
      const int index = row + group * 3;
      grid->Add (new wxStaticText (this, wxID_ANY, names[index]),
                 0, wxALIGN_CENTER_VERTICAL);
      auto* value = new wxStaticText (this, wxID_ANY, "--");
      value->SetMinSize (wxSize (120, -1));
      m_value_labels[index] = value;
      grid->Add (value, 0, wxALIGN_CENTER_VERTICAL);
    }
  }
  root->Add (grid, 0, wxEXPAND | wxBOTTOM, 6);

  m_warning_text = new wxStaticText (this, wxID_ANY, "");
  m_warning_text->SetForegroundColour (wxColour (210, 120, 20));
  root->Add (m_warning_text, 0, wxEXPAND | wxBOTTOM, 6);

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
  m_pose = robot_model::Build_Xyzabc_From_Zyx_Matrix (world_from_flange);
  m_has_pose = true;
  Update_Labels ( );
}

void Cartesian_Pose_Panel::Clear ( )
{
  m_has_pose = false;
  Update_Labels ( );
}

void Cartesian_Pose_Panel::Set_On_World_Frame_Visibility_Changed (
  std::function<void (bool)> callback)
{
  m_on_world_frame_visibility_changed = std::move (callback);
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
    const char* unit = index < 3 ? " mm" : " deg";
    m_value_labels[index]->SetLabel (
      wxString::Format ("%.3f%s", m_pose[index], unit));
  }
  const double b_radians = m_pose[4] * 3.14159265358979323846 / 180.0;
  m_warning_text->SetLabel (
    std::abs (std::cos (b_radians)) < 0.02
      ? wxString::FromUTF8 (u8"提示：姿态接近 ZYX 欧拉角奇异点")
      : wxString (""));
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

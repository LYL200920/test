#ifndef includeguard_cartesian_pose_panel_h_includeguard
#define includeguard_cartesian_pose_panel_h_includeguard

#include "pose_transform.h"

#include <wx/panel.h>

#include <array>
#include <functional>

class wxStaticText;
class wxToggleButton;

class Cartesian_Pose_Panel : public wxPanel
{
public:
  explicit Cartesian_Pose_Panel (wxWindow* parent);

  void Set_World_From_Flange (const robot_model::Matrix4& world_from_flange);
  void Clear ( );
  void Set_On_World_Frame_Visibility_Changed (
    std::function<void (bool)> callback);

private:
  void Update_Labels ( );
  void On_Copy (wxCommandEvent& event);
  void On_Toggle_World_Frame (wxCommandEvent& event);

private:
  std::array<wxStaticText*, 6> m_value_labels = { };
  wxStaticText* m_warning_text = nullptr;
  wxToggleButton* m_world_frame_button = nullptr;
  robot_model::XyzabcPose m_pose = { };
  std::function<void (bool)> m_on_world_frame_visibility_changed;
  bool m_has_pose = false;
};

#endif

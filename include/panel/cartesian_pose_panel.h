#ifndef includeguard_cartesian_pose_panel_h_includeguard
#define includeguard_cartesian_pose_panel_h_includeguard

#include "pose_transform.h"

#include <wx/panel.h>

#include <array>
#include <functional>

class wxStaticText;
class wxToggleButton;
class wxSlider;
class Fine_Adjust_Control;

class Cartesian_Pose_Panel : public wxPanel
{
public:
  explicit Cartesian_Pose_Panel(wxWindow *parent);

  void Set_World_From_Flange(const robot_model::Matrix4 &world_from_flange);
  void Clear();
  void Set_On_World_Frame_Visibility_Changed(std::function<void(bool)> callback);
  void Set_On_Pose_Changed(std::function<void(const robot_model::XyzabcPose &)> callback);
  void Set_Position_Range(int absolute_limit_mm);
  void Set_Pose_Controls_Enabled(bool enabled);

private:
  void Update_Labels();
  void Update_Sliders();
  void On_Copy(wxCommandEvent &event);
  void On_Toggle_World_Frame(wxCommandEvent &event);
  void Begin_Pose_Drag(size_t index);
  void End_Pose_Drag(size_t index);
  void Notify_Pose_Changed(size_t index);
  void Adjust_Pose(size_t index, double delta);

private:
  std::array<wxStaticText *, 6> m_value_labels = {};
  std::array<wxSlider *, 6> m_pose_sliders = {};
  std::array<Fine_Adjust_Control *, 6> m_fine_adjust_controls = {};
  wxStaticText *m_warning_text = nullptr;
  wxToggleButton *m_world_frame_button = nullptr;
  robot_model::XyzabcPose m_pose = {};
  robot_model::XyzabcPose m_actual_pose = {};
  robot_model::XyzabcPose m_drag_start_pose = {};
  std::function<void(bool)> m_on_world_frame_visibility_changed;
  std::function<void(const robot_model::XyzabcPose &)> m_on_pose_changed;
  bool m_has_pose = false;
  bool m_has_actual_pose = false;
  bool m_pose_dragging = false;
  size_t m_pose_drag_index = 0;
  bool m_updating_controls = false;
};

#endif

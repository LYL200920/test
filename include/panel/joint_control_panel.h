#ifndef includeguard_joint_control_panel_h_includeguard
#define includeguard_joint_control_panel_h_includeguard

#include <wx/panel.h>
#include <wx/string.h>

#include <array>
#include <cstddef>
#include <functional>

class wxSlider;
class wxStaticText;
class Fine_Adjust_Control;

class Joint_Control_Panel : public wxPanel
{
public:
  explicit Joint_Control_Panel(wxWindow *parent);

  void Set_On_Joint_Changed(std::function<void()> callback);

  std::array<double, 6> Read_Input_Angles() const;
  void Set_Input_Angles(const std::array<double, 6> &input_angles_deg);
  void Set_Joint_Range_And_Value(size_t index,
                                 int min_value,
                                 int max_value,
                                 int value);
  void Set_Joint_Value_Label(size_t index, double input_angle, double effective_angle);
  void Set_Joint_Controls_Enabled(bool enabled);

private:
  void On_Joint_Slider_Changed(size_t index);
  void Adjust_Joint(size_t index, double delta_deg);
  void Notify_Joint_Changed();

private:
  std::function<void()> m_on_joint_changed;
  std::array<wxSlider *, 6> m_joint_sliders = {};
  std::array<wxStaticText *, 6> m_joint_value_labels = {};
  std::array<Fine_Adjust_Control *, 6> m_fine_adjust_controls = {};
  std::array<double, 6> m_input_angles_deg = {};
};

#endif

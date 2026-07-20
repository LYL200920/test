#ifndef includeguard_fine_adjust_control_h_includeguard
#define includeguard_fine_adjust_control_h_includeguard

#include <wx/panel.h>

#include <functional>
#include <vector>

class wxButton;
class wxChoice;

// Compact reusable control: decrement / precision choice / increment.
class Fine_Adjust_Control : public wxPanel
{
public:
  Fine_Adjust_Control (wxWindow* parent,
                       const std::vector<double>& steps,
                       const wxString& unit,
                       double default_step = 0.1);

  void Set_On_Adjust (std::function<void (double)> callback);
  void Set_Adjust_Enabled (bool enabled);
  double Selected_Step ( ) const;

private:
  void Notify_Adjust (double direction);

private:
  wxButton* m_decrease_button = nullptr;
  wxChoice* m_step_choice = nullptr;
  wxButton* m_increase_button = nullptr;
  std::vector<double> m_steps;
  std::function<void (double)> m_on_adjust;
};

#endif

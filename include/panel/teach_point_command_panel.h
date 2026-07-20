#ifndef includeguard_teach_point_command_panel_h_includeguard
#define includeguard_teach_point_command_panel_h_includeguard

#include <wx/panel.h>

#include <functional>

class wxButton;

class Teach_Point_Command_Panel : public wxPanel
{
public:
  explicit Teach_Point_Command_Panel (wxWindow* parent);

  void Set_On_Record (std::function<void ( )> callback);
  void Set_Record_Enabled (bool enabled);

private:
  wxButton* m_record_button = nullptr;
  std::function<void ( )> m_on_record;
};

#endif

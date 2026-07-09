#ifndef includeguard_camera_parameter_panel_h_includeguard
#define includeguard_camera_parameter_panel_h_includeguard

#include "camera_parameter.h"

#include <wx/scrolwin.h>

#include <functional>
#include <string>
#include <vector>

class wxBoxSizer;
class wxButton;
class wxPanel;
class wxStaticText;
class wxWindow;

class Camera_Parameter_Panel : public wxScrolledWindow
{
public:
  struct Callbacks
  {
    std::function<void ( )> refresh;
    std::function<void (std::vector<Camera_Parameter_Update>)> apply;
  };

  explicit Camera_Parameter_Panel (wxWindow* parent);

  void Set_Callbacks (Callbacks callbacks);
  void Update_View (
    const std::vector<Camera_Parameter>& parameters,
    bool parameters_loaded,
    const std::string& parameter_status,
    bool device_open,
    bool grabbing,
    bool busy);

private:
  struct Editor
  {
    Camera_Parameter parameter;
    wxWindow* control = nullptr;
  };

  bool Needs_Rebuild (
    const std::vector<Camera_Parameter>& parameters) const;
  void Rebuild_Editors (const std::vector<Camera_Parameter>& parameters);
  wxWindow* Create_Editor_Control (const Camera_Parameter& parameter);
  wxString Range_Label (const Camera_Parameter& parameter) const;
  bool Collect_Updates (
    std::vector<Camera_Parameter_Update>* updates,
    wxString* error_message) const;
  void On_Refresh (wxCommandEvent& event);
  void On_Apply (wxCommandEvent& event);

private:
  Callbacks m_callbacks;
  wxPanel* m_fields_panel = nullptr;
  wxBoxSizer* m_fields_sizer = nullptr;
  wxStaticText* m_hint_text = nullptr;
  wxButton* m_refresh_button = nullptr;
  wxButton* m_apply_button = nullptr;
  std::vector<Editor> m_editors;
};

#endif

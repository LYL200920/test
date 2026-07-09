#ifndef includeguard_camera_info_panel_h_includeguard
#define includeguard_camera_info_panel_h_includeguard

#include "camera_device_detail.h"

#include <wx/scrolwin.h>

#include <functional>
#include <optional>

class wxButton;
class wxStaticText;

class Camera_Info_Panel : public wxScrolledWindow
{
public:
  explicit Camera_Info_Panel (wxWindow* parent);

  void Set_On_Refresh (std::function<void ( )> callback);
  void Update_View (
    const std::optional<Camera_Device_Detail>& detail,
    bool device_open,
    bool busy);

private:
  void On_Refresh (wxCommandEvent& event);

private:
  std::function<void ( )> m_on_refresh;
  wxStaticText* m_detail_text = nullptr;
  wxButton* m_refresh_button = nullptr;
};

#endif

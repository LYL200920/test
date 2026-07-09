#ifndef includeguard_camera_config_dialog_h_includeguard
#define includeguard_camera_config_dialog_h_includeguard

#include <wx/dialog.h>

#include <cstdint>

class Camera_Service;
class wxButton;
class wxListCtrl;
class wxListEvent;
class wxShowEvent;
class wxStaticText;

class Camera_Config_Dialog : public wxDialog
{
public:
  Camera_Config_Dialog(wxWindow *parent, Camera_Service &camera_service);
  ~Camera_Config_Dialog() override;

private:
  void Build_Ui();
  void Load_Devices();
  void Populate_Device_List();
  void Set_Busy(bool busy);
  void Update_State_Text();
  void Update_Selection_State();
  long Selected_Row() const;

  void On_Show(wxShowEvent &event);
  void On_Refresh(wxCommandEvent &event);
  void On_Device_Selection_Changed(wxListEvent &event);
  void On_Device_Activated(wxListEvent &event);
  void On_Confirm(wxCommandEvent &event);
  void On_Camera_Updated(wxThreadEvent &event);

private:
  Camera_Service &m_camera_service;
  wxStaticText *m_version_text = nullptr;
  wxStaticText *m_state_text = nullptr;
  wxStaticText *m_status_text = nullptr;
  wxListCtrl *m_device_list = nullptr;
  wxButton *m_refresh_button = nullptr;
  wxButton *m_ok_button = nullptr;
  bool m_initial_load_started = false;
  bool m_busy = false;
  bool m_close_on_success = false;
  std::uint64_t m_pending_request = 0;
};

#endif

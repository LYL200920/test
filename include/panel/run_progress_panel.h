#ifndef includeguard_run_progress_panel_h_includeguard
#define includeguard_run_progress_panel_h_includeguard

#include <wx/panel.h>

#include <chrono>
#include <functional>
#include <string>

class wxButton;
class wxCheckBox;
class wxStaticText;

class Run_Progress_Panel : public wxPanel
{
public:
  struct Callbacks
  {
    std::function<void()> start;
    std::function<void()> emergency_stop;
  };

  explicit Run_Progress_Panel(wxWindow *parent);

  void Set_Callbacks(Callbacks callbacks);
  bool Save_Images() const;
  void Set_Progress_Ready(bool ready);
  void Set_Robot_Ready(bool ready, const std::string &reason);
  void Set_Running(bool running, bool stopping = false);
  void Set_Elapsed(std::chrono::milliseconds elapsed);
  void Set_Status(const std::string &status, bool error = false);

private:
  void Refresh_Run_Button();

private:
  Callbacks m_callbacks;
  wxStaticText *m_ct_value = nullptr;
  wxCheckBox *m_save_images = nullptr;
  wxButton *m_run_button = nullptr;
  wxStaticText *m_status = nullptr;
  bool m_progress_ready = false;
  bool m_robot_ready = false;
  bool m_running = false;
  bool m_stopping = false;
  std::string m_robot_reason;
};

#endif

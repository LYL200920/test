#ifndef includeguard_run_progress_panel_h_includeguard
#define includeguard_run_progress_panel_h_includeguard

#include <wx/panel.h>

#include <chrono>
#include <functional>
#include <string>

class wxButton;
class wxCheckBox;
class wxChoice;
class wxSlider;
class wxStaticBitmap;
class wxStaticText;
class wxImage;

class Run_Progress_Panel : public wxPanel
{
public:
  enum class Motion_Mode
  {
    Ptp,
    Linear
  };

  struct Callbacks
  {
    std::function<void()> start;
    std::function<void()> emergency_stop;
  };

  explicit Run_Progress_Panel(wxWindow *parent);

  void Set_Callbacks(Callbacks callbacks);
  bool Save_Images() const;
  bool Build_Mosaic() const;
  Motion_Mode Selected_Motion_Mode() const;
  int Motion_Speed() const;
  void Set_Progress_Ready(bool ready);
  void Set_Robot_Ready(bool ready, const std::string &reason);
  void Set_Running(bool running, bool stopping = false);
  void Set_Elapsed(std::chrono::milliseconds elapsed);
  void Set_Status(const std::string &status, bool error = false);
  void Clear_Mosaic();
  void Set_Mosaic_Image(const wxImage &image);

private:
  void Refresh_Run_Button();
  void Refresh_Motion_Controls();

private:
  Callbacks m_callbacks;
  wxStaticText *m_ct_value = nullptr;
  wxCheckBox *m_save_images = nullptr;
  wxCheckBox *m_build_mosaic = nullptr;
  wxChoice *m_motion_mode = nullptr;
  wxSlider *m_motion_speed = nullptr;
  wxStaticText *m_motion_speed_label = nullptr;
  wxButton *m_run_button = nullptr;
  wxStaticText *m_status = nullptr;
  wxStaticText *m_mosaic_title = nullptr;
  wxStaticBitmap *m_mosaic_bitmap = nullptr;
  bool m_progress_ready = false;
  bool m_robot_ready = false;
  bool m_running = false;
  bool m_stopping = false;
  int m_ptp_speed_percent = 100;
  int m_linear_speed_mm_s = 500;
  std::string m_robot_reason;
};

#endif

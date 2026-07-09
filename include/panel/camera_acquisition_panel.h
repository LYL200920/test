#ifndef includeguard_camera_acquisition_panel_h_includeguard
#define includeguard_camera_acquisition_panel_h_includeguard

#include "camera_stream_config.h"

#include <wx/scrolwin.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

enum class Camera_State;
struct Camera_Frame;
class wxButton;
class wxStaticText;

class Camera_Acquisition_Panel : public wxScrolledWindow
{
public:
  struct Callbacks
  {
    std::function<void ( )> start;
    std::function<void ( )> stop;
    std::function<void ( )> close;
    std::function<void ( )> refresh_streams;
  };

  explicit Camera_Acquisition_Panel (wxWindow* parent);

  void Set_Callbacks (Callbacks callbacks);
  void Update_View (
    Camera_State state,
    bool device_open,
    bool busy,
    const std::string& serial_number,
    const std::vector<Camera_Stream_Config>& stream_configuration,
    bool stream_configuration_loaded);
  void Update_Frame (const std::shared_ptr<const Camera_Frame>& frame);

private:
  void On_Start (wxCommandEvent& event);
  void On_Stop (wxCommandEvent& event);
  void On_Close (wxCommandEvent& event);
  void On_Refresh_Streams (wxCommandEvent& event);

private:
  Callbacks m_callbacks;
  wxStaticText* m_device_text = nullptr;
  wxStaticText* m_state_text = nullptr;
  wxStaticText* m_stream_config_text = nullptr;
  wxStaticText* m_frame_text = nullptr;
  wxButton* m_refresh_streams_button = nullptr;
  wxButton* m_start_button = nullptr;
  wxButton* m_stop_button = nullptr;
  wxButton* m_close_button = nullptr;
  std::uint64_t m_last_generation = 0;
  std::chrono::steady_clock::time_point m_last_frame_time{};
  double m_measured_fps = 0.0;
};

#endif

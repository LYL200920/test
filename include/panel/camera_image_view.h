#ifndef includeguard_camera_image_view_h_includeguard
#define includeguard_camera_image_view_h_includeguard

#include "camera_image_converter.h"

#include <wx/panel.h>
#include <wx/timer.h>

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

class Camera_Bitmap_Canvas;
class Camera_Service;
class wxChoice;
class wxStaticText;

class Camera_Image_View : public wxPanel
{
public:
  Camera_Image_View (wxWindow* parent, Camera_Service& camera_service);
  ~Camera_Image_View ( ) override;

private:
  struct Conversion_Job
  {
    std::uint64_t id = 0;
    std::uint64_t generation = 0;
    Camera_Image_Display_Mode mode = Camera_Image_Display_Mode::Automatic;
    std::shared_ptr<const Camera_Frame> frame;
  };

  struct Conversion_Result
  {
    std::uint64_t id = 0;
    std::uint64_t generation = 0;
    Camera_Image_Display_Mode mode = Camera_Image_Display_Mode::Automatic;
    bool success = false;
    std::string error;
    Camera_Display_Image image;
  };

  Camera_Image_Display_Mode Selected_Mode ( ) const;
  void Submit_Frame (std::shared_ptr<const Camera_Frame> frame);
  void Worker_Loop ( );
  void Consume_Result ( );
  void On_Mode_Changed (wxCommandEvent& event);
  void On_Timer (wxTimerEvent& event);

private:
  Camera_Service& m_camera_service;
  Camera_Bitmap_Canvas* m_canvas = nullptr;
  wxChoice* m_mode_choice = nullptr;
  wxStaticText* m_status_text = nullptr;
  wxTimer m_timer;

  std::thread m_worker;
  std::mutex m_worker_mutex;
  std::condition_variable m_worker_ready;
  bool m_exit_requested = false;
  std::optional<Conversion_Job> m_pending_job;
  std::optional<Conversion_Result> m_pending_result;
  std::uint64_t m_next_job_id = 1;
  std::uint64_t m_last_submitted_generation = 0;
  Camera_Image_Display_Mode m_last_submitted_mode =
    Camera_Image_Display_Mode::Automatic;
  std::uint64_t m_last_displayed_job = 0;
};

#endif

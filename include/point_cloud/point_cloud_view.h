#ifndef includeguard_point_cloud_view_h_includeguard
#define includeguard_point_cloud_view_h_includeguard

#include "camera_point_cloud_converter.h"

#include <wx/panel.h>
#include <wx/timer.h>

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

class Camera_Service;
class Point_Cloud_Canvas;
class wxButton;
class wxStaticText;

class Point_Cloud_View : public wxPanel
{
public:
  Point_Cloud_View (wxWindow* parent, Camera_Service& camera_service);
  ~Point_Cloud_View ( ) override;

private:
  enum class Source_Mode
  {
    None,
    Live_Camera,
    Ply_File
  };

  struct Conversion_Job
  {
    std::uint64_t id = 0;
    std::uint64_t epoch = 0;
    std::uint64_t generation = 0;
    std::shared_ptr<const Camera_Frame> frame;
  };

  struct Conversion_Result
  {
    std::uint64_t id = 0;
    std::uint64_t epoch = 0;
    std::uint64_t generation = 0;
    bool success = false;
    std::string error;
    Camera_Point_Cloud cloud;
  };

  void Submit_Frame (std::shared_ptr<const Camera_Frame> frame);
  void Worker_Loop ( );
  void Consume_Result ( );
  void On_Live_Camera (wxCommandEvent& event);
  void On_Load_Ply (wxCommandEvent& event);
  void On_Clear (wxCommandEvent& event);
  void On_Fit_View (wxCommandEvent& event);
  void On_Timer (wxTimerEvent& event);

private:
  Camera_Service& m_camera_service;
  Point_Cloud_Canvas* m_canvas = nullptr;
  wxButton* m_live_button = nullptr;
  wxStaticText* m_status_text = nullptr;
  wxTimer m_timer;
  Source_Mode m_source_mode = Source_Mode::Live_Camera;

  std::thread m_worker;
  std::mutex m_worker_mutex;
  std::condition_variable m_worker_ready;
  bool m_exit_requested = false;
  std::optional<Conversion_Job> m_pending_job;
  std::optional<Conversion_Result> m_pending_result;
  std::uint64_t m_next_job_id = 1;
  std::uint64_t m_live_epoch = 1;
  std::uint64_t m_last_submitted_generation = 0;
  std::uint64_t m_last_displayed_job = 0;
};

#endif

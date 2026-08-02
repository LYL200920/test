#ifndef includeguard_progress_run_controller_h_includeguard
#define includeguard_progress_run_controller_h_includeguard

#include <cstddef>
#include <string>
#include <vector>

namespace application
{

enum class Progress_Run_State
{
  Idle,
  Moving,
  Waiting_For_Image,
  Stopping,
  Processing_Images,
  Completed,
  Failed
};

enum class Progress_Run_Action
{
  None,
  Move_Point,
  Trigger_Image,
  Request_Stop,
  Start_Image_Processing,
  Finish
};

struct Progress_Run_Transition
{
  Progress_Run_State state = Progress_Run_State::Idle;
  Progress_Run_Action action = Progress_Run_Action::None;
  std::size_t point_index = 0;
  std::string message;
};

// Owns only the deterministic Progress workflow. Device commands, timers,
// image buffers and UI updates are effects performed by the caller from the
// returned action.
class Progress_Run_Controller
{
public:
  Progress_Run_Transition Start(std::vector<bool> capture_image_at_point);
  Progress_Run_Transition Motion_Completed();
  Progress_Run_Transition Image_Captured();
  Progress_Run_Transition Image_Timed_Out();
  Progress_Run_Transition Request_Emergency_Stop();
  Progress_Run_Transition Stop_Confirmed();
  Progress_Run_Transition Robot_Became_Unavailable(std::string message);
  Progress_Run_Transition Effect_Failed(std::string message);
  Progress_Run_Transition Image_Processing_Completed(
    bool success,
    std::string message = {});
  void Reset();

  Progress_Run_State State() const { return m_state; }
  std::size_t Current_Point_Index() const { return m_point_index; }
  std::size_t Point_Count() const { return m_capture_image_at_point.size(); }
  std::size_t Captured_Image_Count() const { return m_captured_image_count; }
  bool Is_Motion_Active() const;
  bool Is_Terminal() const;

private:
  Progress_Run_Transition Advance();
  Progress_Run_Transition Fail(std::string message);
  Progress_Run_Transition Transition(
    Progress_Run_Action action,
    std::string message = {}) const;

private:
  Progress_Run_State m_state = Progress_Run_State::Idle;
  std::vector<bool> m_capture_image_at_point;
  std::size_t m_point_index = 0;
  std::size_t m_captured_image_count = 0;
};

const char *To_String(Progress_Run_State state);

} // namespace application

#endif

#ifndef APPLICATION_CAMERA_CALIBRATION_PROGRESS_CONTROLLER_H_
#define APPLICATION_CAMERA_CALIBRATION_PROGRESS_CONTROLLER_H_

#include <cstddef>
#include <string>
#include <vector>

namespace application
{

enum class Camera_Calibration_Progress_State
{
  Idle,
  Moving,
  Settling,
  Waiting_For_Image,
  Processing_Image,
  Stopping,
  Completed,
  Failed
};

enum class Camera_Calibration_Progress_Action
{
  None,
  Move_Point,
  Wait_For_Settling,
  Trigger_Image,
  Process_Image,
  Request_Stop,
  Finish
};

struct Camera_Calibration_Progress_Transition
{
  Camera_Calibration_Progress_State state =
    Camera_Calibration_Progress_State::Idle;
  Camera_Calibration_Progress_Action action =
    Camera_Calibration_Progress_Action::None;
  std::size_t point_index = 0;
  std::size_t retry_index = 0;
  std::string message;
};

// Deterministic state machine for a robot-driven calibration capture. Device
// commands, timers, image conversion and UI updates remain caller effects.
class Camera_Calibration_Progress_Controller
{
public:
  Camera_Calibration_Progress_Transition Start(
    std::vector<bool> capture_image_at_point,
    std::size_t maximum_retries = 1);
  Camera_Calibration_Progress_Transition Motion_Completed();
  Camera_Calibration_Progress_Transition Settling_Completed();
  Camera_Calibration_Progress_Transition Image_Arrived();
  Camera_Calibration_Progress_Transition Image_Processed(bool accepted);
  Camera_Calibration_Progress_Transition Image_Timed_Out();
  Camera_Calibration_Progress_Transition Request_Stop();
  Camera_Calibration_Progress_Transition Stop_Confirmed();
  Camera_Calibration_Progress_Transition Effect_Failed(std::string message);
  void Reset();

  Camera_Calibration_Progress_State State() const { return state_; }
  std::size_t Current_Point_Index() const { return point_index_; }
  std::size_t Point_Count() const { return capture_image_at_point_.size(); }
  std::size_t Current_Retry_Index() const { return retry_index_; }
  std::size_t Accepted_Image_Count() const { return accepted_image_count_; }
  std::size_t Rejected_Point_Count() const { return rejected_point_count_; }
  bool Is_Active() const;
  bool Is_Terminal() const;

private:
  Camera_Calibration_Progress_Transition Advance();
  Camera_Calibration_Progress_Transition Fail(std::string message);
  Camera_Calibration_Progress_Transition Transition(
    Camera_Calibration_Progress_Action action,
    std::string message = {}) const;

private:
  Camera_Calibration_Progress_State state_ =
    Camera_Calibration_Progress_State::Idle;
  std::vector<bool> capture_image_at_point_;
  std::size_t point_index_ = 0;
  std::size_t retry_index_ = 0;
  std::size_t maximum_retries_ = 1;
  std::size_t accepted_image_count_ = 0;
  std::size_t rejected_point_count_ = 0;
};

const char *To_String(Camera_Calibration_Progress_State state);

} // namespace application

#endif // APPLICATION_CAMERA_CALIBRATION_PROGRESS_CONTROLLER_H_

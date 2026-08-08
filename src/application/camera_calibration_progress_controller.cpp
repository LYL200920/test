#include "camera_calibration_progress_controller.h"

#include <utility>

namespace application
{

Camera_Calibration_Progress_Transition
Camera_Calibration_Progress_Controller::Start(
  std::vector<bool> capture_image_at_point,
  std::size_t maximum_retries)
{
  if (Is_Active())
  {
    return Transition(
      Camera_Calibration_Progress_Action::None,
      "Calibration progress is already active.");
  }
  if (capture_image_at_point.empty())
  {
    capture_image_at_point_.clear();
    return Fail("Calibration progress has no runnable points.");
  }
  capture_image_at_point_ = std::move(capture_image_at_point);
  maximum_retries_ = maximum_retries;
  point_index_ = 0;
  retry_index_ = 0;
  accepted_image_count_ = 0;
  rejected_point_count_ = 0;
  state_ = Camera_Calibration_Progress_State::Moving;
  return Transition(Camera_Calibration_Progress_Action::Move_Point);
}

Camera_Calibration_Progress_Transition
Camera_Calibration_Progress_Controller::Motion_Completed()
{
  if (state_ != Camera_Calibration_Progress_State::Moving)
  {
    return Transition(Camera_Calibration_Progress_Action::None);
  }
  if (point_index_ >= capture_image_at_point_.size())
  {
    return Fail("Calibration progress point index is invalid.");
  }
  if (!capture_image_at_point_[point_index_])
  {
    return Advance();
  }
  state_ = Camera_Calibration_Progress_State::Settling;
  return Transition(Camera_Calibration_Progress_Action::Wait_For_Settling);
}

Camera_Calibration_Progress_Transition
Camera_Calibration_Progress_Controller::Settling_Completed()
{
  if (state_ != Camera_Calibration_Progress_State::Settling)
  {
    return Transition(Camera_Calibration_Progress_Action::None);
  }
  state_ = Camera_Calibration_Progress_State::Waiting_For_Image;
  return Transition(Camera_Calibration_Progress_Action::Trigger_Image);
}

Camera_Calibration_Progress_Transition
Camera_Calibration_Progress_Controller::Image_Arrived()
{
  if (state_ != Camera_Calibration_Progress_State::Waiting_For_Image)
  {
    return Transition(Camera_Calibration_Progress_Action::None);
  }
  state_ = Camera_Calibration_Progress_State::Processing_Image;
  return Transition(Camera_Calibration_Progress_Action::Process_Image);
}

Camera_Calibration_Progress_Transition
Camera_Calibration_Progress_Controller::Image_Processed(bool accepted)
{
  if (state_ != Camera_Calibration_Progress_State::Processing_Image)
  {
    return Transition(Camera_Calibration_Progress_Action::None);
  }
  if (accepted)
  {
    ++accepted_image_count_;
    return Advance();
  }
  if (retry_index_ < maximum_retries_)
  {
    ++retry_index_;
    state_ = Camera_Calibration_Progress_State::Settling;
    return Transition(
      Camera_Calibration_Progress_Action::Wait_For_Settling,
      "Calibration board was not detected; retrying the current point.");
  }
  ++rejected_point_count_;
  return Advance();
}

Camera_Calibration_Progress_Transition
Camera_Calibration_Progress_Controller::Image_Timed_Out()
{
  if (state_ != Camera_Calibration_Progress_State::Waiting_For_Image)
  {
    return Transition(Camera_Calibration_Progress_Action::None);
  }
  return Fail("Timed out waiting for a calibration camera image.");
}

Camera_Calibration_Progress_Transition
Camera_Calibration_Progress_Controller::Request_Stop()
{
  if (!Is_Active() || state_ == Camera_Calibration_Progress_State::Stopping)
  {
    return Transition(Camera_Calibration_Progress_Action::None);
  }
  state_ = Camera_Calibration_Progress_State::Stopping;
  return Transition(Camera_Calibration_Progress_Action::Request_Stop);
}

Camera_Calibration_Progress_Transition
Camera_Calibration_Progress_Controller::Stop_Confirmed()
{
  if (state_ != Camera_Calibration_Progress_State::Stopping)
  {
    return Transition(Camera_Calibration_Progress_Action::None);
  }
  return Fail("Calibration progress was stopped.");
}

Camera_Calibration_Progress_Transition
Camera_Calibration_Progress_Controller::Effect_Failed(std::string message)
{
  if (!Is_Active())
  {
    return Transition(Camera_Calibration_Progress_Action::None);
  }
  return Fail(std::move(message));
}

void Camera_Calibration_Progress_Controller::Reset()
{
  state_ = Camera_Calibration_Progress_State::Idle;
  capture_image_at_point_.clear();
  point_index_ = 0;
  retry_index_ = 0;
  accepted_image_count_ = 0;
  rejected_point_count_ = 0;
}

bool Camera_Calibration_Progress_Controller::Is_Active() const
{
  return state_ == Camera_Calibration_Progress_State::Moving ||
         state_ == Camera_Calibration_Progress_State::Settling ||
         state_ == Camera_Calibration_Progress_State::Waiting_For_Image ||
         state_ == Camera_Calibration_Progress_State::Processing_Image ||
         state_ == Camera_Calibration_Progress_State::Stopping;
}

bool Camera_Calibration_Progress_Controller::Is_Terminal() const
{
  return state_ == Camera_Calibration_Progress_State::Completed ||
         state_ == Camera_Calibration_Progress_State::Failed;
}

Camera_Calibration_Progress_Transition
Camera_Calibration_Progress_Controller::Advance()
{
  ++point_index_;
  retry_index_ = 0;
  if (point_index_ < capture_image_at_point_.size())
  {
    state_ = Camera_Calibration_Progress_State::Moving;
    return Transition(Camera_Calibration_Progress_Action::Move_Point);
  }
  state_ = Camera_Calibration_Progress_State::Completed;
  return Transition(Camera_Calibration_Progress_Action::Finish);
}

Camera_Calibration_Progress_Transition
Camera_Calibration_Progress_Controller::Fail(std::string message)
{
  state_ = Camera_Calibration_Progress_State::Failed;
  return Transition(
    Camera_Calibration_Progress_Action::Finish, std::move(message));
}

Camera_Calibration_Progress_Transition
Camera_Calibration_Progress_Controller::Transition(
  Camera_Calibration_Progress_Action action,
  std::string message) const
{
  return {state_, action, point_index_, retry_index_, std::move(message)};
}

const char *To_String(Camera_Calibration_Progress_State state)
{
  switch (state)
  {
    case Camera_Calibration_Progress_State::Idle: return "Idle";
    case Camera_Calibration_Progress_State::Moving: return "Moving";
    case Camera_Calibration_Progress_State::Settling: return "Settling";
    case Camera_Calibration_Progress_State::Waiting_For_Image:
      return "WaitingForImage";
    case Camera_Calibration_Progress_State::Processing_Image:
      return "ProcessingImage";
    case Camera_Calibration_Progress_State::Stopping: return "Stopping";
    case Camera_Calibration_Progress_State::Completed: return "Completed";
    case Camera_Calibration_Progress_State::Failed: return "Failed";
  }
  return "Unknown";
}

} // namespace application

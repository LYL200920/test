#include "progress_run_controller.h"

#include <utility>

namespace application
{

Progress_Run_Transition Progress_Run_Controller::Start(
  std::vector<bool> capture_image_at_point)
{
  if( Is_Motion_Active() || m_state == Progress_Run_State::Processing_Images )
    return Transition(Progress_Run_Action::None, "Progress is already active.");
  if( capture_image_at_point.empty() )
  {
    m_capture_image_at_point.clear();
    m_point_index = 0;
    m_captured_image_count = 0;
    return Fail("Progress has no runnable points.");
  }

  m_capture_image_at_point = std::move(capture_image_at_point);
  m_point_index = 0;
  m_captured_image_count = 0;
  m_state = Progress_Run_State::Moving;
  return Transition(Progress_Run_Action::Move_Point);
}

Progress_Run_Transition Progress_Run_Controller::Motion_Completed()
{
  if( m_state != Progress_Run_State::Moving )
    return Transition(Progress_Run_Action::None);
  if( m_point_index >= m_capture_image_at_point.size() )
    return Fail("Progress point index is invalid.");

  if( m_capture_image_at_point[m_point_index] )
  {
    m_state = Progress_Run_State::Waiting_For_Image;
    return Transition(Progress_Run_Action::Trigger_Image);
  }
  return Advance();
}

Progress_Run_Transition Progress_Run_Controller::Image_Captured()
{
  if( m_state != Progress_Run_State::Waiting_For_Image )
    return Transition(Progress_Run_Action::None);
  ++m_captured_image_count;
  return Advance();
}

Progress_Run_Transition Progress_Run_Controller::Image_Timed_Out()
{
  if( m_state != Progress_Run_State::Waiting_For_Image )
    return Transition(Progress_Run_Action::None);
  return Fail("Timed out waiting for a new 2D camera image.");
}

Progress_Run_Transition Progress_Run_Controller::Request_Emergency_Stop()
{
  if( m_state != Progress_Run_State::Moving &&
      m_state != Progress_Run_State::Waiting_For_Image )
  {
    return Transition(Progress_Run_Action::None);
  }
  m_state = Progress_Run_State::Stopping;
  return Transition(Progress_Run_Action::Request_Stop);
}

Progress_Run_Transition Progress_Run_Controller::Stop_Confirmed()
{
  if( m_state != Progress_Run_State::Stopping )
    return Transition(Progress_Run_Action::None);
  return Fail("Progress was stopped.");
}

Progress_Run_Transition Progress_Run_Controller::Robot_Became_Unavailable(
  std::string message)
{
  if( !Is_Motion_Active() )
    return Transition(Progress_Run_Action::None);
  return Fail(std::move(message));
}

Progress_Run_Transition Progress_Run_Controller::Effect_Failed(
  std::string message)
{
  if( !Is_Motion_Active() )
    return Transition(Progress_Run_Action::None);
  return Fail(std::move(message));
}

Progress_Run_Transition Progress_Run_Controller::Image_Processing_Completed(
  bool success,
  std::string message)
{
  if( m_state != Progress_Run_State::Processing_Images )
    return Transition(Progress_Run_Action::None);
  m_state = success
    ? Progress_Run_State::Completed
    : Progress_Run_State::Failed;
  return Transition(Progress_Run_Action::None, std::move(message));
}

void Progress_Run_Controller::Reset()
{
  m_state = Progress_Run_State::Idle;
  m_capture_image_at_point.clear();
  m_point_index = 0;
  m_captured_image_count = 0;
}

bool Progress_Run_Controller::Is_Motion_Active() const
{
  return m_state == Progress_Run_State::Moving ||
         m_state == Progress_Run_State::Waiting_For_Image ||
         m_state == Progress_Run_State::Stopping;
}

bool Progress_Run_Controller::Is_Terminal() const
{
  return m_state == Progress_Run_State::Completed ||
         m_state == Progress_Run_State::Failed;
}

Progress_Run_Transition Progress_Run_Controller::Advance()
{
  ++m_point_index;
  if( m_point_index < m_capture_image_at_point.size() )
  {
    m_state = Progress_Run_State::Moving;
    return Transition(Progress_Run_Action::Move_Point);
  }

  if( m_captured_image_count != 0 )
  {
    m_state = Progress_Run_State::Processing_Images;
    return Transition(Progress_Run_Action::Start_Image_Processing);
  }

  m_state = Progress_Run_State::Completed;
  return Transition(Progress_Run_Action::Finish);
}

Progress_Run_Transition Progress_Run_Controller::Fail(std::string message)
{
  m_state = Progress_Run_State::Failed;
  return Transition(Progress_Run_Action::Finish, std::move(message));
}

Progress_Run_Transition Progress_Run_Controller::Transition(
  Progress_Run_Action action,
  std::string message) const
{
  return {m_state, action, m_point_index, std::move(message)};
}

const char *To_String(Progress_Run_State state)
{
  switch( state )
  {
    case Progress_Run_State::Idle: return "Idle";
    case Progress_Run_State::Moving: return "Moving";
    case Progress_Run_State::Waiting_For_Image: return "WaitingForImage";
    case Progress_Run_State::Stopping: return "Stopping";
    case Progress_Run_State::Processing_Images: return "ProcessingImages";
    case Progress_Run_State::Completed: return "Completed";
    case Progress_Run_State::Failed: return "Failed";
  }
  return "Unknown";
}

} // namespace application

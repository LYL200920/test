#include "camera_calibration_progress_controller.h"

#include <iostream>
#include <stdexcept>

namespace
{

using application::Camera_Calibration_Progress_Action;
using application::Camera_Calibration_Progress_Controller;
using application::Camera_Calibration_Progress_State;

void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

void Test_Transition_And_Capture_Points()
{
  Camera_Calibration_Progress_Controller controller;
  auto transition = controller.Start({false, true, false});
  Require(
    transition.action == Camera_Calibration_Progress_Action::Move_Point,
    "start move");
  transition = controller.Motion_Completed();
  Require(transition.point_index == 1, "skip transition point");
  Require(
    transition.action == Camera_Calibration_Progress_Action::Move_Point,
    "move capture point");
  transition = controller.Motion_Completed();
  Require(
    transition.action ==
      Camera_Calibration_Progress_Action::Wait_For_Settling,
    "wait for settling");
  transition = controller.Settling_Completed();
  Require(
    transition.action == Camera_Calibration_Progress_Action::Trigger_Image,
    "trigger image");
  transition = controller.Image_Arrived();
  Require(
    transition.action == Camera_Calibration_Progress_Action::Process_Image,
    "process image");
  transition = controller.Image_Processed(true);
  Require(controller.Accepted_Image_Count() == 1, "accepted count");
  Require(transition.point_index == 2, "move final transition");
  transition = controller.Motion_Completed();
  Require(
    transition.state == Camera_Calibration_Progress_State::Completed,
    "completed");
}

void Test_Detection_Retry_Then_Continue()
{
  Camera_Calibration_Progress_Controller controller;
  controller.Start({true}, 1);
  controller.Motion_Completed();
  controller.Settling_Completed();
  controller.Image_Arrived();
  auto transition = controller.Image_Processed(false);
  Require(transition.retry_index == 1, "retry index");
  Require(
    transition.action ==
      Camera_Calibration_Progress_Action::Wait_For_Settling,
    "retry settling");
  controller.Settling_Completed();
  controller.Image_Arrived();
  transition = controller.Image_Processed(false);
  Require(
    transition.state == Camera_Calibration_Progress_State::Completed,
    "complete after rejected point");
  Require(controller.Rejected_Point_Count() == 1, "rejected point count");
}

void Test_Timeout_And_Stop_Fail_Safely()
{
  Camera_Calibration_Progress_Controller controller;
  controller.Start({true});
  controller.Motion_Completed();
  controller.Settling_Completed();
  auto transition = controller.Image_Timed_Out();
  Require(
    transition.state == Camera_Calibration_Progress_State::Failed,
    "timeout failure");

  controller.Reset();
  controller.Start({false});
  transition = controller.Request_Stop();
  Require(
    transition.action == Camera_Calibration_Progress_Action::Request_Stop,
    "request stop");
  transition = controller.Stop_Confirmed();
  Require(
    transition.state == Camera_Calibration_Progress_State::Failed,
    "confirmed stop");
}

} // namespace

int main()
{
  try
  {
    Test_Transition_And_Capture_Points();
    Test_Detection_Retry_Then_Continue();
    Test_Timeout_And_Stop_Fail_Safely();
  }
  catch (const std::exception &error)
  {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }
  std::cout << "Camera calibration progress controller tests passed.\n";
  return 0;
}

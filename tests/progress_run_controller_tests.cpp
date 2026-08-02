#include "progress_run_controller.h"

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using application::Progress_Run_Action;
using application::Progress_Run_Controller;
using application::Progress_Run_State;

void Require(bool condition, const char *message)
{
  if( !condition ) throw std::runtime_error(message);
}

void Test_Run_Without_Images_Completes()
{
  Progress_Run_Controller controller;
  auto transition = controller.Start({false, false});
  Require(transition.state == Progress_Run_State::Moving, "start state");
  Require(transition.action == Progress_Run_Action::Move_Point, "first move");
  Require(transition.point_index == 0, "first point index");

  transition = controller.Motion_Completed();
  Require(transition.action == Progress_Run_Action::Move_Point, "second move");
  Require(transition.point_index == 1, "second point index");

  transition = controller.Motion_Completed();
  Require(transition.state == Progress_Run_State::Completed, "completed state");
  Require(transition.action == Progress_Run_Action::Finish, "finish action");
}

void Test_Image_Points_Wait_And_Process()
{
  Progress_Run_Controller controller;
  controller.Start({true, false, true});

  auto transition = controller.Motion_Completed();
  Require(
    transition.state == Progress_Run_State::Waiting_For_Image,
    "wait for first image");
  Require(
    transition.action == Progress_Run_Action::Trigger_Image,
    "trigger first image");

  transition = controller.Image_Captured();
  Require(transition.action == Progress_Run_Action::Move_Point, "move point two");
  Require(transition.point_index == 1, "point two index");
  transition = controller.Motion_Completed();
  Require(transition.point_index == 2, "point three index");
  transition = controller.Motion_Completed();
  Require(
    transition.state == Progress_Run_State::Waiting_For_Image,
    "wait for second image");
  transition = controller.Image_Captured();
  Require(
    transition.state == Progress_Run_State::Processing_Images,
    "processing state");
  Require(
    transition.action == Progress_Run_Action::Start_Image_Processing,
    "processing action");
  Require(controller.Captured_Image_Count() == 2, "capture count");

  controller.Image_Processing_Completed(true);
  Require(controller.State() == Progress_Run_State::Completed, "processed state");
}

void Test_Timeout_Fails_Run()
{
  Progress_Run_Controller controller;
  controller.Start({true});
  controller.Motion_Completed();
  const auto transition = controller.Image_Timed_Out();
  Require(transition.state == Progress_Run_State::Failed, "timeout failed state");
  Require(transition.action == Progress_Run_Action::Finish, "timeout finish action");
  Require(!transition.message.empty(), "timeout message");
}

void Test_Emergency_Stop_Waits_For_Confirmation()
{
  Progress_Run_Controller controller;
  controller.Start({false});
  auto transition = controller.Request_Emergency_Stop();
  Require(transition.state == Progress_Run_State::Stopping, "stopping state");
  Require(transition.action == Progress_Run_Action::Request_Stop, "stop action");
  Require(controller.Is_Motion_Active(), "stopping remains active");

  transition = controller.Stop_Confirmed();
  Require(transition.state == Progress_Run_State::Failed, "stopped state");
  Require(transition.action == Progress_Run_Action::Finish, "stopped finish action");
}

void Test_Invalid_Events_Do_Not_Advance()
{
  Progress_Run_Controller controller;
  auto transition = controller.Image_Captured();
  Require(transition.state == Progress_Run_State::Idle, "idle invalid event");
  Require(transition.action == Progress_Run_Action::None, "idle no action");

  transition = controller.Start({});
  Require(transition.state == Progress_Run_State::Failed, "empty run fails");
  controller.Reset();
  Require(controller.State() == Progress_Run_State::Idle, "reset state");
}

void Test_Robot_And_Processing_Failures_Are_Explicit()
{
  Progress_Run_Controller controller;
  controller.Start({false});
  auto transition = controller.Robot_Became_Unavailable("robot fault");
  Require(transition.state == Progress_Run_State::Failed, "robot failed state");
  Require(transition.action == Progress_Run_Action::Finish, "robot finish action");
  Require(transition.message == "robot fault", "robot failure message");

  controller.Start({true});
  controller.Motion_Completed();
  controller.Image_Captured();
  Require(
    controller.State() == Progress_Run_State::Processing_Images,
    "processing before failure");
  transition = controller.Image_Processing_Completed(false, "image failure");
  Require(transition.state == Progress_Run_State::Failed, "processing failed state");
  Require(transition.message == "image failure", "processing failure message");
}
} // namespace

int main()
{
  try
  {
    Test_Run_Without_Images_Completes();
    Test_Image_Points_Wait_And_Process();
    Test_Timeout_Fails_Run();
    Test_Emergency_Stop_Waits_For_Confirmation();
    Test_Invalid_Events_Do_Not_Advance();
    Test_Robot_And_Processing_Failures_Are_Explicit();
  }
  catch( const std::exception &error )
  {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }
  std::cout << "Progress run controller tests passed.\n";
  return 0;
}

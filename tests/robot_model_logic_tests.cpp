#include "robot_joint_state_builder.h"
#include "robot_trajectory_io.h"
#include "robot_trajectory_planner.h"
#include "robot_trajectory_session.h"

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

void require (bool condition, const std::string& message)
{
  if( !condition )
  {
    throw std::runtime_error (message);
  }
}

void require_near (double actual, double expected, const std::string& message)
{
  require (std::abs (actual - expected) < 1.0e-9, message);
}

void require_point_near (
  const std::array<double, 6>& actual,
  const std::array<double, 6>& expected,
  const std::string& message)
{
  for( size_t i = 0; i < actual.size ( ); ++i )
  {
    if( std::abs (actual[i] - expected[i]) >= 1.0e-9 )
    {
      throw std::runtime_error (message);
    }
  }
}

void test_joint_state_builder ( )
{
  robot_model::Robot_Kinematic_Params params;
  params.joint_offsets = { 10.0, -20.0, 30.0 };
  params.joint_directions = { 1, -1, 0 };

  const std::array<double, 6> input = { 5.0, 10.0, 1.0, 2.0, 3.0, 4.0 };
  const auto state = robot_model::Build_Joint_State_From_Input_Angles (
    params, input);

  require_near (state.input_angles_deg[0], 5.0, "A1 input mismatch");
  require_near (state.effective_angles_deg[0], 15.0, "A1 effective mismatch");
  require_near (state.delta_angles_deg[0], 15.0, "A1 delta mismatch");

  require_near (state.effective_angles_deg[1], -30.0, "A2 effective mismatch");
  require_near (state.delta_angles_deg[1], -30.0, "A2 delta mismatch");

  require_near (state.effective_angles_deg[2], 31.0, "A3 effective mismatch");
  require_near (state.delta_angles_deg[2], 31.0, "A3 delta mismatch");
}

void test_trajectory_planner ( )
{
  const std::array<double, 6> start = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  const std::array<double, 6> target = { 10.0, 20.0, 30.0, 40.0, 50.0, 60.0 };

  const auto frames = robot_model::Build_Joint_Ptp_Trajectory (
    start, target, 3);
  require (frames.size ( ) == 3, "PTP frame count mismatch");
  require_point_near (frames[0], start, "PTP start frame mismatch");
  require_point_near (
    frames[1],
    { 5.0, 10.0, 15.0, 20.0, 25.0, 30.0 },
    "PTP midpoint mismatch");
  require_point_near (frames[2], target, "PTP target frame mismatch");

  const robot_model::Robot_Joint_Trajectory points =
  {
    start,
    target,
    { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 }
  };
  const auto multi_frames =
    robot_model::Build_Multi_Point_Joint_Ptp_Trajectory (points, 3);
  require (multi_frames.size ( ) == 5, "Multi-point frame count mismatch");
  require_point_near (multi_frames.front ( ), points.front ( ),
                      "Multi-point start mismatch");
  require_point_near (multi_frames.back ( ), points.back ( ),
                      "Multi-point end mismatch");
}

void test_trajectory_session ( )
{
  robot_model::Robot_Trajectory_Session session;
  const std::array<double, 6> p0 = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  const std::array<double, 6> p1 = { 10.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  const std::array<double, 6> p2 = { 10.0, 10.0, 0.0, 0.0, 0.0, 0.0 };

  session.Add_Point (p0);
  session.Add_Point (p1);
  session.Add_Point (p2);
  require (session.Point_Count ( ) == 3, "Session point count mismatch");
  require (session.Has_Playable_Path ( ), "Session should be playable");
  require (session.Start_Playback (3), "Session playback should start");

  size_t stepped_frames = 0;
  while( session.Step ( ) != nullptr )
  {
    ++stepped_frames;
  }
  require (stepped_frames == 5, "Session playback frame count mismatch");
  require (session.Is_Finished ( ), "Session should be finished");

  require (session.Start_Go_To (p0, 1, 3), "Session Go To should start");
  const auto* first = session.Step ( );
  require (first != nullptr, "Session Go To first frame missing");
  require_point_near (*first, p0, "Session Go To start frame mismatch");
}

void test_trajectory_csv_io ( )
{
  const auto path = std::filesystem::temp_directory_path ( ) /
    "test_robot_trajectory_io_test.csv";
  const std::vector<std::array<double, 6>> points =
  {
    { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 },
    { -1.5, 0.0, 2.5, 3.5, 4.5, 5.5 }
  };

  std::string error;
  require (robot_model::Save_Joint_Trajectory_Csv (path, points, &error),
           "CSV save failed: " + error);

  std::vector<std::array<double, 6>> loaded_points;
  require (robot_model::Load_Joint_Trajectory_Csv (
             path, &loaded_points, &error),
           "CSV load failed: " + error);
  require (loaded_points.size ( ) == points.size ( ),
           "CSV loaded point count mismatch");
  require_point_near (loaded_points[0], points[0], "CSV first point mismatch");
  require_point_near (loaded_points[1], points[1], "CSV second point mismatch");

  {
    std::ofstream invalid_file (path);
    invalid_file << "A1,A2,A3,A4,A5,A6\n1,2,3\n";
  }
  require (!robot_model::Load_Joint_Trajectory_Csv (
             path, &loaded_points, &error),
           "Invalid CSV should fail");

  std::filesystem::remove (path);
}

} // namespace

int main ( )
{
  try
  {
    test_joint_state_builder ( );
    test_trajectory_planner ( );
    test_trajectory_session ( );
    test_trajectory_csv_io ( );
  }
  catch( const std::exception& e )
  {
    std::cerr << "FAILED: " << e.what ( ) << '\n';
    return 1;
  }

  std::cout << "All robot model logic tests passed.\n";
  return 0;
}

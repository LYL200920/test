#include "robot_trajectory_session.h"

#include <utility>

namespace robot_model
{

  void Robot_Trajectory_Session::Add_Point(const Joint_Point &point)
  {
    m_points.push_back(point);
  }

  void Robot_Trajectory_Session::Clear_Points()
  {
    m_points.clear();
  }

  bool Robot_Trajectory_Session::Delete_Point(size_t index)
  {
    if (index >= m_points.size())
    {
      return false;
    }

    m_points.erase(m_points.begin() + static_cast<std::ptrdiff_t>(index));
    return true;
  }

  void Robot_Trajectory_Session::Set_Points(std::vector<Joint_Point> points)
  {
    m_points = std::move(points);
  }

  const std::vector<Robot_Trajectory_Session::Joint_Point> &
  Robot_Trajectory_Session::Points() const
  {
    return m_points;
  }

  size_t Robot_Trajectory_Session::Point_Count() const
  {
    return m_points.size();
  }

  bool Robot_Trajectory_Session::Has_Playable_Path() const
  {
    return m_points.size() >= 2;
  }

  bool Robot_Trajectory_Session::Start_Go_To(const Joint_Point &current_angles,
                                             size_t target_index,
                                             size_t frame_count)
  {
    if (target_index >= m_points.size())
    {
      return false;
    }

    auto frames = Build_Joint_Ptp_Trajectory(current_angles, m_points[target_index], frame_count);
    if (frames.empty())
    {
      return false;
    }

    m_player.Start(std::move(frames));
    return true;
  }

  bool Robot_Trajectory_Session::Start_Playback(size_t frame_count)
  {
    if (!Has_Playable_Path())
    {
      return false;
    }

    auto frames = Build_Multi_Point_Joint_Ptp_Trajectory(m_points, frame_count);
    if (frames.empty())
    {
      return false;
    }

    m_player.Start(std::move(frames));
    return true;
  }

  void Robot_Trajectory_Session::Pause()
  {
    m_player.Pause();
  }

  void Robot_Trajectory_Session::Resume()
  {
    m_player.Resume();
  }

  void Robot_Trajectory_Session::Stop()
  {
    m_player.Stop();
  }

  bool Robot_Trajectory_Session::Is_Playing() const
  {
    return m_player.Is_Playing();
  }

  bool Robot_Trajectory_Session::Is_Paused() const
  {
    return m_player.Is_Paused();
  }

  bool Robot_Trajectory_Session::Is_Active() const
  {
    return m_player.Is_Active();
  }

  bool Robot_Trajectory_Session::Is_Finished() const
  {
    return m_player.Is_Finished();
  }

  const Robot_Trajectory_Session::Joint_Point *Robot_Trajectory_Session::Step()
  {
    return m_player.Step();
  }

} // namespace robot_model

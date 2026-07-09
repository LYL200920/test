#include "robot_trajectory_player.h"

#include <utility>

namespace robot_model
{

void Robot_Trajectory_Player::Start (
  std::vector<std::array<double, 6>> frames)
{
  m_frames = std::move (frames);
  m_frame_index = 0;
  m_paused = false;
}

void Robot_Trajectory_Player::Pause ( )
{
  if( Is_Active ( ) )
  {
    m_paused = true;
  }
}

void Robot_Trajectory_Player::Resume ( )
{
  if( Is_Active ( ) )
  {
    m_paused = false;
  }
}

void Robot_Trajectory_Player::Stop ( )
{
  m_frames.clear ( );
  m_frame_index = 0;
  m_paused = false;
}

bool Robot_Trajectory_Player::Is_Playing ( ) const
{
  return Is_Active ( ) && !m_paused;
}

bool Robot_Trajectory_Player::Is_Paused ( ) const
{
  return Is_Active ( ) && m_paused;
}

bool Robot_Trajectory_Player::Is_Active ( ) const
{
  return !m_frames.empty ( ) && !Is_Finished ( );
}

bool Robot_Trajectory_Player::Is_Finished ( ) const
{
  return m_frame_index >= m_frames.size ( );
}

const std::array<double, 6>* Robot_Trajectory_Player::Step ( )
{
  if( !Is_Playing ( ) )
  {
    return nullptr;
  }

  if( m_frame_index >= m_frames.size ( ) )
  {
    return nullptr;
  }

  const auto* frame = &m_frames[m_frame_index];
  ++m_frame_index;
  return frame;
}

size_t Robot_Trajectory_Player::Frame_Index ( ) const
{
  return m_frame_index;
}

size_t Robot_Trajectory_Player::Frame_Count ( ) const
{
  return m_frames.size ( );
}

} // namespace robot_model

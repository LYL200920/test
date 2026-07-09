#ifndef includeguard_robot_trajectory_player_h_includeguard
#define includeguard_robot_trajectory_player_h_includeguard

#include <array>
#include <cstddef>
#include <vector>

namespace robot_model
{

class Robot_Trajectory_Player
{
public:
  void Start (std::vector<std::array<double, 6>> frames);
  void Pause ( );
  void Resume ( );
  void Stop ( );

  bool Is_Playing ( ) const;
  bool Is_Paused ( ) const;
  bool Is_Active ( ) const;
  bool Is_Finished ( ) const;

  const std::array<double, 6>* Step ( );
  size_t Frame_Index ( ) const;
  size_t Frame_Count ( ) const;

private:
  std::vector<std::array<double, 6>> m_frames;
  size_t m_frame_index = 0;
  bool m_paused = false;
};

} // namespace robot_model

#endif

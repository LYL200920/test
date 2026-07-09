#ifndef includeguard_robot_trajectory_session_h_includeguard
#define includeguard_robot_trajectory_session_h_includeguard

#include "robot_trajectory_planner.h"
#include "robot_trajectory_player.h"

#include <cstddef>

namespace robot_model
{

class Robot_Trajectory_Session
{
public:
  using Joint_Point = Robot_Joint_Trajectory_Point;

  void Add_Point (const Joint_Point& point);
  void Clear_Points ( );
  bool Delete_Point (size_t index);
  void Set_Points (std::vector<Joint_Point> points);

  const std::vector<Joint_Point>& Points ( ) const;
  size_t Point_Count ( ) const;
  bool Has_Playable_Path ( ) const;

  bool Start_Go_To (const Joint_Point& current_angles,
                    size_t target_index,
                    size_t frame_count);
  bool Start_Playback (size_t frame_count);
  void Pause ( );
  void Resume ( );
  void Stop ( );

  bool Is_Playing ( ) const;
  bool Is_Paused ( ) const;
  bool Is_Active ( ) const;
  bool Is_Finished ( ) const;

  const Joint_Point* Step ( );

private:
  Robot_Joint_Trajectory m_points;
  Robot_Trajectory_Player m_player;
};

} // namespace robot_model

#endif

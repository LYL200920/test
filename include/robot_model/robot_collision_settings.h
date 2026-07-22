#ifndef includeguard_robot_collision_settings_h_includeguard
#define includeguard_robot_collision_settings_h_includeguard

#include "robot_collision_detector.h"

namespace robot_model
{

struct Robot_Collision_Settings
{
  double clearance_mm = 10.0;
  Robot_Collision_Point_Cloud_Options point_cloud;
};

} // namespace robot_model

#endif

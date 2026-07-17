#include "robot_kinematic_params.h"

#include <cmath>

namespace robot_model
{

double link_length_at (const Robot_Kinematic_Params& params,
                       std::size_t index,
                       double fallback)
{
  if( index < params.link_lengths.size ( ) &&
      std::fabs (params.link_lengths[index]) > 1.0e-6 )
  {
    return params.link_lengths[index];
  }
  return fallback;
}

} // namespace robot_model

#ifndef includeguard_robot_simulation_state_h_includeguard
#define includeguard_robot_simulation_state_h_includeguard

#include "robot_model_state.h"

namespace robot_model
{

class Robot_Simulation_State
{
public:
  Robot_Model_State& Robot_Model ( ) { return m_robot_model; }
  const Robot_Model_State& Robot_Model ( ) const { return m_robot_model; }

private:
  Robot_Model_State m_robot_model;
};

} // namespace robot_model

#endif

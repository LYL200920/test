#include "robot_model_state.h"

namespace robot_model
{

  void Robot_Model_State::Set_Current_Model(const Robot_Model_Info &model,
                                            const Robot_Kinematic_Params &params)
  {
    m_current_model = model;
    m_params = params;
    m_model_name = model.display_name;
    m_has_current_model = true;
  }

  void Robot_Model_State::Set_Joint_State(const Robot_Joint_State &joint_state)
  {
    m_joint_state = joint_state;
  }

  void Robot_Model_State::Set_Assembly_Calibration(const Robot_Assembly_Calibration &assembly_calibration)
  {
    m_assembly_calibration = assembly_calibration;
    m_has_assembly_calibration = true;
  }

  void Robot_Model_State::Clear_Assembly_Calibration()
  {
    m_assembly_calibration = Robot_Assembly_Calibration();
    m_has_assembly_calibration = false;
  }

} // namespace robot_model

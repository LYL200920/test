#ifndef includeguard_robot_model_state_h_includeguard
#define includeguard_robot_model_state_h_includeguard

#include "robot_model_data.h"

#include <string>

namespace robot_model
{

  class Robot_Model_State
  {
  public:
    void Set_Current_Model(const Robot_Model_Info &model, const Robot_Kinematic_Params &params);

    bool Has_Current_Model() const { return m_has_current_model; }
    const Robot_Model_Info &Current_Model() const { return m_current_model; }
    const Robot_Kinematic_Params &Params() const { return m_params; }
    const std::string &Model_Name() const { return m_model_name; }

    void Set_Joint_State(const Robot_Joint_State &joint_state);
    const Robot_Joint_State &Joint_State() const { return m_joint_state; }

    void Set_Assembly_Calibration(const Robot_Assembly_Calibration &assembly_calibration);
    void Clear_Assembly_Calibration();
    bool Has_Assembly_Calibration() const { return m_has_assembly_calibration; }
    const Robot_Assembly_Calibration &Assembly_Calibration() const
    {
      return m_assembly_calibration;
    }

  private:
    Robot_Model_Info m_current_model;
    Robot_Kinematic_Params m_params;
    Robot_Assembly_Calibration m_assembly_calibration;
    Robot_Joint_State m_joint_state;
    std::string m_model_name;
    bool m_has_current_model = false;
    bool m_has_assembly_calibration = false;
  };

} // namespace robot_model

#endif

#ifndef includeguard_robot_render_controller_h_includeguard
#define includeguard_robot_render_controller_h_includeguard

#include "robot_model_data.h"
#include "robot_scene_assembly.h"
#include "robot_simulation_state.h"

namespace robot_model
{

class Vtk_Scene;

class Robot_Render_Controller
{
public:
  void Attach_Scene (Vtk_Scene* scene);
  void Detach_Scene ( );

  void Load_Model (const Robot_Model_Info& model);
  void Set_Joint_State (const Robot_Joint_State& joint_state);

private:
  void Rebuild_Current_Model ( );
  void Apply_Joint_Pose ( );

private:
  Vtk_Scene* m_scene = nullptr;
  Robot_Simulation_State m_state;
  Robot_Scene_Assembly m_assembly;
};

} // namespace robot_model

#endif

#ifndef includeguard_robot_render_controller_h_includeguard
#define includeguard_robot_render_controller_h_includeguard

#include "robot_model_data.h"
#include "robot_forward_kinematics.h"
#include "robot_inverse_kinematics.h"
#include "robot_scene_assembly.h"
#include "robot_simulation_state.h"
#include "pose_transform.h"

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
  bool Has_Current_Model ( ) const;
  const Robot_Kinematic_Params& Kinematic_Params ( ) const;
  const Robot_Joint_State& Joint_State ( ) const;
  bool Has_Flange_Pose ( ) const;
  const Matrix4& World_From_Flange ( ) const;
  Robot_Position_IK_Result Move_Flange_To (const Point3& target_world);
  Robot_Pose_IK_Result Move_Flange_To_Pose (
    const Matrix4& target_world_from_flange);

private:
  void Rebuild_Current_Model ( );
  void Apply_Joint_Pose ( );

private:
  Vtk_Scene* m_scene = nullptr;
  Robot_Simulation_State m_state;
  Robot_Scene_Assembly m_assembly;
  Robot_Forward_Kinematics_Model m_forward_model;
  bool m_has_forward_model = false;
};

} // namespace robot_model

#endif

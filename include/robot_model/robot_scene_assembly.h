#ifndef includeguard_robot_scene_assembly_h_includeguard
#define includeguard_robot_scene_assembly_h_includeguard

#include "robot_model_data.h"
#include "robot_visual_data.h"
#include "robot_calibration_builder.h"
#include "robot_forward_kinematics.h"
#include "pose_transform.h"

#include <vtkRenderer.h>

namespace robot_model
{

class Robot_Scene_Assembly
{
public:
  void Load_Mesh (const Robot_Model_Info& model, vtkRenderer* renderer);
  void Build_Fallback_Demo (const Robot_Kinematic_Params& params,
                            vtkRenderer* renderer);

  void Apply_Calibration (const Robot_Assembly_Calibration& calibration);
  void Apply_Joint_State (const Robot_Assembly_Calibration& calibration,
                          const Robot_Kinematic_Params& params,
                          const Robot_Joint_State& joint_state);
  void Apply_Forward_Kinematics (
    const Robot_Forward_Kinematics_Result& transforms);

  void Clear ( );

  const std::vector<Robot_Visual_Part>& Parts ( ) const { return m_parts; }
  bool Has_Flange_Pose ( ) const { return m_has_flange_pose; }
  const Matrix4& World_From_Flange ( ) const { return m_world_from_flange; }

private:
  void Add_Link (double x_length, double y_length, double z_length,
                 double x, double y, double z,
                 const double color[3], vtkRenderer* renderer);
  void Add_Joint (double x, double y, double z, double radius,
                  vtkRenderer* renderer);

private:
  std::vector<Robot_Visual_Part> m_parts;
  Matrix4 m_world_from_flange = { };
  bool m_has_flange_pose = false;
};

} // namespace robot_model

#endif

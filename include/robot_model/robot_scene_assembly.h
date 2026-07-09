#ifndef includeguard_robot_scene_assembly_h_includeguard
#define includeguard_robot_scene_assembly_h_includeguard

#include "robot_model_data.h"
#include "robot_calibration_builder.h"

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

  void Clear ( );

  const std::vector<Robot_Visual_Part>& Parts ( ) const { return m_parts; }

private:
  void Add_Link (double x_length, double y_length, double z_length,
                 double x, double y, double z,
                 const double color[3], vtkRenderer* renderer);
  void Add_Joint (double x, double y, double z, double radius,
                  vtkRenderer* renderer);

private:
  std::vector<Robot_Visual_Part> m_parts;
};

} // namespace robot_model

#endif

#ifndef includeguard_robot_part_transform_h_includeguard
#define includeguard_robot_part_transform_h_includeguard

#include "pose_transform.h"
#include "robot_model_data.h"

namespace robot_model
{

  // Reproduces the mesh calibration order Rx * Ry * Rz * T using only the
  // renderer-independent Matrix4 representation.
  Matrix4 Build_Part_Calibration_Matrix(const Robot_Part_Calibration &calibration);

  Point3 Transform_Part_Point(const Robot_Part_Calibration &calibration, const Point3 &point);

} // namespace robot_model

#endif

#ifndef includeguard_transform_utils_h_includeguard
#define includeguard_transform_utils_h_includeguard

#include "robot_model_data.h"

#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

#include <array>

namespace robot_model
{

vtkSmartPointer<vtkMatrix4x4> Build_Static_Part_Matrix (
  const Robot_Part_Calibration& part_calibration);

vtkSmartPointer<vtkMatrix4x4> Build_Rotation_About_Point_Matrix (
  double angle_deg,
  const std::array<double, 3>& axis,
  const std::array<double, 3>& point);

void Prepend_Matrix (vtkMatrix4x4* chain, vtkMatrix4x4* next);

std::array<double, 3> Transform_Point (vtkMatrix4x4* matrix,
                                       const std::array<double, 3>& point);

std::array<double, 3> Transform_Vector (vtkMatrix4x4* matrix,
                                        const std::array<double, 3>& vector);

std::array<double, 3> Transform_Part_Point (
  const Robot_Part_Calibration& part_calibration,
  const std::array<double, 3>& point);

} // namespace robot_model

#endif

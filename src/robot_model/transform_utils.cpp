#include "transform_utils.h"

#include <vtkTransform.h>

#include <cmath>

namespace robot_model
{
namespace
{
void apply_part_calibration (vtkTransform* transform,
                             const Robot_Part_Calibration& part_calibration)
{
  if( transform == nullptr )
  {
    return;
  }

  transform->Identity ( );
  transform->PostMultiply ( );
  transform->RotateX (part_calibration.rotate_xyz[0]);
  transform->RotateY (part_calibration.rotate_xyz[1]);
  transform->RotateZ (part_calibration.rotate_xyz[2]);
  transform->Translate (part_calibration.translate[0],
                        part_calibration.translate[1],
                        part_calibration.translate[2]);
}
} // namespace

vtkSmartPointer<vtkMatrix4x4> Build_Static_Part_Matrix (
  const Robot_Part_Calibration& part_calibration)
{
  auto transform = vtkSmartPointer<vtkTransform>::New ( );
  apply_part_calibration (transform, part_calibration);

  auto matrix = vtkSmartPointer<vtkMatrix4x4>::New ( );
  matrix->DeepCopy (transform->GetMatrix ( ));
  return matrix;
}

vtkSmartPointer<vtkMatrix4x4> Build_Rotation_About_Point_Matrix (
  double angle_deg,
  const std::array<double, 3>& axis,
  const std::array<double, 3>& point)
{
  double ax = axis[0];
  double ay = axis[1];
  double az = axis[2];
  const double length = std::sqrt (ax * ax + ay * ay + az * az);
  if( length < 1.0e-9 )
  {
    ax = 0.0;
    ay = 0.0;
    az = 1.0;
  }
  else
  {
    ax /= length;
    ay /= length;
    az /= length;
  }

  const double radians = angle_deg * 3.14159265358979323846 / 180.0;
  const double c = std::cos (radians);
  const double s = std::sin (radians);
  const double t = 1.0 - c;

  const double r00 = t * ax * ax + c;
  const double r01 = t * ax * ay - s * az;
  const double r02 = t * ax * az + s * ay;
  const double r10 = t * ax * ay + s * az;
  const double r11 = t * ay * ay + c;
  const double r12 = t * ay * az - s * ax;
  const double r20 = t * ax * az - s * ay;
  const double r21 = t * ay * az + s * ax;
  const double r22 = t * az * az + c;

  const double px = point[0];
  const double py = point[1];
  const double pz = point[2];
  const double tx = px - ( r00 * px + r01 * py + r02 * pz );
  const double ty = py - ( r10 * px + r11 * py + r12 * pz );
  const double tz = pz - ( r20 * px + r21 * py + r22 * pz );

  auto matrix = vtkSmartPointer<vtkMatrix4x4>::New ( );
  matrix->Identity ( );
  matrix->SetElement (0, 0, r00);
  matrix->SetElement (0, 1, r01);
  matrix->SetElement (0, 2, r02);
  matrix->SetElement (0, 3, tx);
  matrix->SetElement (1, 0, r10);
  matrix->SetElement (1, 1, r11);
  matrix->SetElement (1, 2, r12);
  matrix->SetElement (1, 3, ty);
  matrix->SetElement (2, 0, r20);
  matrix->SetElement (2, 1, r21);
  matrix->SetElement (2, 2, r22);
  matrix->SetElement (2, 3, tz);
  return matrix;
}

void Prepend_Matrix (vtkMatrix4x4* chain, vtkMatrix4x4* next)
{
  auto tmp = vtkSmartPointer<vtkMatrix4x4>::New ( );
  vtkMatrix4x4::Multiply4x4 (next, chain, tmp);
  chain->DeepCopy (tmp);
}

std::array<double, 3> Transform_Point (vtkMatrix4x4* matrix,
                                       const std::array<double, 3>& point)
{
  double in[4] = { point[0], point[1], point[2], 1.0 };
  double out[4] = {};
  matrix->MultiplyPoint (in, out);
  if( std::abs (out[3]) > 1.0e-9 )
  {
    return { out[0] / out[3], out[1] / out[3], out[2] / out[3] };
  }
  return { out[0], out[1], out[2] };
}

std::array<double, 3> Transform_Vector (vtkMatrix4x4* matrix,
                                        const std::array<double, 3>& vector)
{
  double in[4] = { vector[0], vector[1], vector[2], 0.0 };
  double out[4] = {};
  matrix->MultiplyPoint (in, out);
  return { out[0], out[1], out[2] };
}

std::array<double, 3> Transform_Part_Point (
  const Robot_Part_Calibration& part_calibration,
  const std::array<double, 3>& point)
{
  auto matrix = Build_Static_Part_Matrix (part_calibration);
  return Transform_Point (matrix, point);
}

} // namespace robot_model

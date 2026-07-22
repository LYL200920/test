#ifndef includeguard_robot_visual_data_h_includeguard
#define includeguard_robot_visual_data_h_includeguard

#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

#include <array>
#include <filesystem>

namespace robot_model
{

// VTK-owned representation of a loaded mesh part. This type belongs to the
// rendering boundary and must not be used by forward or inverse kinematics.
struct Robot_Visual_Part
{
  std::filesystem::path mesh_path;
  vtkSmartPointer<vtkPolyData> mesh_data;
  vtkSmartPointer<vtkActor> actor;
  vtkSmartPointer<vtkTransform> local_transform;
  std::array<double, 3> base_color = { 0.8, 0.8, 0.8 };
  std::array<double, 6> raw_bounds = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  bool has_raw_bounds = false;
};

} // namespace robot_model

#endif

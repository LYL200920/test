#include "robot_mesh_loader.h"

#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSTLReader.h>
#include <vtkTransform.h>

namespace robot_model
{

Robot_Visual_Part Create_STL_Part (const std::filesystem::path& stl_path,
                                   int index)
{
  auto reader = vtkSmartPointer<vtkSTLReader>::New ( );
  reader->SetFileName (stl_path.string ( ).c_str ( ));
  reader->Update ( );

  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New ( );
  mapper->SetInputConnection (reader->GetOutputPort ( ));

  auto actor = vtkSmartPointer<vtkActor>::New ( );
  actor->SetMapper (mapper);

  auto local_transform = vtkSmartPointer<vtkTransform>::New ( );
  local_transform->Identity ( );
  actor->SetUserTransform (local_transform);

  const double colors[][3] =
  {
    { 0.16, 0.16, 0.17 },
    { 0.94, 0.48, 0.10 },
    { 0.98, 0.58, 0.12 },
    { 0.93, 0.44, 0.08 },
    { 0.90, 0.90, 0.86 },
    { 0.75, 0.77, 0.78 },
    { 0.12, 0.12, 0.13 },
  };
  const auto& color = colors[index % ( sizeof (colors) / sizeof (colors[0]) )];
  actor->GetProperty ( )->SetColor (color[0], color[1], color[2]);
  actor->GetProperty ( )->SetAmbient (0.22);
  actor->GetProperty ( )->SetDiffuse (0.78);
  actor->GetProperty ( )->SetSpecular (0.25);
  actor->GetProperty ( )->SetSpecularPower (24.0);

  Robot_Visual_Part part;
  part.mesh_path = stl_path;
  part.mesh_data = reader->GetOutput ( );
  part.actor = actor;
  part.local_transform = local_transform;
  part.base_color = { color[0], color[1], color[2] };
  if( auto* output = reader->GetOutput ( ) )
  {
    output->GetBounds (part.raw_bounds.data ( ));
    part.has_raw_bounds = true;
  }
  return part;
}

} // namespace robot_model

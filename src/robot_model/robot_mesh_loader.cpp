#include "robot_mesh_loader.h"
#include "robot_model_diagnostics.h"

#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkErrorCode.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSTLReader.h>
#include <vtkTransform.h>

#include <sstream>

namespace robot_model
{
namespace
{
void log_reader_event(vtkObject *, unsigned long event_id,
                      void *client_data, void *call_data)
{
  const auto *path = static_cast<const std::filesystem::path *>(client_data);
  std::ostringstream message;
  message << "VTK " << vtkCommand::GetStringFromEventId(event_id)
          << " while reading " << (path ? path->u8string() : "<unknown>");
  if (call_data) message << ": " << static_cast<const char *>(call_data);
  Write_Robot_Model_Diagnostic(message.str());
}
} // namespace

Robot_Visual_Part Create_STL_Part (const std::filesystem::path& stl_path,
                                   int index)
{
  std::error_code file_error;
  const bool exists = std::filesystem::is_regular_file(stl_path, file_error);
  const auto size = exists ? std::filesystem::file_size(stl_path, file_error) : 0;
  std::ostringstream start_message;
  start_message << "STL begin index=" << index
                << " path=" << stl_path.u8string()
                << " exists=" << (exists ? "true" : "false")
                << " size=" << size;
  if (file_error) start_message << " file_error=" << file_error.message();
  Write_Robot_Model_Diagnostic(start_message.str());

  auto reader = vtkSmartPointer<vtkSTLReader>::New ( );
  auto observer = vtkSmartPointer<vtkCallbackCommand>::New();
  observer->SetClientData(const_cast<std::filesystem::path *>(&stl_path));
  observer->SetCallback(&log_reader_event);
  reader->AddObserver(vtkCommand::ErrorEvent, observer);
  reader->AddObserver(vtkCommand::WarningEvent, observer);
  reader->SetFileName (stl_path.string ( ).c_str ( ));
  reader->Update ( );

  auto *output = reader->GetOutput();
  std::ostringstream result_message;
  result_message << "STL end index=" << index
                 << " vtk_error_code=" << reader->GetErrorCode()
                 << " vtk_error="
                 << vtkErrorCode::GetStringFromErrorCode(reader->GetErrorCode())
                 << " points=" << (output ? output->GetNumberOfPoints() : 0)
                 << " cells=" << (output ? output->GetNumberOfCells() : 0);
  if (output && output->GetNumberOfPoints() > 0)
  {
    double bounds[6] = {};
    output->GetBounds(bounds);
    result_message << " bounds=[" << bounds[0] << ',' << bounds[1] << ','
                   << bounds[2] << ',' << bounds[3] << ',' << bounds[4] << ','
                   << bounds[5] << ']';
  }
  Write_Robot_Model_Diagnostic(result_message.str());

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
  if( output )
  {
    output->GetBounds (part.raw_bounds.data ( ));
    part.has_raw_bounds = true;
  }
  return part;
}

} // namespace robot_model

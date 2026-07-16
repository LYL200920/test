#include "robot_scene_assembly.h"

#include "robot_mesh_loader.h"

#include <vtkCubeSource.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>

#include <algorithm>

namespace robot_model
{

void Robot_Scene_Assembly::Load_Mesh (const Robot_Model_Info& model,
                                      vtkRenderer* renderer)
{
  Clear ( );

  for( size_t i = 0; i < model.stl_files.size ( ); ++i )
  {
    auto part = Create_STL_Part (model.stl_files[i], static_cast<int> (i));
    if( renderer && part.actor )
    {
      renderer->AddActor (part.actor);
    }
    m_parts.push_back (std::move (part));
  }
}

void Robot_Scene_Assembly::Build_Fallback_Demo (
  const Robot_Kinematic_Params& params,
  vtkRenderer* renderer)
{
  auto effective_params = params;
  if( effective_params.link_lengths.empty ( ) )
  {
    effective_params.link_lengths = { 400.0, 560.0, 25.0, 515.0, 90.0, 25.0 };
  }

  const double orange[3] = { 0.94, 0.48, 0.10 };
  const double dark[3] = { 0.17, 0.18, 0.20 };
  const double metal[3] = { 0.72, 0.74, 0.76 };

  Add_Joint (0.0, 0.0, 0.0, 115.0, renderer);
  Add_Link (220.0, 220.0, 140.0, 0.0, 0.0, 70.0, dark, renderer);

  double x = 0.0;
  double z = 140.0;

  const auto l1 = link_length_at (effective_params, 0, 400.0);
  Add_Link (130.0, 130.0, l1, x, 0.0, z + l1 * 0.5, orange, renderer);
  z += l1;
  Add_Joint (x, 0.0, z, 80.0, renderer);

  const auto l2 = link_length_at (effective_params, 1, 560.0);
  Add_Link (l2, 120.0, 120.0, x + l2 * 0.5, 0.0, z, orange, renderer);
  x += l2;
  Add_Joint (x, 0.0, z, 72.0, renderer);

  const auto l3 = link_length_at (effective_params, 3, 515.0);
  Add_Link (120.0, 120.0, l3, x, 0.0, z + l3 * 0.5, orange, renderer);
  z += l3;
  Add_Joint (x, 0.0, z, 64.0, renderer);

  const auto wrist = std::max (
    link_length_at (effective_params, 4, 90.0), 120.0);
  Add_Link (wrist + 160.0, 95.0, 95.0,
            x + ( wrist + 160.0 ) * 0.5, 0.0, z, metal, renderer);
  x += wrist + 160.0;
  Add_Joint (x, 0.0, z, 54.0, renderer);
}

void Robot_Scene_Assembly::Apply_Calibration (
  const Robot_Assembly_Calibration& calibration)
{
  Robot_Joint_State zero_state;
  Robot_Kinematic_Params params;
  Apply_Joint_State (calibration, params, zero_state);
}

void Robot_Scene_Assembly::Apply_Joint_State (
  const Robot_Assembly_Calibration& calibration,
  const Robot_Kinematic_Params& params,
  const Robot_Joint_State& joint_state)
{
  const auto model = Build_Forward_Kinematics_Model (
    m_parts, calibration, params);
  Apply_Forward_Kinematics (Compute_Forward_Kinematics (model, joint_state));
}

void Robot_Scene_Assembly::Apply_Forward_Kinematics (
  const Robot_Forward_Kinematics_Result& transforms)
{
  m_world_from_flange = transforms.world_from_flange;
  m_has_flange_pose = transforms.has_flange;
  const auto count = std::min (
    m_parts.size ( ), transforms.world_from_parts.size ( ));
  for( size_t i = 0; i < count; ++i )
  {
    auto& part = m_parts[i];
    if( !part.local_transform )
    {
      continue;
    }

    auto matrix = vtkSmartPointer<vtkMatrix4x4>::New ( );
    for( int row = 0; row < 4; ++row )
    {
      for( int column = 0; column < 4; ++column )
      {
        matrix->SetElement (
          row, column, transforms.world_from_parts[i][row][column]);
      }
    }
    part.local_transform->SetMatrix (matrix);

    if( part.actor )
    {
      part.actor->Modified ( );
    }
  }
}

void Robot_Scene_Assembly::Clear ( )
{
  m_parts.clear ( );
  m_world_from_flange = { };
  m_has_flange_pose = false;
}

void Robot_Scene_Assembly::Add_Link (double x_length, double y_length,
                                     double z_length,
                                     double x, double y, double z,
                                     const double color[3],
                                     vtkRenderer* renderer)
{
  auto source = vtkSmartPointer<vtkCubeSource>::New ( );
  source->SetXLength (x_length);
  source->SetYLength (y_length);
  source->SetZLength (z_length);
  source->SetCenter (x, y, z);

  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New ( );
  mapper->SetInputConnection (source->GetOutputPort ( ));

  auto actor = vtkSmartPointer<vtkActor>::New ( );
  actor->SetMapper (mapper);
  actor->GetProperty ( )->SetColor (color[0], color[1], color[2]);
  actor->GetProperty ( )->SetAmbient (0.28);
  actor->GetProperty ( )->SetDiffuse (0.78);
  actor->GetProperty ( )->SetSpecular (0.25);
  actor->GetProperty ( )->SetSpecularPower (24.0);
  renderer->AddActor (actor);
}

void Robot_Scene_Assembly::Add_Joint (double x, double y, double z,
                                      double radius,
                                      vtkRenderer* renderer)
{
  auto source = vtkSmartPointer<vtkSphereSource>::New ( );
  source->SetCenter (x, y, z);
  source->SetRadius (radius);
  source->SetThetaResolution (32);
  source->SetPhiResolution (16);

  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New ( );
  mapper->SetInputConnection (source->GetOutputPort ( ));

  auto actor = vtkSmartPointer<vtkActor>::New ( );
  actor->SetMapper (mapper);
  actor->GetProperty ( )->SetColor (0.12, 0.12, 0.13);
  actor->GetProperty ( )->SetAmbient (0.35);
  actor->GetProperty ( )->SetDiffuse (0.72);
  actor->GetProperty ( )->SetSpecular (0.35);
  actor->GetProperty ( )->SetSpecularPower (32.0);
  renderer->AddActor (actor);
}

} // namespace robot_model

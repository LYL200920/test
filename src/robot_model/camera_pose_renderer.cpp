#include "camera_pose_renderer.h"

#include <vtkCubeSource.h>
#include <vtkLineSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>

#include <array>

namespace robot_model
{
namespace
{

vtkSmartPointer<vtkActor> make_line_actor (
  const std::array<double, 3>& start,
  const std::array<double, 3>& end,
  const std::array<double, 3>& color,
  double width)
{
  auto source = vtkSmartPointer<vtkLineSource>::New ( );
  source->SetPoint1 (start.data ( ));
  source->SetPoint2 (end.data ( ));

  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New ( );
  mapper->SetInputConnection (source->GetOutputPort ( ));

  auto actor = vtkSmartPointer<vtkActor>::New ( );
  actor->SetMapper (mapper);
  actor->GetProperty ( )->SetColor (color[0], color[1], color[2]);
  actor->GetProperty ( )->SetLineWidth (width);
  actor->PickableOff ( );
  return actor;
}

} // namespace

void Camera_Pose_Renderer::Attach_Renderer (vtkRenderer* renderer)
{
  Remove_Actors ( );
  m_renderer = renderer;
  Ensure_Actors ( );
  Add_Actors ( );
}

void Camera_Pose_Renderer::Detach_Renderer ( )
{
  Remove_Actors ( );
  m_renderer = nullptr;
}

void Camera_Pose_Renderer::Set_World_From_Camera (
  const Matrix4& world_from_camera)
{
  if( !m_world_from_camera )
  {
    m_world_from_camera = vtkSmartPointer<vtkMatrix4x4>::New ( );
  }
  for( int row = 0; row < 4; ++row )
  {
    for( int column = 0; column < 4; ++column )
    {
      m_world_from_camera->SetElement (
        row, column, world_from_camera[row][column]);
    }
  }
  m_world_from_camera->Modified ( );
  Ensure_Actors ( );
  for( const auto& actor : m_actors )
  {
    actor->SetUserMatrix (m_world_from_camera);
    actor->Modified ( );
  }
}

void Camera_Pose_Renderer::Set_Visible (bool visible)
{
  m_visible = visible;
  Ensure_Actors ( );
  for( const auto& actor : m_actors )
  {
    actor->SetVisibility (visible ? 1 : 0);
  }
}

void Camera_Pose_Renderer::Ensure_Actors ( )
{
  if( !m_actors.empty ( ) )
  {
    return;
  }

  constexpr double axis_length = 300.0;
  m_actors.push_back (make_line_actor (
    { 0.0, 0.0, 0.0 }, { axis_length, 0.0, 0.0 },
    { 1.0, 0.1, 0.1 }, 4.0));
  m_actors.push_back (make_line_actor (
    { 0.0, 0.0, 0.0 }, { 0.0, axis_length, 0.0 },
    { 0.1, 1.0, 0.1 }, 4.0));
  m_actors.push_back (make_line_actor (
    { 0.0, 0.0, 0.0 }, { 0.0, 0.0, axis_length },
    { 0.1, 0.35, 1.0 }, 4.0));

  auto body_source = vtkSmartPointer<vtkCubeSource>::New ( );
  body_source->SetCenter (0.0, 0.0, -55.0);
  body_source->SetXLength (180.0);
  body_source->SetYLength (120.0);
  body_source->SetZLength (110.0);
  auto body_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ( );
  body_mapper->SetInputConnection (body_source->GetOutputPort ( ));
  auto body_actor = vtkSmartPointer<vtkActor>::New ( );
  body_actor->SetMapper (body_mapper);
  body_actor->GetProperty ( )->SetColor (0.72, 0.75, 0.80);
  body_actor->GetProperty ( )->SetOpacity (0.72);
  body_actor->PickableOff ( );
  m_actors.push_back (body_actor);

  auto origin_source = vtkSmartPointer<vtkSphereSource>::New ( );
  origin_source->SetCenter (0.0, 0.0, 0.0);
  origin_source->SetRadius (18.0);
  origin_source->SetThetaResolution (20);
  origin_source->SetPhiResolution (20);
  auto origin_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ( );
  origin_mapper->SetInputConnection (origin_source->GetOutputPort ( ));
  auto origin_actor = vtkSmartPointer<vtkActor>::New ( );
  origin_actor->SetMapper (origin_mapper);
  origin_actor->GetProperty ( )->SetColor (1.0, 0.85, 0.1);
  origin_actor->PickableOff ( );
  m_actors.push_back (origin_actor);

  constexpr double depth = 300.0;
  constexpr double half_width = 180.0;
  constexpr double half_height = 130.0;
  const std::array<std::array<double, 3>, 4> corners = {
    std::array<double, 3> { -half_width, -half_height, depth },
    std::array<double, 3> {  half_width, -half_height, depth },
    std::array<double, 3> {  half_width,  half_height, depth },
    std::array<double, 3> { -half_width,  half_height, depth }
  };
  const std::array<double, 3> origin = { 0.0, 0.0, 0.0 };
  const std::array<double, 3> frustum_color = { 1.0, 0.75, 0.15 };
  for( std::size_t i = 0; i < corners.size ( ); ++i )
  {
    m_actors.push_back (make_line_actor (
      origin, corners[i], frustum_color, 2.0));
    m_actors.push_back (make_line_actor (
      corners[i], corners[( i + 1 ) % corners.size ( )],
      frustum_color, 2.0));
  }

  for( const auto& actor : m_actors )
  {
    actor->SetVisibility (m_visible ? 1 : 0);
    if( m_world_from_camera )
    {
      actor->SetUserMatrix (m_world_from_camera);
    }
  }
}

void Camera_Pose_Renderer::Add_Actors ( )
{
  if( !m_renderer )
  {
    return;
  }
  for( const auto& actor : m_actors )
  {
    m_renderer->AddActor (actor);
  }
}

void Camera_Pose_Renderer::Remove_Actors ( )
{
  if( !m_renderer )
  {
    return;
  }
  for( const auto& actor : m_actors )
  {
    m_renderer->RemoveActor (actor);
  }
}

} // namespace robot_model

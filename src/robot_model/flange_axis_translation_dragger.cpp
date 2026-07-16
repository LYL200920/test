#include "flange_axis_translation_dragger.h"

#include <vtkRenderer.h>

#include <cmath>
#include <limits>

namespace robot_model
{

Flange_Axis_Translation_Dragger::Flange_Axis_Translation_Dragger (
  double axis_length_mm)
  : m_axis_length_mm (axis_length_mm)
{
}

void Flange_Axis_Translation_Dragger::Attach_Renderer (vtkRenderer* renderer)
{
  End_Drag ( );
  m_renderer = renderer;
}

void Flange_Axis_Translation_Dragger::Detach_Renderer ( )
{
  End_Drag ( );
  m_renderer = nullptr;
}

void Flange_Axis_Translation_Dragger::Set_Enabled (bool enabled)
{
  m_enabled = enabled;
  if( !enabled ) End_Drag ( );
}

bool Flange_Axis_Translation_Dragger::Begin_Drag (
  double display_x,
  double display_y,
  const Matrix4& world_from_flange)
{
  if( !m_enabled || !m_renderer ) return false;
  m_start_origin_world = {
    world_from_flange[0][3],
    world_from_flange[1][3],
    world_from_flange[2][3]
  };
  Point3 origin_display = { };
  if( !Display_From_World (m_start_origin_world, &origin_display) ) return false;
  const int best_axis = Hit_Test (display_x, display_y, world_from_flange);
  if( best_axis < 0 ) return false;
  Point3 endpoint_local = { };
  endpoint_local[best_axis] = m_axis_length_mm;
  Point3 best_endpoint_display = { };
  if( !Display_From_World (
        Transform_Position (world_from_flange, endpoint_local),
        &best_endpoint_display) ) return false;

  m_projected_axis_display = {
    best_endpoint_display[0] - origin_display[0],
    best_endpoint_display[1] - origin_display[1],
    0.0
  };
  m_projected_axis_length_squared =
    m_projected_axis_display[0] * m_projected_axis_display[0] +
    m_projected_axis_display[1] * m_projected_axis_display[1];
  if( m_projected_axis_length_squared < 20.0 * 20.0 ) return false;
  m_axis_world = {
    world_from_flange[0][best_axis],
    world_from_flange[1][best_axis],
    world_from_flange[2][best_axis]
  };
  const double length = std::sqrt (
    m_axis_world[0] * m_axis_world[0] +
    m_axis_world[1] * m_axis_world[1] +
    m_axis_world[2] * m_axis_world[2]);
  if( length < 1.0e-12 ) return false;
  for( double& value : m_axis_world ) value /= length;
  m_start_pointer_display = { display_x, display_y, 0.0 };
  m_selected_axis = best_axis;
  m_dragging = true;
  return true;
}

int Flange_Axis_Translation_Dragger::Hit_Test (
  double display_x,
  double display_y,
  const Matrix4& world_from_flange) const
{
  if( !m_enabled || !m_renderer ) return -1;
  int best_axis = -1;
  double best_distance_squared = std::numeric_limits<double>::infinity ( );
  for( int axis = 0; axis < 3; ++axis )
  {
    Point3 endpoint_local = { };
    endpoint_local[axis] = m_axis_length_mm;
    Point3 endpoint_display = { };
    if( !Display_From_World (
          Transform_Position (world_from_flange, endpoint_local),
          &endpoint_display) ) continue;
    const double dx = display_x - endpoint_display[0];
    const double dy = display_y - endpoint_display[1];
    const double distance_squared = dx * dx + dy * dy;
    if( distance_squared < best_distance_squared )
    {
      best_distance_squared = distance_squared;
      best_axis = axis;
    }
  }
  return best_distance_squared <= m_hit_radius_pixels * m_hit_radius_pixels
    ? best_axis : -1;
}

bool Flange_Axis_Translation_Dragger::Update_Drag (
  double display_x,
  double display_y,
  Point3* target_position_world) const
{
  if( !m_dragging || !target_position_world ) return false;
  const double delta_x = display_x - m_start_pointer_display[0];
  const double delta_y = display_y - m_start_pointer_display[1];
  const double distance_world =
    ( delta_x * m_projected_axis_display[0] +
      delta_y * m_projected_axis_display[1] ) /
    m_projected_axis_length_squared * m_axis_length_mm;
  for( int axis = 0; axis < 3; ++axis )
  {
    (*target_position_world)[axis] =
      m_start_origin_world[axis] + m_axis_world[axis] * distance_world;
  }
  return true;
}

void Flange_Axis_Translation_Dragger::End_Drag ( )
{
  m_dragging = false;
  m_selected_axis = -1;
}

bool Flange_Axis_Translation_Dragger::Display_From_World (
  const Point3& world,
  Point3* display) const
{
  if( !m_renderer || !display ) return false;
  m_renderer->SetWorldPoint (world[0], world[1], world[2], 1.0);
  m_renderer->WorldToDisplay ( );
  const double* value = m_renderer->GetDisplayPoint ( );
  if( !value ) return false;
  *display = { value[0], value[1], value[2] };
  return std::isfinite (value[0]) && std::isfinite (value[1]);
}

} // namespace robot_model

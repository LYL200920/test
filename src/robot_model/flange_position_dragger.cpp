#include "flange_position_dragger.h"

#include <vtkRenderer.h>

#include <cmath>
#include <cstddef>

namespace robot_model
{

void Flange_Position_Dragger::Attach_Renderer (vtkRenderer* renderer)
{
  End_Drag ( );
  m_renderer = renderer;
}

void Flange_Position_Dragger::Detach_Renderer ( )
{
  End_Drag ( );
  m_renderer = nullptr;
}

void Flange_Position_Dragger::Set_Enabled (bool enabled)
{
  m_enabled = enabled;
  if( !enabled )
  {
    End_Drag ( );
  }
}

bool Flange_Position_Dragger::Begin_Drag (
  double display_x,
  double display_y,
  const Point3& flange_position_world)
{
  if( !m_enabled || !m_renderer )
  {
    return false;
  }

  if( !Hit_Test (display_x, display_y, flange_position_world) )
  {
    return false;
  }
  Point3 flange_display = { };
  if( !Display_From_World (flange_position_world, &flange_display) )
  {
    return false;
  }
  Point3 pointer_world = { };
  if( !World_From_Display (
        display_x, display_y, flange_display[2], &pointer_world) )
  {
    return false;
  }
  for( std::size_t axis = 0; axis < 3; ++axis )
  {
    m_pointer_to_flange_offset_world[axis] =
      flange_position_world[axis] - pointer_world[axis];
  }
  m_drag_display_depth = flange_display[2];
  m_dragging = true;
  return true;
}

bool Flange_Position_Dragger::Hit_Test (
  double display_x,
  double display_y,
  const Point3& flange_position_world) const
{
  if( !m_enabled || !m_renderer ) return false;
  Point3 flange_display = { };
  if( !Display_From_World (flange_position_world, &flange_display) ) return false;
  const double dx = display_x - flange_display[0];
  const double dy = display_y - flange_display[1];
  return dx * dx + dy * dy <= m_hit_radius_pixels * m_hit_radius_pixels;
}

bool Flange_Position_Dragger::Update_Drag (
  double display_x,
  double display_y,
  Point3* target_position_world) const
{
  if( !m_dragging || !target_position_world )
  {
    return false;
  }
  Point3 pointer_world = { };
  if( !World_From_Display (
        display_x, display_y, m_drag_display_depth, &pointer_world) )
  {
    return false;
  }
  for( std::size_t axis = 0; axis < 3; ++axis )
  {
    (*target_position_world)[axis] =
      pointer_world[axis] + m_pointer_to_flange_offset_world[axis];
  }
  return true;
}

void Flange_Position_Dragger::End_Drag ( )
{
  m_dragging = false;
}

bool Flange_Position_Dragger::Display_From_World (
  const Point3& world,
  Point3* display) const
{
  if( !m_renderer || !display )
  {
    return false;
  }
  m_renderer->SetWorldPoint (world[0], world[1], world[2], 1.0);
  m_renderer->WorldToDisplay ( );
  const double* value = m_renderer->GetDisplayPoint ( );
  if( !value )
  {
    return false;
  }
  *display = { value[0], value[1], value[2] };
  return std::isfinite (value[0]) && std::isfinite (value[1]) &&
         std::isfinite (value[2]);
}

bool Flange_Position_Dragger::World_From_Display (
  double display_x,
  double display_y,
  double display_z,
  Point3* world) const
{
  if( !m_renderer || !world )
  {
    return false;
  }
  m_renderer->SetDisplayPoint (display_x, display_y, display_z);
  m_renderer->DisplayToWorld ( );
  const double* homogeneous = m_renderer->GetWorldPoint ( );
  if( !homogeneous || std::abs (homogeneous[3]) < 1.0e-12 )
  {
    return false;
  }
  *world = {
    homogeneous[0] / homogeneous[3],
    homogeneous[1] / homogeneous[3],
    homogeneous[2] / homogeneous[3]
  };
  return std::isfinite ((*world)[0]) && std::isfinite ((*world)[1]) &&
         std::isfinite ((*world)[2]);
}

} // namespace robot_model

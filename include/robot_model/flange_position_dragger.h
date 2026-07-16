#ifndef includeguard_flange_position_dragger_h_includeguard
#define includeguard_flange_position_dragger_h_includeguard

#include "pose_transform.h"

class vtkRenderer;

namespace robot_model
{

// Maps a screen drag near the flange origin onto the camera-parallel world
// plane through its start position. Rendering and IK remain separate layers.
class Flange_Position_Dragger
{
public:
  explicit Flange_Position_Dragger (double hit_radius_pixels = 24.0)
    : m_hit_radius_pixels (hit_radius_pixels) { }
  void Attach_Renderer (vtkRenderer* renderer);
  void Detach_Renderer ( );
  void Set_Enabled (bool enabled);

  bool Begin_Drag (double display_x,
                   double display_y,
                   const Point3& flange_position_world);
  bool Hit_Test (double display_x,
                 double display_y,
                 const Point3& flange_position_world) const;
  bool Update_Drag (double display_x,
                    double display_y,
                    Point3* target_position_world) const;
  void End_Drag ( );
  bool Is_Dragging ( ) const { return m_dragging; }

private:
  bool Display_From_World (const Point3& world, Point3* display) const;
  bool World_From_Display (double display_x,
                           double display_y,
                           double display_z,
                           Point3* world) const;

private:
  vtkRenderer* m_renderer = nullptr;
  Point3 m_pointer_to_flange_offset_world = { };
  double m_drag_display_depth = 0.0;
  double m_hit_radius_pixels = 24.0;
  bool m_enabled = false;
  bool m_dragging = false;
};

} // namespace robot_model

#endif

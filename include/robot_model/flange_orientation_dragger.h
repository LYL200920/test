#ifndef includeguard_flange_orientation_dragger_h_includeguard
#define includeguard_flange_orientation_dragger_h_includeguard

#include "pose_transform.h"

class vtkRenderer;

namespace robot_model
{

class Flange_Orientation_Dragger
{
public:
  explicit Flange_Orientation_Dragger (double ring_radius_mm = 145.0);

  void Attach_Renderer (vtkRenderer* renderer);
  void Detach_Renderer ( );
  void Set_Enabled (bool enabled);

  bool Begin_Drag (double display_x,
                   double display_y,
                   const Matrix4& world_from_flange);
  int Hit_Test (double display_x,
                double display_y,
                const Matrix4& world_from_flange) const;
  bool Update_Drag (double display_x,
                    double display_y,
                    Matrix4* target_world_from_flange) const;
  void End_Drag ( );
  bool Is_Dragging ( ) const { return m_dragging; }
  int Selected_Axis ( ) const { return m_selected_axis; }

private:
  bool Display_From_World (const Point3& world, Point3* display) const;

private:
  vtkRenderer* m_renderer = nullptr;
  Matrix4 m_start_world_from_flange = { };
  Point3 m_rotation_axis_world = { };
  Point3 m_origin_display = { };
  double m_ring_radius_mm = 145.0;
  double m_start_pointer_angle = 0.0;
  double m_hit_radius_pixels = 24.0;
  int m_selected_axis = -1;
  bool m_enabled = false;
  bool m_dragging = false;
};

} // namespace robot_model

#endif

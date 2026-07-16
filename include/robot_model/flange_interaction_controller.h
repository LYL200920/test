#ifndef includeguard_flange_interaction_controller_h_includeguard
#define includeguard_flange_interaction_controller_h_includeguard

#include "coordinate_frame_renderer.h"
#include "flange_axis_translation_dragger.h"
#include "flange_orientation_dragger.h"
#include "flange_position_dragger.h"
#include "flange_transform_gizmo_renderer.h"

class vtkRenderer;

namespace robot_model
{

enum class Flange_Interaction_Mode
{
  Observe,
  Free_Translation,
  Transform_6D
};

struct Flange_Drag_Update
{
  enum class Target_Type
  {
    None,
    Position,
    Pose
  };

  Target_Type target_type = Target_Type::None;
  Point3 target_position_world = { };
  Matrix4 target_world_from_flange = { };
};

// Owns flange interaction visuals, hit testing, and drag lifecycle. Robot IK
// and window-system event handling deliberately remain outside this class.
class Flange_Interaction_Controller
{
public:
  void Attach_Renderer (vtkRenderer* renderer);
  void Detach_Renderer ( );

  void Set_Mode (Flange_Interaction_Mode mode);
  Flange_Interaction_Mode Mode ( ) const { return m_mode; }
  void Set_Frame_Visible (bool visible);
  void Update_Flange_Pose (const Matrix4* world_from_flange);

  bool Begin_Drag (double display_x,
                   double display_y,
                   const Matrix4& world_from_flange);
  bool Update_Drag (double display_x,
                    double display_y,
                    Flange_Drag_Update* update) const;
  bool Is_Dragging ( ) const;
  void End_Drag ( );

  // Returns whether the pointer is over an active flange handle.
  bool Update_Hover (double display_x,
                     double display_y,
                     const Matrix4* world_from_flange,
                     bool* changed);

private:
  void Update_Visibility ( );
  void Clear_Hover ( );

private:
  Coordinate_Frame_Renderer m_flange_frame_renderer { 220.0 };
  Coordinate_Frame_Renderer m_free_drag_handle_renderer {
    0.0, 32.0, false };
  Flange_Transform_Gizmo_Renderer m_gizmo_renderer { 220.0, 145.0 };
  Flange_Position_Dragger m_position_dragger { 40.0 };
  Flange_Axis_Translation_Dragger m_axis_translation_dragger { 220.0 };
  Flange_Orientation_Dragger m_orientation_dragger { 145.0 };
  Flange_Interaction_Mode m_mode = Flange_Interaction_Mode::Observe;
  bool m_has_flange_pose = false;
  bool m_frame_visible = false;
  int m_hover_translation_axis = -1;
  int m_hover_rotation_axis = -1;
  bool m_hover_free_translation = false;
};

} // namespace robot_model

#endif

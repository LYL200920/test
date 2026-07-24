#include "flange_interaction_controller.h"

namespace robot_model
{

  void Flange_Interaction_Controller::Attach_Renderer(vtkRenderer *renderer)
  {
    m_flange_frame_renderer.Attach_Renderer(renderer);
    m_free_drag_handle_renderer.Attach_Renderer(renderer);
    m_gizmo_renderer.Attach_Renderer(renderer);
    m_position_dragger.Attach_Renderer(renderer);
    m_axis_translation_dragger.Attach_Renderer(renderer);
    m_orientation_dragger.Attach_Renderer(renderer);
    Set_Mode(m_mode);
    Update_Visibility();
  }

  void Flange_Interaction_Controller::Detach_Renderer()
  {
    End_Drag();
    m_orientation_dragger.Detach_Renderer();
    m_axis_translation_dragger.Detach_Renderer();
    m_position_dragger.Detach_Renderer();
    m_gizmo_renderer.Detach_Renderer();
    m_free_drag_handle_renderer.Detach_Renderer();
    m_flange_frame_renderer.Detach_Renderer();
  }

  void Flange_Interaction_Controller::Set_Mode(Flange_Interaction_Mode mode)
  {
    End_Drag();
    m_mode = mode;
    m_position_dragger.Set_Enabled(mode == Flange_Interaction_Mode::Free_Translation);
    m_axis_translation_dragger.Set_Enabled(mode == Flange_Interaction_Mode::Transform_6D);
    m_orientation_dragger.Set_Enabled(mode == Flange_Interaction_Mode::Transform_6D);
    Clear_Hover();
    Update_Visibility();
  }

  void Flange_Interaction_Controller::Set_Frame_Visible(bool visible)
  {
    m_frame_visible = visible;
    Update_Visibility();
  }

  void Flange_Interaction_Controller::Update_Flange_Pose(const Matrix4 *world_from_flange)
  {
    m_has_flange_pose = world_from_flange != nullptr;
    if (world_from_flange)
    {
      m_flange_frame_renderer.Set_World_From_Frame(*world_from_flange);
      m_free_drag_handle_renderer.Set_World_From_Frame(*world_from_flange);
      m_gizmo_renderer.Set_World_From_Flange(*world_from_flange);
    }
    else
    {
      End_Drag();
      Clear_Hover();
    }
    Update_Visibility();
  }

  bool Flange_Interaction_Controller::Begin_Drag(double display_x,
                                                 double display_y,
                                                 const Matrix4 &world_from_flange)
  {
    if (!m_has_flange_pose)
      return false;

    if (m_mode == Flange_Interaction_Mode::Transform_6D)
    {
      return m_axis_translation_dragger.Begin_Drag(display_x, display_y, world_from_flange) ||
             m_orientation_dragger.Begin_Drag(display_x, display_y, world_from_flange);
    }
    if (m_mode == Flange_Interaction_Mode::Free_Translation)
    {
      const Point3 flange_world = {world_from_flange[0][3],
                                   world_from_flange[1][3],
                                   world_from_flange[2][3]};
      return m_position_dragger.Begin_Drag(display_x, display_y, flange_world);
    }
    return false;
  }

  bool Flange_Interaction_Controller::Update_Drag(double display_x,
                                                  double display_y,
                                                  Flange_Drag_Update *update) const
  {
    if (!update)
      return false;
    *update = {};

    if (m_axis_translation_dragger.Is_Dragging() ||
        m_position_dragger.Is_Dragging())
    {
      Point3 target = {};
      const bool updated = m_axis_translation_dragger.Is_Dragging() ? m_axis_translation_dragger.Update_Drag(display_x, display_y, &target)
                                                                    : m_position_dragger.Update_Drag(display_x, display_y, &target);
      if (!updated)
        return false;
      update->target_type = Flange_Drag_Update::Target_Type::Position;
      update->target_position_world = target;
      return true;
    }

    if (m_orientation_dragger.Is_Dragging())
    {
      Matrix4 target = {};
      if (!m_orientation_dragger.Update_Drag(display_x, display_y, &target))
        return false;
      update->target_type = Flange_Drag_Update::Target_Type::Pose;
      update->target_world_from_flange = target;
      return true;
    }
    return false;
  }

  bool Flange_Interaction_Controller::Is_Dragging() const
  {
    return m_position_dragger.Is_Dragging() ||
           m_axis_translation_dragger.Is_Dragging() ||
           m_orientation_dragger.Is_Dragging();
  }

  void Flange_Interaction_Controller::End_Drag()
  {
    m_axis_translation_dragger.End_Drag();
    m_orientation_dragger.End_Drag();
    m_position_dragger.End_Drag();
  }

  bool Flange_Interaction_Controller::Update_Hover(double display_x,
                                                   double display_y,
                                                   const Matrix4 *world_from_flange,
                                                   bool *changed)
  {
    int translation_axis = -1;
    int rotation_axis = -1;
    bool free_translation = false;
    if (world_from_flange)
    {
      if (m_mode == Flange_Interaction_Mode::Transform_6D)
      {
        translation_axis = m_axis_translation_dragger.Hit_Test(display_x, display_y, *world_from_flange);
        if (translation_axis < 0)
        {
          rotation_axis = m_orientation_dragger.Hit_Test(display_x, display_y, *world_from_flange);
        }
      }
      else if (m_mode == Flange_Interaction_Mode::Free_Translation)
      {
        const Point3 origin = {(*world_from_flange)[0][3],
                               (*world_from_flange)[1][3],
                               (*world_from_flange)[2][3]};
        free_translation = m_position_dragger.Hit_Test(display_x, display_y, origin);
      }
    }

    const bool hover_changed = translation_axis != m_hover_translation_axis ||
                               rotation_axis != m_hover_rotation_axis ||
                               free_translation != m_hover_free_translation;
    if (hover_changed)
    {
      m_hover_translation_axis = translation_axis;
      m_hover_rotation_axis = rotation_axis;
      m_hover_free_translation = free_translation;
      m_gizmo_renderer.Set_Highlighted_Handle(translation_axis, rotation_axis);
      m_free_drag_handle_renderer.Set_Origin_Highlighted(free_translation);
    }
    if (changed)
      *changed = hover_changed;
    return translation_axis >= 0 || rotation_axis >= 0 || free_translation;
  }

  void Flange_Interaction_Controller::Update_Visibility()
  {
    m_flange_frame_renderer.Set_Visible(m_frame_visible && m_has_flange_pose);
    m_gizmo_renderer.Set_Visible(m_mode == Flange_Interaction_Mode::Transform_6D && m_has_flange_pose);
    m_free_drag_handle_renderer.Set_Visible(m_mode == Flange_Interaction_Mode::Free_Translation &&
                                            m_has_flange_pose);
  }

  void Flange_Interaction_Controller::Clear_Hover()
  {
    m_hover_translation_axis = -1;
    m_hover_rotation_axis = -1;
    m_hover_free_translation = false;
    m_gizmo_renderer.Set_Highlighted_Handle(-1, -1);
    m_free_drag_handle_renderer.Set_Origin_Highlighted(false);
  }

} // namespace robot_model

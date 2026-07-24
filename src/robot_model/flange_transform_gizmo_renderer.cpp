#include "flange_transform_gizmo_renderer.h"

#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRegularPolygonSource.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

#include <array>

namespace robot_model
{
  namespace
  {

    vtkSmartPointer<vtkActor> make_handle(const std::array<double, 3> &center,
                                          const std::array<double, 3> &color)
    {
      auto source = vtkSmartPointer<vtkSphereSource>::New();
      source->SetCenter(center.data());
      source->SetRadius(23.0);
      source->SetThetaResolution(20);
      source->SetPhiResolution(20);
      auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      mapper->SetInputConnection(source->GetOutputPort());
      auto actor = vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);
      actor->GetProperty()->SetColor(color[0], color[1], color[2]);
      actor->PickableOff();
      return actor;
    }

    vtkSmartPointer<vtkActor> make_ring(double radius,
                                        const std::array<double, 3> &normal,
                                        const std::array<double, 3> &color)
    {
      auto source = vtkSmartPointer<vtkRegularPolygonSource>::New();
      source->SetCenter(0.0, 0.0, 0.0);
      source->SetNormal(normal.data());
      source->SetRadius(radius);
      source->SetNumberOfSides(96);
      source->GeneratePolygonOff();
      auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      mapper->SetInputConnection(source->GetOutputPort());
      auto actor = vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);
      actor->GetProperty()->SetColor(color[0], color[1], color[2]);
      actor->GetProperty()->SetLineWidth(3.0);
      actor->GetProperty()->SetOpacity(0.8);
      actor->PickableOff();
      return actor;
    }

  } // namespace

  Flange_Transform_Gizmo_Renderer::Flange_Transform_Gizmo_Renderer(double axis_length_mm, double ring_radius_mm)
      : m_axis_length_mm(axis_length_mm), m_ring_radius_mm(ring_radius_mm)
  {
  }

  void Flange_Transform_Gizmo_Renderer::Attach_Renderer(vtkRenderer *renderer)
  {
    Remove_Actors();
    m_renderer = renderer;
    Ensure_Actors();
    Add_Actors();
  }

  void Flange_Transform_Gizmo_Renderer::Detach_Renderer()
  {
    Remove_Actors();
    m_renderer = nullptr;
  }

  void Flange_Transform_Gizmo_Renderer::Set_World_From_Flange(const Matrix4 &world_from_flange)
  {
    if (!m_world_from_flange)
    {
      m_world_from_flange = vtkSmartPointer<vtkMatrix4x4>::New();
    }
    for (int row = 0; row < 4; ++row)
    {
      for (int column = 0; column < 4; ++column)
      {
        m_world_from_flange->SetElement(row, column, world_from_flange[row][column]);
      }
    }
    m_world_from_flange->Modified();
    Ensure_Actors();
    for (const auto &actor : m_actors)
      actor->SetUserMatrix(m_world_from_flange);
  }

  void Flange_Transform_Gizmo_Renderer::Set_Visible(bool visible)
  {
    m_visible = visible;
    Ensure_Actors();
    for (const auto &actor : m_actors)
      actor->SetVisibility(visible ? 1 : 0);
  }

  void Flange_Transform_Gizmo_Renderer::Set_Highlighted_Handle(int translation_axis, int rotation_axis)
  {
    if (translation_axis == m_translation_highlight &&
        rotation_axis == m_rotation_highlight)
      return;
    m_translation_highlight = translation_axis;
    m_rotation_highlight = rotation_axis;
    Ensure_Actors();
    const double colors[3][3] = {{1.0, 0.1, 0.1}, {0.1, 1.0, 0.1}, {0.1, 0.35, 1.0}};
    for (int axis = 0; axis < 3; ++axis)
    {
      auto *handle_property = m_actors[axis * 2]->GetProperty();
      const bool handle_highlighted = axis == translation_axis;
      handle_property->SetColor(handle_highlighted ? 1.0 : colors[axis][0],
                                handle_highlighted ? 0.9 : colors[axis][1],
                                handle_highlighted ? 0.2 : colors[axis][2]);
      m_actors[axis * 2]->SetScale(handle_highlighted ? 1.25 : 1.0);

      auto *ring_property = m_actors[axis * 2 + 1]->GetProperty();
      const bool ring_highlighted = axis == rotation_axis;
      ring_property->SetColor(ring_highlighted ? 1.0 : colors[axis][0],
                              ring_highlighted ? 0.9 : colors[axis][1],
                              ring_highlighted ? 0.2 : colors[axis][2]);
      ring_property->SetLineWidth(ring_highlighted ? 6.0 : 3.0);
      ring_property->SetOpacity(ring_highlighted ? 1.0 : 0.8);
    }
  }

  void Flange_Transform_Gizmo_Renderer::Ensure_Actors()
  {
    if (!m_actors.empty())
      return;
    const std::array<std::array<double, 3>, 3> colors = {std::array<double, 3>{1.0, 0.1, 0.1},
                                                         std::array<double, 3>{0.1, 1.0, 0.1},
                                                         std::array<double, 3>{0.1, 0.35, 1.0}};
    for (int axis = 0; axis < 3; ++axis)
    {
      std::array<double, 3> center = {};
      center[axis] = m_axis_length_mm;
      m_actors.push_back(make_handle(center, colors[axis]));
      std::array<double, 3> normal = {};
      normal[axis] = 1.0;
      m_actors.push_back(make_ring(m_ring_radius_mm, normal, colors[axis]));
    }
    for (const auto &actor : m_actors)
    {
      actor->SetVisibility(m_visible ? 1 : 0);
      if (m_world_from_flange)
        actor->SetUserMatrix(m_world_from_flange);
    }
  }

  void Flange_Transform_Gizmo_Renderer::Add_Actors()
  {
    if (!m_renderer)
      return;
    for (const auto &actor : m_actors)
      m_renderer->AddActor(actor);
  }

  void Flange_Transform_Gizmo_Renderer::Remove_Actors()
  {
    if (!m_renderer)
      return;
    for (const auto &actor : m_actors)
      m_renderer->RemoveActor(actor);
  }

} // namespace robot_model

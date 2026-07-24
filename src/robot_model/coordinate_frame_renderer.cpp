#include "coordinate_frame_renderer.h"

#include <vtkLineSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>

#include <array>

namespace robot_model
{
  namespace
  {

    vtkSmartPointer<vtkActor> make_axis(const std::array<double, 3> &end, const std::array<double, 3> &color)
    {
      auto source = vtkSmartPointer<vtkLineSource>::New();
      source->SetPoint1(0.0, 0.0, 0.0);
      source->SetPoint2(end.data());
      auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      mapper->SetInputConnection(source->GetOutputPort());
      auto actor = vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);
      actor->GetProperty()->SetColor(color[0], color[1], color[2]);
      actor->GetProperty()->SetLineWidth(5.0);
      actor->PickableOff();
      return actor;
    }

  } // namespace

  Coordinate_Frame_Renderer::Coordinate_Frame_Renderer(double axis_length_mm,
                                                       double origin_radius_mm,
                                                       bool show_axes)
      : m_axis_length_mm(axis_length_mm),
        m_origin_radius_mm(origin_radius_mm),
        m_show_axes(show_axes)
  {
  }

  void Coordinate_Frame_Renderer::Set_Origin_Highlighted(bool highlighted)
  {
    Ensure_Actors();
    if (!m_origin_actor)
      return;
    m_origin_actor->GetProperty()->SetColor(highlighted ? 1.0 : 1.0,
                                            highlighted ? 1.0 : 0.85,
                                            highlighted ? 0.35 : 0.1);
    m_origin_actor->SetScale(highlighted ? 1.18 : 1.0);
  }

  void Coordinate_Frame_Renderer::Attach_Renderer(vtkRenderer *renderer)
  {
    Remove_Actors();
    m_renderer = renderer;
    Ensure_Actors();
    Add_Actors();
  }

  void Coordinate_Frame_Renderer::Detach_Renderer()
  {
    Remove_Actors();
    m_renderer = nullptr;
  }

  void Coordinate_Frame_Renderer::Set_World_From_Frame(const Matrix4 &world_from_frame)
  {
    if (!m_world_from_frame)
    {
      m_world_from_frame = vtkSmartPointer<vtkMatrix4x4>::New();
    }
    for (int row = 0; row < 4; ++row)
    {
      for (int column = 0; column < 4; ++column)
      {
        m_world_from_frame->SetElement(row, column, world_from_frame[row][column]);
      }
    }
    m_world_from_frame->Modified();
    Ensure_Actors();
    for (const auto &actor : m_actors)
    {
      actor->SetUserMatrix(m_world_from_frame);
      actor->Modified();
    }
  }

  void Coordinate_Frame_Renderer::Set_Visible(bool visible)
  {
    m_visible = visible;
    Ensure_Actors();
    for (const auto &actor : m_actors)
    {
      actor->SetVisibility(visible ? 1 : 0);
    }
  }

  void Coordinate_Frame_Renderer::Ensure_Actors()
  {
    if (!m_actors.empty())
    {
      return;
    }
    if (m_show_axes)
    {
      m_actors.push_back(make_axis({m_axis_length_mm, 0.0, 0.0}, {1.0, 0.1, 0.1}));
      m_actors.push_back(make_axis({0.0, m_axis_length_mm, 0.0}, {0.1, 1.0, 0.1}));
      m_actors.push_back(make_axis({0.0, 0.0, m_axis_length_mm}, {0.1, 0.35, 1.0}));
    }

    auto origin = vtkSmartPointer<vtkSphereSource>::New();
    origin->SetRadius(m_origin_radius_mm);
    origin->SetThetaResolution(20);
    origin->SetPhiResolution(20);
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(origin->GetOutputPort());
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.85, 0.1);
    actor->PickableOff();
    m_origin_actor = actor;
    m_actors.push_back(m_origin_actor);

    for (const auto &item : m_actors)
    {
      item->SetVisibility(m_visible ? 1 : 0);
      if (m_world_from_frame)
      {
        item->SetUserMatrix(m_world_from_frame);
      }
    }
  }

  void Coordinate_Frame_Renderer::Add_Actors()
  {
    if (!m_renderer)
    {
      return;
    }
    for (const auto &actor : m_actors)
    {
      m_renderer->AddActor(actor);
    }
  }

  void Coordinate_Frame_Renderer::Remove_Actors()
  {
    if (!m_renderer)
    {
      return;
    }
    for (const auto &actor : m_actors)
    {
      m_renderer->RemoveActor(actor);
    }
  }

} // namespace robot_model

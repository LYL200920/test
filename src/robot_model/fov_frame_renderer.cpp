#include "fov_frame_renderer.h"

#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>

#include <algorithm>
#include <array>

namespace robot_model
{
namespace
{

vtkSmartPointer<vtkActor> Make_Line_Actor(
  vtkLineSource *source,
  const std::array<double, 3> &color,
  double width)
{
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(source->GetOutputPort());
  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(
    color[0], color[1], color[2]);
  actor->GetProperty()->SetLineWidth(width);
  actor->PickableOff();
  return actor;
}

} // namespace

void Fov_Frame_Renderer::Attach_Renderer(vtkRenderer *renderer)
{
  Remove_Actors();
  m_renderer = renderer;
  Ensure_Actors();
  Add_Actors();
}

void Fov_Frame_Renderer::Detach_Renderer()
{
  Remove_Actors();
  m_renderer = nullptr;
}

void Fov_Frame_Renderer::Set_Configuration(
  const Fov_Visualization_Configuration &configuration)
{
  m_configuration = configuration;
  Tool_Visualization_Configuration wrapper;
  wrapper.fov = m_configuration;
  Normalize_Tool_Visualization_Configuration(&wrapper);
  m_configuration = wrapper.fov;
  Ensure_Actors();
  Update_Geometry();
}

void Fov_Frame_Renderer::Set_World_From_Fov(
  const Matrix4 &world_from_fov)
{
  if (!m_world_from_fov)
  {
    m_world_from_fov = vtkSmartPointer<vtkMatrix4x4>::New();
  }
  for (int row = 0; row < 4; ++row)
  {
    for (int column = 0; column < 4; ++column)
    {
      m_world_from_fov->SetElement(
        row, column, world_from_fov[row][column]);
    }
  }
  m_world_from_fov->Modified();
  Ensure_Actors();
  for (const auto &actor : m_actors)
  {
    actor->SetUserMatrix(m_world_from_fov);
    actor->Modified();
  }
}

void Fov_Frame_Renderer::Set_Visible(bool visible)
{
  m_visible = visible;
  Ensure_Actors();
  for (const auto &actor : m_actors)
  {
    actor->SetVisibility(visible ? 1 : 0);
  }
}

void Fov_Frame_Renderer::Ensure_Actors()
{
  if (!m_actors.empty())
  {
    return;
  }

  constexpr std::array<double, 3> fov_color = {
    1.0, 0.75, 0.15};
  constexpr std::array<std::array<double, 3>, 3> axis_colors = {{
    {1.0, 0.1, 0.1},
    {0.1, 1.0, 0.1},
    {0.1, 0.35, 1.0}}};

  for (std::size_t index = 0; index < 7; ++index)
  {
    auto source = vtkSmartPointer<vtkLineSource>::New();
    m_line_sources.push_back(source);
    m_actors.push_back(Make_Line_Actor(
      source,
      index < 4 ? fov_color : axis_colors[index - 4],
      index < 4 ? 3.0 : 4.0));
  }

  auto origin_source = vtkSmartPointer<vtkSphereSource>::New();
  origin_source->SetRadius(8.0);
  origin_source->SetThetaResolution(20);
  origin_source->SetPhiResolution(20);
  auto origin_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  origin_mapper->SetInputConnection(origin_source->GetOutputPort());
  auto origin_actor = vtkSmartPointer<vtkActor>::New();
  origin_actor->SetMapper(origin_mapper);
  origin_actor->GetProperty()->SetColor(1.0, 0.85, 0.1);
  origin_actor->PickableOff();
  m_actors.push_back(origin_actor);

  Update_Geometry();
  for (const auto &actor : m_actors)
  {
    actor->SetVisibility(m_visible ? 1 : 0);
    if (m_world_from_fov)
    {
      actor->SetUserMatrix(m_world_from_fov);
    }
  }
}

void Fov_Frame_Renderer::Update_Geometry()
{
  if (m_line_sources.size() < 7)
  {
    return;
  }
  const auto corners = Build_Fov_Local_Corners(m_configuration);
  for (std::size_t edge = 0; edge < 4; ++edge)
  {
    m_line_sources[edge]->SetPoint1(corners[edge].data());
    m_line_sources[edge]->SetPoint2(
      corners[(edge + 1) % corners.size()].data());
    m_line_sources[edge]->Modified();
  }

  const double axis_length = std::clamp(
    std::min(m_configuration.width_mm,
             m_configuration.length_mm) * 0.25,
    30.0,
    150.0);
  const Point3 origin = {};
  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    Point3 endpoint = {};
    endpoint[axis] = axis_length;
    m_line_sources[axis + 4]->SetPoint1(origin.data());
    m_line_sources[axis + 4]->SetPoint2(endpoint.data());
    m_line_sources[axis + 4]->Modified();
  }
}

void Fov_Frame_Renderer::Add_Actors()
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

void Fov_Frame_Renderer::Remove_Actors()
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

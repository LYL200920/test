#include "view_cube_controller.h"

#include <vtkArrowSource.h>
#include <vtkCamera.h>
#include <vtkCoordinate.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkMath.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace robot_model
{
  namespace
  {
    constexpr double kViewCubeViewportLeft = 0.80;
    constexpr double kViewCubeViewportBottom = 0.02;
    constexpr double kViewCubeViewportRight = 0.99;
    constexpr double kViewCubeViewportTop = 0.28;
    constexpr double kViewCubeArrowLength = 0.36;
    constexpr double kViewCubeHalfSize = kViewCubeArrowLength * 0.25;
    constexpr double kViewCubeLabelOffset = 0.045;
    constexpr double kViewCubeFaceLabelVisibleDot = 0.08;
    constexpr double kViewCubeCameraDistance = 2.0;
    constexpr double kViewCubeParallelScale = 0.55;
    constexpr double kViewCubeArrowOrigin[3] = {-kViewCubeHalfSize,
                                                -kViewCubeHalfSize,
                                                -kViewCubeHalfSize};

    enum class View_Cube_Face
    {
      none,
      front,
      back,
      left,
      right,
      top,
      bottom
    };

    struct View_Cube_Axis_Info
    {
      const char *label;
      double direction[3];
      double color[3];
    };

    struct View_Cube_Face_Info
    {
      View_Cube_Face face;
      int axis;
      double sign;
      const char *label;
    };

    constexpr std::array<View_Cube_Axis_Info, 3> kViewCubeAxes = {{{"X", {1.0, 0.0, 0.0}, {0.95, 0.16, 0.18}},
                                                                   {"Y", {0.0, 1.0, 0.0}, {0.10, 0.80, 0.24}},
                                                                   {"Z", {0.0, 0.0, 1.0}, {0.10, 0.45, 0.95}}}};

    constexpr std::array<View_Cube_Face_Info, 6> kViewCubeFaces = {{{View_Cube_Face::front, 1, -1.0, "L"},
                                                                    {View_Cube_Face::back, 1, 1.0, "R"},
                                                                    {View_Cube_Face::left, 0, -1.0, "B"},
                                                                    {View_Cube_Face::right, 0, 1.0, "F"},
                                                                    {View_Cube_Face::top, 2, 1.0, "U"},
                                                                    {View_Cube_Face::bottom, 2, -1.0, "D"}}};

    constexpr double kDirectionXPositive[3] = {1.0, 0.0, 0.0};
    constexpr double kDirectionXNegative[3] = {-1.0, 0.0, 0.0};
    constexpr double kDirectionYPositive[3] = {0.0, 1.0, 0.0};
    constexpr double kDirectionYNegative[3] = {0.0, -1.0, 0.0};
    constexpr double kDirectionZPositive[3] = {0.0, 0.0, 1.0};
    constexpr double kDirectionZNegative[3] = {0.0, 0.0, -1.0};

    const View_Cube_Face_Info *find_view_cube_face_info(View_Cube_Face face)
    {
      for (const auto &face_info : kViewCubeFaces)
      {
        if (face_info.face == face)
        {
          return &face_info;
        }
      }
      return nullptr;
    }

    bool display_to_world(vtkRenderer *renderer,
                          double x,
                          double y,
                          double z,
                          double world[3])
    {
      if (!renderer)
      {
        return false;
      }

      renderer->SetDisplayPoint(x, y, z);
      renderer->DisplayToWorld();

      const auto *world_point = renderer->GetWorldPoint();
      if (!world_point || std::abs(world_point[3]) < 1.0e-12)
      {
        return false;
      }

      world[0] = world_point[0] / world_point[3];
      world[1] = world_point[1] / world_point[3];
      world[2] = world_point[2] / world_point[3];
      return true;
    }

    bool world_to_display(vtkRenderer *renderer,
                          const double world[3],
                          double display[3])
    {
      if (!renderer)
      {
        return false;
      }

      renderer->SetWorldPoint(world[0], world[1], world[2], 1.0);
      renderer->WorldToDisplay();

      const auto *display_point = renderer->GetDisplayPoint();
      if (!display_point)
      {
        return false;
      }

      display[0] = display_point[0];
      display[1] = display_point[1];
      display[2] = display_point[2];
      return true;
    }

    void axis_label_position(int axis, double position[3])
    {
      position[0] = kViewCubeArrowOrigin[0];
      position[1] = kViewCubeArrowOrigin[1];
      position[2] = kViewCubeArrowOrigin[2];
      if (axis >= 0 && axis < 3)
      {
        position[axis] += kViewCubeArrowLength + kViewCubeLabelOffset;
      }
    }

    bool face_label_position_and_normal(View_Cube_Face face,
                                        double position[3],
                                        double normal[3])
    {
      position[0] = 0.0;
      position[1] = 0.0;
      position[2] = 0.0;
      normal[0] = 0.0;
      normal[1] = 0.0;
      normal[2] = 0.0;

      const auto *face_info = find_view_cube_face_info(face);
      if (!face_info)
      {
        return false;
      }

      position[face_info->axis] = face_info->sign * kViewCubeHalfSize;
      normal[face_info->axis] = face_info->sign;
      return true;
    }

    View_Cube_Face pick_view_cube_face(vtkRenderer *renderer,
                                       double x,
                                       double y)
    {
      double near_point[3] = {0.0, 0.0, 0.0};
      double far_point[3] = {0.0, 0.0, 0.0};
      if (!display_to_world(renderer, x, y, 0.0, near_point) ||
          !display_to_world(renderer, x, y, 1.0, far_point))
      {
        return View_Cube_Face::none;
      }

      const double direction[3] = {far_point[0] - near_point[0],
                                   far_point[1] - near_point[1],
                                   far_point[2] - near_point[2]};

      View_Cube_Face best_face = View_Cube_Face::none;
      double best_t = std::numeric_limits<double>::max();
      const double tolerance = kViewCubeHalfSize * 0.03;

      auto test_plane = [&](int axis, double plane_position, View_Cube_Face face)
      {
        if (std::abs(direction[axis]) < 1.0e-12)
        {
          return;
        }

        const double t = (plane_position - near_point[axis]) / direction[axis];
        if (t < 0.0 || t >= best_t)
        {
          return;
        }

        double hit[3] = {near_point[0] + direction[0] * t,
                         near_point[1] + direction[1] * t,
                         near_point[2] + direction[2] * t};

        for (int i = 0; i < 3; ++i)
        {
          if (i == axis)
          {
            continue;
          }
          if (hit[i] < -kViewCubeHalfSize - tolerance ||
              hit[i] > kViewCubeHalfSize + tolerance)
          {
            return;
          }
        }

        best_t = t;
        best_face = face;
      };

      for (const auto &face_info : kViewCubeFaces)
      {
        test_plane(face_info.axis,
                   face_info.sign * kViewCubeHalfSize,
                   face_info.face);
      }

      return best_face;
    }

    void compute_scene_center_and_radius(vtkRenderer *renderer,
                                         double center[3],
                                         double &radius)
    {
      double bounds[6] = {-1000.0, 1000.0, -1000.0, 1000.0, 0.0, 1500.0};
      renderer->ComputeVisiblePropBounds(bounds);

      center[0] = (bounds[0] + bounds[1]) * 0.5;
      center[1] = (bounds[2] + bounds[3]) * 0.5;
      center[2] = (bounds[4] + bounds[5]) * 0.5;

      const double x_size = std::max(bounds[1] - bounds[0], 1.0);
      const double y_size = std::max(bounds[3] - bounds[2], 1.0);
      const double z_size = std::max(bounds[5] - bounds[4], 1.0);
      radius = std::max({x_size, y_size, z_size});
    }

    vtkSmartPointer<vtkActor> make_plane_actor(const double origin[3],
                                               const double point1[3],
                                               const double point2[3],
                                               const double color[3])
    {
      auto plane = vtkSmartPointer<vtkPlaneSource>::New();
      plane->SetOrigin(origin[0], origin[1], origin[2]);
      plane->SetPoint1(point1[0], point1[1], point1[2]);
      plane->SetPoint2(point2[0], point2[1], point2[2]);
      plane->Update();

      auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      mapper->SetInputConnection(plane->GetOutputPort());

      auto actor = vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);
      actor->GetProperty()->SetColor(color[0], color[1], color[2]);
      actor->GetProperty()->SetOpacity(1.0);
      actor->GetProperty()->LightingOff();
      actor->GetProperty()->SetEdgeVisibility(true);
      actor->GetProperty()->SetEdgeColor(0.08, 0.09, 0.10);
      actor->GetProperty()->SetLineWidth(1.5);
      return actor;
    }

    vtkSmartPointer<vtkTextActor> make_text_label_actor(const char *text,
                                                        const double color[3],
                                                        int font_size = 9,
                                                        double opacity = 0.80)
    {
      auto actor = vtkSmartPointer<vtkTextActor>::New();
      actor->SetInput(text);
      actor->GetPositionCoordinate()->SetCoordinateSystemToDisplay();
      actor->PickableOff();
      actor->UseBoundsOff();

      auto *property = actor->GetTextProperty();
      if (property)
      {
        property->SetFontFamilyToArial();
        property->SetFontSize(font_size);
        property->BoldOff();
        property->ItalicOff();
        property->ShadowOff();
        property->SetColor(color[0], color[1], color[2]);
        property->SetOpacity(opacity);
        property->SetBackgroundOpacity(0.0);
        property->SetJustificationToCentered();
        property->SetVerticalJustificationToCentered();
        property->UseTightBoundingBoxOn();
      }

      return actor;
    }

    void add_text_label_actor(vtkRenderer *renderer,
                              vtkSmartPointer<vtkTextActor> &actor,
                              const char *text,
                              const double color[3],
                              int font_size = 9,
                              double opacity = 0.80)
    {
      if (!renderer)
      {
        return;
      }

      actor = make_text_label_actor(text, color, font_size, opacity);
      renderer->AddViewProp(actor);
    }

    bool set_projected_label_position(vtkRenderer *renderer,
                                      vtkTextActor *actor,
                                      const double world[3])
    {
      if (!actor)
      {
        return false;
      }

      double display[3] = {0.0, 0.0, 0.0};
      if (!world_to_display(renderer, world, display))
      {
        return false;
      }

      actor->SetPosition(display[0], display[1]);
      return true;
    }

    vtkSmartPointer<vtkActor> make_arrow_actor(const double start[3],
                                               const double direction[3],
                                               double length,
                                               const double color[3])
    {
      double normalized_direction[3] = {direction[0],
                                        direction[1],
                                        direction[2]};
      if (vtkMath::Normalize(normalized_direction) == 0.0)
      {
        normalized_direction[0] = 1.0;
        normalized_direction[1] = 0.0;
        normalized_direction[2] = 0.0;
      }

      auto arrow = vtkSmartPointer<vtkArrowSource>::New();
      arrow->SetTipLength(0.24);
      arrow->SetTipRadius(0.045);
      arrow->SetShaftRadius(0.015);

      double rotation_axis[3] = {0.0, 0.0, 0.0};
      const double x_axis[3] = {1.0, 0.0, 0.0};
      vtkMath::Cross(x_axis, normalized_direction, rotation_axis);
      double rotation_angle = vtkMath::DegreesFromRadians(std::acos(std::clamp(vtkMath::Dot(x_axis, normalized_direction),
                                                                               -1.0,
                                                                               1.0)));

      auto transform = vtkSmartPointer<vtkTransform>::New();
      transform->PostMultiply();
      transform->Scale(length, length, length);
      if (vtkMath::Norm(rotation_axis) > 1.0e-6)
      {
        transform->RotateWXYZ(rotation_angle,
                              rotation_axis[0],
                              rotation_axis[1],
                              rotation_axis[2]);
      }
      else if (normalized_direction[0] < 0.0)
      {
        transform->RotateWXYZ(180.0, 0.0, 0.0, 1.0);
      }
      transform->Translate(start[0], start[1], start[2]);

      auto transform_filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
      transform_filter->SetInputConnection(arrow->GetOutputPort());
      transform_filter->SetTransform(transform);
      transform_filter->Update();

      auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      mapper->SetInputConnection(transform_filter->GetOutputPort());

      auto actor = vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);
      actor->PickableOff();
      actor->GetProperty()->SetColor(color[0], color[1], color[2]);
      actor->GetProperty()->LightingOff();
      return actor;
    }

    void set_camera_view(vtkRenderer *renderer,
                         const double direction[3],
                         const double view_up[3])
    {
      if (!renderer)
      {
        return;
      }

      auto *camera = renderer->GetActiveCamera();
      if (!camera)
      {
        return;
      }

      double center[3] = {0.0, 0.0, 0.0};
      double radius = 1.0;
      compute_scene_center_and_radius(renderer, center, radius);

      double normalized_direction[3] = {direction[0],
                                        direction[1],
                                        direction[2]};
      if (vtkMath::Normalize(normalized_direction) == 0.0)
      {
        normalized_direction[0] = 1.0;
        normalized_direction[1] = 0.0;
        normalized_direction[2] = 0.0;
      }

      const double distance = radius * 2.4;
      camera->SetFocalPoint(center[0], center[1], center[2]);
      camera->SetPosition(center[0] + normalized_direction[0] * distance,
                          center[1] + normalized_direction[1] * distance,
                          center[2] + normalized_direction[2] * distance);
      camera->SetViewUp(view_up[0], view_up[1], view_up[2]);
      renderer->ResetCameraClippingRange();
    }

    bool set_camera_for_view_cube_face(vtkRenderer *renderer, View_Cube_Face face)
    {
      switch (face)
      {
      case View_Cube_Face::top:
        set_camera_view(renderer, kDirectionZPositive, kDirectionXNegative);
        return true;
      case View_Cube_Face::bottom:
        set_camera_view(renderer, kDirectionZNegative, kDirectionYPositive);
        return true;
      case View_Cube_Face::left:
        set_camera_view(renderer, kDirectionXNegative, kDirectionZPositive);
        return true;
      case View_Cube_Face::right:
        set_camera_view(renderer, kDirectionXPositive, kDirectionZPositive);
        return true;
      case View_Cube_Face::back:
        set_camera_view(renderer, kDirectionYPositive, kDirectionZPositive);
        return true;
      case View_Cube_Face::front:
        set_camera_view(renderer, kDirectionYNegative, kDirectionZPositive);
        return true;
      case View_Cube_Face::none:
        return false;
      }

      return false;
    }

  } // namespace

  void View_Cube_Controller::Setup(vtkGenericOpenGLRenderWindow *render_window,
                                   vtkRenderer *scene_renderer)
  {
    if (!render_window || !scene_renderer || m_view_cube_renderer)
    {
      return;
    }

    m_render_window = render_window;
    m_scene_renderer = scene_renderer;
    m_view_cube_renderer = vtkSmartPointer<vtkRenderer>::New();
    m_view_cube_renderer->SetLayer(1);
    m_view_cube_renderer->SetViewport(kViewCubeViewportLeft,
                                      kViewCubeViewportBottom,
                                      kViewCubeViewportRight,
                                      kViewCubeViewportTop);
    m_view_cube_renderer->SetPreserveColorBuffer(true);
    m_view_cube_renderer->SetPreserveDepthBuffer(false);
    m_view_cube_renderer->SetBackground(0.0, 0.0, 0.0);
    m_view_cube_renderer->InteractiveOff();
    m_render_window->AddRenderer(m_view_cube_renderer);

    m_view_cube_assembly = vtkSmartPointer<vtkPropAssembly>::New();

    const double front_color[3] = {0.62, 0.66, 0.70};
    const double back_color[3] = {0.50, 0.54, 0.58};
    const double left_color[3] = {0.56, 0.60, 0.64};
    const double right_color[3] = {0.68, 0.72, 0.76};
    const double top_color[3] = {0.64, 0.68, 0.72};
    const double bottom_color[3] = {0.46, 0.50, 0.54};

    const double p000[3] = {-kViewCubeHalfSize, -kViewCubeHalfSize, -kViewCubeHalfSize};
    const double p001[3] = {-kViewCubeHalfSize, -kViewCubeHalfSize, kViewCubeHalfSize};
    const double p010[3] = {-kViewCubeHalfSize, kViewCubeHalfSize, -kViewCubeHalfSize};
    const double p011[3] = {-kViewCubeHalfSize, kViewCubeHalfSize, kViewCubeHalfSize};
    const double p100[3] = {kViewCubeHalfSize, -kViewCubeHalfSize, -kViewCubeHalfSize};
    const double p101[3] = {kViewCubeHalfSize, -kViewCubeHalfSize, kViewCubeHalfSize};
    const double p110[3] = {kViewCubeHalfSize, kViewCubeHalfSize, -kViewCubeHalfSize};
    const double p111[3] = {kViewCubeHalfSize, kViewCubeHalfSize, kViewCubeHalfSize};

    m_view_cube_assembly->AddPart(make_plane_actor(p100, p000, p101, front_color));
    m_view_cube_assembly->AddPart(make_plane_actor(p010, p110, p011, back_color));
    m_view_cube_assembly->AddPart(make_plane_actor(p000, p010, p001, left_color));
    m_view_cube_assembly->AddPart(make_plane_actor(p110, p100, p111, right_color));
    m_view_cube_assembly->AddPart(make_plane_actor(p101, p001, p111, top_color));
    m_view_cube_assembly->AddPart(make_plane_actor(p000, p100, p010, bottom_color));

    for (const auto &axis_info : kViewCubeAxes)
    {
      m_view_cube_assembly->AddPart(make_arrow_actor(kViewCubeArrowOrigin,
                                                     axis_info.direction,
                                                     kViewCubeArrowLength,
                                                     axis_info.color));
    }

    m_view_cube_renderer->AddViewProp(m_view_cube_assembly);
    for (std::size_t i = 0; i < kViewCubeAxes.size(); ++i)
    {
      add_text_label_actor(m_view_cube_renderer,
                           m_axis_labels[i],
                           kViewCubeAxes[i].label,
                           kViewCubeAxes[i].color);
    }

    const double face_label_color[3] = {0.04, 0.05, 0.06};
    for (std::size_t i = 0; i < kViewCubeFaces.size(); ++i)
    {
      add_text_label_actor(m_view_cube_renderer,
                           m_face_labels[i],
                           kViewCubeFaces[i].label,
                           face_label_color,
                           9,
                           0.90);
    }
    Update_Camera();
  }

  void View_Cube_Controller::Shutdown()
  {
    if (m_render_window && m_view_cube_renderer)
    {
      m_render_window->RemoveRenderer(m_view_cube_renderer);
    }

    m_view_cube_assembly = nullptr;
    m_axis_labels = {};
    m_face_labels = {};
    m_view_cube_renderer = nullptr;
    m_scene_renderer = nullptr;
    m_render_window = nullptr;
  }

  void View_Cube_Controller::Update_Camera()
  {
    if (!m_scene_renderer || !m_view_cube_renderer)
    {
      return;
    }

    auto *scene_camera = m_scene_renderer->GetActiveCamera();
    auto *cube_camera = m_view_cube_renderer->GetActiveCamera();
    if (!scene_camera || !cube_camera)
    {
      return;
    }

    double scene_position[3] = {0.0, 0.0, 1.0};
    double scene_focal_point[3] = {0.0, 0.0, 0.0};
    double scene_view_up[3] = {0.0, 0.0, 1.0};
    scene_camera->GetPosition(scene_position);
    scene_camera->GetFocalPoint(scene_focal_point);
    scene_camera->GetViewUp(scene_view_up);

    double direction[3] = {scene_position[0] - scene_focal_point[0],
                           scene_position[1] - scene_focal_point[1],
                           scene_position[2] - scene_focal_point[2]};
    if (vtkMath::Normalize(direction) == 0.0)
    {
      direction[0] = 1.0;
      direction[1] = -1.0;
      direction[2] = 0.7;
      vtkMath::Normalize(direction);
    }

    cube_camera->SetFocalPoint(0.0, 0.0, 0.0);
    cube_camera->SetPosition(direction[0] * kViewCubeCameraDistance,
                             direction[1] * kViewCubeCameraDistance,
                             direction[2] * kViewCubeCameraDistance);
    cube_camera->SetViewUp(scene_view_up[0],
                           scene_view_up[1],
                           scene_view_up[2]);
    cube_camera->OrthogonalizeViewUp();
    cube_camera->SetParallelProjection(true);
    cube_camera->SetParallelScale(kViewCubeParallelScale);
    m_view_cube_renderer->ResetCameraClippingRange();

    for (std::size_t i = 0; i < m_axis_labels.size(); ++i)
    {
      double world[3] = {0.0, 0.0, 0.0};
      axis_label_position(static_cast<int>(i), world);
      set_projected_label_position(m_view_cube_renderer,
                                   m_axis_labels[i].GetPointer(),
                                   world);
    }

    for (std::size_t i = 0; i < kViewCubeFaces.size(); ++i)
    {
      auto *actor = m_face_labels[i].GetPointer();
      if (!actor)
      {
        continue;
      }

      double world[3] = {0.0, 0.0, 0.0};
      double normal[3] = {0.0, 0.0, 0.0};
      const bool visible = face_label_position_and_normal(kViewCubeFaces[i].face, world, normal) &&
                           vtkMath::Dot(normal, direction) > kViewCubeFaceLabelVisibleDot &&
                           set_projected_label_position(m_view_cube_renderer, actor, world);
      actor->SetVisibility(visible ? 1 : 0);
    }
  }

  bool View_Cube_Controller::Handle_Click(int x, int y)
  {
    if (!m_scene_renderer || !m_render_window)
    {
      return false;
    }

    const int *size = m_render_window->GetSize();
    if (!size || size[0] <= 0 || size[1] <= 0)
    {
      return false;
    }

    const double y_from_bottom = static_cast<double>(size[1] - y);
    const double left = kViewCubeViewportLeft * static_cast<double>(size[0]);
    const double right = kViewCubeViewportRight * static_cast<double>(size[0]);
    const double bottom = kViewCubeViewportBottom * static_cast<double>(size[1]);
    const double top = kViewCubeViewportTop * static_cast<double>(size[1]);
    if (x < left || x > right || y_from_bottom < bottom || y_from_bottom > top)
    {
      return false;
    }

    if (!m_view_cube_renderer)
    {
      return true;
    }

    Update_Camera();
    const auto picked_face = pick_view_cube_face(m_view_cube_renderer,
                                                 static_cast<double>(x),
                                                 y_from_bottom);
    if (!set_camera_for_view_cube_face(m_scene_renderer, picked_face))
    {
      return true;
    }

    Update_Camera();
    return true;
  }

} // namespace robot_model

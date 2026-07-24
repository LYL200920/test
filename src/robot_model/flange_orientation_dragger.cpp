#include "flange_orientation_dragger.h"

#include <vtkRenderer.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace robot_model
{
  namespace
  {

    double wrapped_angle(double angle)
    {
      constexpr double pi = 3.14159265358979323846;
      while (angle > pi)
        angle -= 2.0 * pi;
      while (angle < -pi)
        angle += 2.0 * pi;
      return angle;
    }

    Matrix4 rotate_orientation_world(const Matrix4 &start,
                                     const Point3 &axis,
                                     double angle)
    {
      const double c = std::cos(angle);
      const double s = std::sin(angle);
      const double t = 1.0 - c;
      const double x = axis[0];
      const double y = axis[1];
      const double z = axis[2];
      const double rotation[3][3] = {
          {t * x * x + c, t * x * y - s * z, t * x * z + s * y},
          {t * x * y + s * z, t * y * y + c, t * y * z - s * x},
          {t * x * z - s * y, t * y * z + s * x, t * z * z + c}};
      Matrix4 target = start;
      for (std::size_t row = 0; row < 3; ++row)
      {
        for (std::size_t column = 0; column < 3; ++column)
        {
          target[row][column] = 0.0;
          for (std::size_t item = 0; item < 3; ++item)
          {
            target[row][column] += rotation[row][item] * start[item][column];
          }
        }
      }
      return target;
    }

    double point_segment_distance_squared(double px, double py,
                                          double ax, double ay,
                                          double bx, double by)
    {
      const double dx = bx - ax;
      const double dy = by - ay;
      const double length_squared = dx * dx + dy * dy;
      const double amount = length_squared > 1.0e-12 ? std::clamp(((px - ax) * dx + (py - ay) * dy) / length_squared, 0.0, 1.0)
                                                     : 0.0;
      const double nearest_x = ax + amount * dx;
      const double nearest_y = ay + amount * dy;
      const double error_x = px - nearest_x;
      const double error_y = py - nearest_y;
      return error_x * error_x + error_y * error_y;
    }

  } // namespace

  Flange_Orientation_Dragger::Flange_Orientation_Dragger(double ring_radius_mm)
      : m_ring_radius_mm(ring_radius_mm)
  {
  }

  void Flange_Orientation_Dragger::Attach_Renderer(vtkRenderer *renderer)
  {
    End_Drag();
    m_renderer = renderer;
  }

  void Flange_Orientation_Dragger::Detach_Renderer()
  {
    End_Drag();
    m_renderer = nullptr;
  }

  void Flange_Orientation_Dragger::Set_Enabled(bool enabled)
  {
    m_enabled = enabled;
    if (!enabled)
      End_Drag();
  }

  bool Flange_Orientation_Dragger::Begin_Drag(double display_x,
                                              double display_y,
                                              const Matrix4 &world_from_flange)
  {
    if (!m_enabled || !m_renderer)
    {
      return false;
    }
    const Point3 origin_world = {world_from_flange[0][3],
                                 world_from_flange[1][3],
                                 world_from_flange[2][3]};
    if (!Display_From_World(origin_world, &m_origin_display))
    {
      return false;
    }

    const int best_axis = Hit_Test(display_x, display_y, world_from_flange);
    if (best_axis < 0)
      return false;

    m_start_world_from_flange = world_from_flange;
    m_rotation_axis_world = {world_from_flange[0][best_axis],
                             world_from_flange[1][best_axis],
                             world_from_flange[2][best_axis]};
    const double axis_length = std::sqrt(m_rotation_axis_world[0] * m_rotation_axis_world[0] +
                                         m_rotation_axis_world[1] * m_rotation_axis_world[1] +
                                         m_rotation_axis_world[2] * m_rotation_axis_world[2]);
    if (axis_length < 1.0e-12)
      return false;
    for (double &value : m_rotation_axis_world)
      value /= axis_length;
    m_start_pointer_angle = std::atan2(display_y - m_origin_display[1], display_x - m_origin_display[0]);
    m_selected_axis = best_axis;
    m_dragging = true;
    return true;
  }

  int Flange_Orientation_Dragger::Hit_Test(double display_x,
                                           double display_y,
                                           const Matrix4 &world_from_flange) const
  {
    if (!m_enabled || !m_renderer)
      return -1;
    double best_distance_squared = std::numeric_limits<double>::infinity();
    int best_axis = -1;
    constexpr int segment_count = 72;
    constexpr double two_pi = 2.0 * 3.14159265358979323846;
    for (int axis = 0; axis < 3; ++axis)
    {
      Point3 previous_display = {};
      bool has_previous = false;
      for (int segment = 0; segment <= segment_count; ++segment)
      {
        const double angle = two_pi * segment / segment_count;
        Point3 local_point = {};
        const int first = (axis + 1) % 3;
        const int second = (axis + 2) % 3;
        local_point[first] = m_ring_radius_mm * std::cos(angle);
        local_point[second] = m_ring_radius_mm * std::sin(angle);
        Point3 current_display = {};
        if (!Display_From_World(Transform_Position(world_from_flange, local_point),
                                &current_display))
        {
          has_previous = false;
          continue;
        }
        if (has_previous)
        {
          const double distance_squared = point_segment_distance_squared(display_x, display_y,
                                                                         previous_display[0], previous_display[1],
                                                                         current_display[0], current_display[1]);
          if (distance_squared < best_distance_squared)
          {
            best_distance_squared = distance_squared;
            best_axis = axis;
          }
        }
        previous_display = current_display;
        has_previous = true;
      }
    }
    return best_distance_squared <= m_hit_radius_pixels * m_hit_radius_pixels ? best_axis
                                                                              : -1;
  }

  bool Flange_Orientation_Dragger::Update_Drag(double display_x,
                                               double display_y,
                                               Matrix4 *target_world_from_flange) const
  {
    if (!m_dragging || !target_world_from_flange)
    {
      return false;
    }
    const double current_angle = std::atan2(display_y - m_origin_display[1], display_x - m_origin_display[0]);
    *target_world_from_flange = rotate_orientation_world(m_start_world_from_flange,
                                                         m_rotation_axis_world,
                                                         wrapped_angle(current_angle - m_start_pointer_angle));
    return true;
  }

  void Flange_Orientation_Dragger::End_Drag()
  {
    m_dragging = false;
    m_selected_axis = -1;
  }

  bool Flange_Orientation_Dragger::Display_From_World(const Point3 &world,
                                                      Point3 *display) const
  {
    if (!m_renderer || !display)
    {
      return false;
    }
    m_renderer->SetWorldPoint(world[0], world[1], world[2], 1.0);
    m_renderer->WorldToDisplay();
    const double *value = m_renderer->GetDisplayPoint();
    if (!value)
    {
      return false;
    }
    *display = {value[0], value[1], value[2]};
    return std::isfinite(value[0]) && std::isfinite(value[1]);
  }

} // namespace robot_model

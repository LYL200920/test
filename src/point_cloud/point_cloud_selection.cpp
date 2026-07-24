#include "point_cloud_selection.h"

#include <algorithm>
#include <cmath>

namespace point_cloud
{
namespace
{

bool Is_Point_On_Segment(
  const Display_Point &point,
  const Display_Point &start,
  const Display_Point &end)
{
  const double segment_x = end.x - start.x;
  const double segment_y = end.y - start.y;
  const double point_x = point.x - start.x;
  const double point_y = point.y - start.y;
  const double cross = segment_x * point_y - segment_y * point_x;
  const double scale = std::max(
    1.0, std::abs(segment_x) + std::abs(segment_y));
  if (std::abs(cross) > 1e-9 * scale)
  {
    return false;
  }

  return point.x >= std::min(start.x, end.x) - 1e-9 &&
         point.x <= std::max(start.x, end.x) + 1e-9 &&
         point.y >= std::min(start.y, end.y) - 1e-9 &&
         point.y <= std::max(start.y, end.y) + 1e-9;
}

}

bool Is_Point_In_Display_Polygon(
  const Display_Point &point,
  const std::vector<Display_Point> &polygon)
{
  if (polygon.size() < 3)
  {
    return false;
  }

  double twice_area = 0.0;
  std::size_t area_previous_index = polygon.size() - 1;
  for (std::size_t index = 0; index < polygon.size(); ++index)
  {
    twice_area +=
      polygon[area_previous_index].x * polygon[index].y -
      polygon[index].x * polygon[area_previous_index].y;
    area_previous_index = index;
  }
  if (std::abs(twice_area) <= 1e-9)
  {
    return false;
  }

  bool inside = false;
  std::size_t previous_index = polygon.size() - 1;
  for (std::size_t index = 0; index < polygon.size(); ++index)
  {
    const Display_Point &start = polygon[previous_index];
    const Display_Point &end = polygon[index];
    if (Is_Point_On_Segment(point, start, end))
    {
      return true;
    }

    const bool crosses_y =
      (start.y > point.y) != (end.y > point.y);
    if (crosses_y)
    {
      const double crossing_x =
        (end.x - start.x) * (point.y - start.y) /
          (end.y - start.y) +
        start.x;
      if (point.x < crossing_x)
      {
        inside = !inside;
      }
    }
    previous_index = index;
  }
  return inside;
}

} // namespace point_cloud

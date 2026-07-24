#ifndef includeguard_point_cloud_selection_h_includeguard
#define includeguard_point_cloud_selection_h_includeguard

#include <vector>

namespace point_cloud
{

struct Display_Point
{
  double x = 0.0;
  double y = 0.0;
};

bool Is_Point_In_Display_Polygon(
  const Display_Point &point,
  const std::vector<Display_Point> &polygon);

} // namespace point_cloud

#endif

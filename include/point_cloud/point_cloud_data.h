#ifndef includeguard_point_cloud_data_h_includeguard
#define includeguard_point_cloud_data_h_includeguard

#include <cstddef>
#include <cstdint>
#include <vector>

namespace point_cloud
{

struct Point_Cloud_Data
{
  std::vector<float> xyz;
  std::vector<std::uint8_t> rgb;

  std::size_t Point_Count ( ) const { return xyz.size ( ) / 3; }
  bool Has_Color ( ) const
  {
    return !rgb.empty ( ) && rgb.size ( ) == Point_Count ( ) * 3;
  }
};

} // namespace point_cloud

#endif

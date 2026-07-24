#include "camera_point_cloud_converter.h"

#include "Mv3dRgbdDefine.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>

namespace
{
  struct Point_Cloud_Format
  {
    std::size_t record_size = 0;
    bool has_color = false;
  };

  bool Get_Point_Cloud_Format(std::uint32_t image_type, Point_Cloud_Format *format)
  {
    if (format == nullptr)
    {
      return false;
    }

    if (image_type == static_cast<std::uint32_t>(ImageType_PointCloud))
    {
      *format = {sizeof(MV3D_RGBD_POINT_3D_F32), false};
      return true;
    }
    if (image_type == static_cast<std::uint32_t>(ImageType_PointCloudWithNormals))
    {
      *format = {sizeof(MV3D_RGBD_POINT_XYZ_NORMALS), false};
      return true;
    }
    if (image_type == static_cast<std::uint32_t>(ImageType_TexturedPointCloud))
    {
      *format = {sizeof(MV3D_RGBD_POINT_XYZ_RGB), true};
      return true;
    }
    if (image_type == static_cast<std::uint32_t>(ImageType_TexturedPointCloudWithNormals))
    {
      *format = {sizeof(MV3D_RGBD_POINT_XYZ_RGB_NORMALS), true};
      return true;
    }
    return false;
  }

  const Camera_Image_Frame *Find_Point_Cloud_Image(const Camera_Frame &frame,
                                                   Point_Cloud_Format *format,
                                                   std::uint32_t required_coordinate_type)
  {
    const Camera_Image_Frame *untextured = nullptr;
    Point_Cloud_Format untextured_format;
    for (const auto &image : frame.images)
    {
      Point_Cloud_Format candidate;
      if (!Get_Point_Cloud_Format(image.image_type, &candidate))
      {
        continue;
      }
      if (required_coordinate_type != 0 && image.coordinate_type != required_coordinate_type)
      {
        continue;
      }
      if (candidate.has_color)
      {
        *format = candidate;
        return &image;
      }
      if (untextured == nullptr)
      {
        untextured = &image;
        untextured_format = candidate;
      }
    }

    if (untextured)
    {
      *format = untextured_format;
    }
    return untextured;
  }

  float Read_Float(const std::uint8_t *data)
  {
    float value = 0.0f;
    std::memcpy(&value, data, sizeof(value));
    return value;
  }

  bool Is_Valid_Point(float x, float y, float z)
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && !(x == 0.0f && y == 0.0f && z == 0.0f);
  }
} // namespace

bool Convert_Camera_Frame_To_Point_Cloud(const Camera_Frame &frame,
                                         Camera_Point_Cloud *destination,
                                         std::string *error,
                                         std::size_t maximum_points,
                                         std::uint32_t required_coordinate_type)
{
  if (error)
  {
    error->clear();
  }
  if (destination == nullptr)
  {
    if (error)
    {
      *error = "Point cloud destination is null";
    }
    return false;
  }
  *destination = {};
  if (maximum_points == 0)
  {
    if (error)
    {
      *error = "Maximum point count must be greater than zero";
    }
    return false;
  }

  Point_Cloud_Format format;
  const auto *image = Find_Point_Cloud_Image(frame, &format, required_coordinate_type);
  if (image == nullptr)
  {
    if (error)
    {
      *error = required_coordinate_type == 0 ? "Current frame does not contain point-cloud data"
                                             : "Current frame does not contain point-cloud data in the required coordinate system";
    }
    return false;
  }
  if (format.record_size < sizeof(float) * 3 || image->data.size() < format.record_size)
  {
    if (error)
    {
      *error = "Point-cloud buffer is too small";
    }
    return false;
  }

  std::size_t point_count = image->data.size() / format.record_size;
  const std::uint64_t dimension_count = static_cast<std::uint64_t>(image->width) * image->height;
  if (dimension_count > 0 && dimension_count <= std::numeric_limits<std::size_t>::max())
  {
    point_count = std::min(point_count, static_cast<std::size_t>(dimension_count));
  }
  if (point_count == 0)
  {
    if (error)
    {
      *error = "Point-cloud buffer contains no complete points";
    }
    return false;
  }

  const std::size_t sample_step = std::max<std::size_t>(1, (point_count + maximum_points - 1) / maximum_points);
  const std::size_t reserve_count = std::min(point_count, maximum_points);
  destination->xyz.reserve(reserve_count * 3);
  if (format.has_color)
  {
    destination->rgb.reserve(reserve_count * 3);
  }

  for (std::size_t i = 0; i < point_count; i += sample_step)
  {
    const auto *record = image->data.data() + i * format.record_size;
    const float x = Read_Float(record);
    const float y = Read_Float(record + sizeof(float));
    const float z = Read_Float(record + sizeof(float) * 2);
    if (!Is_Valid_Point(x, y, z))
    {
      continue;
    }

    destination->xyz.insert(destination->xyz.end(), {x, y, z});
    if (format.has_color)
    {
      const auto *rgba = record + sizeof(float) * 3;
      destination->rgb.insert(destination->rgb.end(), {rgba[0], rgba[1], rgba[2]});
    }
  }

  if (destination->xyz.empty())
  {
    *destination = {};
    if (error)
    {
      *error = "Point-cloud buffer contains no valid points";
    }
    return false;
  }

  destination->source_image_type = image->image_type;
  destination->source_frame_number = image->frame_number;
  destination->source_coordinate_type = image->coordinate_type;
  destination->source_width = image->width;
  destination->source_height = image->height;
  destination->source_point_count = point_count;
  return true;
}

bool Prepare_Camera_Point_Cloud_For_Overlay(Camera_Point_Cloud *cloud,
                                            const Camera_Point_Cloud_Filter_Options &options,
                                            Camera_Point_Cloud_Filter_Stats *stats,
                                            std::string *error)
{
  if (error)
  {
    error->clear();
  }
  if (stats)
  {
    *stats = {};
  }
  if (cloud == nullptr || cloud->xyz.empty() || (cloud->xyz.size() % 3) != 0)
  {
    if (error)
      *error = "Point cloud is empty or malformed";
    return false;
  }
  const std::size_t input_count = cloud->Point_Count();
  if (!cloud->rgb.empty() && cloud->rgb.size() != input_count * 3)
  {
    if (error)
      *error = "Point-cloud color array size does not match coordinates";
    return false;
  }
  if (!std::isfinite(options.coordinate_to_mm) ||
      options.coordinate_to_mm <= 0.0 ||
      options.minimum_depth_mm < 0.0 ||
      options.maximum_depth_mm <= options.minimum_depth_mm ||
      options.maximum_absolute_coordinate_mm <= 0.0 ||
      options.maximum_points == 0)
  {
    if (error)
      *error = "Point-cloud overlay filter options are invalid";
    return false;
  }

  std::vector<std::size_t> valid_indices;
  std::vector<double> depths;
  std::vector<double> raw_positive_depths;
  valid_indices.reserve(input_count);
  depths.reserve(input_count);
  raw_positive_depths.reserve(input_count);
  std::array<double, 6> bounds = {std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity()};
  for (std::size_t i = 0; i < input_count; ++i)
  {
    const double raw_z = cloud->xyz[i * 3 + 2];
    if (std::isfinite(raw_z) && raw_z > 0.0)
    {
      raw_positive_depths.push_back(raw_z);
    }
    const double x = cloud->xyz[i * 3] * options.coordinate_to_mm;
    const double y = cloud->xyz[i * 3 + 1] * options.coordinate_to_mm;
    const double z = cloud->xyz[i * 3 + 2] * options.coordinate_to_mm;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        z < options.minimum_depth_mm || z > options.maximum_depth_mm ||
        std::abs(x) > options.maximum_absolute_coordinate_mm ||
        std::abs(y) > options.maximum_absolute_coordinate_mm ||
        std::abs(z) > options.maximum_absolute_coordinate_mm)
    {
      continue;
    }
    valid_indices.push_back(i);
    depths.push_back(z);
    bounds[0] = std::min(bounds[0], x);
    bounds[1] = std::max(bounds[1], x);
    bounds[2] = std::min(bounds[2], y);
    bounds[3] = std::max(bounds[3], y);
    bounds[4] = std::min(bounds[4], z);
    bounds[5] = std::max(bounds[5], z);
  }
  double raw_depth_min = 0.0;
  double raw_depth_median = 0.0;
  double raw_depth_max = 0.0;
  if (!raw_positive_depths.empty())
  {
    const auto raw_min_max = std::minmax_element(raw_positive_depths.begin(), raw_positive_depths.end());
    raw_depth_min = *raw_min_max.first;
    raw_depth_max = *raw_min_max.second;
    const auto raw_middle = raw_positive_depths.begin() + raw_positive_depths.size() / 2;
    std::nth_element(raw_positive_depths.begin(), raw_middle,
                     raw_positive_depths.end());
    raw_depth_median = *raw_middle;
  }
  if (stats)
  {
    stats->input_points = input_count;
    stats->valid_points = valid_indices.size();
    stats->rejected_points = input_count - valid_indices.size();
    stats->raw_positive_depth_min = raw_depth_min;
    stats->raw_positive_depth_median = raw_depth_median;
    stats->raw_positive_depth_max = raw_depth_max;
  }
  if (valid_indices.empty())
  {
    if (error)
    {
      std::ostringstream message;
      message << std::setprecision(6)
              << "No usable Depth points remain after filtering. Raw positive Z[min/median/max]=["
              << raw_depth_min << '/' << raw_depth_median << '/'
              << raw_depth_max << "], PointCloudUnitToMm="
              << options.coordinate_to_mm
              << "; check point-cloud units and ImageAlign";
      *error = message.str();
    }
    return false;
  }

  const auto middle = depths.begin() + depths.size() / 2;
  std::nth_element(depths.begin(), middle, depths.end());
  const double median_depth = *middle;
  const std::size_t output_count = std::min(valid_indices.size(), options.maximum_points);
  std::vector<float> filtered_xyz;
  std::vector<std::uint8_t> filtered_rgb;
  filtered_xyz.reserve(output_count * 3);
  if (cloud->Has_Color())
    filtered_rgb.reserve(output_count * 3);
  for (std::size_t output_index = 0; output_index < output_count; ++output_index)
  {
    const std::size_t valid_position = output_index * valid_indices.size() / output_count;
    const std::size_t source_index = valid_indices[valid_position];
    filtered_xyz.push_back(static_cast<float>(cloud->xyz[source_index * 3] * options.coordinate_to_mm));
    filtered_xyz.push_back(static_cast<float>(cloud->xyz[source_index * 3 + 1] * options.coordinate_to_mm));
    filtered_xyz.push_back(static_cast<float>(cloud->xyz[source_index * 3 + 2] * options.coordinate_to_mm));
    if (cloud->Has_Color())
    {
      filtered_rgb.insert(filtered_rgb.end(), {cloud->rgb[source_index * 3], cloud->rgb[source_index * 3 + 1],
                                               cloud->rgb[source_index * 3 + 2]});
    }
  }
  cloud->xyz = std::move(filtered_xyz);
  cloud->rgb = std::move(filtered_rgb);
  if (stats)
  {
    stats->output_points = output_count;
    stats->median_depth_mm = median_depth;
    stats->camera_bounds_mm = bounds;
  }
  return true;
}

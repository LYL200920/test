#include "camera_point_cloud_converter.h"

#include "Mv3dRgbdDefine.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace
{
struct Point_Cloud_Format
{
  std::size_t record_size = 0;
  bool has_color = false;
};

bool Get_Point_Cloud_Format (
  std::uint32_t image_type,
  Point_Cloud_Format* format)
{
  if( format == nullptr )
  {
    return false;
  }

  if( image_type == static_cast<std::uint32_t> (ImageType_PointCloud) )
  {
    *format = { sizeof (MV3D_RGBD_POINT_3D_F32), false };
    return true;
  }
  if( image_type ==
      static_cast<std::uint32_t> (ImageType_PointCloudWithNormals) )
  {
    *format = { sizeof (MV3D_RGBD_POINT_XYZ_NORMALS), false };
    return true;
  }
  if( image_type ==
      static_cast<std::uint32_t> (ImageType_TexturedPointCloud) )
  {
    *format = { sizeof (MV3D_RGBD_POINT_XYZ_RGB), true };
    return true;
  }
  if( image_type == static_cast<std::uint32_t> (
        ImageType_TexturedPointCloudWithNormals) )
  {
    *format = { sizeof (MV3D_RGBD_POINT_XYZ_RGB_NORMALS), true };
    return true;
  }
  return false;
}

const Camera_Image_Frame* Find_Point_Cloud_Image (
  const Camera_Frame& frame,
  Point_Cloud_Format* format)
{
  const Camera_Image_Frame* untextured = nullptr;
  Point_Cloud_Format untextured_format;
  for( const auto& image : frame.images )
  {
    Point_Cloud_Format candidate;
    if( !Get_Point_Cloud_Format (image.image_type, &candidate) )
    {
      continue;
    }
    if( candidate.has_color )
    {
      *format = candidate;
      return &image;
    }
    if( untextured == nullptr )
    {
      untextured = &image;
      untextured_format = candidate;
    }
  }

  if( untextured )
  {
    *format = untextured_format;
  }
  return untextured;
}

float Read_Float (const std::uint8_t* data)
{
  float value = 0.0f;
  std::memcpy (&value, data, sizeof (value));
  return value;
}

bool Is_Valid_Point (float x, float y, float z)
{
  return std::isfinite (x) && std::isfinite (y) && std::isfinite (z) &&
         !( x == 0.0f && y == 0.0f && z == 0.0f );
}
} // namespace

bool Convert_Camera_Frame_To_Point_Cloud (
  const Camera_Frame& frame,
  Camera_Point_Cloud* destination,
  std::string* error,
  std::size_t maximum_points)
{
  if( error )
  {
    error->clear ( );
  }
  if( destination == nullptr )
  {
    if( error )
    {
      *error = "Point cloud destination is null";
    }
    return false;
  }
  *destination = {};
  if( maximum_points == 0 )
  {
    if( error )
    {
      *error = "Maximum point count must be greater than zero";
    }
    return false;
  }

  Point_Cloud_Format format;
  const auto* image = Find_Point_Cloud_Image (frame, &format);
  if( image == nullptr )
  {
    if( error )
    {
      *error = "Current frame does not contain point-cloud data";
    }
    return false;
  }
  if( format.record_size < sizeof (float) * 3 ||
      image->data.size ( ) < format.record_size )
  {
    if( error )
    {
      *error = "Point-cloud buffer is too small";
    }
    return false;
  }

  std::size_t point_count = image->data.size ( ) / format.record_size;
  const std::uint64_t dimension_count =
    static_cast<std::uint64_t> (image->width) * image->height;
  if( dimension_count > 0 &&
      dimension_count <= std::numeric_limits<std::size_t>::max ( ) )
  {
    point_count = std::min (
      point_count, static_cast<std::size_t> (dimension_count));
  }
  if( point_count == 0 )
  {
    if( error )
    {
      *error = "Point-cloud buffer contains no complete points";
    }
    return false;
  }

  const std::size_t sample_step = std::max<std::size_t> (
    1, ( point_count + maximum_points - 1 ) / maximum_points);
  const std::size_t reserve_count =
    std::min (point_count, maximum_points);
  destination->xyz.reserve (reserve_count * 3);
  if( format.has_color )
  {
    destination->rgb.reserve (reserve_count * 3);
  }

  for( std::size_t i = 0; i < point_count; i += sample_step )
  {
    const auto* record = image->data.data ( ) + i * format.record_size;
    const float x = Read_Float (record);
    const float y = Read_Float (record + sizeof (float));
    const float z = Read_Float (record + sizeof (float) * 2);
    if( !Is_Valid_Point (x, y, z) )
    {
      continue;
    }

    destination->xyz.insert (destination->xyz.end ( ), { x, y, z });
    if( format.has_color )
    {
      const auto* rgba = record + sizeof (float) * 3;
      destination->rgb.insert (
        destination->rgb.end ( ), { rgba[0], rgba[1], rgba[2] });
    }
  }

  if( destination->xyz.empty ( ) )
  {
    *destination = {};
    if( error )
    {
      *error = "Point-cloud buffer contains no valid points";
    }
    return false;
  }

  destination->source_image_type = image->image_type;
  destination->source_frame_number = image->frame_number;
  destination->source_width = image->width;
  destination->source_height = image->height;
  destination->source_point_count = point_count;
  return true;
}

#include "camera_stream_config.h"

#include <algorithm>

bool Build_Camera_Stream_Configuration(
    const MV3D_RGBD_STREAM_CFG_LIST &source,
    std::vector<Camera_Stream_Config> *destination)
{
  if (destination == nullptr)
  {
    return false;
  }

  destination->clear();
  const auto count = std::min<std::size_t>(source.nStreamCfgCount, MV3D_RGBD_MAX_IMAGE_COUNT);
  destination->reserve(count);
  for (std::size_t index = 0; index < count; ++index)
  {
    const auto &stream = source.stStreamCfg[index];
    destination->push_back({static_cast<std::uint32_t>(stream.enImageType),
                            stream.nWidth,
                            stream.nHeight});
  }
  return true;
}

std::string Camera_Image_Type_Name(std::uint32_t image_type)
{
  switch (image_type)
  {
  case static_cast<std::uint32_t>(ImageType_Mono8):
    return "Mono8";
  case static_cast<std::uint32_t>(ImageType_Mono16):
    return "Mono16";
  case static_cast<std::uint32_t>(ImageType_Depth):
    return "Depth";
  case static_cast<std::uint32_t>(ImageType_YUV422):
    return "YUV422";
  case static_cast<std::uint32_t>(ImageType_YUV420SP_NV12):
    return "NV12";
  case static_cast<std::uint32_t>(ImageType_YUV420SP_NV21):
    return "NV21";
  case static_cast<std::uint32_t>(ImageType_RGB8_Planar):
    return "RGB8 Planar";
  case static_cast<std::uint32_t>(ImageType_PointCloud):
    return "Point Cloud";
  case static_cast<std::uint32_t>(ImageType_PointCloudWithNormals):
    return "Point Cloud + Normals";
  case static_cast<std::uint32_t>(ImageType_TexturedPointCloud):
    return "Textured Point Cloud";
  case static_cast<std::uint32_t>(ImageType_TexturedPointCloudWithNormals):
    return "Textured Point Cloud + Normals";
  case static_cast<std::uint32_t>(ImageType_Jpeg):
    return "JPEG";
  case static_cast<std::uint32_t>(ImageType_Rgbd):
    return "RGBD";
  default:
    return "Unknown";
  }
}

std::string Camera_Stream_Type_Name(std::uint32_t stream_type)
{
  switch (stream_type)
  {
  case StreamType_Depth:
    return "Depth";
  case StreamType_Color:
    return "Color";
  case StreamType_Ir_Left:
    return "IR Left";
  case StreamType_Ir_Right:
    return "IR Right";
  case StreamType_Imu:
    return "IMU";
  case StreamType_LeftMono:
    return "Left Mono";
  case StreamType_Mask:
    return "Mask";
  case StreamType_Mono:
    return "Mono";
  case StreamType_Phase:
    return "Phase";
  case StreamType_Rgbd:
    return "RGBD";
  default:
    return "Undefined";
  }
}

std::string Camera_Coordinate_Type_Name(std::uint32_t coordinate_type)
{
  switch (coordinate_type)
  {
  case CoordinateType_Depth:
    return "Depth";
  case CoordinateType_RGB:
    return "RGB";
  default:
    return "Undefined";
  }
}

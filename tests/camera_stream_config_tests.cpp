#include "camera_stream_config.h"

#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
void Require (bool condition, const char* message)
{
  if( !condition )
  {
    throw std::runtime_error (message);
  }
}
} // namespace

int main ( )
{
  try
  {
    MV3D_RGBD_STREAM_CFG_LIST source{};
    source.nStreamCfgCount = 2;
    source.stStreamCfg[0].enImageType = ImageType_Depth;
    source.stStreamCfg[0].nWidth = 640;
    source.stStreamCfg[0].nHeight = 480;
    source.stStreamCfg[1].enImageType = ImageType_PointCloud;
    source.stStreamCfg[1].nWidth = 320;
    source.stStreamCfg[1].nHeight = 240;

    std::vector<Camera_Stream_Config> streams;
    Require (
      Build_Camera_Stream_Configuration (source, &streams),
      "Stream configuration conversion failed");
    Require (streams.size ( ) == 2, "Stream count mismatch");
    Require (
      streams[0].image_type ==
        static_cast<std::uint32_t> (ImageType_Depth),
      "Depth image type mismatch");
    Require (
      streams[0].width == 640 && streams[0].height == 480,
      "Depth dimensions mismatch");
    Require (
      Camera_Image_Type_Name (streams[1].image_type) == "Point Cloud",
      "Point cloud type label mismatch");
    Require (
      Camera_Stream_Type_Name (StreamType_Color) == "Color",
      "Stream type label mismatch");
    Require (
      Camera_Coordinate_Type_Name (CoordinateType_RGB) == "RGB",
      "Coordinate type label mismatch");
  }
  catch( const std::exception& error )
  {
    std::cerr << "FAILED: " << error.what ( ) << '\n';
    return 1;
  }

  std::cout << "Camera stream configuration tests passed.\n";
  return 0;
}

#ifndef includeguard_camera_stream_config_h_includeguard
#define includeguard_camera_stream_config_h_includeguard

#include "Mv3dRgbdDefine.h"

#include <cstdint>
#include <string>
#include <vector>

struct Camera_Stream_Config
{
  std::uint32_t image_type = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
};

bool Build_Camera_Stream_Configuration(const MV3D_RGBD_STREAM_CFG_LIST &source,
                                       std::vector<Camera_Stream_Config> *destination);

std::string Camera_Image_Type_Name(std::uint32_t image_type);
std::string Camera_Stream_Type_Name(std::uint32_t stream_type);
std::string Camera_Coordinate_Type_Name(std::uint32_t coordinate_type);

#endif

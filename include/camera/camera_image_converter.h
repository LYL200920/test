#ifndef includeguard_camera_image_converter_h_includeguard
#define includeguard_camera_image_converter_h_includeguard

#include "camera_frame.h"

#include <cstdint>
#include <string>
#include <vector>

enum class Camera_Image_Display_Mode
{
  Automatic,
  Color,
  Depth
};

struct Camera_Display_Image
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t source_image_type = 0;
  std::uint32_t source_frame_number = 0;
  std::vector<std::uint8_t> rgb;
};

bool Convert_Camera_Frame_To_Display_Image (
  const Camera_Frame& frame,
  Camera_Image_Display_Mode mode,
  Camera_Display_Image* destination,
  std::string* error = nullptr);

#endif

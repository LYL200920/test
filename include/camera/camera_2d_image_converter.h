#ifndef includeguard_camera_2d_image_converter_h_includeguard
#define includeguard_camera_2d_image_converter_h_includeguard

#include <cstdint>
#include <string>
#include <vector>

namespace jutze_camera
{
struct camera_frame;
}

struct Camera_2D_Display_Image
{
  unsigned int width = 0;
  unsigned int height = 0;
  unsigned long long frame_number = 0;
  std::vector<std::uint8_t> rgb;
};

bool Convert_Camera_2D_Frame(
  const jutze_camera::camera_frame &frame,
  Camera_2D_Display_Image *destination,
  std::string *error_message = nullptr);

#endif

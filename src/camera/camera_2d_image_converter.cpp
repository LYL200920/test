#include "camera_2d_image_converter.h"

#include "camera_params.h"

#include <algorithm>
#include <cstddef>

namespace
{
using jutze_camera::pixel_format;

enum class Bayer_Color
{
  Red,
  Green,
  Blue
};

bool Is_Bayer(pixel_format format)
{
  switch (format)
  {
  case pixel_format::bayer_gr8:
  case pixel_format::bayer_rg8:
  case pixel_format::bayer_gb8:
  case pixel_format::bayer_bg8:
  case pixel_format::bayer_gr10:
  case pixel_format::bayer_rg10:
  case pixel_format::bayer_gb10:
  case pixel_format::bayer_bg10:
  case pixel_format::bayer_gr12:
  case pixel_format::bayer_rg12:
  case pixel_format::bayer_gb12:
  case pixel_format::bayer_bg12:
  case pixel_format::bayer_gr16:
  case pixel_format::bayer_rg16:
  case pixel_format::bayer_gb16:
  case pixel_format::bayer_bg16:
    return true;
  default:
    return false;
  }
}

Bayer_Color Bayer_At(
  pixel_format format,
  unsigned int x,
  unsigned int y)
{
  const bool even_x = (x % 2) == 0;
  const bool even_y = (y % 2) == 0;
  const unsigned int cell = (even_y ? 0U : 2U) + (even_x ? 0U : 1U);
  const char *pattern = "RGGB";
  switch (format)
  {
  case pixel_format::bayer_gr8:
  case pixel_format::bayer_gr10:
  case pixel_format::bayer_gr12:
  case pixel_format::bayer_gr16:
    pattern = "GRBG";
    break;
  case pixel_format::bayer_gb8:
  case pixel_format::bayer_gb10:
  case pixel_format::bayer_gb12:
  case pixel_format::bayer_gb16:
    pattern = "GBRG";
    break;
  case pixel_format::bayer_bg8:
  case pixel_format::bayer_bg10:
  case pixel_format::bayer_bg12:
  case pixel_format::bayer_bg16:
    pattern = "BGGR";
    break;
  default:
    break;
  }
  return pattern[cell] == 'R'
    ? Bayer_Color::Red
    : (pattern[cell] == 'B' ? Bayer_Color::Blue : Bayer_Color::Green);
}

std::uint8_t Read_Sample(
  const jutze_camera::camera_frame &frame,
  std::size_t index,
  int bit_depth)
{
  const auto *data = frame.m_data.get();
  if (bit_depth <= 8)
  {
    return data[index];
  }
  const std::size_t byte_index = index * 2;
  const auto value = static_cast<unsigned int>(data[byte_index]) |
    (static_cast<unsigned int>(data[byte_index + 1]) << 8);
  const unsigned int maximum = (1U << std::min(bit_depth, 16)) - 1U;
  return static_cast<std::uint8_t>(
    (static_cast<unsigned long long>(value) * 255ULL) / maximum);
}

int Effective_Bit_Depth(pixel_format format)
{
  switch (format)
  {
  case pixel_format::mono10:
  case pixel_format::bayer_gr10:
  case pixel_format::bayer_rg10:
  case pixel_format::bayer_gb10:
  case pixel_format::bayer_bg10:
    return 10;
  case pixel_format::mono12:
  case pixel_format::bayer_gr12:
  case pixel_format::bayer_rg12:
  case pixel_format::bayer_gb12:
  case pixel_format::bayer_bg12:
    return 12;
  case pixel_format::mono16:
  case pixel_format::bayer_gr16:
  case pixel_format::bayer_rg16:
  case pixel_format::bayer_gb16:
  case pixel_format::bayer_bg16:
    return 16;
  default:
    return 8;
  }
}

void Demosaic_Bayer_Pixel(
  const jutze_camera::camera_frame &frame,
  pixel_format format,
  int bit_depth,
  unsigned int x,
  unsigned int y,
  std::uint8_t *rgb)
{
  unsigned int sums[3] = {0, 0, 0};
  unsigned int counts[3] = {0, 0, 0};
  for (int dy = -1; dy <= 1; ++dy)
  {
    const int sample_y = static_cast<int>(y) + dy;
    if (sample_y < 0 || sample_y >= static_cast<int>(frame.m_height))
    {
      continue;
    }
    for (int dx = -1; dx <= 1; ++dx)
    {
      const int sample_x = static_cast<int>(x) + dx;
      if (sample_x < 0 || sample_x >= static_cast<int>(frame.m_width))
      {
        continue;
      }
      const auto color = Bayer_At(
        format,
        static_cast<unsigned int>(sample_x),
        static_cast<unsigned int>(sample_y));
      const int channel = color == Bayer_Color::Red
        ? 0
        : (color == Bayer_Color::Green ? 1 : 2);
      const auto index =
        static_cast<std::size_t>(sample_y) * frame.m_width +
        static_cast<std::size_t>(sample_x);
      sums[channel] += Read_Sample(frame, index, bit_depth);
      ++counts[channel];
    }
  }
  for (int channel = 0; channel < 3; ++channel)
  {
    rgb[channel] = counts[channel] == 0
      ? 0
      : static_cast<std::uint8_t>(sums[channel] / counts[channel]);
  }
}

bool Fail(const std::string &message, std::string *error_message)
{
  if (error_message)
  {
    *error_message = message;
  }
  return false;
}
}

bool Convert_Camera_2D_Frame(
  const jutze_camera::camera_frame &frame,
  Camera_2D_Display_Image *destination,
  std::string *error_message)
{
  if (!destination || !frame.m_data || frame.m_width == 0 ||
      frame.m_height == 0)
  {
    return Fail("2D相机图像数据无效", error_message);
  }

  const auto format = static_cast<pixel_format>(frame.m_pixel_type);
  const auto pixels =
    static_cast<std::size_t>(frame.m_width) * frame.m_height;
  const int depth = Effective_Bit_Depth(format);
  const std::size_t sample_bytes = depth <= 8 ? 1U : 2U;
  const bool bayer = Is_Bayer(format);
  const bool mono = !bayer && (
    format == pixel_format::mono8 ||
    format == pixel_format::mono10 ||
    format == pixel_format::mono12 ||
    format == pixel_format::mono16);

  std::size_t required_bytes = 0;
  if (bayer || mono)
  {
    required_bytes = pixels * sample_bytes;
  }
  else if (format == pixel_format::rgb8 ||
           format == pixel_format::bgr8)
  {
    required_bytes = pixels * 3;
  }
  else if (format == pixel_format::rgba8 ||
           format == pixel_format::bgra8)
  {
    required_bytes = pixels * 4;
  }
  else
  {
    return Fail(
      std::string("暂不支持像素格式：") +
        jutze_camera::pixel_format_name(format),
      error_message);
  }
  if (frame.m_data_size < required_bytes)
  {
    return Fail("2D相机图像缓冲区长度不足", error_message);
  }

  Camera_2D_Display_Image result;
  result.width = frame.m_width;
  result.height = frame.m_height;
  result.frame_number = frame.m_frame_num;
  result.rgb.resize(pixels * 3);

  if (mono)
  {
    for (std::size_t index = 0; index < pixels; ++index)
    {
      const auto value = Read_Sample(frame, index, depth);
      result.rgb[index * 3] = value;
      result.rgb[index * 3 + 1] = value;
      result.rgb[index * 3 + 2] = value;
    }
  }
  else if (bayer)
  {
    for (unsigned int y = 0; y < frame.m_height; ++y)
    {
      for (unsigned int x = 0; x < frame.m_width; ++x)
      {
        const auto index =
          static_cast<std::size_t>(y) * frame.m_width + x;
        Demosaic_Bayer_Pixel(
          frame, format, depth, x, y, result.rgb.data() + index * 3);
      }
    }
  }
  else
  {
    const auto *source = frame.m_data.get();
    const bool bgr =
      format == pixel_format::bgr8 || format == pixel_format::bgra8;
    const std::size_t channels =
      (format == pixel_format::rgba8 || format == pixel_format::bgra8)
        ? 4U
        : 3U;
    for (std::size_t index = 0; index < pixels; ++index)
    {
      result.rgb[index * 3] =
        source[index * channels + (bgr ? 2U : 0U)];
      result.rgb[index * 3 + 1] = source[index * channels + 1U];
      result.rgb[index * 3 + 2] =
        source[index * channels + (bgr ? 0U : 2U)];
    }
  }

  *destination = std::move(result);
  if (error_message)
  {
    error_message->clear();
  }
  return true;
}

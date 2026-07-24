#include "camera_image_converter.h"

#include "camera_stream_config.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace
{
  bool Return_Error(const std::string &message, std::string *error)
  {
    if (error)
    {
      *error = message;
    }
    return false;
  }

  bool Pixel_Count(const Camera_Image_Frame &image, std::size_t *pixel_count, std::string *error)
  {
    if (image.width == 0 || image.height == 0)
    {
      return Return_Error("Image dimensions are zero", error);
    }
    const auto width = static_cast<std::size_t>(image.width);
    const auto height = static_cast<std::size_t>(image.height);
    if (width > std::numeric_limits<std::size_t>::max() / height)
    {
      return Return_Error("Image dimensions overflow", error);
    }
    *pixel_count = width * height;
    if (*pixel_count > std::numeric_limits<std::size_t>::max() / 3)
    {
      return Return_Error("RGB image size overflow", error);
    }
    return true;
  }

  std::uint8_t Clamp_Byte(float value)
  {
    return static_cast<std::uint8_t>(std::clamp(std::lround(value), 0L, 255L));
  }

  void Yuv_To_Rgb(std::uint8_t y,
                  std::uint8_t u,
                  std::uint8_t v,
                  std::uint8_t *rgb)
  {
    const float yf = static_cast<float>(y);
    const float uf = static_cast<float>(u) - 128.0F;
    const float vf = static_cast<float>(v) - 128.0F;
    rgb[0] = Clamp_Byte(yf + 1.370705F * vf);
    rgb[1] = Clamp_Byte(yf - 0.337633F * uf - 0.698001F * vf);
    rgb[2] = Clamp_Byte(yf + 1.732446F * uf);
  }

  void Depth_Color(float value, std::uint8_t *rgb)
  {
    const float normalized = std::clamp(value, 0.0F, 1.0F);
    const float scaled = normalized * 4.0F;
    rgb[0] = Clamp_Byte(255.0F * std::clamp(scaled - 1.5F, 0.0F, 1.0F));
    rgb[1] = Clamp_Byte(255.0F * std::clamp(1.5F - std::fabs(scaled - 2.0F), 0.0F, 1.0F));
    rgb[2] = Clamp_Byte(255.0F * std::clamp(1.5F - scaled, 0.0F, 1.0F));
  }

  bool Is_Color_Type(std::uint32_t image_type)
  {
    return image_type == static_cast<std::uint32_t>(ImageType_RGB8_Planar) ||
           image_type == static_cast<std::uint32_t>(ImageType_YUV422) ||
           image_type == static_cast<std::uint32_t>(ImageType_YUV420SP_NV12) ||
           image_type == static_cast<std::uint32_t>(ImageType_YUV420SP_NV21);
  }

  bool Is_Depth_Type(std::uint32_t image_type)
  {
    return image_type == static_cast<std::uint32_t>(ImageType_Depth) ||
           image_type == static_cast<std::uint32_t>(ImageType_Mono16) ||
           image_type == static_cast<std::uint32_t>(ImageType_Mono8);
  }

  const Camera_Image_Frame *Select_Image(const Camera_Frame &frame,
                                         Camera_Image_Display_Mode mode)
  {
    const auto find_type = [&frame](bool (*predicate)(std::uint32_t))
    {
      const auto found = std::find_if(frame.images.begin(), frame.images.end(),
                                      [predicate](const Camera_Image_Frame &image)
                                      {
                                        return predicate(image.image_type);
                                      });
      return found == frame.images.end() ? nullptr : &*found;
    };

    if (mode == Camera_Image_Display_Mode::Color)
    {
      return find_type(Is_Color_Type);
    }
    if (mode == Camera_Image_Display_Mode::Depth)
    {
      return find_type(Is_Depth_Type);
    }
    if (const auto *color = find_type(Is_Color_Type))
    {
      return color;
    }
    return find_type(Is_Depth_Type);
  }

  bool Convert_Rgb_Planar(const Camera_Image_Frame &image,
                          std::size_t pixel_count,
                          Camera_Display_Image *destination,
                          std::string *error)
  {
    if (image.data.size() < pixel_count * 3)
    {
      return Return_Error("RGB8 Planar buffer is too small", error);
    }
    destination->rgb.resize(pixel_count * 3);
    for (std::size_t pixel = 0; pixel < pixel_count; ++pixel)
    {
      destination->rgb[pixel * 3] = image.data[pixel];
      destination->rgb[pixel * 3 + 1] = image.data[pixel_count + pixel];
      destination->rgb[pixel * 3 + 2] = image.data[pixel_count * 2 + pixel];
    }
    return true;
  }

  bool Convert_Yuyv(const Camera_Image_Frame &image,
                    std::size_t pixel_count,
                    Camera_Display_Image *destination,
                    std::string *error)
  {
    if (image.width % 2 != 0 || image.data.size() < pixel_count * 2)
    {
      return Return_Error("YUV422 buffer or width is invalid", error);
    }
    destination->rgb.resize(pixel_count * 3);
    for (std::size_t pixel = 0; pixel < pixel_count; pixel += 2)
    {
      const auto source = pixel * 2;
      const auto y0 = image.data[source];
      const auto u = image.data[source + 1];
      const auto y1 = image.data[source + 2];
      const auto v = image.data[source + 3];
      Yuv_To_Rgb(y0, u, v, destination->rgb.data() + pixel * 3);
      Yuv_To_Rgb(y1, u, v, destination->rgb.data() + (pixel + 1) * 3);
    }
    return true;
  }

  bool Convert_Nv(const Camera_Image_Frame &image,
                  std::size_t pixel_count,
                  bool nv21,
                  Camera_Display_Image *destination,
                  std::string *error)
  {
    if (image.width % 2 != 0 || image.height % 2 != 0 ||
        image.data.size() < pixel_count + pixel_count / 2)
    {
      return Return_Error("NV12/NV21 buffer or dimensions are invalid", error);
    }
    destination->rgb.resize(pixel_count * 3);
    const auto width = static_cast<std::size_t>(image.width);
    for (std::size_t y = 0; y < image.height; ++y)
    {
      for (std::size_t x = 0; x < width; ++x)
      {
        const auto pixel = y * width + x;
        const auto chroma = pixel_count + (y / 2) * width + (x / 2) * 2;
        const auto first = image.data[chroma];
        const auto second = image.data[chroma + 1];
        const auto u = nv21 ? second : first;
        const auto v = nv21 ? first : second;
        Yuv_To_Rgb(
            image.data[pixel], u, v, destination->rgb.data() + pixel * 3);
      }
    }
    return true;
  }

  bool Convert_Mono8(const Camera_Image_Frame &image,
                     std::size_t pixel_count,
                     Camera_Display_Image *destination,
                     std::string *error)
  {
    if (image.data.size() < pixel_count)
    {
      return Return_Error("Mono8 buffer is too small", error);
    }
    destination->rgb.resize(pixel_count * 3);
    for (std::size_t pixel = 0; pixel < pixel_count; ++pixel)
    {
      const auto value = image.data[pixel];
      destination->rgb[pixel * 3] = value;
      destination->rgb[pixel * 3 + 1] = value;
      destination->rgb[pixel * 3 + 2] = value;
    }
    return true;
  }

  bool Convert_Depth16(const Camera_Image_Frame &image,
                       std::size_t pixel_count,
                       Camera_Display_Image *destination,
                       std::string *error)
  {
    if (image.data.size() < pixel_count * 2)
    {
      return Return_Error("16-bit depth buffer is too small", error);
    }

    std::uint16_t minimum = std::numeric_limits<std::uint16_t>::max();
    std::uint16_t maximum = 0;
    for (std::size_t pixel = 0; pixel < pixel_count; ++pixel)
    {
      const auto value = static_cast<std::uint16_t>(image.data[pixel * 2] |
                                                    (static_cast<std::uint16_t>(image.data[pixel * 2 + 1]) << 8));
      if (value == 0)
        continue;
      minimum = std::min(minimum, value);
      maximum = std::max(maximum, value);
    }

    destination->rgb.assign(pixel_count * 3, 0);
    if (maximum == 0)
    {
      return true;
    }
    const float range = maximum > minimum ? static_cast<float>(maximum - minimum)
                                          : 1.0F;
    for (std::size_t pixel = 0; pixel < pixel_count; ++pixel)
    {
      const auto value = static_cast<std::uint16_t>(image.data[pixel * 2] |
                                                    (static_cast<std::uint16_t>(image.data[pixel * 2 + 1]) << 8));
      if (value == 0)
        continue;
      const float normalized = static_cast<float>(value - minimum) / range;
      Depth_Color(normalized, destination->rgb.data() + pixel * 3);
    }
    return true;
  }
} // namespace

bool Convert_Camera_Frame_To_Display_Image(const Camera_Frame &frame,
                                           Camera_Image_Display_Mode mode,
                                           Camera_Display_Image *destination,
                                           std::string *error)
{
  if (error)
  {
    error->clear();
  }
  if (destination == nullptr)
  {
    return Return_Error("Display image destination is null", error);
  }
  *destination = {};

  const auto *image = Select_Image(frame, mode);
  if (image == nullptr)
  {
    return Return_Error("Requested color or depth stream is unavailable", error);
  }

  std::size_t pixel_count = 0;
  if (!Pixel_Count(*image, &pixel_count, error))
  {
    return false;
  }

  destination->width = image->width;
  destination->height = image->height;
  destination->source_image_type = image->image_type;
  destination->source_frame_number = image->frame_number;

  if (image->image_type == static_cast<std::uint32_t>(ImageType_RGB8_Planar))
    return Convert_Rgb_Planar(*image, pixel_count, destination, error);
  if (image->image_type == static_cast<std::uint32_t>(ImageType_YUV422))
    return Convert_Yuyv(*image, pixel_count, destination, error);
  if (image->image_type == static_cast<std::uint32_t>(ImageType_YUV420SP_NV12))
    return Convert_Nv(*image, pixel_count, false, destination, error);
  if (image->image_type == static_cast<std::uint32_t>(ImageType_YUV420SP_NV21))
    return Convert_Nv(*image, pixel_count, true, destination, error);
  if (image->image_type == static_cast<std::uint32_t>(ImageType_Mono8))
    return Convert_Mono8(*image, pixel_count, destination, error);
  if (image->image_type == static_cast<std::uint32_t>(ImageType_Depth) ||
      image->image_type == static_cast<std::uint32_t>(ImageType_Mono16))
    return Convert_Depth16(*image, pixel_count, destination, error);

  return Return_Error("Image type is not supported by the preview converter", error);
}

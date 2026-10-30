#include "run_image_processing.h"
#include "camera_params.h"
#include "pose_transform.h"

#include <wx/image.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace application
{
namespace
{
constexpr int RUN_MOSAIC_MAX_DIMENSION = 6000;
constexpr std::size_t RUN_MOSAIC_MAX_PIXELS = 20000000;

struct Mosaic_Placed_Image
{
  Camera_2D_Display_Image image;
  double center_u = 0.0;
  double center_v = 0.0;
  double horizontal_u = 0.0;
  double horizontal_v = 0.0;
  double vertical_u = 0.0;
  double vertical_v = 0.0;
};

double dot3(
  const std::array<double, 3>& lhs,
  const std::array<double, 3>& rhs)
{
  return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
}

double norm3(const std::array<double, 3>& value)
{
  return std::sqrt(dot3(value, value));
}

std::array<double, 3> normalized3(
  const std::array<double, 3>& value)
{
  const double length = norm3(value);
  if( length <= 1.0e-12 ) return {0.0, 0.0, 0.0};
  return {
    value[0] / length,
    value[1] / length,
    value[2] / length};
}

std::array<double, 3> matrix_axis(
  const robot_model::Matrix4& matrix,
  std::size_t axis,
  double length)
{
  return {
    matrix[0][axis] * length,
    matrix[1][axis] * length,
    matrix[2][axis] * length};
}

bool save_rgb_image(
  const Camera_2D_Display_Image& image_data,
  const std::filesystem::path& path,
  std::string* error_message)
{
  wxImage image(
    static_cast<int>(image_data.width),
    static_cast<int>(image_data.height));
  if( !image.IsOk() || !image.GetData() ||
      image_data.rgb.size() !=
        static_cast<std::size_t>(image_data.width) *
        image_data.height * 3 )
  {
    if( error_message ) *error_message = "创建待保存图像失败";
    return false;
  }
  std::memcpy(
    image.GetData(), image_data.rgb.data(), image_data.rgb.size());
  if( !image.SaveFile(wxString(path.wstring()), wxBITMAP_TYPE_PNG) )
  {
    if( error_message )
      *error_message = "保存图片失败：" + path.string();
    return false;
  }
  return true;
}

Camera_2D_Display_Image make_image_preview(
  const Camera_2D_Display_Image& source,
  unsigned int maximum_width,
  unsigned int maximum_height)
{
  if( source.width == 0 || source.height == 0 ||
      source.rgb.empty() )
    return {};
  const double scale = std::min(
    1.0,
    std::min(
      static_cast<double>(maximum_width) / source.width,
      static_cast<double>(maximum_height) / source.height));
  Camera_2D_Display_Image preview;
  preview.width = std::max(
    1U, static_cast<unsigned int>(std::lround(source.width * scale)));
  preview.height = std::max(
    1U, static_cast<unsigned int>(std::lround(source.height * scale)));
  preview.rgb.resize(
    static_cast<std::size_t>(preview.width) * preview.height * 3);
  for( unsigned int y = 0; y < preview.height; ++y )
  {
    const unsigned int source_y = std::min(
      source.height - 1,
      static_cast<unsigned int>(
        static_cast<std::uint64_t>(y) * source.height /
        preview.height));
    for( unsigned int x = 0; x < preview.width; ++x )
    {
      const unsigned int source_x = std::min(
        source.width - 1,
        static_cast<unsigned int>(
          static_cast<std::uint64_t>(x) * source.width /
          preview.width));
      const std::size_t source_index =
        (static_cast<std::size_t>(source_y) * source.width +
         source_x) * 3;
      const std::size_t target_index =
        (static_cast<std::size_t>(y) * preview.width + x) * 3;
      preview.rgb[target_index] = source.rgb[source_index];
      preview.rgb[target_index + 1] = source.rgb[source_index + 1];
      preview.rgb[target_index + 2] = source.rgb[source_index + 2];
    }
  }
  return preview;
}

bool build_pose_mosaic(
  std::vector<std::pair<
    Camera_2D_Display_Image,
    robot_model::Robot_Teach_Point>> captures,
  const robot_model::Fov_Visualization_Configuration& fov_configuration,
  const robot_model::XyzabcPose& flange_from_camera_pose,
  Camera_2D_Display_Image* output,
  std::string* error_message)
{
  if( !output || captures.empty() )
  {
    if( error_message ) *error_message = "没有可用于拼图的运动点图像";
    return false;
  }
  auto fov = fov_configuration;
  robot_model::Tool_Visualization_Configuration wrapper;
  wrapper.fov = fov;
  robot_model::Normalize_Tool_Visualization_Configuration(&wrapper);
  fov = wrapper.fov;

  struct World_Image
  {
    Camera_2D_Display_Image image;
    std::array<double, 3> center;
    std::array<double, 3> horizontal;
    std::array<double, 3> vertical;
  };
  std::vector<World_Image> world_images;
  world_images.reserve(captures.size());
  for( auto& capture : captures )
  {
    auto& image = capture.first;
    const auto& point = capture.second;
    if( image.width == 0 || image.height == 0 || !point.has_world_pose )
      continue;
    const auto world_from_camera = robot_model::Multiply_Matrices(
      robot_model::Build_Zyx_Pose_Matrix(point.world_pose),
      robot_model::Build_Zyx_Pose_Matrix(flange_from_camera_pose));
    // The configured FOV is deterministic: image columns follow the
    // configured length axis/span and image rows follow width. Do not infer
    // this from image aspect ratio because that can transpose an entire run.
    constexpr bool horizontal_is_length = true;
    const auto horizontal_axis = robot_model::Coordinate_Axis_Index(
      horizontal_is_length ? fov.length_axis : fov.width_axis);
    const auto vertical_axis = robot_model::Coordinate_Axis_Index(
      horizontal_is_length ? fov.width_axis : fov.length_axis);
    const double horizontal_span =
      horizontal_is_length ? fov.length_mm : fov.width_mm;
    const double vertical_span =
      horizontal_is_length ? fov.width_mm : fov.length_mm;
    // The 2D camera SDK delivers pixels with right/down pointing opposite to
    // the positive FOV tool X/Y axes. Keep the exact FOV centre from the tool
    // pose, but apply the camera pixel-axis signs when constructing its image
    // plane. Without these signs a C~=180 degree scan is laid out P[n]..P[1].
    world_images.push_back({
      std::move(image),
      {
        world_from_camera[0][3],
        world_from_camera[1][3],
        world_from_camera[2][3]},
      matrix_axis(world_from_camera, horizontal_axis, -horizontal_span),
      matrix_axis(world_from_camera, vertical_axis, -vertical_span)});
  }
  if( world_images.empty() )
  {
    if( error_message ) *error_message = "运动点图像或位姿数据无效";
    return false;
  }

  const auto origin = world_images.front().center;
  // Keep the canvas axes identical to the camera pixel axes. Robot poses only
  // determine the image-centre offsets; they must never rotate every source
  // image merely to make the Progress scan direction run left-to-right.
  const auto basis_u = normalized3(world_images.front().horizontal);
  const auto basis_v = normalized3(world_images.front().vertical);
  if( norm3(basis_u) <= 0.0 || norm3(basis_v) <= 0.0 )
  {
    if( error_message ) *error_message = "相机图像平面轴向无效";
    return false;
  }

  std::vector<Mosaic_Placed_Image> placed;
  placed.reserve(world_images.size());
  double minimum_u = std::numeric_limits<double>::max();
  double maximum_u = std::numeric_limits<double>::lowest();
  double minimum_v = std::numeric_limits<double>::max();
  double maximum_v = std::numeric_limits<double>::lowest();
  double native_pixels_per_mm = std::numeric_limits<double>::max();
  for( auto& item : world_images )
  {
    const std::array<double, 3> relative = {
      item.center[0] - origin[0],
      item.center[1] - origin[1],
      item.center[2] - origin[2]};
    Mosaic_Placed_Image projected;
    projected.image = std::move(item.image);
    projected.center_u = dot3(relative, basis_u);
    projected.center_v = dot3(relative, basis_v);
    projected.horizontal_u = dot3(item.horizontal, basis_u);
    projected.horizontal_v = dot3(item.horizontal, basis_v);
    projected.vertical_u = dot3(item.vertical, basis_u);
    projected.vertical_v = dot3(item.vertical, basis_v);
    native_pixels_per_mm = std::min(
      native_pixels_per_mm,
      std::min(
        static_cast<double>(projected.image.width) /
          std::max(norm3(item.horizontal), 1.0e-9),
        static_cast<double>(projected.image.height) /
          std::max(norm3(item.vertical), 1.0e-9)));
    for( const double sx : {-0.5, 0.5} )
    {
      for( const double sy : {-0.5, 0.5} )
      {
        const double u = projected.center_u +
          sx * projected.horizontal_u + sy * projected.vertical_u;
        const double v = projected.center_v +
          sx * projected.horizontal_v + sy * projected.vertical_v;
        minimum_u = std::min(minimum_u, u);
        maximum_u = std::max(maximum_u, u);
        minimum_v = std::min(minimum_v, v);
        maximum_v = std::max(maximum_v, v);
      }
    }
    placed.push_back(std::move(projected));
  }

  const double span_u = std::max(maximum_u - minimum_u, 1.0e-6);
  const double span_v = std::max(maximum_v - minimum_v, 1.0e-6);
  double pixels_per_mm = native_pixels_per_mm;
  pixels_per_mm = std::min(
    pixels_per_mm,
    static_cast<double>(RUN_MOSAIC_MAX_DIMENSION) /
      std::max(span_u, span_v));
  pixels_per_mm = std::min(
    pixels_per_mm,
    std::sqrt(
      static_cast<double>(RUN_MOSAIC_MAX_PIXELS) /
      (span_u * span_v)));
  if( !std::isfinite(pixels_per_mm) || pixels_per_mm <= 0.0 )
  {
    if( error_message ) *error_message = "计算拼图分辨率失败";
    return false;
  }
  const unsigned int output_width = static_cast<unsigned int>(
    std::max(1.0, std::ceil(span_u * pixels_per_mm)));
  const unsigned int output_height = static_cast<unsigned int>(
    std::max(1.0, std::ceil(span_v * pixels_per_mm)));
  Camera_2D_Display_Image mosaic;
  mosaic.width = output_width;
  mosaic.height = output_height;
  mosaic.rgb.assign(
    static_cast<std::size_t>(output_width) * output_height * 3, 0);
  // Keep a sharp seam in overlap regions. Averaging two slightly misaligned
  // robot-positioned images creates a wide double edge; selecting the source
  // whose pixel is farther from its image border confines any residual pose
  // error to one seam and preserves the original camera detail.
  std::vector<float> best_source_scores(
    static_cast<std::size_t>(output_width) * output_height,
    -std::numeric_limits<float>::infinity());

  for( const auto& item : placed )
  {
    const double determinant =
      item.horizontal_u * item.vertical_v -
      item.horizontal_v * item.vertical_u;
    if( std::abs(determinant) <= 1.0e-9 )
      continue;
    double image_min_u = std::numeric_limits<double>::max();
    double image_max_u = std::numeric_limits<double>::lowest();
    double image_min_v = std::numeric_limits<double>::max();
    double image_max_v = std::numeric_limits<double>::lowest();
    for( const double sx : {-0.5, 0.5} )
    {
      for( const double sy : {-0.5, 0.5} )
      {
        const double u = item.center_u +
          sx * item.horizontal_u + sy * item.vertical_u;
        const double v = item.center_v +
          sx * item.horizontal_v + sy * item.vertical_v;
        image_min_u = std::min(image_min_u, u);
        image_max_u = std::max(image_max_u, u);
        image_min_v = std::min(image_min_v, v);
        image_max_v = std::max(image_max_v, v);
      }
    }
    const int x0 = std::max(
      0, static_cast<int>(std::floor(
        (image_min_u - minimum_u) * pixels_per_mm)));
    const int x1 = std::min(
      static_cast<int>(output_width) - 1,
      static_cast<int>(std::ceil(
        (image_max_u - minimum_u) * pixels_per_mm)));
    const int y0 = std::max(
      0, static_cast<int>(std::floor(
        (image_min_v - minimum_v) * pixels_per_mm)));
    const int y1 = std::min(
      static_cast<int>(output_height) - 1,
      static_cast<int>(std::ceil(
        (image_max_v - minimum_v) * pixels_per_mm)));
    for( int y = y0; y <= y1; ++y )
    {
      const double plane_v =
        minimum_v + (static_cast<double>(y) + 0.5) / pixels_per_mm;
      for( int x = x0; x <= x1; ++x )
      {
        const double plane_u =
          minimum_u + (static_cast<double>(x) + 0.5) / pixels_per_mm;
        const double du = plane_u - item.center_u;
        const double dv = plane_v - item.center_v;
        const double sx =
          (du * item.vertical_v - dv * item.vertical_u) / determinant;
        const double sy =
          (item.horizontal_u * dv - item.horizontal_v * du) / determinant;
        if( sx < -0.5 || sx > 0.5 || sy < -0.5 || sy > 0.5 )
          continue;
        const auto source_x = static_cast<unsigned int>(std::clamp(
          (sx + 0.5) * static_cast<double>(item.image.width - 1),
          0.0,
          static_cast<double>(item.image.width - 1)));
        const auto source_y = static_cast<unsigned int>(std::clamp(
          (sy + 0.5) * static_cast<double>(item.image.height - 1),
          0.0,
          static_cast<double>(item.image.height - 1)));
        const std::size_t source_index =
          (static_cast<std::size_t>(source_y) * item.image.width +
           source_x) * 3;
        const std::size_t target_pixel =
          static_cast<std::size_t>(y) * output_width +
          static_cast<std::size_t>(x);
        const std::size_t target_index = target_pixel * 3;
        const float source_score = static_cast<float>(std::min(
          0.5 - std::abs(sx), 0.5 - std::abs(sy)));
        if( source_score <= best_source_scores[target_pixel] )
          continue;
        for( std::size_t channel = 0; channel < 3; ++channel )
        {
          mosaic.rgb[target_index + channel] =
            item.image.rgb[source_index + channel];
        }
        best_source_scores[target_pixel] = source_score;
      }
    }
  }
  *output = std::move(mosaic);
  if( error_message ) error_message->clear();
  return true;
}

} // namespace

Run_Image_Processing_Result Process_Run_Images(
  std::vector<Run_Captured_Frame> captures,
  const Run_Image_Processing_Options &options)
{
  if( captures.empty() )
    return {false, "后台处理失败：没有运动点图像", {}, 0};
  const auto &image_directory = options.image_directory;
  const bool save_original_images = options.save_original_images;
  const bool build_mosaic = options.build_mosaic;
  const auto &fov_configuration = options.fov_configuration;
  const auto &flange_from_camera_pose = options.flange_from_camera_pose;
  Run_Image_Processing_Result result;
  try
  {
    std::vector<std::pair<
      Camera_2D_Display_Image,
      robot_model::Robot_Teach_Point>> converted;
    if( build_mosaic )
      converted.reserve(captures.size());

    std::string error;
    for( auto& capture : captures )
    {
      if( !capture.frame )
      {
        result.message = "后台处理失败：运动点图像为空";
        break;
      }
      Camera_2D_Display_Image image;
      if( !Convert_Camera_2D_Frame(*capture.frame, &image, &error) )
      {
        result.message = "后台图像转换失败：" + error;
        break;
      }
      if( save_original_images )
      {
        std::ostringstream name;
        name << "P" << std::setw(3) << std::setfill('0')
             << capture.point.id
             << "_frame_" << capture.frame->m_frame_num << ".png";
        if( !save_rgb_image(
              image, image_directory / name.str(), &error) )
        {
          result.message = "后台" + error;
          break;
        }
      }
      if( build_mosaic )
        converted.emplace_back(
          std::move(image), std::move(capture.point));
      ++result.processed_count;
      capture.frame.reset();
    }

    if( result.message.empty() && build_mosaic )
    {
      const std::size_t converted_count = converted.size();
      if( !build_pose_mosaic(
            std::move(converted),
            fov_configuration,
            flange_from_camera_pose,
            &result.mosaic,
            &error) )
      {
        result.message = "拼图失败：" + error;
      }
      else if( !save_rgb_image(
                 result.mosaic,
                 image_directory / "mosaic.png",
                 &error) )
      {
        result.message = "拼图" + error;
      }
      else
      {
        result.success = true;
        result.message =
          "图片与拼图处理完成，共 " +
          std::to_string(converted_count) +
          " 个运动点；目录：" + image_directory.string();
        result.mosaic =
          make_image_preview(result.mosaic, 360, 260);
      }
    }
    else if( result.message.empty() )
    {
      result.success = true;
      result.message =
        "运动点原图保存完成，共 " +
        std::to_string(result.processed_count) +
        " 张；目录：" + image_directory.string();
    }
  }
  catch( const std::exception& error )
  {
    result.success = false;
    result.message =
      std::string("后台图片/拼图处理异常：") + error.what();
  }

  return result;
}
} // namespace application

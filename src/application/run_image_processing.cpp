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
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace application
{
namespace
{
constexpr int RUN_MOSAIC_MAX_DIMENSION = 6000;
constexpr std::size_t RUN_MOSAIC_MAX_PIXELS = 20000000;

struct Converted_Capture
{
  Camera_2D_Display_Image image;
  robot_model::Robot_Teach_Point point;
  std::string error;
};

struct Mosaic_Placed_Image
{
  const Camera_2D_Display_Image* image = nullptr;
  std::size_t point_id = 0;
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
  return std::hypot(value[0], value[1], value[2]);
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

template<typename Values>
bool all_finite(const Values& values)
{
  return std::all_of(values.begin(), values.end(),
    [](double value) { return std::isfinite(value); });
}

bool valid_axis(robot_model::Coordinate_Axis axis)
{
  return axis == robot_model::Coordinate_Axis::X ||
    axis == robot_model::Coordinate_Axis::Y ||
    axis == robot_model::Coordinate_Axis::Z;
}

bool build_pose_mosaic(
  const std::vector<const Converted_Capture*>& captures,
  const robot_model::Fov_Visualization_Configuration& fov_configuration,
  const robot_model::XyzabcPose& flange_from_camera_pose,
  const robot_model::Mosaic_Group& group,
  Camera_2D_Display_Image* output,
  std::string* error_message)
{
  if( !output || captures.empty() )
  {
    if( error_message ) *error_message = "没有可用于拼图的运动点图像";
    return false;
  }
  const auto& fov = fov_configuration;
  if( !std::isfinite(fov.length_mm) || !std::isfinite(fov.width_mm) ||
      fov.length_mm <= 0.0 || fov.width_mm <= 0.0 ||
      !valid_axis(fov.length_axis) || !valid_axis(fov.width_axis) ||
      fov.length_axis == fov.width_axis )
  {
    if( error_message ) *error_message = "视野尺寸或轴配置无效";
    return false;
  }

  std::unordered_map<std::size_t, const robot_model::Mosaic_Point_Correction*> corrections;
  const std::unordered_set<std::size_t> member_ids(
    group.point_ids.begin(), group.point_ids.end());
  for( const auto& correction : group.corrections )
  {
    if( member_ids.count(correction.point_id) == 0 ||
        !corrections.emplace(correction.point_id, &correction).second ||
        !std::isfinite(correction.offset_u_mm) ||
        !std::isfinite(correction.offset_v_mm) )
    {
      if( error_message ) *error_message = "P" +
        std::to_string(correction.point_id) + "：拼图补偿必须属于组内点、唯一且为有限数值";
      return false;
    }
  }

  struct World_Image
  {
    const Camera_2D_Display_Image* image;
    std::size_t point_id;
    std::array<double, 3> center;
    std::array<double, 3> horizontal;
    std::array<double, 3> vertical;
  };
  std::vector<World_Image> world_images;
  world_images.reserve(captures.size());
  for( const auto* capture : captures )
  {
    const auto& image = capture->image;
    const auto& point = capture->point;
    const auto fail_point = [&](const std::string& reason) {
      if( error_message )
        *error_message = "P" + std::to_string(point.id) + "：" + reason;
      return false;
    };
    if( !capture->error.empty() ) return fail_point(capture->error);
    if( image.width == 0 || image.height == 0 || image.rgb.size() !=
        static_cast<std::size_t>(image.width) * image.height * 3 )
      return fail_point("图像数据无效");
    if( !point.has_world_pose || !all_finite(point.world_pose) ||
        !all_finite(flange_from_camera_pose) )
      return fail_point("运动点或相机位姿无效");
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
      &image,
      point.id,
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
  auto basis_u = normalized3(world_images.front().horizontal);
  auto basis_v = normalized3(world_images.front().vertical);
  if( group.reference == robot_model::Mosaic_Reference::Robot_Base )
  {
    basis_u = {0.0, 0.0, 0.0};
    basis_v = {0.0, 0.0, 0.0};
    basis_u[robot_model::Coordinate_Axis_Index(group.horizontal_axis)] = 1.0;
    basis_v[robot_model::Coordinate_Axis_Index(group.vertical_axis)] = 1.0;
  }
  if( group.reverse_horizontal )
    for( auto& value : basis_u ) value = -value;
  if( group.reverse_vertical )
    for( auto& value : basis_v ) value = -value;
  if( !all_finite(basis_u) || !all_finite(basis_v) ||
      norm3(basis_u) <= 0.0 || norm3(basis_v) <= 0.0 )
  {
    if( error_message ) *error_message = "P" +
      std::to_string(world_images.front().point_id) + "：相机图像平面轴向无效";
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
    projected.image = item.image;
    projected.point_id = item.point_id;
    projected.center_u = dot3(relative, basis_u);
    projected.center_v = dot3(relative, basis_v);
    // Corrections are millimetres in the final signed mosaic plane, not robot
    // motion offsets: never change the camera basis or the captured point pose.
    const auto correction = corrections.find(item.point_id);
    if( correction != corrections.end() )
    {
      projected.center_u += correction->second->offset_u_mm;
      projected.center_v += correction->second->offset_v_mm;
    }
    projected.horizontal_u = dot3(item.horizontal, basis_u);
    projected.horizontal_v = dot3(item.horizontal, basis_v);
    projected.vertical_u = dot3(item.vertical, basis_u);
    projected.vertical_v = dot3(item.vertical, basis_v);
    const double determinant = projected.horizontal_u * projected.vertical_v -
      projected.horizontal_v * projected.vertical_u;
    const double source_area = norm3(item.horizontal) * norm3(item.vertical);
    if( !all_finite(item.center) || !all_finite(relative) ||
        !all_finite(item.horizontal) || !all_finite(item.vertical) ||
        !std::isfinite(projected.center_u) || !std::isfinite(projected.center_v) ||
        !std::isfinite(determinant) || !std::isfinite(source_area) ||
        source_area <= 0.0 || std::abs(determinant) / source_area < 1.0e-6 ||
        projected.center_u + 0.5 * (std::abs(projected.horizontal_u) +
          std::abs(projected.vertical_u)) == projected.center_u ||
        projected.center_v + 0.5 * (std::abs(projected.horizontal_v) +
          std::abs(projected.vertical_v)) == projected.center_v )
    {
      if( error_message ) *error_message = "P" + std::to_string(item.point_id) +
        "：投影接近垂直、退化或包含无效数值";
      return false;
    }
    native_pixels_per_mm = std::min(
      native_pixels_per_mm,
      std::min(
        static_cast<double>(projected.image->width) /
          std::max(norm3(item.horizontal), 1.0e-9),
        static_cast<double>(projected.image->height) /
          std::max(norm3(item.vertical), 1.0e-9)));
    for( const double sx : {-0.5, 0.5} )
    {
      for( const double sy : {-0.5, 0.5} )
      {
        const double u = projected.center_u +
          sx * projected.horizontal_u + sy * projected.vertical_u;
        const double v = projected.center_v +
          sx * projected.horizontal_v + sy * projected.vertical_v;
        if( !std::isfinite(u) || !std::isfinite(v) )
        {
          if( error_message ) *error_message = "P" +
            std::to_string(item.point_id) + "：投影边界无效";
          return false;
        }
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
  if( !std::isfinite(span_u) || !std::isfinite(span_v) ||
      !std::isfinite(span_u * span_v) )
  {
    if( error_message ) *error_message = "拼图范围过大或无效";
    return false;
  }
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
  const double width_pixels = std::max(1.0, std::ceil(span_u * pixels_per_mm));
  const double height_pixels = std::max(1.0, std::ceil(span_v * pixels_per_mm));
  if( !std::isfinite(width_pixels) || !std::isfinite(height_pixels) ||
      width_pixels > RUN_MOSAIC_MAX_DIMENSION + 1.0 ||
      height_pixels > RUN_MOSAIC_MAX_DIMENSION + 1.0 )
  {
    if( error_message ) *error_message = "拼图尺寸无效";
    return false;
  }
  const unsigned int output_width = std::min(
    static_cast<unsigned int>(RUN_MOSAIC_MAX_DIMENSION),
    static_cast<unsigned int>(width_pixels));
  const unsigned int output_height = std::min({
    static_cast<unsigned int>(RUN_MOSAIC_MAX_DIMENSION),
    static_cast<unsigned int>(height_pixels),
    static_cast<unsigned int>(RUN_MOSAIC_MAX_PIXELS / output_width)});
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
        if( !std::isfinite(sx) || !std::isfinite(sy) )
        {
          if( error_message ) *error_message = "P" +
            std::to_string(item.point_id) + "：图像逆投影数值无效";
          return false;
        }
        if( sx < -0.5 || sx > 0.5 || sy < -0.5 || sy > 0.5 )
          continue;
        const auto source_x = static_cast<unsigned int>(std::clamp(
          (sx + 0.5) * static_cast<double>(item.image->width - 1),
          0.0,
          static_cast<double>(item.image->width - 1)));
        const auto source_y = static_cast<unsigned int>(std::clamp(
          (sy + 0.5) * static_cast<double>(item.image->height - 1),
          0.0,
          static_cast<double>(item.image->height - 1)));
        const std::size_t source_index =
          (static_cast<std::size_t>(source_y) * item.image->width +
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
            item.image->rgb[source_index + channel];
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
  Run_Image_Processing_Result result;
  const auto& image_directory = options.image_directory;
  try
  {
    // Convert/save each input once. Groups borrow these images, never duplicate
    // full-resolution buffers, and only one full-resolution mosaic is live.
    std::vector<Converted_Capture> converted;
    if( options.build_mosaic ) converted.reserve(captures.size());
    for( auto& capture : captures )
    {
      Converted_Capture item;
      item.point = std::move(capture.point);
      if( !capture.frame )
        item.error = "运动点图像为空";
      else if( !Convert_Camera_2D_Frame(*capture.frame, &item.image, &item.error) )
        item.error = "图像转换失败：" + item.error;
      else if( options.save_original_images && !options.preview_only )
      {
        std::ostringstream name;
        name << "P" << std::setw(3) << std::setfill('0') << item.point.id
             << "_frame_" << capture.frame->m_frame_num << ".png";
        save_rgb_image(item.image, image_directory / name.str(), &item.error);
      }
      if( item.error.empty() )
        ++result.processed_count;
      else if( result.message.empty() )
        result.message = "P" + std::to_string(item.point.id) + "：" + item.error;
      capture.frame.reset();
      if( options.build_mosaic ) converted.push_back(std::move(item));
    }
    if( !options.build_mosaic )
    {
      result.success = !captures.empty() && result.message.empty();
      if( result.message.empty() )
        result.message = result.success ?
          "运动点原图处理完成，共 " + std::to_string(result.processed_count) + " 张" :
          "后台处理失败：没有运动点图像";
      return result;
    }

    const bool implicit_group = options.mosaic_configuration.groups.empty();
    auto groups = options.mosaic_configuration.groups;
    if( implicit_group )
    {
      robot_model::Mosaic_Group group;
      group.id = 0;
      group.name = "默认拼图";
      groups.push_back(std::move(group));
    }
    std::unordered_map<std::size_t, std::size_t> group_id_counts;
    std::unordered_map<std::size_t, std::size_t> point_id_counts;
    for( const auto& group : groups ) ++group_id_counts[group.id];
    for( const auto& item : converted ) ++point_id_counts[item.point.id];
    bool all_success = result.message.empty();
    for( const auto& group : groups )
    {
      Mosaic_Group_Result group_result;
      group_result.group_id = group.id;
      group_result.name = group.name;
      try
      {
        std::string error;
        std::unordered_set<std::size_t> members;
        if( !implicit_group )
        {
          if( group.id == 0 || group_id_counts[group.id] != 1 )
            error = "拼图组ID无效或重复";
          else if( group.name.empty() )
            error = "拼图组名称不能为空";
          else if( group.point_ids.empty() )
            error = "拼图组没有运动点";
          for( const auto id : group.point_ids )
          {
            if( !error.empty() ) break;
            if( id == 0 )
              error = "P0：采图点号无效";
            else if( !members.insert(id).second )
              error = "P" + std::to_string(id) + "：组内点号重复";
            else if( point_id_counts[id] == 0 )
              error = "P" + std::to_string(id) + "：缺少采图数据";
          }
        }
        if( error.empty() &&
            (group.reference != robot_model::Mosaic_Reference::First_Camera &&
             group.reference != robot_model::Mosaic_Reference::Robot_Base) )
          error = "拼图参考系无效";
        if( error.empty() && group.reference == robot_model::Mosaic_Reference::Robot_Base &&
            (!valid_axis(group.horizontal_axis) || !valid_axis(group.vertical_axis) ||
             group.horizontal_axis == group.vertical_axis) )
          error = "拼图水平轴和垂直轴必须是不同的有效轴";

        std::vector<const Converted_Capture*> selected;
        // Stable point IDs select membership, but capture order determines the
        // first-camera reference and tie breaking in overlaps.
        for( const auto& item : converted )
        {
          if( implicit_group ?
              item.point.type != robot_model::Robot_Teach_Point_Type::Motion :
              members.count(item.point.id) == 0 ) continue;
          if( error.empty() && point_id_counts[item.point.id] != 1 )
            error = "P" + std::to_string(item.point.id) + "：采图点号重复";
          if( error.empty() && item.point.type != robot_model::Robot_Teach_Point_Type::Motion )
            error = "P" + std::to_string(item.point.id) + "：不是运动采图点";
          selected.push_back(&item);
        }
        Camera_2D_Display_Image mosaic;
        const auto path = image_directory / (implicit_group ? "mosaic.png" :
          "mosaic_group_" + std::to_string(group.id) + ".png");
        if( error.empty() && build_pose_mosaic(selected, options.fov_configuration,
              options.flange_from_camera_pose, group, &mosaic, &error) &&
            (options.preview_only || save_rgb_image(mosaic, path, &error)) )
        {
          group_result.mosaic = options.preview_only ? std::move(mosaic) : make_image_preview(mosaic, 360, 260);
          if (!options.preview_only) group_result.mosaic_path = path;
          group_result.success = true;
          group_result.message = "拼图完成，共 " + std::to_string(selected.size()) + " 个运动点";
        }
        else
          group_result.message = "拼图失败：" + error;
      }
      catch( const std::exception& error )
      {
        group_result.message = std::string("拼图处理异常：") + error.what();
      }
      if( group_result.success && result.mosaic.rgb.empty() )
      {
        result.mosaic = options.preview_only
          ? make_image_preview(group_result.mosaic, 360, 260) : group_result.mosaic;
        result.mosaic_path = group_result.mosaic_path;
      }
      if( !group_result.success )
      {
        all_success = false;
        if( !result.message.empty() ) result.message += "；";
        result.message += "组 " + std::to_string(group.id) + " " +
          group.name + "：" + group_result.message;
      }
      result.groups.push_back(std::move(group_result));
    }
    result.success = all_success;
    if( result.success )
      result.message = options.preview_only
        ? "实时纠偏预览已更新（未覆盖原文件）"
        : "图片与拼图处理完成，共 " + std::to_string(result.groups.size()) +
          " 组；目录：" + image_directory.string();
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

#include "robot_model_panel_controller.h"

#include "app_paths.h"
#include "camera_control_panel.h"
#include "camera_image_view.h"
#include "camera_service.h"
#include "camera_2d_control_panel.h"
#ifdef CAMERA_INTRINSIC_CALIBRATION_ENABLED
#include "camera_intrinsic_calibration_dialog.h"
#endif
#include "camera_2d_image_view.h"
#include "camera_2d_template_panel.h"
#include "robot_joint_state_builder.h"
#include "robot_model_config_dialog.h"
#include "robot_model_repository.h"
#include "robot_progress_io.h"
#include "tool_coordinate_repository.h"
#include "right_tool_panel.h"
#include "net_panel.h"
#include "run_progress_panel.h"
#include "camera_2d_image_converter.h"
#include "camera_params.h"
#include "point_cloud_view.h"
#include "point_cloud_overlay_toolbar.h"
#include "teach_point_command_panel.h"
#include "teach_point_list_panel.h"
#include "template_configuration_repository.h"
#include "tool_panel.h"
#include "tool_visualization_repository.h"

#include <wx/button.h>
#include <wx/choice.h>
#include <wx/choicdlg.h>
#include <wx/datetime.h>
#include <wx/filedlg.h>
#include <wx/filename.h>
#include <wx/image.h>
#include <wx/grid.h>
#include <wx/msgdlg.h>
#include <wx/minifram.h>
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/simplebook.h>
#include <wx/slider.h>
#include <wx/splitter.h>
#include <wx/tglbtn.h>
#include <wx/utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <utility>

wxDEFINE_EVENT(wxEVT_KUKA_MODEL_STATE, wxThreadEvent);
wxDEFINE_EVENT(wxEVT_KUKA_SERVICE_STATUS, wxThreadEvent);
wxDEFINE_EVENT(wxEVT_RUN_IMAGE_PROCESSING_RESULT, wxThreadEvent);

namespace
{
constexpr int DEFAULT_JOINT_MIN = -180;
constexpr int DEFAULT_JOINT_MAX = 180;
constexpr int TRAJECTORY_FRAME_COUNT = 120;
constexpr int TRAJECTORY_TIMER_MS = 16;
constexpr int TRAJECTORY_SPEED_DEFAULT_CM_PER_SECOND = 100;
constexpr std::size_t ROBOT_JOINT_COUNT = 6;
constexpr int RIGHT_TOOL_COLLAPSED_WIDTH = 72;
constexpr int DISPLAY_MINIMUM_WIDTH = 300;
constexpr int TEACH_POINT_COLLAPSED_WIDTH = 38;
constexpr int TEACH_POINT_DEFAULT_EXPANDED_WIDTH = 240;
constexpr int WORKSPACE_MINIMUM_WIDTH = 500;
constexpr const char* DEFAULT_ROBOT_MODEL_ID = "KR10_R1100_2";
constexpr double RUN_HOME_POSITION_TOLERANCE_MM = 2.0;
constexpr double RUN_HOME_ANGLE_TOLERANCE_DEG = 1.0;
constexpr int RUN_TIMER_INTERVAL_MS = 40;
constexpr int RUN_IMAGE_TIMEOUT_MS = 3000;
constexpr int RUN_MOSAIC_MAX_DIMENSION = 6000;
constexpr std::size_t RUN_MOSAIC_MAX_PIXELS = 20000000;

struct Run_Image_Processing_Result
{
  bool success = false;
  std::string message;
  Camera_2D_Display_Image mosaic;
};

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

std::size_t frame_count_for_one_meter_per_second (
  const robot_model::XyzabcPose& start,
  const robot_model::XyzabcPose& target)
{
  const double dx = target[0] - start[0];
  const double dy = target[1] - start[1];
  const double dz = target[2] - start[2];
  const double distance_mm = std::sqrt (dx * dx + dy * dy + dz * dz);
  if( distance_mm <= 0.0 ) return TRAJECTORY_FRAME_COUNT;
  const double millimeters_per_tick =
    1000.0 * static_cast<double> (TRAJECTORY_TIMER_MS) / 1000.0;
  return std::max<std::size_t> (
    2,
    static_cast<std::size_t> (
      std::ceil (distance_mm / millimeters_per_tick)) + 1);
}

std::size_t frame_count_for_joint_step (
  const std::array<double, 6>& start,
  const std::array<double, 6>& target,
  int speed_percent)
{
  double maximum_delta_deg = 0.0;
  for( std::size_t index = 0; index < start.size ( ); ++index )
  {
    maximum_delta_deg = std::max (
      maximum_delta_deg,
      std::abs (target[index] - start[index]));
  }
  constexpr double maximum_preview_speed_deg_per_second = 180.0;
  const double speed_deg_per_second =
    maximum_preview_speed_deg_per_second *
    static_cast<double> (std::clamp (speed_percent, 10, 100)) / 100.0;
  const double duration_seconds =
    maximum_delta_deg / speed_deg_per_second;
  return std::max<std::size_t> (
    2,
    static_cast<std::size_t> (std::ceil (
      duration_seconds * 1000.0 /
      static_cast<double> (TRAJECTORY_TIMER_MS))) + 1);
}

wxString teach_point_type_label (
  robot_model::Robot_Teach_Point_Type type)
{
  switch( type )
  {
    case robot_model::Robot_Teach_Point_Type::Transition:
      return wxString::FromUTF8 (u8"过渡点");
    case robot_model::Robot_Teach_Point_Type::Wait:
      return wxString::FromUTF8 (u8"等待点");
    case robot_model::Robot_Teach_Point_Type::Motion:
    default:
      return wxString::FromUTF8 (u8"运动点");
  }
}

std::array<double, 6> configured_home_input_angles (
  const robot_model::Robot_Kinematic_Params& params)
{
  if( params.has_home_input_angles ) return params.home_input_angles_deg;

  std::array<double, 6> fallback = { };
  for( size_t index = 0; index < fallback.size ( ); ++index )
    fallback[index] = robot_model::Neutral_Joint_Input_At (params, index);
  return fallback;
}

int slider_limit_at (const std::vector<double>& values, size_t index,
                     int fallback)
{
  if( index >= values.size ( ) )
  {
    return fallback;
  }

  return static_cast<int> (std::lround (values[index]));
}

std::string model_id (const robot_model::Robot_Model_Info& model)
{
  return model.model_dir.filename ( ).string ( );
}

bool validate_model_files (
  const robot_model::Robot_Model_Info& model,
  std::string* error_message)
{
  if( error_message )
  {
    error_message->clear ( );
  }

  if( model.stl_files.empty ( ) )
  {
    if( error_message )
    {
      *error_message = "The robot model does not contain STL files";
    }
    return false;
  }

  if( model.xml_path.empty ( ) || !std::filesystem::is_regular_file (model.xml_path) )
  {
    if( error_message )
    {
      *error_message = "The robot model configuration XML is missing";
    }
    return false;
  }

  const auto missing_mesh = std::find_if (
    model.stl_files.begin ( ), model.stl_files.end ( ),
    [] (const std::filesystem::path& path)
    {
      return !std::filesystem::is_regular_file (path);
    });
  if( missing_mesh != model.stl_files.end ( ) )
  {
    if( error_message )
    {
      *error_message = "One or more robot STL files are missing";
    }
    return false;
  }

  return true;
}

int cartesian_position_limit_mm (
  const robot_model::Robot_Kinematic_Params& params)
{
  double estimated_reach = 0.0;
  for( const double length : params.link_lengths )
  {
    estimated_reach += std::abs (length);
  }
  const double limit = std::max (1000.0, estimated_reach * 1.25);
  return static_cast<int> (std::ceil (limit / 100.0) * 100.0);
}

wxString collision_summary (
  const robot_model::Robot_Collision_Result& collision)
{
  switch( collision.type )
  {
    case robot_model::Robot_Collision_Type::Self_Collision:
      return wxString::Format (
        wxString::FromUTF8 (u8"部件 %zu 与部件 %zu 自碰撞"),
        collision.robot_part_index,
        collision.other_robot_part_index);
    case robot_model::Robot_Collision_Type::Ground:
      return wxString::Format (
        wxString::FromUTF8 (u8"部件 %zu 接近地面，距离 %.2f mm"),
        collision.robot_part_index,
        collision.minimum_distance_mm);
    case robot_model::Robot_Collision_Type::Obstacle_Point:
      return wxString::Format (
        wxString::FromUTF8 (u8"部件 %zu 与点云碰撞，距离 %.2f mm"),
        collision.robot_part_index,
        collision.minimum_distance_mm);
    default:
      return wxString::FromUTF8 (u8"检测到碰撞");
  }
}

} // namespace

Robot_Model_Panel_Controller::Robot_Model_Panel_Controller (
  wxWindow* parent,
  Camera_Service& camera_service,
  Camera_2D_Service& camera_2d_service,
  Camera_2D_Cross_Template_Service& camera_2d_template_service,
  std::shared_ptr<application::Robot_Connection_Controller> robot_connection,
  wxWindowID id)
  : wxPanel(parent, id)
{
  m_camera_2d_service = &camera_2d_service;
  m_camera_2d_template_service = &camera_2d_template_service;
  m_camera_service = &camera_service;
  m_robot_connection = std::move(robot_connection);
  if( !m_robot_connection )
    throw std::invalid_argument("Robot connection controller is null.");
  this->Bind (
    wxEVT_KUKA_MODEL_STATE,
    &Robot_Model_Panel_Controller::On_Kuka_Model_State,
    this);
  this->Bind (
    wxEVT_KUKA_SERVICE_STATUS,
    &Robot_Model_Panel_Controller::On_Kuka_Service_Status,
    this);
  this->Bind (
    wxEVT_RUN_IMAGE_PROCESSING_RESULT,
    &Robot_Model_Panel_Controller::On_Run_Image_Processing_Result,
    this);

  application::Robot_Connection_Observer robot_observer;
  robot_observer.robot_state =
    [this] (const application::Robot_Actual_State& state,
            std::uint64_t revision)
    {
      auto* event = new wxThreadEvent (wxEVT_KUKA_MODEL_STATE);
      event->SetPayload (std::make_pair (state, revision));
      wxQueueEvent (this, event);
    };
  robot_observer.status =
    [this] (const application::Robot_Connection_Status& status)
    {
      auto* event = new wxThreadEvent (wxEVT_KUKA_SERVICE_STATUS);
      event->SetPayload (status);
      wxQueueEvent (this, event);
    };
  m_robot_connection_observer_token =
    m_robot_connection->Subscribe(std::move(robot_observer));

  m_model_name_text = new wxStaticText (
    this, wxID_ANY, wxString::FromUTF8 (u8"当前机械臂：未加载"));
  m_workspace_splitter = new wxSplitterWindow (
    this,
    wxID_ANY,
    wxDefaultPosition,
    wxDefaultSize,
    wxSP_LIVE_UPDATE | wxSP_3DSASH | wxBORDER_NONE);
  m_workspace_splitter->SetMinimumPaneSize (TEACH_POINT_COLLAPSED_WIDTH);
  m_workspace_splitter->SetSashGravity (0.0);
  m_workspace_splitter->SetSashSize (6);

  m_teach_point_list_panel = new Teach_Point_List_Panel (
    m_workspace_splitter);
  m_teach_point_list_panel->Set_On_Selection_Changed (
    [this] { On_Teach_Point_Selection_Changed ( ); });
  m_teach_point_list_panel->Set_On_Pose_Coordinate_Changed (
    [this] (int selection)
    {
      On_Teach_Pose_Coordinate_Changed (selection);
    });
  m_teach_point_list_panel->Set_On_Collapsed_Changed (
    [this] (bool collapsed) { Resize_Teach_Point_List (collapsed); });
  m_teach_point_list_panel->Set_On_Bind_Cloud_Template (
    [this] (int point_index)
    {
      if( point_index >= 0 )
      {
        Bind_Template_To_Teach_Point_Cloud (
          static_cast<std::size_t> (point_index));
      }
    });
  m_teach_point_list_panel->Set_On_Unbind_Cloud_Template (
    [this] (int point_index)
    {
      if( point_index >= 0 )
      {
        Unbind_Template_From_Teach_Point_Cloud (
          static_cast<std::size_t> (point_index));
      }
    });

  m_content_splitter = new wxSplitterWindow (
    m_workspace_splitter,
    wxID_ANY,
    wxDefaultPosition,
    wxDefaultSize,
    wxSP_LIVE_UPDATE | wxSP_3DSASH | wxBORDER_NONE);
  m_content_splitter->SetMinSize (wxSize (WORKSPACE_MINIMUM_WIDTH, -1));
  m_content_splitter->SetMinimumPaneSize (RIGHT_TOOL_COLLAPSED_WIDTH);
  m_content_splitter->SetSashGravity (1.0);
  m_content_splitter->SetSashSize (6);

  auto* display_panel = new wxPanel (m_content_splitter, wxID_ANY);
  m_display_panel = display_panel;
  m_display_book = new wxSimplebook (display_panel, wxID_ANY);
  m_display_book->SetMinSize (wxSize (DISPLAY_MINIMUM_WIDTH, -1));
  auto* status_panel = new wxPanel (
    display_panel, wxID_ANY, wxDefaultPosition, wxDefaultSize,
    wxBORDER_SIMPLE);
  m_status_text = new wxStaticText (status_panel, wxID_ANY, "");
  auto* status_sizer = new wxBoxSizer (wxHORIZONTAL);
  status_sizer->Add (m_status_text, 1, wxALIGN_CENTER_VERTICAL | wxALL, 6);
  status_panel->SetSizer (status_sizer);
  status_panel->SetMinSize (wxSize (-1, 30));

  auto* display_sizer = new wxBoxSizer (wxVERTICAL);
  display_sizer->Add (m_display_book, 1, wxEXPAND);
  display_sizer->Add (status_panel, 0, wxEXPAND | wxTOP, 4);
  display_panel->SetSizer (display_sizer);
  m_view = new Robot_Model_View (m_display_book);
  m_view->Set_On_Collision_Rebuild_Completed (
    [this] (
      bool success,
      const robot_model::Robot_Collision_Point_Cloud_Stats& stats,
      const std::string& error_message)
    {
      if( !success && m_waiting_for_playback_cloud &&
          m_trajectory_session.Is_Active ( ) )
      {
        m_trajectory_timer.Stop ( );
        m_trajectory_session.Pause ( );
        m_waiting_for_playback_cloud = false;
        m_playback_cloud_switch_blocked = true;
        Set_Joint_Controls_Enabled (true);
        Update_Trajectory_Status ( );
      }
      if( !m_status_text ) return;
      if( !success )
      {
        m_status_text->SetLabel (
          wxString::FromUTF8 (u8"碰撞索引后台构建失败：") +
          wxString::FromUTF8 (error_message.c_str ( )));
        return;
      }
      m_status_text->SetLabel (wxString::Format (
        wxString::FromUTF8 (
          u8"碰撞索引已更新：剔除本体点 %zu，碰撞点 %zu，耗时 %.0f ms"),
        stats.excluded_robot_point_count,
        stats.collision_point_count,
        stats.build_time_ms));
    });
  m_view->Set_On_Flange_Dragged (
    [this] (const robot_model::Robot_Position_IK_Result& result)
    {
      Apply_Flange_Drag_Result (result);
    });
  m_view->Set_On_Flange_Pose_Dragged (
    [this] (const robot_model::Robot_Pose_IK_Result& result)
    {
      Apply_Flange_Pose_Drag_Result (result);
    });
  Ensure_Camera_2D_Floating_Preview ( );
  display_panel->Bind (
    wxEVT_SIZE,
    [this] (wxSizeEvent& event)
    {
      event.Skip ( );
      CallAfter (&Robot_Model_Panel_Controller::Layout_Camera_2D_Preview);
    });
  Point_Cloud_Overlay_Toolbar::Callbacks overlay_callbacks;
  overlay_callbacks.renderer = [this]
  {
    return m_view ? m_view->Scene_Renderer ( ) : nullptr;
  };
  overlay_callbacks.robot_model_id = [this]
  {
    return m_current_model_id;
  };
  overlay_callbacks.show_robot_page = [this]
  {
    Select_Display_Page (Main_Display_Page::Robot);
  };
  overlay_callbacks.render_scene = [this]
  {
    if( m_view ) m_view->Render_Scene ( );
  };
  overlay_callbacks.set_status = [this] (const wxString& status)
  {
    if( m_status_text ) m_status_text->SetLabel (status);
  };
  overlay_callbacks.set_camera_pose_visible = [this] (bool visible)
  {
    return Set_Camera_Pose_Visible (visible);
  };
  overlay_callbacks.set_collision_obstacle_points =
    [this] (std::shared_ptr<const std::vector<float>> xyz,
            std::string* error_message)
    {
      if( !m_view )
      {
        if( error_message ) *error_message = "Robot model view is unavailable";
        return false;
      }
      return m_view->Set_Collision_Obstacle_Points (xyz, error_message);
    };
  overlay_callbacks.clear_collision_obstacle_points = [this]
  {
    if( m_view ) m_view->Clear_Collision_Obstacle_Points ( );
  };
  overlay_callbacks.set_collision_enabled = [this] (bool enabled)
  {
    if( !m_view ) return;
    // Reset while collision is still disabled so an already-colliding pose
    // cannot prevent the robot from reaching the known safe home state.
    if( enabled ) Reset_Robot_To_Home ( );
    m_view->Set_Collision_Enabled (enabled);
  };
  overlay_callbacks.collision_rebuild_in_progress = [this]
  {
    return m_view && m_view->Collision_Rebuild_In_Progress ( );
  };
  overlay_callbacks.apply_collision_settings =
    [this] (double clearance_mm,
            double voxel_size_mm,
            double robot_exclusion_distance_mm,
            bool exclude_robot_points,
            std::size_t* excluded_robot_point_count,
            std::size_t* collision_point_count,
            std::string* error_message)
    {
      if( !m_view )
      {
        if( error_message ) *error_message = "Robot model view is unavailable";
        return false;
      }

      auto settings = m_view->Collision_Settings ( );
      settings.clearance_mm = clearance_mm;
      settings.point_cloud.voxel_size_mm = voxel_size_mm;
      settings.point_cloud.robot_exclusion_distance_mm =
        robot_exclusion_distance_mm;
      settings.point_cloud.exclude_robot_points = exclude_robot_points;
      if( !m_view->Set_Collision_Settings (settings, error_message) )
      {
        return false;
      }

      const auto& stats = m_view->Collision_Point_Cloud_Stats ( );
      if( excluded_robot_point_count )
      {
        *excluded_robot_point_count = stats.excluded_robot_point_count;
      }
      if( collision_point_count )
      {
        *collision_point_count = stats.collision_point_count;
      }
      return true;
    };
  overlay_callbacks.set_point_cloud_edit_mode =
    [this] (bool enabled, bool polygon)
  {
    if( m_view )
    {
      m_view->Set_Point_Cloud_Edit_Mode (
        enabled,
        polygon
          ? Robot_Model_View::Point_Cloud_Selection_Shape::Polygon
          : Robot_Model_View::Point_Cloud_Selection_Shape::Rectangle);
    }
  };
  overlay_callbacks.clear_point_cloud_selection_outline = [this]
  {
    if( m_view ) m_view->Clear_Point_Cloud_Selection_Outline ( );
  };
  m_display_book->AddPage (m_view, wxEmptyString, true);
  m_right_tool_panel = new Right_Tool_Panel (m_content_splitter);
  m_right_tool_panel->Set_On_Width_Changed (
    [this] (int width) { Resize_Right_Tool (width); });

  m_tcp_panel = new Net_Panel (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Tcp),
    m_robot_connection);
  m_run_progress_panel = new Run_Progress_Panel (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Run));
  Run_Progress_Panel::Callbacks run_callbacks;
  run_callbacks.start = [this] { Start_Progress_Run ( ); };
  run_callbacks.emergency_stop =
    [this] { Request_Progress_Emergency_Stop ( ); };
  m_run_progress_panel->Set_Callbacks (std::move (run_callbacks));
  m_camera_control_panel = new Camera_Control_Panel (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Camera),
    camera_service);
  m_camera_2d_control_panel = new Camera_2D_Control_Panel (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Camera2D),
    camera_2d_service);
  m_camera_2d_control_panel->Set_On_Show_Image (
    [this] { Select_Display_Page (Main_Display_Page::Camera_2D_Image); });
#ifdef CAMERA_INTRINSIC_CALIBRATION_ENABLED
  m_camera_2d_control_panel->Set_On_Calibrate (
    [this]
    {
      if( !m_camera_2d_service ) return;
      Camera_Intrinsic_Calibration_Dialog dialog (
        this, *m_camera_2d_service);
      dialog.ShowModal ( );
    });
#endif
  m_camera_2d_template_panel = new Camera_2D_Template_Panel (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Template2D),
    camera_2d_service,
    camera_2d_template_service,
    *m_camera_2d_image_view);
  m_camera_2d_template_panel->Set_On_Show_Image (
    [this] { Select_Display_Page (Main_Display_Page::Camera_2D_Image); });
  m_point_cloud_overlay_toolbar = new Point_Cloud_Overlay_Toolbar (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::PointCloud),
    camera_service,
    std::move (overlay_callbacks));
  m_tool_panel = new Tool_Panel (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Tool));
  m_tool_panel->Set_On_Configuration_Changed (
    [this] (
      const robot_model::Tool_Visualization_Configuration& configuration)
    {
      std::string error_message;
      if( !robot_model::Save_Tool_Visualization_Configuration (
            robot_model::Tool_Visualization_Config_Path ( ),
            configuration,
            &error_message) )
      {
        if( m_status_text )
        {
          m_status_text->SetLabel (
            wxString::FromUTF8 (error_message.c_str ( )));
        }
        return;
      }
      m_tool_visualization_configuration = configuration;
      Apply_Tool_Visualization ( );
      if( m_status_text )
      {
        m_status_text->SetLabel (
          wxString::FromUTF8 (u8"FOV 设置已应用"));
      }
    });
  m_view->Set_On_Point_Cloud_Area_Selected (
    [this] (
      int start_x,
      int start_y,
      int end_x,
      int end_y,
      bool add,
      bool toggle)
    {
      if( m_point_cloud_overlay_toolbar )
      {
        m_point_cloud_overlay_toolbar->Handle_Area_Selected (
          start_x, start_y, end_x, end_y, add, toggle);
      }
    });
  m_view->Set_On_Point_Cloud_Polygon_Selected (
    [this] (
      const std::vector<std::array<int, 2>>& polygon,
      bool add,
      bool toggle)
    {
      if( m_point_cloud_overlay_toolbar )
      {
        m_point_cloud_overlay_toolbar->Handle_Polygon_Selected (
          polygon, add, toggle);
      }
    });
  m_view->Set_On_Point_Cloud_Delete ([this]
  {
    if( m_point_cloud_overlay_toolbar )
      m_point_cloud_overlay_toolbar->Delete_Selected ( );
  });
  m_view->Set_On_Point_Cloud_Undo ([this]
  {
    if( m_point_cloud_overlay_toolbar )
      m_point_cloud_overlay_toolbar->Undo_Edit ( );
  });
  m_view->Set_On_Point_Cloud_Redo ([this]
  {
    if( m_point_cloud_overlay_toolbar )
      m_point_cloud_overlay_toolbar->Redo_Edit ( );
  });
  m_view->Set_On_Flange_Drag_State_Changed ([this] (bool dragging)
  {
    if( m_point_cloud_overlay_toolbar )
      m_point_cloud_overlay_toolbar->Set_Interactive_LOD (dragging);
    if( m_interaction_coordinate_choice )
      m_interaction_coordinate_choice->Enable (!dragging);
  });
  m_view->Set_On_Flange_Drag_Performance (
    [this] (const robot_model::Robot_Drag_Performance_Stats& stats)
    {
      if( !m_status_text || stats.update_count == 0 ) return;
      const double count = static_cast<double> (stats.update_count);
      m_status_text->SetLabel (wxString::Format (
        wxString::FromUTF8 (
          u8"拖动性能：更新 %zu，平均 %.2f ms，最大 %.2f ms；IK %.2f，碰撞 %.2f，渲染 %.2f ms；姿态 %zu，候选/距离=%zu/%zu，精确网格 %zu；阻挡 点云/自碰/地面=%zu/%zu/%zu"),
        stats.update_count,
        stats.total_update_time_ms / count,
        stats.maximum_update_time_ms,
        stats.total_ik_time_ms / count,
        stats.total_collision_time_ms / count,
        stats.total_render_time_ms / count,
        stats.checked_pose_count,
        stats.obstacle_candidate_points,
        stats.obstacle_distance_queries,
        stats.self_exact_pair_queries,
        stats.obstacle_blocked_update_count,
        stats.self_blocked_update_count,
        stats.ground_blocked_update_count));
    });
  auto* robot_tool_page = Build_Robot_Tool_Page (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Robot));
  auto* teach_tool_page = Build_Teach_Tool_Page (
    m_right_tool_panel->Page_Parent (Right_Tool_Page::Teach));
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Robot, robot_tool_page);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Teach, teach_tool_page);
  m_right_tool_panel->Add_Page (Right_Tool_Page::Tcp, m_tcp_panel);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Run, m_run_progress_panel);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Camera, m_camera_control_panel);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Camera2D, m_camera_2d_control_panel);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Template2D, m_camera_2d_template_panel);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::PointCloud, m_point_cloud_overlay_toolbar);
  m_right_tool_panel->Add_Page (
    Right_Tool_Page::Tool, m_tool_panel);
  m_camera_control_panel->Set_On_Availability_Changed (
    [this] (bool enabled)
    {
      m_right_tool_panel->Set_Camera_Tool_Enabled (enabled);
      if( m_point_cloud_overlay_toolbar )
      {
        m_point_cloud_overlay_toolbar->Set_Camera_Connected (enabled);
      }
    });
  m_camera_2d_control_panel->Set_On_Availability_Changed (
    [this] (bool enabled)
    {
      if( m_right_tool_panel )
        m_right_tool_panel->Set_Template_2D_Tool_Enabled (enabled);
    });
  m_right_tool_panel->Set_Robot_Tool_Enabled (false);
  m_content_splitter->SplitVertically (
    display_panel,
    m_right_tool_panel,
    -RIGHT_TOOL_COLLAPSED_WIDTH);
  m_workspace_splitter->SplitVertically (
    m_teach_point_list_panel,
    m_content_splitter,
    TEACH_POINT_DEFAULT_EXPANDED_WIDTH);

  m_trajectory_timer.SetOwner (this);
  this->Bind (wxEVT_TIMER, &Robot_Model_Panel_Controller::On_Trajectory_Timer, this,
        m_trajectory_timer.GetId ( ));
  m_run_timer.SetOwner (this);
  this->Bind (wxEVT_TIMER, &Robot_Model_Panel_Controller::On_Run_Timer, this,
        m_run_timer.GetId ( ));

  auto* toolbar_sizer = new wxBoxSizer (wxHORIZONTAL);
  toolbar_sizer->AddStretchSpacer (1);
  toolbar_sizer->Add (m_model_name_text, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 12);

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (toolbar_sizer, 0, wxEXPAND | wxALL, 6);
  sizer->Add (
    m_workspace_splitter,
    1,
    wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM,
    6);
  this->SetSizer (sizer);

  Select_Display_Page (Main_Display_Page::Robot);

  std::string tool_error;
  if( !robot_model::Load_Tool_Coordinate_Configuration (
        robot_model::Tool_Coordinate_Config_Path ( ),
        &m_tool_configuration,
        &tool_error) )
  {
    m_tool_configuration =
      robot_model::Default_Tool_Coordinate_Configuration ( );
  }
  Apply_Active_Tool ( );

  std::string tool_visualization_error;
  if( !robot_model::Load_Tool_Visualization_Configuration (
        robot_model::Tool_Visualization_Config_Path ( ),
        &m_tool_visualization_configuration,
        &tool_visualization_error) )
  {
    m_tool_visualization_configuration = { };
  }
  if( m_tool_panel )
  {
    m_tool_panel->Set_Configuration (
      m_tool_visualization_configuration);
  }
  Apply_Tool_Visualization ( );

  Load_Model_List ( );
  std::string connection_error;
  if( !m_robot_connection->Load_Configuration(&connection_error) )
  {
    if( m_status_text )
      m_status_text->SetLabel (wxString::FromUTF8 (connection_error.c_str ( )));
  }
  Refresh_Kuka_Status_Table ( );
}

Robot_Model_Panel_Controller::~Robot_Model_Panel_Controller()
{
  if( m_camera_2d_preview_frame )
  {
    m_camera_2d_preview_frame->SetEvtHandlerEnabled (false);
    m_camera_2d_preview_frame->Destroy ( );
    m_camera_2d_preview_frame = nullptr;
    m_camera_preview_book = nullptr;
  }
  if( m_run_timer.IsRunning() )
    m_run_timer.Stop();
  if( m_run_image_processing_thread.joinable() )
    m_run_image_processing_thread.join();
  if( m_robot_connection )
  {
    m_robot_connection->Unsubscribe(m_robot_connection_observer_token);
    m_robot_connection_observer_token = 0;
    m_robot_connection->Disconnect();
  }
  this->DeletePendingEvents ( );
  this->Unbind (
    wxEVT_KUKA_MODEL_STATE,
    &Robot_Model_Panel_Controller::On_Kuka_Model_State,
    this);
  this->Unbind (
    wxEVT_KUKA_SERVICE_STATUS,
    &Robot_Model_Panel_Controller::On_Kuka_Service_Status,
    this);
  this->Unbind (
    wxEVT_RUN_IMAGE_PROCESSING_RESULT,
    &Robot_Model_Panel_Controller::On_Run_Image_Processing_Result,
    this);
  this->Unbind (
    wxEVT_TIMER,
    &Robot_Model_Panel_Controller::On_Run_Timer,
    this,
    m_run_timer.GetId());
}

void Robot_Model_Panel_Controller::Initialize_After_Layout()
{
  if( m_initialization_completed ) return;

  // The host invokes this from CallAfter, once the final parent hierarchy has
  // been laid out and the top-level frame has been shown.  Creating VTK's
  // native OpenGL resources in this controller's constructor is unreliable on
  // Windows, especially with a wxGLCanvas nested inside a wxSimplebook.
  this->Layout();
  if( m_display_book ) m_display_book->Layout();
  Load_Default_Model ( );
  m_initialization_completed = true;
  if( m_view )
  {
    m_view->Refresh(false);
    m_view->Render_Scene ( );
  }
}

void Robot_Model_Panel_Controller::On_Kuka_Model_State(wxThreadEvent &event)
{
  const auto payload =
    event.GetPayload<std::pair<
      application::Robot_Actual_State, std::uint64_t>>();
  Apply_Robot_Actual_State(payload.first);
}

void Robot_Model_Panel_Controller::On_Kuka_Service_Status(wxThreadEvent &event)
{
  Refresh_Robot_Command_Controls(
    event.GetPayload<application::Robot_Connection_Status>());
}

void Robot_Model_Panel_Controller::Apply_Robot_Actual_State(
  const application::Robot_Actual_State &state)
{
  Refresh_Kuka_Status_Table ( );
  if( m_hardware_step_preview_active ) return;
  if( !m_view || !m_view->Has_Current_Model ( ) ) return;

  const auto joint_state = robot_model::Build_Joint_State_From_Input_Angles (
    m_view->Kinematic_Params ( ), state.axis);
  // This is measured hardware state, not a proposed target. Always display
  // it even if the local collision guard would reject a commanded move.
  m_view->Set_Joint_State (joint_state);
  Sync_Joint_Controls_From_State ( );

  if( m_status_text )
  {
    m_status_text->SetLabel (wxString::Format (
      "KUKA actual state synchronized: "
      "A=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
      state.axis[0], state.axis[1], state.axis[2],
      state.axis[3], state.axis[4], state.axis[5]));
  }
  Refresh_Run_Readiness();
}

void Robot_Model_Panel_Controller::Refresh_Robot_Command_Controls(
  const application::Robot_Connection_Status &status)
{
  m_robot_connection_status = status;
  if( m_progress_run_controller.Is_Motion_Active() )
  {
    if( m_progress_run_controller.State() ==
          application::Progress_Run_State::Stopping &&
        ( status.state == application::Robot_Control_State::Ready ||
          status.state == application::Robot_Control_State::Fault ||
          status.state == application::Robot_Control_State::Disconnected ) )
    {
      Apply_Progress_Run_Transition(
        m_progress_run_controller.Stop_Confirmed());
      return;
    }
    if( m_progress_run_controller.State() !=
          application::Progress_Run_State::Stopping &&
        ( status.state == application::Robot_Control_State::Fault ||
          status.state == application::Robot_Control_State::Disconnected ) )
    {
      Apply_Progress_Run_Transition(
        m_progress_run_controller.Robot_Became_Unavailable(
        status.state == application::Robot_Control_State::Disconnected
          ? "机械臂连接断开，Progress 已中止"
          : "机械臂故障，Progress 已中止：" + status.detail));
      return;
    }
    if( m_progress_run_controller.State() ==
          application::Progress_Run_State::Moving &&
        status.state == application::Robot_Control_State::Ready )
    {
      Apply_Progress_Run_Transition(
        m_progress_run_controller.Motion_Completed());
    }
  }
  const bool finish_hardware_preview =
    m_hardware_step_preview_active &&
    ( status.state == application::Robot_Control_State::Ready ||
      status.state == application::Robot_Control_State::Fault ||
      status.state == application::Robot_Control_State::Disconnected );
  if( finish_hardware_preview )
  {
    m_hardware_step_preview_active = false;
    Stop_Trajectory_Playback ( );
    application::Robot_Actual_State latest_state;
    if( m_robot_connection->Latest_State(&latest_state) )
      Apply_Robot_Actual_State(latest_state);
  }
  const bool ready = status.state == application::Robot_Control_State::Ready;
  const bool connected =
    status.state != application::Robot_Control_State::Disconnected;
  const bool busy =
    status.state == application::Robot_Control_State::Command_Sent ||
    status.state == application::Robot_Control_State::Running ||
    status.state == application::Robot_Control_State::Completed;

  const bool safe_controls =
    !m_progress_run_controller.Is_Motion_Active();
  if( m_kuka_move_joint_button )
    m_kuka_move_joint_button->Enable (safe_controls && ready);
  if( m_kuka_move_ptp_button )
    m_kuka_move_ptp_button->Enable (safe_controls && ready);
  if( m_kuka_move_linear_button )
    m_kuka_move_linear_button->Enable (safe_controls && ready);
  if( m_kuka_sync_button )
    m_kuka_sync_button->Enable (safe_controls && connected && !busy);
  if( m_kuka_stop_button )
    m_kuka_stop_button->Enable (safe_controls && connected);
  if( m_kuka_ptp_velocity_slider )
    m_kuka_ptp_velocity_slider->Enable (safe_controls && !busy);
  if( m_kuka_linear_velocity_slider )
    m_kuka_linear_velocity_slider->Enable (safe_controls && !busy);
  if( m_kuka_acceleration_slider )
    m_kuka_acceleration_slider->Enable (safe_controls && !busy);
  if( m_kuka_connect_button )
  {
    m_kuka_connect_button->SetLabel (
      connected ? "Disconnect" :
        (m_robot_connection->Is_Connecting() ? "Connecting..." : "Connect"));
    m_kuka_connect_button->Enable (
      safe_controls && (!m_robot_connection->Is_Connecting() || connected));
  }
  if( m_joint_panel )
    m_joint_panel->Set_Joint_Controls_Enabled (safe_controls && !busy);
  if( m_cartesian_pose_panel )
    m_cartesian_pose_panel->Set_Pose_Controls_Enabled (
      safe_controls && !busy);

  if( m_kuka_status_text )
  {
    wxString label =
      "KUKA: " + wxString::FromUTF8(application::To_String(status.state));
    if( status.active_sequence != 0 )
      label += wxString::Format ("  seq=%u", status.active_sequence);
    if( !status.detail.empty ( ) )
      label += "  " + wxString::FromUTF8 (status.detail);
    m_kuka_status_text->SetLabel (label);
  }
  Refresh_Kuka_Status_Table ( );
  Refresh_Run_Readiness();
}

void Robot_Model_Panel_Controller::Refresh_Kuka_Status_Table ( )
{
  if( !m_kuka_status_table ) return;
  const bool connected =
    m_robot_connection->Is_Connected();
  const auto set_value = [this] (long row, const wxString& value)
  {
    m_kuka_status_table->SetCellValue (
      static_cast<int> (row), 1, value);
  };
  set_value (0, connected ? "Connected" :
    (m_robot_connection->Is_Connecting() ? "Connecting" : "Disconnected"));
  set_value (
    1,
    wxString::FromUTF8(
      application::To_String(m_robot_connection_status.state)));
}

void Robot_Model_Panel_Controller::On_Kuka_Connect (wxCommandEvent&)
{
  if( m_robot_connection->Is_Connected() ||
      m_robot_connection->Is_Connecting() )
  {
    m_robot_connection->Disconnect();
    Refresh_Robot_Command_Controls(m_robot_connection->Status());
    return;
  }

  std::string error_message;
  if( !Load_Bound_Robot_Model (&error_message) )
  {
    m_status_text->SetLabel (
      "KUKA connect failed: " + wxString::FromUTF8 (error_message.c_str ( )));
    return;
  }

  try
  {
    m_robot_connection->Connect();
    Refresh_Robot_Command_Controls(m_robot_connection->Status());
  }
  catch( const std::exception& error )
  {
    m_status_text->SetLabel (
      "KUKA connect failed: " + wxString::FromUTF8 (error.what ( )));
    Refresh_Robot_Command_Controls(m_robot_connection->Status());
  }
}

void Robot_Model_Panel_Controller::On_Robot_Display (wxCommandEvent&)
{
  Select_Display_Page (Main_Display_Page::Robot);
}

void Robot_Model_Panel_Controller::On_Camera_Image_Display (wxCommandEvent&)
{
  Select_Display_Page (Main_Display_Page::Camera_Image);
}

void Robot_Model_Panel_Controller::On_Camera_2D_Image_Display (wxCommandEvent&)
{
  Select_Display_Page (Main_Display_Page::Camera_2D_Image);
}

void Robot_Model_Panel_Controller::On_Point_Cloud_Display (wxCommandEvent&)
{
  Select_Display_Page (Main_Display_Page::Point_Cloud);
}

void Robot_Model_Panel_Controller::On_Reset_Robot (wxCommandEvent&)
{
  if( Reset_Robot_To_Home ( ) && m_status_text )
    m_status_text->SetLabel (wxString::FromUTF8 (u8"机械臂已复位"));
}

bool Robot_Model_Panel_Controller::Reset_Robot_To_Home ( )
{
  if( !m_view || !m_view->Has_Current_Model ( ) ) return false;
  if( Is_Trajectory_Active ( ) ) Stop_Trajectory_Playback ( );

  const auto& params = m_view->Kinematic_Params ( );
  if( params.has_home_pose )
  {
    // Seed pose IK with the configured reset joint solution so reset remains
    // deterministic and converges to the precise Cartesian target.
    Apply_Joint_Input_Angles_To_Sliders (
      configured_home_input_angles (params));
    return Apply_Cartesian_Pose_Target (params.home_pose_xyzabc);
  }

  Apply_Joint_Input_Angles_To_Sliders (
    configured_home_input_angles (params));
  Select_Display_Page (Main_Display_Page::Robot);
  return true;
}

bool Robot_Model_Panel_Controller::Set_Camera_Pose_Visible (bool visible)
{
  if( !visible )
  {
    m_camera_pose_controller.Hide ( );
    m_status_text->SetLabel (wxString::FromUTF8 (u8"相机位姿已隐藏"));
    m_view->Render_Scene ( );
    return true;
  }

  std::string error;
  if( !m_camera_pose_controller.Show (m_view->Scene_Renderer ( ), &error) )
  {
    wxMessageBox (
      wxString::FromUTF8 (error.c_str ( )),
      wxString::FromUTF8 (u8"相机位姿加载失败"),
      wxOK | wxICON_ERROR,
      this);
    return false;
  }

  Select_Display_Page (Main_Display_Page::Robot);
  m_status_text->SetLabel (wxString::FromUTF8 (u8"相机位姿已显示"));
  m_view->Render_Scene ( );
  return true;
}

void Robot_Model_Panel_Controller::On_Toggle_Flange_Frame (wxCommandEvent&)
{
  const bool visible =
    m_flange_frame_button && m_flange_frame_button->GetValue ( );
  m_view->Set_Flange_Frame_Visible (visible);
  if( visible && !m_view->Has_Flange_Pose ( ) )
  {
    m_flange_frame_button->SetValue (false);
    m_flange_frame_button->SetLabel (
      wxString::FromUTF8 (u8"显示法兰坐标系"));
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"当前模型没有可用的法兰位姿"));
    return;
  }

  m_flange_frame_button->SetLabel (
    visible
      ? wxString::FromUTF8 (u8"隐藏法兰坐标系")
      : wxString::FromUTF8 (u8"显示法兰坐标系"));
  m_status_text->SetLabel (
    visible
      ? wxString::FromUTF8 (u8"法兰坐标系已显示")
      : wxString::FromUTF8 (u8"法兰坐标系已隐藏"));

  if( !visible && m_flange_6d_button && m_flange_6d_button->GetValue ( ) )
  {
    Set_Flange_Interaction_Mode (
      Robot_Model_View::Flange_Interaction_Mode::Observe);
  }
}

void Robot_Model_Panel_Controller::On_Toggle_Flange_Free_Drag (wxCommandEvent&)
{
  const bool enabled = m_flange_free_drag_button &&
    m_flange_free_drag_button->GetValue ( );
  if( enabled && !m_view->Has_Flange_Pose ( ) )
  {
    m_flange_free_drag_button->SetValue (false);
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"当前模型没有可拖拽的法兰位姿"));
    return;
  }

  if( enabled && Is_Trajectory_Active ( ) )
  {
    Stop_Trajectory_Playback ( );
  }
  if( enabled )
  {
    Select_Display_Page (Main_Display_Page::Robot);
    Set_Flange_Interaction_Mode (
      Robot_Model_View::Flange_Interaction_Mode::Free_Translation);
  }
  else
  {
    Set_Flange_Interaction_Mode (
      Robot_Model_View::Flange_Interaction_Mode::Observe);
  }
  m_status_text->SetLabel (
    enabled
      ? wxString::FromUTF8 (u8"自由拖拽：拖动黄色手柄")
      : wxString::FromUTF8 (u8"观察模式"));
}

void Robot_Model_Panel_Controller::On_Toggle_Flange_6D (wxCommandEvent&)
{
  const bool enabled = m_flange_6d_button && m_flange_6d_button->GetValue ( );
  if( enabled && !m_view->Has_Flange_Pose ( ) )
  {
    m_flange_6d_button->SetValue (false);
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"当前模型没有可操纵的法兰位姿"));
    return;
  }
  if( enabled && Is_Trajectory_Active ( ) ) Stop_Trajectory_Playback ( );
  if( enabled ) Select_Display_Page (Main_Display_Page::Robot);
  Set_Flange_Interaction_Mode (enabled
    ? Robot_Model_View::Flange_Interaction_Mode::Transform_6D
    : Robot_Model_View::Flange_Interaction_Mode::Observe);
  m_status_text->SetLabel (enabled
    ? wxString::FromUTF8 (u8"6D 操纵：彩色轴端平移，圆环旋转")
    : wxString::FromUTF8 (u8"观察模式"));
}

void Robot_Model_Panel_Controller::Set_Flange_Interaction_Mode (
  Robot_Model_View::Flange_Interaction_Mode mode)
{
  const bool free_translation =
    mode == Robot_Model_View::Flange_Interaction_Mode::Free_Translation;
  const bool transform_6d =
    mode == Robot_Model_View::Flange_Interaction_Mode::Transform_6D;
  m_flange_free_drag_button->SetValue (free_translation);
  m_flange_6d_button->SetValue (transform_6d);
  m_flange_free_drag_button->SetLabel (wxString::FromUTF8 (
    free_translation ? u8"停止自由拖拽" : u8"自由拖拽"));
  m_flange_6d_button->SetLabel (wxString::FromUTF8 (
    transform_6d ? u8"停止 6D 操纵" : u8"6D 操纵"));
  m_view->Set_Flange_Interaction_Mode (mode);

  if( free_translation )
  {
    m_view->Set_Flange_Frame_Visible (false);
    m_flange_frame_button->SetValue (false);
    m_flange_frame_button->SetLabel (
      wxString::FromUTF8 (u8"显示法兰坐标系"));
  }
  else if( transform_6d )
  {
    m_view->Set_Flange_Frame_Visible (true);
    m_flange_frame_button->SetValue (true);
    m_flange_frame_button->SetLabel (
      wxString::FromUTF8 (u8"隐藏法兰坐标系"));
  }
}

void Robot_Model_Panel_Controller::Select_Display_Page (Main_Display_Page page)
{
  if( m_display_book )
    m_display_book->SetSelection (0);
  m_display_page = Main_Display_Page::Robot;
  if( page != Main_Display_Page::Robot )
  {
    Select_Camera_Preview (page);
    if( m_camera_preview_state == Camera_Preview_State::Launcher )
      m_camera_preview_state = Camera_Preview_State::Quarter;
    m_camera_preview_menu_visible = false;
  }
  Layout_Camera_2D_Preview ( );
  Update_Display_Menu ( );
}

void Robot_Model_Panel_Controller::Layout_Camera_2D_Preview ( )
{
  if( !m_display_book || !m_display_panel )
    return;
  Ensure_Camera_2D_Floating_Preview ( );
  if( !m_camera_2d_preview_frame ) return;

  const wxRect area = m_display_book->GetRect ( );
  if( area.width <= 0 || area.height <= 0 ) return;

  const bool launcher =
    m_camera_preview_state == Camera_Preview_State::Launcher;
  const bool full = m_camera_preview_state == Camera_Preview_State::Full;
  const bool show_menu = launcher && m_camera_preview_menu_visible;

  if( m_camera_preview_book ) m_camera_preview_book->Show (!launcher);
  if( m_camera_preview_launcher_button )
    m_camera_preview_launcher_button->Show (launcher);
  if( m_camera_preview_3d_button )
    m_camera_preview_3d_button->Show (!launcher || show_menu);
  if( m_camera_preview_2d_button )
    m_camera_preview_2d_button->Show (!launcher || show_menu);
  if( m_camera_preview_cloud_button )
    m_camera_preview_cloud_button->Show (!launcher || show_menu);
  if( m_camera_2d_preview_toggle )
    m_camera_2d_preview_toggle->Show (!launcher);
  if( m_camera_preview_collapse_button )
    m_camera_preview_collapse_button->Show (!launcher);
  m_camera_2d_preview_frame->Layout ( );

  constexpr int margin = 12;
  int width = 0;
  int height = 0;
  int x = area.x;
  int y = area.y;
  if( launcher )
  {
    width = show_menu ? std::min (460, area.width) : std::min (130, area.width);
    height = std::min (42, area.height);
    x = area.x + margin;
    y = area.y + area.height - height - margin;
  }
  else if( full )
  {
    width = area.width;
    height = area.height;
  }
  else
  {
    width = std::min (area.width, std::max (360, area.width / 2));
    height = std::min (area.height, std::max (260, area.height / 2));
    x = area.x + std::max (0, area.width - width - margin);
    y = area.y + margin;
  }
  const wxPoint screen_position =
    m_display_panel->ClientToScreen (wxPoint (x, y));
  m_camera_2d_preview_frame->SetSize (
    screen_position.x, screen_position.y, width, height);
  if( m_camera_2d_preview_toggle )
  {
    m_camera_2d_preview_toggle->SetLabel (wxString::FromUTF8 (
      full ? u8"恢复1/4" : u8"100%显示"));
  }
  m_camera_2d_preview_frame->Show ( );
  m_camera_2d_preview_frame->Raise ( );
}

void Robot_Model_Panel_Controller::Toggle_Camera_2D_Preview ( )
{
  if( m_camera_preview_state == Camera_Preview_State::Quarter )
    m_camera_preview_state = Camera_Preview_State::Full;
  else if( m_camera_preview_state == Camera_Preview_State::Full )
    m_camera_preview_state = Camera_Preview_State::Quarter;
  Layout_Camera_2D_Preview ( );
}

void Robot_Model_Panel_Controller::Collapse_Camera_2D_Preview ( )
{
  m_camera_preview_state = Camera_Preview_State::Launcher;
  m_camera_preview_menu_visible = false;
  Layout_Camera_2D_Preview ( );
}

void Robot_Model_Panel_Controller::Hide_Camera_Preview_Menu_If_Pointer_Left ( )
{
  if( m_camera_preview_state != Camera_Preview_State::Launcher ||
      !m_camera_preview_menu_visible || !m_camera_2d_preview_frame )
    return;
  if( m_camera_2d_preview_frame->GetScreenRect ( ).Contains (
        wxGetMousePosition ()))
    return;
  m_camera_preview_menu_visible = false;
  Layout_Camera_2D_Preview ( );
}

void Robot_Model_Panel_Controller::Ensure_Camera_2D_Floating_Preview ( )
{
  if( m_camera_2d_preview_frame || !m_camera_service ||
      !m_camera_2d_service || !m_camera_2d_template_service )
    return;

  m_camera_2d_preview_frame = new wxMiniFrame (
    wxGetTopLevelParent (this),
    wxID_ANY,
    wxEmptyString,
    wxDefaultPosition,
    wxDefaultSize,
    wxBORDER_SIMPLE | wxFRAME_FLOAT_ON_PARENT | wxFRAME_NO_TASKBAR);
  m_camera_preview_book = new wxSimplebook (
    m_camera_2d_preview_frame, wxID_ANY);
  m_camera_image_view = new Camera_Image_View (
    m_camera_preview_book, *m_camera_service);
  m_camera_2d_image_view = new Camera_2D_Image_View (
    m_camera_preview_book,
    *m_camera_2d_service,
    *m_camera_2d_template_service);
  m_point_cloud_view = new Point_Cloud_View (
    m_camera_preview_book, *m_camera_service);
  m_camera_preview_book->AddPage (m_camera_image_view, wxEmptyString, false);
  m_camera_preview_book->AddPage (m_camera_2d_image_view, wxEmptyString, true);
  m_camera_preview_book->AddPage (m_point_cloud_view, wxEmptyString, false);

  m_camera_preview_3d_button = new wxButton (
    m_camera_2d_preview_frame, wxID_ANY, wxString::FromUTF8 (u8"3D图像"));
  m_camera_preview_2d_button = new wxButton (
    m_camera_2d_preview_frame, wxID_ANY, wxString::FromUTF8 (u8"2D图像"));
  m_camera_preview_cloud_button = new wxButton (
    m_camera_2d_preview_frame, wxID_ANY, wxString::FromUTF8 (u8"点云"));
  m_camera_preview_launcher_button = new wxButton (
    m_camera_2d_preview_frame, wxID_ANY, wxString::FromUTF8 (u8"画面预览"));
  m_camera_2d_preview_toggle = new wxButton (
    m_camera_2d_preview_frame, wxID_ANY, wxString::FromUTF8 (u8"100%显示"));
  m_camera_preview_collapse_button = new wxButton (
    m_camera_2d_preview_frame, wxID_ANY, wxString::FromUTF8 (u8"收起"));
  auto* controls = new wxBoxSizer (wxHORIZONTAL);
  controls->Add (m_camera_preview_launcher_button, 1, wxEXPAND | wxRIGHT, 3);
  controls->Add (m_camera_preview_3d_button, 1, wxEXPAND | wxRIGHT, 3);
  controls->Add (m_camera_preview_2d_button, 1, wxEXPAND | wxRIGHT, 3);
  controls->Add (m_camera_preview_cloud_button, 1, wxEXPAND | wxRIGHT, 3);
  controls->Add (m_camera_2d_preview_toggle, 1, wxEXPAND | wxRIGHT, 3);
  controls->Add (m_camera_preview_collapse_button, 1, wxEXPAND);
  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (m_camera_preview_book, 1, wxEXPAND);
  sizer->Add (controls, 0, wxEXPAND | wxALL, 4);
  m_camera_2d_preview_frame->SetSizer (sizer);
  m_camera_preview_3d_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&) {
      Select_Camera_Preview (Main_Display_Page::Camera_Image);
      if( m_camera_preview_state == Camera_Preview_State::Launcher )
        m_camera_preview_state = Camera_Preview_State::Quarter;
      m_camera_preview_menu_visible = false;
      Layout_Camera_2D_Preview ( ); });
  m_camera_preview_2d_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&) {
      Select_Camera_Preview (Main_Display_Page::Camera_2D_Image);
      if( m_camera_preview_state == Camera_Preview_State::Launcher )
        m_camera_preview_state = Camera_Preview_State::Quarter;
      m_camera_preview_menu_visible = false;
      Layout_Camera_2D_Preview ( ); });
  m_camera_preview_cloud_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&) {
      Select_Camera_Preview (Main_Display_Page::Point_Cloud);
      if( m_camera_preview_state == Camera_Preview_State::Launcher )
        m_camera_preview_state = Camera_Preview_State::Quarter;
      m_camera_preview_menu_visible = false;
      Layout_Camera_2D_Preview ( ); });
  m_camera_preview_launcher_button->Bind (
    wxEVT_ENTER_WINDOW,
    [this] (wxMouseEvent& event)
    {
      m_camera_preview_menu_visible = true;
      Layout_Camera_2D_Preview ( );
      event.Skip ( );
    });
  m_camera_2d_preview_toggle->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&) { Toggle_Camera_2D_Preview ( ); });
  m_camera_preview_collapse_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&) { Collapse_Camera_2D_Preview ( ); });

  const auto bind_menu_leave = [this] (wxWindow* window)
  {
    window->Bind (
      wxEVT_LEAVE_WINDOW,
      [this] (wxMouseEvent& event)
      {
        CallAfter ([this] { Hide_Camera_Preview_Menu_If_Pointer_Left ( ); });
        event.Skip ( );
      });
  };
  bind_menu_leave (m_camera_2d_preview_frame);
  bind_menu_leave (m_camera_preview_launcher_button);
  bind_menu_leave (m_camera_preview_3d_button);
  bind_menu_leave (m_camera_preview_2d_button);
  bind_menu_leave (m_camera_preview_cloud_button);
  m_camera_2d_preview_frame->Bind (
    wxEVT_CLOSE_WINDOW,
    [this] (wxCloseEvent& event)
    {
      event.Veto ( );
      Collapse_Camera_2D_Preview ( );
    });
  Select_Camera_Preview (m_camera_preview_page);
}

void Robot_Model_Panel_Controller::Select_Camera_Preview (
  Main_Display_Page page)
{
  if( !m_camera_preview_book ) return;
  int selection = 1;
  if( page == Main_Display_Page::Camera_Image ) selection = 0;
  else if( page == Main_Display_Page::Point_Cloud ) selection = 2;
  else page = Main_Display_Page::Camera_2D_Image;
  m_camera_preview_page = page;
  m_camera_preview_book->SetSelection (selection);
  const wxColour selected_colour (210, 230, 250);
  if( m_camera_preview_3d_button )
    m_camera_preview_3d_button->SetBackgroundColour (
      selection == 0 ? selected_colour : wxNullColour);
  if( m_camera_preview_2d_button )
    m_camera_preview_2d_button->SetBackgroundColour (
      selection == 1 ? selected_colour : wxNullColour);
  if( m_camera_preview_cloud_button )
    m_camera_preview_cloud_button->SetBackgroundColour (
      selection == 2 ? selected_colour : wxNullColour);
}

void Robot_Model_Panel_Controller::Update_Display_Menu ( )
{
  if( m_robot_display_button )
  {
    m_robot_display_button->SetValue (
      m_display_page == Main_Display_Page::Robot);
  }
  if( m_camera_display_button )
  {
    m_camera_display_button->SetValue (
      m_display_page == Main_Display_Page::Camera_Image);
  }
  if( m_camera_2d_display_button )
  {
    m_camera_2d_display_button->SetValue (
      m_display_page == Main_Display_Page::Camera_2D_Image);
  }
  if( m_point_cloud_display_button )
  {
    m_point_cloud_display_button->SetValue (
      m_display_page == Main_Display_Page::Point_Cloud);
  }
}

void Robot_Model_Panel_Controller::Show_Model_Configuration (wxWindow* parent)
{
  Robot_Model_Config_Dialog dialog (
    parent ? parent : this,
    m_models,
    m_current_model_id,
    m_tool_configuration,
    m_robot_connection->Configuration());
  if( dialog.ShowModal ( ) != wxID_OK )
  {
    return;
  }

  if( dialog.Tool_Apply_Requested ( ) )
  {
    std::string error_message;
    if( !robot_model::Save_Tool_Coordinate_Configuration (
          robot_model::Tool_Coordinate_Config_Path ( ),
          dialog.Tool_Configuration ( ),
          &error_message) )
    {
      wxMessageBox (
        wxString::FromUTF8 (error_message.c_str ( )),
        wxString::FromUTF8 (u8"工具坐标保存失败"),
        wxOK | wxICON_ERROR,
        parent ? parent : this);
      return;
    }
    m_tool_configuration = dialog.Tool_Configuration ( );
    Invalidate_Completed_Progress ( );
    Apply_Active_Tool ( );
    Update_Cartesian_Pose ( );
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"当前工具坐标：") +
      wxString::FromUTF8 (Active_Tool ( ).name.c_str ( )));
    return;
  }

  if( dialog.Connection_Save_Requested ( ) )
  {
    std::string error_message;
    if( !m_robot_connection->Save_Configuration(
          dialog.Connection_Configuration(), &error_message) )
    {
      wxMessageBox (
        wxString::FromUTF8 (error_message.c_str ( )),
        "Connection settings",
        wxOK | wxICON_ERROR,
        parent ? parent : this);
      return;
    }
    if( !Load_Bound_Robot_Model (&error_message) )
    {
      wxMessageBox (
        wxString::FromUTF8 (error_message.c_str ( )),
        "Bound robot model",
        wxOK | wxICON_ERROR,
        parent ? parent : this);
      return;
    }
    Refresh_Robot_Command_Controls(m_robot_connection->Status());
    m_status_text->SetLabel ("Robot connection settings saved");
    return;
  }

  if( dialog.Model_Load_Requested ( ) )
  {
    wxBusyCursor busy_cursor;
    std::string error_message;
    if( !Load_Model (dialog.Selected_Model_Index ( ), &error_message) )
    {
      m_status_text->SetLabel (wxString::FromUTF8 (u8"机械臂模型加载失败"));
      wxMessageBox (
        wxString::FromUTF8 (error_message.c_str ( )),
        wxString::FromUTF8 (u8"机械臂加载失败"),
        wxOK | wxICON_ERROR,
        parent ? parent : this);
    }
  }
}

void Robot_Model_Panel_Controller::Refresh_Template_Configuration ( )
{
  if( m_point_cloud_overlay_toolbar )
  {
    m_point_cloud_overlay_toolbar->Refresh_Template_Configuration ( );
  }
}

void Robot_Model_Panel_Controller::On_Add_Trajectory_Point (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) || m_current_model_id.empty ( ) || !m_view )
  {
    return;
  }

  std::array<double, 6> joint_angles = { };
  robot_model::XyzabcPose world_pose = { };
  if( !Read_Current_Teach_Point (&joint_angles, &world_pose) ) return;
  std::string point_cloud_path;
  std::string point_cloud_name;
  robot_model::Tool_Coordinate_Profile coordinate;
  robot_model::Template_Profile template_profile;
  if( !Capture_Current_Teach_Bindings (
        &point_cloud_path, &point_cloud_name, &coordinate,
        &template_profile) ) return;
  const auto& point = m_teach_point_store.Add_Point (
    m_current_model_id,
    joint_angles,
    world_pose,
    point_cloud_path,
    coordinate.id,
    coordinate.name,
    coordinate.flange_from_tool_pose,
    point_cloud_name,
      m_teach_point_command_panel
      ? m_teach_point_command_panel->Selected_Point_Type ( )
      : robot_model::Robot_Teach_Point_Type::Motion,
    template_profile.id,
    template_profile.name,
    template_profile.reference_pose);
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Point_Selection (
      static_cast<int> (
        m_teach_point_store.Point_Count (m_current_model_id) - 1));
    Update_Teach_Point_Details ( );
  }
  Set_Progress_Dirty (true);
  Update_Trajectory_Status ( );
  m_status_text->SetLabel (wxString::FromUTF8 (u8"已记录示教点：") +
    wxString::FromUTF8 (
      robot_model::Format_Teach_Point_Name (point.id).c_str ( )));
}

void Robot_Model_Panel_Controller::On_Update_Teach_Point (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) ) return;
  const int selection = Selected_Teach_Point_Index ( );
  if( selection == wxNOT_FOUND ) return;

  std::array<double, 6> joint_angles = { };
  robot_model::XyzabcPose world_pose = { };
  if( !Read_Current_Teach_Point (&joint_angles, &world_pose) ) return;
  std::string point_cloud_path;
  std::string point_cloud_name;
  robot_model::Tool_Coordinate_Profile coordinate;
  robot_model::Template_Profile template_profile;
  if( !Capture_Current_Teach_Bindings (
        &point_cloud_path, &point_cloud_name, &coordinate,
        &template_profile) ) return;
  if( !m_teach_point_store.Update_Point (
        m_current_model_id,
        static_cast<std::size_t> (selection),
        joint_angles,
        world_pose,
        point_cloud_path,
        coordinate.id,
        coordinate.name,
        coordinate.flange_from_tool_pose,
        point_cloud_name,
        m_teach_point_command_panel
          ? m_teach_point_command_panel->Selected_Point_Type ( )
          : robot_model::Robot_Teach_Point_Type::Motion,
        template_profile.id,
        template_profile.name,
        template_profile.reference_pose) )
  {
    return;
  }
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  Set_Progress_Dirty (true);
  Update_Trajectory_Status ( );
  m_status_text->SetLabel (
    wxString::FromUTF8 (u8"当前示教点已更新"));
}

void Robot_Model_Panel_Controller::On_Insert_Teach_Point (bool before)
{
  if( Is_Trajectory_Active ( ) ) return;
  const int selection = Selected_Teach_Point_Index ( );
  if( selection == wxNOT_FOUND ) return;

  std::array<double, 6> joint_angles = { };
  robot_model::XyzabcPose world_pose = { };
  if( !Read_Current_Teach_Point (&joint_angles, &world_pose) ) return;
  std::string point_cloud_path;
  std::string point_cloud_name;
  robot_model::Tool_Coordinate_Profile coordinate;
  robot_model::Template_Profile template_profile;
  if( !Capture_Current_Teach_Bindings (
        &point_cloud_path, &point_cloud_name, &coordinate,
        &template_profile) ) return;
  const std::size_t insertion_index =
    static_cast<std::size_t> (selection) + (before ? 0 : 1);
  const auto& point = m_teach_point_store.Insert_Point (
    m_current_model_id,
    insertion_index,
    joint_angles,
    world_pose,
    point_cloud_path,
    coordinate.id,
    coordinate.name,
    coordinate.flange_from_tool_pose,
    point_cloud_name,
      m_teach_point_command_panel
      ? m_teach_point_command_panel->Selected_Point_Type ( )
      : robot_model::Robot_Teach_Point_Type::Motion,
    template_profile.id,
    template_profile.name,
    template_profile.reference_pose);
  const std::size_t point_id = point.id;
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Point_Selection (
      static_cast<int> (insertion_index));
    Update_Teach_Point_Details ( );
  }
  Set_Progress_Dirty (true);
  Update_Trajectory_Status ( );
  m_status_text->SetLabel (
    wxString::FromUTF8 (u8"已插入示教点：") +
    wxString::FromUTF8 (
      robot_model::Format_Teach_Point_Name (point_id).c_str ( )));
}

void Robot_Model_Panel_Controller::On_Clear_Trajectory_Points (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) )
  {
    return;
  }

  if( m_teach_point_store.Point_Count (m_current_model_id) == 0 ) return;
  if( wxMessageBox (
        wxString::FromUTF8 (u8"确定清空当前 Progress 吗？"),
        wxString::FromUTF8 (u8"清空 Progress"),
        wxYES_NO | wxNO_DEFAULT | wxICON_WARNING,
        this) != wxYES )
  {
    return;
  }
  m_teach_point_store.Clear_Points (m_current_model_id);
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  Set_Progress_Dirty (true);
  Update_Trajectory_Status ( );
}

void Robot_Model_Panel_Controller::On_Go_To_Trajectory_Point (wxCommandEvent&)
{
  const int selection = Selected_Teach_Point_Index ( );
  if( selection == wxNOT_FOUND ||
      selection < 0 ||
      static_cast<size_t> (selection) >= m_trajectory_session.Point_Count ( ) )
  {
    return;
  }
  Start_Ordered_Go_To_Teach_Point (
    static_cast<std::size_t> (selection));
}

bool Robot_Model_Panel_Controller::Start_Direct_Go_To_Teach_Point (
  std::size_t selection,
  bool apply_bindings,
  std::size_t frame_count_override)
{
  if( Is_Trajectory_Active ( ) ||
      selection >= m_trajectory_session.Point_Count ( ) )
  {
    return false;
  }

  if( apply_bindings &&
      !Apply_Teach_Point_Bindings_Keeping_Display (selection) )
  {
    return false;
  }
  const auto start_angles = Read_Joint_Input_Angles ( );
  std::array<double, 6> ignored_angles = { };
  robot_model::XyzabcPose start_pose = { };
  std::size_t frame_count = frame_count_override > 0
    ? frame_count_override
    : static_cast<std::size_t> (TRAJECTORY_FRAME_COUNT);
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( frame_count_override == 0 &&
      Read_Current_Teach_Point (&ignored_angles, &start_pose) &&
      selection < points.size ( ) &&
      points[selection].has_world_pose )
  {
    frame_count = frame_count_for_one_meter_per_second (
      start_pose,
      points[selection].world_pose);
  }
  if( !m_trajectory_session.Start_Go_To (
        start_angles,
        selection,
        frame_count) )
  {
    return false;
  }

  Set_Joint_Controls_Enabled (false);
  m_trajectory_timer.Start (TRAJECTORY_TIMER_MS);
  Update_Trajectory_Status ( );
  return true;
}

bool Robot_Model_Panel_Controller::Start_Ordered_Go_To_Teach_Point (
  std::size_t target_index)
{
  if( Is_Trajectory_Active ( ) ||
      Get_Trajectory_Speed_Mps ( ) <= 0.0 ||
      target_index >= m_trajectory_session.Point_Count ( ) )
  {
    return false;
  }

  const auto current_angles = Read_Joint_Input_Angles ( );
  const auto waypoint_indices =
    robot_model::Build_Ordered_Go_To_Point_Indices (
      m_trajectory_session.Points ( ),
      current_angles,
      target_index);
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( waypoint_indices.empty ( ) )
  {
    if( !Apply_Teach_Point_Bindings_Keeping_Display (target_index) )
      return false;
    if( m_teach_point_list_panel )
    {
      m_teach_point_list_panel->Set_Point_Selection (
        static_cast<int> (target_index));
      Update_Teach_Point_Details ( );
    }
    if( m_status_text )
    {
      m_status_text->SetLabel (wxString::Format (
        wxString::FromUTF8 (u8"当前已位于 P[%zu]"),
        points[target_index].id));
    }
    Update_Trajectory_Status ( );
    return true;
  }

  const std::size_t initial_point_index =
    waypoint_indices.front ( ) == 0
      ? 0
      : waypoint_indices.front ( ) - 1;
  if( !Apply_Teach_Point_Bindings_Keeping_Display (
        initial_point_index) )
  {
    return false;
  }

  std::array<double, 6> ignored_angles = { };
  robot_model::XyzabcPose previous_pose = { };
  bool has_previous_pose =
    Read_Current_Teach_Point (&ignored_angles, &previous_pose);
  std::vector<std::size_t> frame_counts;
  frame_counts.reserve (waypoint_indices.size ( ));
  for( const std::size_t point_index : waypoint_indices )
  {
    const auto& point = points[point_index];
    frame_counts.push_back (
      has_previous_pose && point.has_world_pose
        ? frame_count_for_one_meter_per_second (
            previous_pose, point.world_pose)
        : static_cast<std::size_t> (TRAJECTORY_FRAME_COUNT));
    if( point.has_world_pose )
    {
      previous_pose = point.world_pose;
      has_previous_pose = true;
    }
    else
    {
      has_previous_pose = false;
    }
  }

  if( !m_trajectory_session.Start_Go_To_Path (
        current_angles, waypoint_indices, frame_counts) )
  {
    return false;
  }

  m_playback_waypoint_frame_indices.clear ( );
  m_playback_waypoint_point_indices.clear ( );
  m_playback_cloud_switches.clear ( );
  std::size_t waypoint_frame_index = 0;
  std::size_t previous_point_index = initial_point_index;
  for( std::size_t path_index = 0;
       path_index < waypoint_indices.size ( );
       ++path_index )
  {
    const std::size_t point_index = waypoint_indices[path_index];
    if( points[previous_point_index].point_cloud_path !=
        points[point_index].point_cloud_path )
    {
      m_playback_cloud_switches.push_back ({
        waypoint_frame_index + 1,
        point_index,
        false});
    }
    waypoint_frame_index +=
      std::max<std::size_t> (frame_counts[path_index], 2) - 1;
    m_playback_waypoint_frame_indices.push_back (
      waypoint_frame_index);
    m_playback_waypoint_point_indices.push_back (point_index);
    previous_point_index = point_index;
  }

  m_next_playback_waypoint_index = 0;
  m_next_playback_cloud_switch = 0;
  m_waiting_for_playback_cloud = false;
  m_playback_cloud_switch_blocked = false;
  m_speed_zero_paused_playback = false;
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Point_Selection (
      static_cast<int> (initial_point_index));
    Update_Teach_Point_Details ( );
  }
  Set_Joint_Controls_Enabled (false);
  if( m_view && m_view->Collision_Rebuild_In_Progress ( ) )
  {
    m_trajectory_session.Pause ( );
    m_waiting_for_playback_cloud = true;
    m_trajectory_timer.Start (50);
  }
  else
  {
    m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
  }
  Update_Trajectory_Status ( );
  return true;
}

void Robot_Model_Panel_Controller::On_Step_Teach_Point (int direction)
{
  if( Is_Trajectory_Active ( ) ) return;

  const auto& points = m_teach_point_store.Points (m_current_model_id);
  const std::size_t current_index = Current_Progress_Point_Index ( );
  if( current_index >= points.size ( ) )
  {
    if( m_status_text )
      m_status_text->SetLabel (
        wxString::FromUTF8 (
          u8"当前姿态不在任何 Progress 点，请先使用 Go To"));
    return;
  }

  const int target_selection =
    static_cast<int> (current_index) + direction;
  if( target_selection < 0 ||
      static_cast<std::size_t> (target_selection) >= points.size ( ) )
  {
    if( m_status_text )
      m_status_text->SetLabel (
        wxString::FromUTF8 (
          direction < 0
            ? u8"当前已是第一个示教点"
            : u8"当前已是最后一个示教点"));
    return;
  }
  const auto target_index =
    static_cast<std::size_t> (target_selection);

  const bool hardware_connected =
    m_robot_connection->Is_Connected();
  if( hardware_connected && !m_robot_connection->Can_Move() )
  {
    if( m_status_text )
      m_status_text->SetLabel (
        wxString::FromUTF8 (
          u8"KUKA 当前未就绪，请等待当前指令完成或先同步状态"));
    return;
  }

  if( !Apply_Teach_Point_Bindings_Keeping_Display (target_index) )
    return;

  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Point_Selection (
      static_cast<int> (target_index));
    Update_Teach_Point_Details ( );
  }
  Update_Trajectory_Status ( );

  const int step_speed_percent = m_teach_point_command_panel
    ? m_teach_point_command_panel->Step_Speed_Percent ( )
    : 50;
  const std::size_t preview_frame_count = frame_count_for_joint_step (
    Read_Joint_Input_Angles ( ),
    points[target_index].joint_angles_deg,
    step_speed_percent);
  if( !Start_Direct_Go_To_Teach_Point (
        target_index, false, preview_frame_count) )
  {
    return;
  }
  m_teach_step_preview_active = true;

  if( hardware_connected )
  {
    m_hardware_step_preview_active = true;
    try
    {
      application::Robot_Joint_Motion_Options options;
      options.velocity_percent =
        static_cast<double> (step_speed_percent);
      options.acceleration_percent = static_cast<double> (
        m_kuka_acceleration_slider
          ? m_kuka_acceleration_slider->GetValue ( )
          : 20);
      const auto sequence = m_robot_connection->Move_Joint(
        points[target_index].joint_angles_deg, options);
      if( m_status_text )
      {
        m_status_text->SetLabel (wxString::Format (
          wxString::FromUTF8 (
            u8"步进至 P[%zu]：KUKA MOVEJ 已发送，sequence=%u"),
          points[target_index].id,
          sequence));
      }
    }
    catch( const std::exception& error )
    {
      m_hardware_step_preview_active = false;
      Stop_Trajectory_Playback ( );
      if( m_status_text )
      {
        m_status_text->SetLabel (
          wxString::FromUTF8 (u8"步进指令发送失败：") +
          wxString::FromUTF8 (error.what ( )));
      }
    }
    return;
  }
}

void Robot_Model_Panel_Controller::On_Delete_Trajectory_Point (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) )
  {
    return;
  }

  const auto selections = Selected_Teach_Point_Indices ( );
  if( selections.empty ( ) )
  {
    return;
  }

  if( selections.size ( ) > 1 )
  {
    const wxString prompt = wxString::Format (
      wxString::FromUTF8 (u8"确定删除选中的 %zu 个示教点吗？"),
      selections.size ( ));
    if( wxMessageBox (
          prompt,
          wxString::FromUTF8 (u8"批量删除"),
          wxYES_NO | wxNO_DEFAULT | wxICON_WARNING,
          this) != wxYES )
    {
      return;
    }
  }

  std::vector<std::size_t> point_indices;
  point_indices.reserve (selections.size ( ));
  for( const int selection : selections )
  {
    if( selection >= 0 )
      point_indices.push_back (static_cast<std::size_t> (selection));
  }
  const std::size_t next_index = *std::min_element (
    point_indices.begin ( ), point_indices.end ( ));
  m_teach_point_store.Delete_Points (
    m_current_model_id, point_indices);
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  Set_Progress_Dirty (true);

  if( m_trajectory_session.Point_Count ( ) > 0 )
  {
    const int next_selection = static_cast<int> (
      std::min (next_index, m_trajectory_session.Point_Count ( ) - 1));
    if( m_teach_point_list_panel )
    {
      m_teach_point_list_panel->Set_Point_Selection (next_selection);
      Update_Teach_Point_Details ( );
    }
  }

  Update_Trajectory_Status ( );
}

void Robot_Model_Panel_Controller::On_Save_Trajectory (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) )
  {
    return;
  }

  if( m_trajectory_session.Point_Count ( ) == 0 )
  {
    if( m_status_text )
    {
      m_status_text->SetLabel (
        wxString::FromUTF8 (u8"没有可保存的示教点"));
    }
    return;
  }

  wxFileDialog dialog (
    this,
    wxString::FromUTF8 (u8"保存 Progress"),
    m_last_progress_directory,
    "progress.xml",
    "Progress XML (*.xml)|*.xml|All files (*.*)|*.*",
    wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
  if( dialog.ShowModal ( ) != wxID_OK )
  {
    return;
  }

  robot_model::Robot_Progress_File progress;
  progress.robot_model_id = m_current_model_id;
  progress.points = m_teach_point_store.Points (m_current_model_id);
  std::string error_message;
  const bool saved = robot_model::Save_Robot_Progress (
    std::filesystem::path (dialog.GetPath ( ).ToStdWstring ( )),
    progress,
    &error_message);
  if( saved )
  {
    m_last_progress_directory =
      wxFileName (dialog.GetPath ( )).GetPath ( );
    Set_Progress_Dirty (false);
  }
  if( m_status_text )
  {
    m_status_text->SetLabel (
      saved
        ? wxString::FromUTF8 (u8"Progress 已保存")
        : wxString::FromUTF8 (u8"保存失败：") +
          wxString::FromUTF8 (error_message.c_str ( )));
  }
}

void Robot_Model_Panel_Controller::On_Load_Trajectory (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) )
  {
    return;
  }

  wxFileDialog dialog (
    this,
    wxString::FromUTF8 (u8"加载 Progress"),
    m_last_progress_directory,
    "",
    "Progress XML (*.xml)|*.xml|All files (*.*)|*.*",
    wxFD_OPEN | wxFD_FILE_MUST_EXIST);
  if( dialog.ShowModal ( ) != wxID_OK )
  {
    return;
  }

  if( m_teach_point_store.Point_Count (m_current_model_id) > 0 &&
      wxMessageBox (
        wxString::FromUTF8 (
          u8"加载会替换当前 Progress，是否继续？"),
        wxString::FromUTF8 (u8"加载 Progress"),
        wxYES_NO | wxNO_DEFAULT | wxICON_WARNING,
        this) != wxYES )
  {
    return;
  }

  robot_model::Robot_Progress_File progress;
  std::string error_message;
  const bool loaded = robot_model::Load_Robot_Progress (
    std::filesystem::path (dialog.GetPath ( ).ToStdWstring ( )),
    &progress,
    &error_message);
  if( !loaded )
  {
    if( m_status_text )
    {
      m_status_text->SetLabel (
        wxString::FromUTF8 (u8"加载失败：") +
        wxString::FromUTF8 (error_message.c_str ( )));
    }
    return;
  }

  if( progress.robot_model_id != m_current_model_id )
  {
    wxMessageBox (
      wxString::FromUTF8 (u8"Progress 对应的机械臂模型为：") +
      wxString::FromUTF8 (progress.robot_model_id.c_str ( )) +
      wxString::FromUTF8 (u8"\n与当前模型不一致，未加载。"),
      wxString::FromUTF8 (u8"模型不匹配"),
      wxOK | wxICON_ERROR,
      this);
    return;
  }

  m_teach_point_store.Replace_Points (
    m_current_model_id, progress.points);
  Invalidate_Completed_Progress ( );
  m_last_progress_directory =
    wxFileName (dialog.GetPath ( )).GetPath ( );
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Point_Selection (0);
    Update_Teach_Point_Details ( );
  }
  Apply_Teach_Point_Bindings (0);
  Set_Progress_Dirty (false);
  Update_Trajectory_Status ( );
  if( m_status_text )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"Progress 已加载"));
  }
}

void Robot_Model_Panel_Controller::On_Complete_Progress ( )
{
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( points.empty ( ) || !m_run_progress_panel || !m_right_tool_panel )
  {
    return;
  }

  robot_model::Template_Configuration templates;
  std::string configuration_error;
  if( !robot_model::Load_Template_Configuration (
        robot_model::Template_Configuration_Path ( ),
        &templates,
        &configuration_error) )
  {
    wxMessageBox (
      wxString::FromUTF8 (configuration_error.c_str ( )),
      wxString::FromUTF8 (u8"Progress 检查失败"),
      wxOK | wxICON_ERROR,
      this);
    return;
  }

  std::unordered_map<std::string, std::string> cloud_templates;
  auto fail = [this] (const robot_model::Robot_Teach_Point& point,
                      const wxString& reason)
  {
    wxMessageBox (
      wxString::FromUTF8 (
        robot_model::Format_Teach_Point_Name (point.id).c_str ( )) +
        wxString::FromUTF8 (u8"：") + reason,
      wxString::FromUTF8 (u8"Progress 检查失败"),
      wxOK | wxICON_WARNING,
      this);
  };

  for( const auto& point : points )
  {
    if( point.point_cloud_path.empty ( ) )
    {
      fail (point, wxString::FromUTF8 (u8"没有绑定点云"));
      return;
    }
    if( !point.has_template || point.template_id.empty ( ) )
    {
      fail (point, wxString::FromUTF8 (u8"点云没有绑定模板"));
      return;
    }
    const auto* template_profile = robot_model::Find_Template_Profile (
      templates, point.template_id);
    if( !template_profile )
    {
      fail (point, wxString::FromUTF8 (
        u8"绑定的模板已不存在，请重新绑定"));
      return;
    }
    for( std::size_t index = 0; index < 6; ++index )
    {
      if( !std::isfinite (point.template_reference_pose[index]) ||
          std::abs (
            point.template_reference_pose[index] -
            template_profile->reference_pose[index]) > 1.0e-9 )
      {
        fail (point, wxString::FromUTF8 (
          u8"模板参考点已变化，请重新绑定该点云模板"));
        return;
      }
    }
    const auto cloud_binding = cloud_templates.emplace (
      point.point_cloud_path, point.template_id);
    if( !cloud_binding.second &&
        cloud_binding.first->second != point.template_id )
    {
      fail (point, wxString::FromUTF8 (
        u8"同一点云存在不同模板，请重新绑定"));
      return;
    }
    if( !point.has_coordinate_frame ||
        point.coordinate_frame_id.empty ( ) ||
        !robot_model::Find_Tool_Coordinate (
          m_tool_configuration, point.coordinate_frame_id) )
    {
      fail (point, wxString::FromUTF8 (
        u8"坐标系无效或已被删除"));
      return;
    }
    if( !point.has_world_pose ||
        !std::all_of (
          point.world_pose.begin ( ),
          point.world_pose.end ( ),
          [] (double value) { return std::isfinite (value); }) )
    {
      fail (point, wxString::FromUTF8 (u8"示教位姿无效"));
      return;
    }
  }

  m_progress_completed = true;
  m_run_progress_panel->Set_Progress_Ready (true);
  m_right_tool_panel->Set_Run_Tool_Enabled (true);
  Refresh_Run_Readiness ( );
  if( m_status_text )
  {
    m_status_text->SetLabel (wxString::Format (
      wxString::FromUTF8 (
        u8"Progress 检查完成：%zu 个点，运行选项卡已使能"),
      points.size ( )));
  }
}

bool Robot_Model_Panel_Controller::Is_Robot_At_Home(std::string *reason) const
{
  auto fail = [reason](const std::string &message)
  {
    if( reason ) *reason = message;
    return false;
  };
  if( !m_robot_connection->Is_Connected() )
    return fail("机械臂未连接，请先连接并复位");
  if( m_robot_connection_status.state !=
        application::Robot_Control_State::Ready )
    return fail("机械臂尚未就绪，请等待状态同步完成");
  application::Robot_Actual_State latest_state;
  if( !m_robot_connection->Latest_State(&latest_state) )
    return fail("尚未收到机械臂实际位姿，请先同步状态");
  if( !m_view || !m_view->Has_Current_Model ( ) )
    return fail("机械臂模型未加载");

  const auto& params = m_view->Kinematic_Params ( );
  if( !params.has_home_pose )
    return fail("当前模型没有配置复位位姿");

  const auto angular_error = [](double actual, double target)
  {
    return std::abs(std::remainder(actual - target, 360.0));
  };
  double position_error = 0.0;
  double angle_error = 0.0;
  for( std::size_t index = 0; index < 3; ++index )
  {
    const double delta =
      latest_state.pose[index] - params.home_pose_xyzabc[index];
    position_error += delta * delta;
  }
  position_error = std::sqrt(position_error);
  for( std::size_t index = 3; index < 6; ++index )
  {
    angle_error = std::max(
      angle_error,
      angular_error(
        latest_state.pose[index],
        params.home_pose_xyzabc[index]));
  }
  if( position_error > RUN_HOME_POSITION_TOLERANCE_MM ||
      angle_error > RUN_HOME_ANGLE_TOLERANCE_DEG )
  {
    std::ostringstream message;
    message << std::fixed << std::setprecision(2)
            << "机械臂不在复位位置（位置偏差 "
            << position_error << " mm，角度偏差 "
            << angle_error << "°），请先复位";
    return fail(message.str());
  }
  if( reason ) reason->clear();
  return true;
}

void Robot_Model_Panel_Controller::Refresh_Run_Readiness()
{
  if( !m_run_progress_panel ||
      m_progress_run_controller.Is_Motion_Active() )
    return;
  std::string reason;
  const bool ready = Is_Robot_At_Home(&reason);
  if( ready )
  {
    application::Robot_Actual_State latest_state;
    m_robot_connection->Latest_State(&latest_state);
    std::ostringstream message;
    message << std::fixed << std::setprecision(0)
            << "机械臂已在复位位置，可以运行；控制器 Override="
            << latest_state.override_percent << "%";
    if( latest_state.override_percent < 99.5 )
      message << "（该值会继续限制实际速度）";
    reason = message.str();
  }
  m_run_progress_panel->Set_Robot_Ready(ready, reason);
}

void Robot_Model_Panel_Controller::Set_Run_Safety_Lock(bool locked)
{
  if( m_right_tool_panel )
    m_right_tool_panel->Set_Run_Locked(locked);
  if( m_robot_display_button ) m_robot_display_button->Enable(!locked);
  if( m_camera_display_button ) m_camera_display_button->Enable(!locked);
  if( m_camera_2d_display_button ) m_camera_2d_display_button->Enable(!locked);
  if( m_point_cloud_display_button ) m_point_cloud_display_button->Enable(!locked);
  if( m_reset_robot_button )
    m_reset_robot_button->Enable(
      !locked && m_view && m_view->Has_Current_Model());
  if( m_flange_frame_button ) m_flange_frame_button->Enable(!locked);
  if( m_flange_free_drag_button ) m_flange_free_drag_button->Enable(!locked);
  if( m_flange_6d_button ) m_flange_6d_button->Enable(!locked);
  if( m_interaction_coordinate_choice )
    m_interaction_coordinate_choice->Enable(!locked);
  if( m_trajectory_panel ) m_trajectory_panel->Enable(!locked);
  if( m_teach_point_command_panel )
    m_teach_point_command_panel->Enable(!locked);
  Update_Trajectory_Status();
  Set_Joint_Controls_Enabled(!locked);
}

void Robot_Model_Panel_Controller::Start_Progress_Run()
{
  if( m_progress_run_controller.Is_Motion_Active() ||
      m_progress_run_controller.State() ==
        application::Progress_Run_State::Processing_Images ||
      !m_progress_completed ||
      !m_run_progress_panel || !m_robot_connection )
    return;
  if( m_run_image_processing.load() )
  {
    m_run_progress_panel->Set_Status(
      "上一轮图片与拼图仍在后台处理中，请稍候", true);
    return;
  }
  if( m_run_image_processing_thread.joinable() )
    m_run_image_processing_thread.join();
  m_run_captured_frames.clear();
  m_run_progress_panel->Clear_Mosaic();

  std::string reason;
  if( !Is_Robot_At_Home(&reason) )
  {
    m_run_progress_panel->Set_Status(reason, true);
    Refresh_Run_Readiness();
    return;
  }

  const auto& points = m_teach_point_store.Points(m_current_model_id);
  if( points.empty() )
  {
    m_run_progress_panel->Set_Status("Progress 中没有可运行的点", true);
    return;
  }

  m_run_save_images = m_run_progress_panel->Save_Images();
  m_run_build_mosaic = m_run_progress_panel->Build_Mosaic();
  const bool capture_images = m_run_save_images || m_run_build_mosaic;
  m_run_linear_motion =
    m_run_progress_panel->Selected_Motion_Mode() ==
    Run_Progress_Panel::Motion_Mode::Linear;
  m_run_motion_speed = m_run_progress_panel->Motion_Speed();
  const bool has_motion_point = std::any_of(
    points.begin(), points.end(),
    [](const robot_model::Robot_Teach_Point& point)
    {
      return point.type == robot_model::Robot_Teach_Point_Type::Motion;
    });
  if( capture_images && has_motion_point )
  {
    if( !m_camera_2d_service || !m_camera_2d_service->Is_Grabbing() )
    {
      m_run_progress_panel->Set_Status(
        "采集运动点图像前请打开 2D 相机并开始采集", true);
      return;
    }
    if( m_camera_2d_service->Status().trigger_mode !=
        jutze_camera::camera_trigger_mode::soft_trigger )
    {
      m_run_progress_panel->Set_Status(
        "采集运动点图像需要将 2D 相机设置为软触发模式", true);
      return;
    }

    m_run_image_directory =
      application::Get_App_Paths().run_image_root /
      std::string(wxDateTime::Now().Format("%Y%m%d_%H%M%S").utf8_str());
    std::error_code directory_error;
    std::filesystem::create_directories(
      m_run_image_directory, directory_error);
    if( directory_error )
    {
      m_run_progress_panel->Set_Status(
        "创建图片目录失败：" + directory_error.message(), true);
      return;
    }
  }

  m_run_started_at = std::chrono::steady_clock::now();
  m_run_progress_panel->Set_Elapsed(std::chrono::milliseconds(0));
  m_run_progress_panel->Set_Running(true);
  m_run_progress_panel->Set_Status(
    std::string("运行中：") +
    (m_run_linear_motion ? "LIN " : "PTP ") +
    "速度=" + std::to_string(m_run_motion_speed) +
    (m_run_linear_motion ? " mm/s" : "%"));
  Set_Run_Safety_Lock(true);
  m_run_timer.Start(RUN_TIMER_INTERVAL_MS);
  std::vector<bool> capture_image_at_point;
  capture_image_at_point.reserve(points.size());
  for( const auto& point : points )
  {
    capture_image_at_point.push_back(
      capture_images &&
      point.type == robot_model::Robot_Teach_Point_Type::Motion);
  }
  Apply_Progress_Run_Transition(
    m_progress_run_controller.Start(std::move(capture_image_at_point)));
}

void Robot_Model_Panel_Controller::Dispatch_Next_Run_Point()
{
  if( m_progress_run_controller.State() !=
        application::Progress_Run_State::Moving )
    return;
  const std::size_t point_index =
    m_progress_run_controller.Current_Point_Index();
  const auto& points = m_teach_point_store.Points(m_current_model_id);
  if( point_index >= points.size() )
  {
    Apply_Progress_Run_Transition(
      m_progress_run_controller.Effect_Failed(
        "运动点索引无效，Progress 已中止"));
    return;
  }
  if( !m_robot_connection->Can_Move() )
  {
    Apply_Progress_Run_Transition(
      m_progress_run_controller.Effect_Failed(
        "机械臂未就绪，Progress 已中止"));
    return;
  }

  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Point_Selection(
      static_cast<int>(point_index));
    Update_Teach_Point_Details();
  }

  try
  {
    application::Robot_Cartesian_Motion_Options options;
    options.velocity = static_cast<double>(m_run_motion_speed);
    options.acceleration_percent = static_cast<double>(
      m_kuka_acceleration_slider
        ? m_kuka_acceleration_slider->GetValue()
        : 50);
    options.blend_mm = 0.0;
    const auto sequence = m_run_linear_motion
      ? m_robot_connection->Move_Linear(
          points[point_index].world_pose, options)
      : m_robot_connection->Move_Pose_Ptp(
          points[point_index].world_pose,
          points[point_index].joint_angles_deg,
          options);
    m_run_progress_panel->Set_Status(
      "运行中：正在执行 " +
      robot_model::Format_Teach_Point_Name(points[point_index].id) +
      (m_run_linear_motion ? "（LIN）" : "（PTP）") +
      "，sequence=" + std::to_string(sequence));
  }
  catch( const std::exception& error )
  {
    Apply_Progress_Run_Transition(
      m_progress_run_controller.Effect_Failed(
        std::string("下发机械臂位姿失败：") + error.what()));
  }
}

void Robot_Model_Panel_Controller::Capture_Run_Point_Image()
{
  if( m_progress_run_controller.State() !=
        application::Progress_Run_State::Waiting_For_Image )
    return;
  const std::size_t point_index =
    m_progress_run_controller.Current_Point_Index();
  const auto& points = m_teach_point_store.Points(m_current_model_id);
  const bool is_motion_point =
    point_index < points.size() &&
    points[point_index].type ==
      robot_model::Robot_Teach_Point_Type::Motion;
  if( !is_motion_point )
  {
    Apply_Progress_Run_Transition(
      m_progress_run_controller.Effect_Failed(
        "非运动点请求了图像采集，Progress 已中止"));
    return;
  }

  const auto previous =
    m_camera_2d_service ? m_camera_2d_service->Latest_Frame() : nullptr;
  m_run_had_previous_frame = static_cast<bool>(previous);
  m_run_previous_frame_number = previous ? previous->m_frame_num : 0;
  std::string error;
  if( !m_camera_2d_service ||
      !m_camera_2d_service->Software_Trigger(&error) )
  {
    Apply_Progress_Run_Transition(
      m_progress_run_controller.Effect_Failed(
        "2D 相机触发失败：" + error));
    return;
  }
  m_run_image_deadline =
    std::chrono::steady_clock::now() +
    std::chrono::milliseconds(RUN_IMAGE_TIMEOUT_MS);
  m_run_progress_panel->Set_Status(
    "机械臂已到位，正在获取 2D 图片...");
}

void Robot_Model_Panel_Controller::On_Run_Timer(wxTimerEvent &)
{
  if( !m_progress_run_controller.Is_Motion_Active() )
    return;
  const auto now = std::chrono::steady_clock::now();
  if( m_run_progress_panel )
  {
    m_run_progress_panel->Set_Elapsed(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_run_started_at));
  }
  if( m_progress_run_controller.State() !=
        application::Progress_Run_State::Waiting_For_Image )
    return;

  const auto frame =
    m_camera_2d_service ? m_camera_2d_service->Latest_Frame() : nullptr;
  const bool has_new_frame =
    frame && (!m_run_had_previous_frame ||
              frame->m_frame_num != m_run_previous_frame_number);
  if( has_new_frame )
  {
    const auto& points = m_teach_point_store.Points(m_current_model_id);
    const std::size_t point_index =
      m_progress_run_controller.Current_Point_Index();
    if( point_index >= points.size() )
    {
      Apply_Progress_Run_Transition(
        m_progress_run_controller.Effect_Failed(
          "运动点索引无效，Progress 已中止"));
      return;
    }
    // Keep the SDK-owned frame alive and continue robot motion immediately.
    // Conversion, PNG compression, disk I/O and mosaic construction run only
    // after robot motion completes, on a worker thread.
    m_run_captured_frames.push_back({frame, points[point_index]});
    Apply_Progress_Run_Transition(
      m_progress_run_controller.Image_Captured());
    return;
  }
  if( now >= m_run_image_deadline )
  {
    Apply_Progress_Run_Transition(
      m_progress_run_controller.Image_Timed_Out());
  }
}

void Robot_Model_Panel_Controller::Request_Progress_Emergency_Stop()
{
  Apply_Progress_Run_Transition(
    m_progress_run_controller.Request_Emergency_Stop());
}

void Robot_Model_Panel_Controller::Apply_Progress_Run_Transition(
  const application::Progress_Run_Transition& transition)
{
  switch( transition.action )
  {
    case application::Progress_Run_Action::None:
      return;
    case application::Progress_Run_Action::Move_Point:
      Dispatch_Next_Run_Point();
      return;
    case application::Progress_Run_Action::Trigger_Image:
      Capture_Run_Point_Image();
      return;
    case application::Progress_Run_Action::Request_Stop:
      if( m_run_progress_panel )
      {
        m_run_progress_panel->Set_Running(true, true);
        m_run_progress_panel->Set_Status(
          "急停命令已发送，正在等待机械臂确认停止", true);
      }
      try
      {
        if( !m_robot_connection )
          throw std::runtime_error("KUKA service is unavailable.");
        m_robot_connection->Request_Stop();
      }
      catch( const std::exception& error )
      {
        Apply_Progress_Run_Transition(
          m_progress_run_controller.Effect_Failed(
            std::string("急停命令发送失败：") + error.what()));
      }
      return;
    case application::Progress_Run_Action::Start_Image_Processing:
      Finish_Progress_Run(true, "Progress 运行完成");
      Start_Run_Image_Processing();
      return;
    case application::Progress_Run_Action::Finish:
      Finish_Progress_Run(
        transition.state == application::Progress_Run_State::Completed,
        transition.message.empty()
          ? "Progress 运行完成"
          : transition.message);
      return;
  }
}

void Robot_Model_Panel_Controller::Start_Run_Image_Processing()
{
  if( m_run_captured_frames.empty() || m_run_image_processing.load() )
    return;
  const auto* camera_tool = m_run_build_mosaic
    ? robot_model::Find_Tool_Coordinate(
        m_tool_configuration,
        m_tool_visualization_configuration.fov.tool_coordinate_id)
    : nullptr;
  if( m_run_build_mosaic && !camera_tool )
  {
    m_run_captured_frames.clear();
    m_progress_run_controller.Image_Processing_Completed(
      false,
      "拼图失败：FOV绑定的相机坐标系不存在，请检查Tool配置");
    if( m_run_progress_panel )
    {
      m_run_progress_panel->Set_Status(
        "拼图失败：FOV绑定的相机坐标系不存在，请检查Tool配置",
        true);
    }
    return;
  }
  if( m_run_image_processing_thread.joinable() )
    m_run_image_processing_thread.join();

  auto captures = std::move(m_run_captured_frames);
  m_run_captured_frames.clear();
  const auto image_directory = m_run_image_directory;
  const bool save_original_images = m_run_save_images;
  const bool build_mosaic = m_run_build_mosaic;
  const auto fov_configuration =
    m_tool_visualization_configuration.fov;
  const robot_model::XyzabcPose flange_from_camera_pose = camera_tool
    ? camera_tool->flange_from_tool_pose
    : robot_model::XyzabcPose{};
  m_run_image_processing.store(true);
  if( m_run_progress_panel )
  {
    m_run_progress_panel->Set_Status(
      build_mosaic
        ? "机械臂运动已完成，正在后台处理图片并生成拼图..."
        : "机械臂运动已完成，正在后台保存运动点原图...");
  }
  m_run_image_processing_thread = std::thread(
    [this,
     captures = std::move(captures),
     image_directory,
     save_original_images,
     build_mosaic,
     fov_configuration,
     flange_from_camera_pose]() mutable
    {
      Run_Image_Processing_Result result;
      try
      {
        std::vector<std::pair<
          Camera_2D_Display_Image,
          robot_model::Robot_Teach_Point>> converted;
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
          converted.emplace_back(
            std::move(image), std::move(capture.point));
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
            std::to_string(converted.size()) +
            " 张；目录：" + image_directory.string();
        }
      }
      catch( const std::exception& error )
      {
        result.success = false;
        result.message =
          std::string("后台图片/拼图处理异常：") + error.what();
      }

      auto* event =
        new wxThreadEvent(wxEVT_RUN_IMAGE_PROCESSING_RESULT);
      event->SetPayload(std::move(result));
      wxQueueEvent(this, event);
    });
}

void Robot_Model_Panel_Controller::On_Run_Image_Processing_Result(
  wxThreadEvent& event)
{
  auto result =
    event.GetPayload<Run_Image_Processing_Result>();
  if( m_run_image_processing_thread.joinable() )
    m_run_image_processing_thread.join();
  m_run_image_processing.store(false);
  m_progress_run_controller.Image_Processing_Completed(
    result.success, result.message);
  if( m_run_progress_panel )
  {
    m_run_progress_panel->Set_Status(
      result.message, !result.success);
    if( result.success && result.mosaic.width > 0 &&
        result.mosaic.height > 0 )
    {
      wxImage image(
        static_cast<int>(result.mosaic.width),
        static_cast<int>(result.mosaic.height));
      if( image.IsOk() && image.GetData() )
      {
        std::memcpy(
          image.GetData(),
          result.mosaic.rgb.data(),
          result.mosaic.rgb.size());
        m_run_progress_panel->Set_Mosaic_Image(image);
      }
    }
  }
  Refresh_Run_Readiness();
}

void Robot_Model_Panel_Controller::Finish_Progress_Run(
  bool success,
  const std::string &message)
{
  if( m_run_timer.IsRunning() )
    m_run_timer.Stop();
  const auto elapsed =
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - m_run_started_at);
  std::string final_message = message;
  if( success )
  {
    if( m_run_captured_frames.empty() )
    {
      final_message += (m_run_save_images || m_run_build_mosaic)
        ? "；没有运动点图片可处理"
        : "；未选择图片保存或拼图";
    }
    else if( m_run_build_mosaic && m_run_save_images )
      final_message += "；原图将在后台保存并生成拼图";
    else if( m_run_build_mosaic )
      final_message += "；将在后台生成拼图";
    else
      final_message += "；原图将在后台保存";
  }
  if( m_run_progress_panel )
  {
    m_run_progress_panel->Set_Elapsed(elapsed);
    m_run_progress_panel->Set_Running(false);
  }
  Set_Run_Safety_Lock(false);
  Refresh_Run_Readiness();
  Refresh_Robot_Command_Controls(m_robot_connection_status);
  if( m_run_progress_panel )
    m_run_progress_panel->Set_Status(final_message, !success);
  if( !success )
  {
    m_run_captured_frames.clear();
  }
}

void Robot_Model_Panel_Controller::On_Play_Trajectory (wxCommandEvent&)
{
  if( Is_Trajectory_Active ( ) ||
      !m_trajectory_session.Has_Playable_Path ( ) ||
      Get_Trajectory_Speed_Mps ( ) <= 0.0 )
  {
    return;
  }

  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( !Apply_Teach_Point_Bindings (0, true) )
  {
    return;
  }
  std::vector<std::size_t> frame_counts;
  frame_counts.reserve (points.size ( ) > 0 ? points.size ( ) - 1 : 0);
  for( std::size_t index = 0; index + 1 < points.size ( ); ++index )
  {
    frame_counts.push_back (
      points[index].has_world_pose && points[index + 1].has_world_pose
        ? frame_count_for_one_meter_per_second (
            points[index].world_pose, points[index + 1].world_pose)
        : static_cast<std::size_t> (TRAJECTORY_FRAME_COUNT));
  }
  m_playback_waypoint_frame_indices.clear ( );
  m_playback_waypoint_point_indices.clear ( );
  m_playback_cloud_switches.clear ( );
  m_playback_waypoint_frame_indices.push_back (0);
  m_playback_waypoint_point_indices.push_back (0);
  std::size_t waypoint_frame_index = 0;
  for( std::size_t index = 0; index < frame_counts.size ( ); ++index )
  {
    if( points[index].point_cloud_path !=
        points[index + 1].point_cloud_path )
    {
      m_playback_cloud_switches.push_back ({
        waypoint_frame_index + 1,
        index + 1,
        true});
    }
    waypoint_frame_index +=
      std::max<std::size_t> (frame_counts[index], 2) - 1;
    m_playback_waypoint_frame_indices.push_back (
      waypoint_frame_index);
    m_playback_waypoint_point_indices.push_back (index + 1);
  }
  if( !m_trajectory_session.Start_Playback (frame_counts) )
  {
    return;
  }

  const auto start_result = Apply_Joint_Input_Angles_To_Sliders (
    m_trajectory_session.Points ( ).front ( ));
  if( !start_result.accepted && start_result.collision.collided )
  {
    m_trajectory_session.Pause ( );
    Set_Joint_Controls_Enabled (true);
    Update_Trajectory_Status ( );
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"轨迹已暂停：起始段，") +
      collision_summary (start_result.collision));
    return;
  }
  m_next_playback_waypoint_index = 1;
  m_next_playback_cloud_switch = 0;
  m_waiting_for_playback_cloud = false;
  m_playback_cloud_switch_blocked = false;
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Point_Selection (0);
    Update_Teach_Point_Details ( );
  }
  m_speed_zero_paused_playback = false;
  Set_Joint_Controls_Enabled (false);
  if( m_view && m_view->Collision_Rebuild_In_Progress ( ) )
  {
    m_trajectory_session.Pause ( );
    m_waiting_for_playback_cloud = true;
    m_trajectory_timer.Start (50);
  }
  else
  {
    m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
  }
  Update_Trajectory_Status ( );
}

void Robot_Model_Panel_Controller::On_Pause_Resume_Trajectory (wxCommandEvent&)
{
  if( !m_trajectory_session.Is_Active ( ) )
  {
    return;
  }
  if( m_playback_cloud_switch_blocked )
  {
    return;
  }

  if( m_trajectory_timer.IsRunning ( ) )
  {
    m_trajectory_timer.Stop ( );
    m_trajectory_session.Pause ( );
  }
  else if( m_trajectory_session.Is_Paused ( ) )
  {
    if( Get_Trajectory_Speed_Mps ( ) <= 0.0 )
    {
      return;
    }
    m_trajectory_session.Resume ( );
    m_speed_zero_paused_playback = false;
    Set_Joint_Controls_Enabled (false);
    m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
  }

  Update_Trajectory_Status ( );
}

void Robot_Model_Panel_Controller::On_Stop_Trajectory (wxCommandEvent&)
{
  Stop_Trajectory_Playback ( );
}

void Robot_Model_Panel_Controller::On_Trajectory_Speed_Changed (wxCommandEvent&)
{
  Update_Trajectory_Speed_Label ( );
  if( m_teach_step_preview_active ) return;
  const double speed_mps = Get_Trajectory_Speed_Mps ( );
  if( m_playback_cloud_switch_blocked )
  {
    Update_Trajectory_Status ( );
    return;
  }
  if( speed_mps <= 0.0 && m_trajectory_timer.IsRunning ( ) )
  {
    m_trajectory_timer.Stop ( );
    m_trajectory_session.Pause ( );
    m_speed_zero_paused_playback = true;
  }
  else if( speed_mps > 0.0 &&
           m_speed_zero_paused_playback &&
           m_trajectory_session.Is_Paused ( ) )
  {
    m_speed_zero_paused_playback = false;
    if( m_waiting_for_playback_cloud )
    {
      m_trajectory_timer.Start (50);
    }
    else
    {
      m_trajectory_session.Resume ( );
      m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
    }
  }
  else if( m_trajectory_timer.IsRunning ( ) )
  {
    m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
  }
  Update_Trajectory_Status ( );
}

void Robot_Model_Panel_Controller::On_Trajectory_Timer (wxTimerEvent&)
{
  if( m_playback_cloud_switch_blocked )
  {
    m_trajectory_timer.Stop ( );
    return;
  }
  if( m_waiting_for_playback_cloud )
  {
    if( m_view && m_view->Collision_Rebuild_In_Progress ( ) )
    {
      return;
    }
    m_waiting_for_playback_cloud = false;
    if( Get_Trajectory_Speed_Mps ( ) <= 0.0 )
    {
      m_trajectory_timer.Stop ( );
      m_speed_zero_paused_playback = true;
      Update_Trajectory_Status ( );
      return;
    }
    m_trajectory_session.Resume ( );
    m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
    Update_Trajectory_Status ( );
    return;
  }

  if( m_next_playback_cloud_switch <
        m_playback_cloud_switches.size ( ) )
  {
    const auto& cloud_switch =
      m_playback_cloud_switches[m_next_playback_cloud_switch];
    if( m_trajectory_session.Frame_Index ( ) >=
        cloud_switch.frame_index )
    {
      m_trajectory_session.Pause ( );
      m_trajectory_timer.Stop ( );
      if( !Apply_Teach_Point_Bindings_Keeping_Display (
            cloud_switch.point_index,
            cloud_switch.require_point_cloud) )
      {
        m_playback_cloud_switch_blocked = true;
        Set_Joint_Controls_Enabled (true);
        Update_Trajectory_Status ( );
        return;
      }
      ++m_next_playback_cloud_switch;
      if( m_view && m_view->Collision_Rebuild_In_Progress ( ) )
      {
        m_waiting_for_playback_cloud = true;
        m_trajectory_timer.Start (50);
      }
      else
      {
        m_trajectory_session.Resume ( );
        m_trajectory_timer.Start (Get_Trajectory_Timer_Interval_Ms ( ));
      }
      Update_Trajectory_Status ( );
      return;
    }
  }

  const auto* frame = m_trajectory_session.Step ( );
  if( frame == nullptr )
  {
    Stop_Trajectory_Playback ( );
    return;
  }

  const auto apply_result = Apply_Joint_Input_Angles_To_Sliders (*frame);
  if( !apply_result.accepted && apply_result.collision.collided )
  {
    m_trajectory_timer.Stop ( );
    m_trajectory_session.Pause ( );
    Set_Joint_Controls_Enabled (true);
    Update_Trajectory_Status ( );
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"轨迹已暂停：") +
      collision_summary (apply_result.collision) +
      wxString::FromUTF8 (u8"，已停在最后安全姿态"));
    return;
  }

  if( m_trajectory_session.Frame_Index ( ) > 0 )
  {
    const std::size_t applied_frame =
      m_trajectory_session.Frame_Index ( ) - 1;
    while( m_next_playback_waypoint_index <
             m_playback_waypoint_frame_indices.size ( ) &&
           applied_frame >=
             m_playback_waypoint_frame_indices[
               m_next_playback_waypoint_index] )
    {
      if( m_teach_point_list_panel )
      {
        m_teach_point_list_panel->Set_Point_Selection (
          static_cast<int> (
            m_playback_waypoint_point_indices[
              m_next_playback_waypoint_index]));
        Update_Teach_Point_Details ( );
      }
      ++m_next_playback_waypoint_index;
    }
  }

  if( m_trajectory_session.Is_Finished ( ) )
  {
    Stop_Trajectory_Playback ( );
  }
}

wxPanel* Robot_Model_Panel_Controller::Build_Robot_Tool_Page (wxWindow* parent)
{
  auto* panel = new wxScrolledWindow (
    parent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
    wxVSCROLL | wxTAB_TRAVERSAL);
  panel->SetMinSize (wxSize (380, -1));
  panel->SetScrollRate (0, 12);

  auto* operation_title = new wxStaticText (
    panel, wxID_ANY, wxString::FromUTF8 (u8"机械臂操作"));
  m_kuka_connect_button = new wxButton (panel, wxID_ANY, "Connect");
  m_kuka_connect_button->Bind (
    wxEVT_BUTTON, &Robot_Model_Panel_Controller::On_Kuka_Connect, this);
  m_flange_frame_button = new wxToggleButton (
    panel, wxID_ANY, wxString::FromUTF8 (u8"显示法兰坐标系"));
  m_flange_free_drag_button = new wxToggleButton (
    panel, wxID_ANY, wxString::FromUTF8 (u8"自由拖拽"));
  m_flange_6d_button = new wxToggleButton (
    panel, wxID_ANY, wxString::FromUTF8 (u8"6D 操纵"));
  auto* interaction_coordinate_label = new wxStaticText (
    panel, wxID_ANY, wxString::FromUTF8 (u8"操作坐标系"));
  m_interaction_coordinate_choice = new wxChoice (panel, wxID_ANY);
  m_reset_robot_button = new wxButton (
    panel, wxID_ANY, wxString::FromUTF8 (u8"机械臂复位"));
  m_reset_robot_button->Enable (false);
  m_flange_frame_button->Bind (
    wxEVT_TOGGLEBUTTON, &Robot_Model_Panel_Controller::On_Toggle_Flange_Frame, this);
  m_flange_free_drag_button->Bind (
    wxEVT_TOGGLEBUTTON,
    &Robot_Model_Panel_Controller::On_Toggle_Flange_Free_Drag,
    this);
  m_flange_6d_button->Bind (
    wxEVT_TOGGLEBUTTON, &Robot_Model_Panel_Controller::On_Toggle_Flange_6D, this);
  m_interaction_coordinate_choice->Bind (
    wxEVT_CHOICE,
    &Robot_Model_Panel_Controller::On_Interaction_Coordinate_Changed,
    this);
  m_reset_robot_button->Bind (
    wxEVT_BUTTON, &Robot_Model_Panel_Controller::On_Reset_Robot, this);

  auto* operation_sizer = new wxGridSizer (2, 6, 6);
  operation_sizer->Add (
    interaction_coordinate_label, 0, wxALIGN_CENTER_VERTICAL);
  operation_sizer->Add (m_interaction_coordinate_choice, 1, wxEXPAND);
  operation_sizer->Add (m_flange_frame_button, 1, wxEXPAND);
  operation_sizer->Add (m_flange_free_drag_button, 1, wxEXPAND);
  operation_sizer->Add (m_flange_6d_button, 1, wxEXPAND);
  operation_sizer->Add (m_reset_robot_button, 1, wxEXPAND);

  m_kuka_status_text =
    new wxStaticText (panel, wxID_ANY, "KUKA: Disconnected");
  m_kuka_move_joint_button =
    new wxButton (panel, wxID_ANY, "MoveJ");
  m_kuka_move_ptp_button =
    new wxButton (panel, wxID_ANY, "MoveP");
  m_kuka_move_linear_button =
    new wxButton (panel, wxID_ANY, "MoveL");
  m_kuka_sync_button = new wxButton (panel, wxID_ANY, "State");
  m_kuka_stop_button = new wxButton (panel, wxID_ANY, "Stop");

  m_kuka_move_joint_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&)
    {
      try
      {
        if( !m_robot_connection )
          throw std::runtime_error ("KUKA service is unavailable.");
        application::Robot_Joint_Motion_Options options;
        options.velocity_percent =
          static_cast<double> (m_kuka_ptp_velocity_slider->GetValue ( ));
        options.acceleration_percent =
          static_cast<double> (m_kuka_acceleration_slider->GetValue ( ));
        const auto sequence =
          m_robot_connection->Move_Joint(
            Read_Joint_Input_Angles ( ), options);
        m_status_text->SetLabel (
          wxString::Format ("KUKA MOVEJ sent, sequence=%u", sequence));
      }
      catch( const std::exception& error )
      {
        m_status_text->SetLabel (
          "KUKA MOVEJ failed: " + wxString::FromUTF8 (error.what ( )));
      }
    });

  const auto send_cartesian =
    [this] (bool linear)
    {
      try
      {
        if( !m_robot_connection )
          throw std::runtime_error ("KUKA service is unavailable.");
        robot_model::Matrix4 world_from_flange = { };
        if( !m_view || !m_view->Get_World_From_Flange (&world_from_flange) )
          throw std::runtime_error ("No valid robot pose is available.");
        // Protocol defaults to KUKA TOOL 0, therefore transmit the flange
        // target. A configured app-tool pose must not be sent as a TOOL 0
        // flange pose because that would introduce the tool offset twice.
        const application::Robot_Pose target =
          robot_model::Build_Xyzabc_From_Zyx_Matrix (world_from_flange);
        application::Robot_Cartesian_Motion_Options options;
        options.velocity = static_cast<double> (
          linear
            ? m_kuka_linear_velocity_slider->GetValue ( )
            : m_kuka_ptp_velocity_slider->GetValue ( ));
        options.acceleration_percent = static_cast<double> (
          m_kuka_acceleration_slider->GetValue ( ));
        const auto sequence = linear
          ? m_robot_connection->Move_Linear(target, options)
          : m_robot_connection->Move_Pose_Ptp(
              target, Read_Joint_Input_Angles ( ), options);
        m_status_text->SetLabel (wxString::Format (
          linear
            ? "KUKA MOVEL sent, sequence=%u"
            : "KUKA MOVEPTP sent, sequence=%u",
          sequence));
      }
      catch( const std::exception& error )
      {
        m_status_text->SetLabel (
          "KUKA Cartesian move failed: " +
          wxString::FromUTF8 (error.what ( )));
      }
    };
  m_kuka_move_ptp_button->Bind (
    wxEVT_BUTTON,
    [send_cartesian] (wxCommandEvent&) { send_cartesian (false); });
  m_kuka_move_linear_button->Bind (
    wxEVT_BUTTON,
    [send_cartesian] (wxCommandEvent&) { send_cartesian (true); });
  m_kuka_sync_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&)
    {
      try
      {
        if( !m_robot_connection )
          throw std::runtime_error ("KUKA service is unavailable.");
        m_robot_connection->Synchronize();
      }
      catch( const std::exception& error )
      {
        m_status_text->SetLabel (
          "KUKA state query failed: " +
          wxString::FromUTF8 (error.what ( )));
      }
    });
  m_kuka_stop_button->Bind (
    wxEVT_BUTTON,
    [this] (wxCommandEvent&)
    {
      try
      {
        if( !m_robot_connection )
          throw std::runtime_error ("KUKA service is unavailable.");
        m_robot_connection->Request_Stop();
      }
      catch( const std::exception& error )
      {
        m_status_text->SetLabel (
          "KUKA stop failed: " + wxString::FromUTF8 (error.what ( )));
      }
    });

  // Keep hardware controls readable on narrower layouts: two columns.
  auto* robot_command_sizer = new wxGridSizer (2, 6, 6);
  robot_command_sizer->Add (m_kuka_move_joint_button, 1, wxEXPAND);
  robot_command_sizer->Add (m_kuka_move_ptp_button, 1, wxEXPAND);
  robot_command_sizer->Add (m_kuka_move_linear_button, 1, wxEXPAND);
  robot_command_sizer->Add (m_kuka_sync_button, 1, wxEXPAND);
  robot_command_sizer->Add (m_kuka_stop_button, 1, wxEXPAND);

  m_joint_panel = new Joint_Control_Panel (panel);
  m_joint_panel->Set_On_Joint_Changed (
    [this] { Update_Joint_State_From_Sliders ( ); });

  m_cartesian_pose_panel = new Cartesian_Pose_Panel (panel);
  m_cartesian_pose_panel->Set_On_World_Frame_Visibility_Changed (
    [this] (bool visible)
    {
      if( m_view ) m_view->Set_World_Frame_Visible (visible);
    });
  m_cartesian_pose_panel->Set_On_Pose_Changed (
    [this] (const robot_model::XyzabcPose& target_pose)
    {
      Apply_Cartesian_Pose_Target (target_pose);
    });

  auto* motion_parameter_sizer = new wxStaticBoxSizer (
    wxVERTICAL, panel, "Motion parameters");
  const auto add_motion_parameter =
    [panel, motion_parameter_sizer] (
      const wxString& name,
      int minimum,
      int maximum,
      int initial,
      const wxString& suffix,
      wxSlider** output)
    {
      auto* row = new wxBoxSizer (wxHORIZONTAL);
      auto* name_text = new wxStaticText (panel, wxID_ANY, name);
      name_text->SetMinSize (wxSize (145, -1));
      auto* slider = new wxSlider (
        panel,
        wxID_ANY,
        initial,
        minimum,
        maximum,
        wxDefaultPosition,
        wxDefaultSize,
        wxSL_HORIZONTAL);
      auto* value_text = new wxStaticText (
        panel,
        wxID_ANY,
        wxString::Format ("%d", initial) + suffix);
      value_text->SetMinSize (wxSize (85, -1));
      slider->Bind (
        wxEVT_SLIDER,
        [slider, value_text, suffix] (wxCommandEvent&)
        {
          value_text->SetLabel (
            wxString::Format ("%d", slider->GetValue ( )) + suffix);
        });
      row->Add (name_text, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 6);
      row->Add (slider, 1, wxALIGN_CENTER_VERTICAL | wxRIGHT, 6);
      row->Add (value_text, 0, wxALIGN_CENTER_VERTICAL);
      motion_parameter_sizer->Add (row, 0, wxEXPAND | wxALL, 4);
      *output = slider;
    };
  add_motion_parameter (
    "PTP speed (MoveJ / MoveP)",
    1,
    100,
    20,
    "%",
    &m_kuka_ptp_velocity_slider);
  add_motion_parameter (
    "Linear speed (MoveL)",
    1,
    2000,
    100,
    " mm/s",
    &m_kuka_linear_velocity_slider);
  add_motion_parameter (
    "Acceleration",
    1,
    100,
    50,
    "%",
    &m_kuka_acceleration_slider);

  m_kuka_status_table = new wxGrid (
    panel, wxID_ANY, wxDefaultPosition, wxSize (-1, 50));
  m_kuka_status_table->CreateGrid (2, 2);
  m_kuka_status_table->EnableEditing (false);
  m_kuka_status_table->EnableGridLines (true);
  m_kuka_status_table->SetRowLabelSize (0);
  m_kuka_status_table->SetColLabelSize (0);
  m_kuka_status_table->DisableDragGridSize ( );
  m_kuka_status_table->DisableDragColSize ( );
  m_kuka_status_table->DisableDragRowSize ( );
  m_kuka_status_table->SetDefaultCellAlignment (
    wxALIGN_CENTER, wxALIGN_CENTER);
  m_kuka_status_table->SetGridLineColour (wxColour (205, 205, 205));
  m_kuka_status_table->SetMargins (0, 0);
  m_kuka_status_table->ShowScrollbars (
    wxSHOW_SB_NEVER, wxSHOW_SB_NEVER);
  m_kuka_status_table->SetColSize (0, 105);
  m_kuka_status_table->SetColSize (1, 520);
  m_kuka_status_table->SetMinSize (wxSize (210, 50));
  const std::array<const char*, 2> status_rows = {
    "Connection", "State"
  };
  for( std::size_t row = 0; row < status_rows.size ( ); ++row )
  {
    m_kuka_status_table->SetCellValue (
      static_cast<int> (row), 0, status_rows[row]);
    m_kuka_status_table->SetCellValue (
      static_cast<int> (row), 1, "--");
    m_kuka_status_table->SetCellBackgroundColour (
      static_cast<int> (row), 0, wxColour (238, 238, 238));
    m_kuka_status_table->SetRowSize (static_cast<int> (row), 24);
  }
  m_kuka_status_table->Bind (
    wxEVT_SIZE,
    [this] (wxSizeEvent& event)
    {
      const int width =
        std::max (210, m_kuka_status_table->GetClientSize ( ).x);
      m_kuka_status_table->SetColSize (0, 105);
      m_kuka_status_table->SetColSize (1, std::max (80, width - 105));
      event.Skip ( );
    });

  auto* connection_sizer = new wxBoxSizer (wxHORIZONTAL);
  connection_sizer->Add (m_kuka_connect_button, 0, wxRIGHT, 8);
  connection_sizer->Add (
    m_kuka_status_text, 1, wxALIGN_CENTER_VERTICAL);

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (
    connection_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 8);
  sizer->Add (operation_title, 0, wxEXPAND | wxALL, 8);
  sizer->Add (operation_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
  sizer->Add (m_joint_panel, 0, wxEXPAND | wxTOP, 6);
  sizer->Add (m_cartesian_pose_panel, 0, wxEXPAND | wxTOP, 14);
  sizer->Add (
    motion_parameter_sizer,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxTOP,
    8);
  sizer->Add (
    robot_command_sizer, 0, wxEXPAND | wxALL, 8);
  sizer->Add (
    new wxStaticText (panel, wxID_ANY, "Robot status"),
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxTOP,
    8);
  sizer->Add (
    m_kuka_status_table,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxTOP | wxBOTTOM,
    8);
  sizer->AddStretchSpacer (1);
  panel->SetSizer (sizer);
  Refresh_Robot_Command_Controls(m_robot_connection->Status());
  panel->FitInside ( );
  return panel;
}

wxPanel* Robot_Model_Panel_Controller::Build_Teach_Tool_Page (wxWindow* parent)
{
  auto* panel = new wxScrolledWindow (
    parent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
    wxVSCROLL | wxTAB_TRAVERSAL);
  panel->SetMinSize (wxSize (380, -1));
  panel->SetScrollRate (0, 12);

  auto* title = new wxStaticText (
    panel, wxID_ANY, "Teach");
  m_teach_point_command_panel =
    new Teach_Point_Command_Panel (panel);
  Teach_Point_Command_Panel::Callbacks edit_callbacks;
  edit_callbacks.add =
    [this] { wxCommandEvent event; On_Add_Trajectory_Point (event); };
  edit_callbacks.update =
    [this] { wxCommandEvent event; On_Update_Teach_Point (event); };
  edit_callbacks.insert_before =
    [this] { On_Insert_Teach_Point (true); };
  edit_callbacks.insert_after =
    [this] { On_Insert_Teach_Point (false); };
  edit_callbacks.delete_selected =
    [this] { wxCommandEvent event; On_Delete_Trajectory_Point (event); };
  edit_callbacks.clear =
    [this] { wxCommandEvent event; On_Clear_Trajectory_Points (event); };
  edit_callbacks.save =
    [this] { wxCommandEvent event; On_Save_Trajectory (event); };
  edit_callbacks.load =
    [this] { wxCommandEvent event; On_Load_Trajectory (event); };
  edit_callbacks.complete =
    [this] { On_Complete_Progress ( ); };
  edit_callbacks.step_back =
    [this] { On_Step_Teach_Point (-1); };
  edit_callbacks.step_next =
    [this] { On_Step_Teach_Point (1); };
  m_teach_point_command_panel->Set_Callbacks (
    std::move (edit_callbacks));

  m_trajectory_panel = new Trajectory_Control_Panel (
    panel,
    TRAJECTORY_SPEED_DEFAULT_CM_PER_SECOND);
  Trajectory_Control_Panel::Callbacks trajectory_callbacks;
  trajectory_callbacks.go_to_point =
    [this] { wxCommandEvent event; On_Go_To_Trajectory_Point (event); };
  trajectory_callbacks.play =
    [this] { wxCommandEvent event; On_Play_Trajectory (event); };
  trajectory_callbacks.pause_resume =
    [this] { wxCommandEvent event; On_Pause_Resume_Trajectory (event); };
  trajectory_callbacks.stop =
    [this] { wxCommandEvent event; On_Stop_Trajectory (event); };
  trajectory_callbacks.speed_changed =
    [this] { wxCommandEvent event; On_Trajectory_Speed_Changed (event); };
  m_trajectory_panel->Set_Callbacks (
    std::move (trajectory_callbacks));

  auto* sizer = new wxBoxSizer (wxVERTICAL);
  sizer->Add (title, 0, wxEXPAND | wxALL, 8);
  sizer->Add (
    m_teach_point_command_panel,
    0,
    wxEXPAND | wxLEFT | wxRIGHT,
    8);
  sizer->Add (
    m_trajectory_panel,
    0,
    wxEXPAND | wxLEFT | wxRIGHT | wxTOP,
    8);
  sizer->AddStretchSpacer (1);
  panel->SetSizer (sizer);
  panel->FitInside ( );
  Update_Trajectory_Speed_Label ( );
  Update_Trajectory_Point_List ( );
  Update_Trajectory_Status ( );
  return panel;
}

void Robot_Model_Panel_Controller::Apply_Joint_Limits (
  const robot_model::Robot_Kinematic_Params& params)
{
  if( m_joint_panel == nullptr )
  {
    return;
  }

  const auto home_angles = configured_home_input_angles (params);
  for( size_t i = 0; i < ROBOT_JOINT_COUNT; ++i )
  {
    int min_value = slider_limit_at (params.joint_mins, i, DEFAULT_JOINT_MIN);
    int max_value = slider_limit_at (params.joint_maxs, i, DEFAULT_JOINT_MAX);
    if( min_value > max_value )
    {
      std::swap (min_value, max_value);
    }

    const int neutral_value =
      static_cast<int> (std::lround (home_angles[i]));
    m_joint_panel->Set_Joint_Range_And_Value (
      i, min_value, max_value, neutral_value);
  }

  Update_Joint_State_From_Sliders ( );
}

void Robot_Model_Panel_Controller::Update_Joint_State_From_Sliders ( )
{
  if( !m_view || !m_view->Has_Current_Model ( ) || !m_joint_panel ) return;
  const auto joint_state = robot_model::Build_Joint_State_From_Input_Angles (
    m_view->Kinematic_Params ( ), m_joint_panel->Read_Input_Angles ( ));
  const auto apply_result = m_view->Try_Set_Joint_State (joint_state);
  Sync_Joint_Controls_From_State ( );
  if( !apply_result.accepted && apply_result.collision.collided &&
      m_status_text )
  {
    auto status =
      wxString::FromUTF8 (u8"关节运动已阻止：") +
      collision_summary (apply_result.collision);
    if( apply_result.collision.type !=
        robot_model::Robot_Collision_Type::Self_Collision )
    {
      status += wxString::Format (
        wxString::FromUTF8 (u8"，安全距离 %.1f mm"),
        apply_result.clearance_mm);
    }
    m_status_text->SetLabel (status);
  }
  else if( apply_result.accepted && apply_result.recovery_motion &&
           m_status_text )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"已允许脱离碰撞方向的恢复运动"));
  }
  else if( apply_result.accepted && apply_result.collision_checked &&
           m_status_text )
  {
    m_status_text->SetLabel (wxString::FromUTF8 (
      u8"关节运动已应用：碰撞安全检测通过"));
  }
}

void Robot_Model_Panel_Controller::Sync_Joint_Controls_From_State ( )
{
  if( !m_view || !m_view->Has_Current_Model ( ) ) return;
  const auto& joint_state = m_view->Joint_State ( );
  if( m_joint_panel )
  {
    m_joint_panel->Set_Input_Angles (joint_state.input_angles_deg);
  }
  for( size_t i = 0; i < joint_state.input_angles_deg.size ( ); ++i )
  {
    Update_Joint_Value_Label (
      i,
      joint_state.input_angles_deg[i],
      joint_state.effective_angles_deg[i]);
  }
  Update_Cartesian_Pose ( );
}

void Robot_Model_Panel_Controller::Update_Cartesian_Pose ( )
{
  if( !m_cartesian_pose_panel || !m_view ) return;
  robot_model::Matrix4 world_from_flange = { };
  if( m_view->Get_World_From_Flange (&world_from_flange) )
  {
    m_cartesian_pose_panel->Set_World_From_Control_Frame (
      robot_model::Build_World_From_Tool (
        world_from_flange,
        Interaction_Tool ( )));
  }
  else
  {
    m_cartesian_pose_panel->Clear ( );
  }
}

bool Robot_Model_Panel_Controller::Apply_Cartesian_Pose_Target (
  const robot_model::XyzabcPose& target_pose)
{
  if( !m_view || !m_view->Has_Current_Model ( ) ) return false;
  if( Is_Trajectory_Active ( ) ) Stop_Trajectory_Playback ( );

  Select_Display_Page (Main_Display_Page::Robot);
  robot_model::Robot_Pose_IK_Options options;
  options.position_tolerance_mm = 0.001;
  options.orientation_tolerance_deg = 0.001;
  options.damping_mm = 1.0;
  options.max_iterations = 100;
  options.time_budget_ms = 8.0;
  const auto result = m_view->Move_Flange_To_Pose (
    robot_model::Build_World_From_Flange_Target (
      robot_model::Build_Zyx_Pose_Matrix (target_pose),
      Interaction_Tool ( )),
    options);
  if( result.status == robot_model::Robot_IK_Status::Invalid_Model )
  {
    m_status_text->SetLabel (wxString::FromUTF8 (
      u8"世界坐标位姿控制失败：模型无效"));
    return false;
  }

  Sync_Joint_Controls_From_State ( );
  const auto& apply_result = m_view->Last_Joint_State_Apply_Result ( );
  if( !apply_result.accepted && apply_result.collision.collided )
  {
    if( apply_result.state_changed ) Sync_Joint_Controls_From_State ( );
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"世界坐标运动已阻止：") +
      collision_summary (apply_result.collision));
    return false;
  }
  m_status_text->SetLabel (wxString::Format (
    result.Converged ( )
      ? wxString::FromUTF8 (
          u8"世界坐标位姿：位置误差 %.2f mm，姿态误差 %.2f°")
      : wxString::FromUTF8 (
          u8"世界坐标目标受限：位置误差 %.2f mm，姿态误差 %.2f°"),
    result.position_error_mm,
    result.orientation_error_deg));
  return result.Converged ( );
}

void Robot_Model_Panel_Controller::Apply_Flange_Drag_Result (
  const robot_model::Robot_Position_IK_Result& result)
{
  if( result.status == robot_model::Robot_IK_Status::Invalid_Model )
  {
    m_status_text->SetLabel (wxString::FromUTF8 (u8"法兰拖拽失败：模型无效"));
    return;
  }

  const auto& apply_result = m_view->Last_Joint_State_Apply_Result ( );
  if( !apply_result.accepted && apply_result.collision.collided )
  {
    if( apply_result.state_changed ) Sync_Joint_Controls_From_State ( );
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"法兰拖拽已阻止：") +
      collision_summary (apply_result.collision));
    return;
  }

  Sync_Joint_Controls_From_State ( );

  m_status_text->SetLabel (wxString::Format (
    result.Converged ( )
      ? wxString::FromUTF8 (u8"法兰拖拽：IK 误差 %.2f mm")
      : wxString::FromUTF8 (u8"法兰目标受限：当前误差 %.2f mm"),
    result.position_error_mm));
}

void Robot_Model_Panel_Controller::Apply_Flange_Pose_Drag_Result (
  const robot_model::Robot_Pose_IK_Result& result)
{
  if( result.status == robot_model::Robot_IK_Status::Invalid_Model )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"法兰姿态拖拽失败：模型无效"));
    return;
  }
  const auto& apply_result = m_view->Last_Joint_State_Apply_Result ( );
  if( !apply_result.accepted && apply_result.collision.collided )
  {
    if( apply_result.state_changed ) Sync_Joint_Controls_From_State ( );
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"姿态拖拽已阻止：") +
      collision_summary (apply_result.collision));
    return;
  }
  Sync_Joint_Controls_From_State ( );
  m_status_text->SetLabel (wxString::Format (
    result.Converged ( )
      ? wxString::FromUTF8 (u8"姿态拖拽：位置 %.2f mm，角度 %.2f°")
      : wxString::FromUTF8 (u8"姿态目标受限：位置 %.2f mm，角度 %.2f°"),
    result.position_error_mm,
    result.orientation_error_deg));
}

void Robot_Model_Panel_Controller::Update_Joint_Value_Label (size_t index,
                                                 double input_angle,
                                                 double effective_angle)
{
  if( m_joint_panel )
  {
    m_joint_panel->Set_Joint_Value_Label (index, input_angle, effective_angle);
  }
}

std::array<double, 6> Robot_Model_Panel_Controller::Read_Joint_Input_Angles ( ) const
{
  if( m_view && m_view->Has_Current_Model ( ) )
  {
    return m_view->Joint_State ( ).input_angles_deg;
  }
  return m_joint_panel ? m_joint_panel->Read_Input_Angles ( )
                       : std::array<double, 6> {};
}

robot_model::Robot_Joint_State_Apply_Result
Robot_Model_Panel_Controller::Apply_Joint_Input_Angles_To_Sliders (
  const std::array<double, 6>& input_angles_deg)
{
  if( m_joint_panel )
  {
    m_joint_panel->Set_Input_Angles (input_angles_deg);
  }

  if( !m_view || !m_view->Has_Current_Model ( ) ) return { };
  const auto joint_state = robot_model::Build_Joint_State_From_Input_Angles (
    m_view->Kinematic_Params ( ), input_angles_deg);
  const auto result = m_view->Try_Set_Joint_State (joint_state);
  Sync_Joint_Controls_From_State ( );
  return result;
}

void Robot_Model_Panel_Controller::Set_Joint_Controls_Enabled (bool enabled)
{
  if( m_joint_panel )
  {
    m_joint_panel->Set_Joint_Controls_Enabled (enabled);
  }
  if( m_cartesian_pose_panel )
  {
    m_cartesian_pose_panel->Set_Pose_Controls_Enabled (enabled);
  }
  if( m_teach_point_command_panel )
  {
    const auto point_count =
      m_teach_point_store.Point_Count (m_current_model_id);
    const auto current_index = Current_Progress_Point_Index ( );
    m_teach_point_command_panel->Refresh_Command_State (
      enabled && !m_current_model_id.empty ( ),
      Selected_Teach_Point_Indices ( ).size ( ),
      point_count,
      current_index < point_count && current_index > 0,
      current_index < point_count &&
        current_index + 1 < point_count);
  }
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_List_Enabled (enabled);
  }
}

void Robot_Model_Panel_Controller::Update_Trajectory_Status ( )
{
  const bool active = Is_Trajectory_Active ( );
  const auto selections = Selected_Teach_Point_Indices ( );
  if( m_teach_point_command_panel )
  {
    const auto point_count =
      m_teach_point_store.Point_Count (m_current_model_id);
    const auto current_index = Current_Progress_Point_Index ( );
    m_teach_point_command_panel->Refresh_Command_State (
      !active && !m_current_model_id.empty ( ),
      selections.size ( ),
      point_count,
      current_index < point_count && current_index > 0,
      current_index < point_count &&
        current_index + 1 < point_count);
  }
  if( m_trajectory_panel == nullptr )
  {
    return;
  }

  wxString suffix = "";
  if( m_playback_cloud_switch_blocked )
  {
    suffix = " / cloud error";
  }
  else if( m_waiting_for_playback_cloud )
  {
    suffix = " / switching cloud";
  }
  else if( m_trajectory_timer.IsRunning ( ) )
  {
    suffix = " / playing";
  }
  else if( m_trajectory_session.Is_Paused ( ) )
  {
    suffix = " / paused";
  }
  else if( !m_trajectory_session.Has_Playable_Path ( ) )
  {
    suffix = " / need 2";
  }

  m_trajectory_panel->Set_Status_Text (
    wxString::Format ("Points: %zu%s",
                      m_trajectory_session.Point_Count ( ),
                      suffix));
  m_trajectory_panel->Refresh_Command_State (
    active,
    m_trajectory_session.Is_Paused ( ),
    m_trajectory_timer.IsRunning ( ),
    m_trajectory_session.Point_Count ( ),
    selections.size ( ) == 1);
}

void Robot_Model_Panel_Controller::Update_Trajectory_Speed_Label ( )
{
  if( m_trajectory_panel == nullptr )
  {
    return;
  }

  m_trajectory_panel->Set_Speed_Label (
    wxString::Format (
      "Speed: %.2f m/s", Get_Trajectory_Speed_Mps ( )));
}

void Robot_Model_Panel_Controller::Update_Trajectory_Point_List ( )
{
  if( !m_teach_point_list_panel ) return;

  std::vector<wxString> labels;
  std::vector<robot_model::Robot_Teach_Point_Type> types;
  std::vector<wxString> cloud_names;
  std::vector<std::string> cloud_keys;
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  labels.reserve (points.size ( ));
  types.reserve (points.size ( ));
  cloud_names.reserve (points.size ( ));
  cloud_keys.reserve (points.size ( ));
  for( const auto& point : points )
  {
    labels.push_back (wxString::FromUTF8 (
      robot_model::Format_Teach_Point_Name (point.id).c_str ( )));
    types.push_back (point.type);
    if( !point.point_cloud_name.empty ( ) )
    {
      cloud_names.push_back (
        wxString::FromUTF8 (point.point_cloud_name.c_str ( )));
    }
    else if( !point.point_cloud_path.empty ( ) )
    {
      cloud_names.push_back (wxString (
        std::filesystem::u8path (point.point_cloud_path)
          .stem ( ).wstring ( )));
    }
    else
    {
      cloud_names.push_back (wxString::FromUTF8 (u8"未绑定点云"));
    }
    cloud_names.back ( ) += wxString::FromUTF8 (u8" · ");
    cloud_names.back ( ) += point.has_template &&
      !point.template_name.empty ( )
        ? wxString::FromUTF8 (point.template_name.c_str ( ))
        : wxString::FromUTF8 (u8"未绑定模板");
    cloud_keys.push_back (
      point.point_cloud_path.empty ( )
        ? std::string ("__unbound__")
        : point.point_cloud_path);
  }
  m_teach_point_list_panel->Set_Point_Names (
    labels, types, cloud_names, cloud_keys);
  m_teach_point_list_panel->Set_Dirty (
    m_dirty_progress_models.count (m_current_model_id) != 0);
  Update_Teach_Point_Details ( );
}

void Robot_Model_Panel_Controller::Update_Teach_Point_Details ( )
{
  if( !m_teach_point_list_panel ) return;
  const auto selections = Selected_Teach_Point_Indices ( );
  if( selections.empty ( ) )
  {
    m_teach_point_list_panel->Set_Point_Details (
      wxString::FromUTF8 (u8"未选择"),
      wxString::FromUTF8 (u8"未选择"),
      wxString::FromUTF8 (u8"未选择"),
      wxString::FromUTF8 (u8"未选择"),
      robot_model::Robot_Teach_Point_Type::Motion,
      false);
    m_teach_point_list_panel->Set_Point_Pose ({ }, false);
    return;
  }
  if( selections.size ( ) > 1 )
  {
    m_teach_point_list_panel->Set_Point_Details (
      wxString::FromUTF8 (u8"多个"),
      wxString::FromUTF8 (u8"多个"),
      wxString::FromUTF8 (u8"多个"),
      wxString::FromUTF8 (u8"多个"),
      robot_model::Robot_Teach_Point_Type::Motion,
      false);
    m_teach_point_list_panel->Set_Point_Pose ({ }, false);
    return;
  }

  const int selection = selections.front ( );
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( selection < 0 ||
      static_cast<std::size_t> (selection) >= points.size ( ) )
  {
    m_teach_point_list_panel->Set_Point_Details (
      wxString::FromUTF8 (u8"未选择"),
      wxString::FromUTF8 (u8"未选择"),
      wxString::FromUTF8 (u8"未选择"),
      wxString::FromUTF8 (u8"未选择"),
      robot_model::Robot_Teach_Point_Type::Motion,
      false);
    m_teach_point_list_panel->Set_Point_Pose ({ }, false);
    return;
  }

  const auto& point = points[static_cast<std::size_t> (selection)];
  if( m_teach_point_command_panel )
  {
    m_teach_point_command_panel->Set_Selected_Point_Type (point.type);
  }
  wxString coordinate = wxString::FromUTF8 (u8"未绑定");
  if( point.has_coordinate_frame )
  {
    coordinate = wxString::FromUTF8 (
      point.coordinate_frame_name.empty ( )
        ? point.coordinate_frame_id.c_str ( )
        : point.coordinate_frame_name.c_str ( ));
  }

  wxString cloud = wxString::FromUTF8 (u8"未绑定");
  if( !point.point_cloud_path.empty ( ) )
  {
    const auto cloud_path =
      std::filesystem::u8path (point.point_cloud_path);
    cloud = point.point_cloud_name.empty ( )
      ? wxString (cloud_path.stem ( ).wstring ( ))
      : wxString::FromUTF8 (point.point_cloud_name.c_str ( ));
  }
  m_teach_point_list_panel->Set_Point_Details (
    teach_point_type_label (point.type),
    coordinate,
    cloud,
    point.has_template && !point.template_name.empty ( )
      ? wxString::FromUTF8 (point.template_name.c_str ( ))
      : wxString::FromUTF8 (u8"未绑定"),
    point.type,
    true);
  robot_model::XyzabcPose display_pose = point.world_pose;
  bool has_display_pose = point.has_world_pose;
  const auto* display_coordinate = robot_model::Find_Tool_Coordinate (
    m_tool_configuration,
    m_teach_pose_coordinate_id);
  if( has_display_pose && display_coordinate )
  {
    display_pose = robot_model::Build_Xyzabc_From_Zyx_Matrix (
      robot_model::Build_World_From_Tool (
        robot_model::Build_Zyx_Pose_Matrix (point.world_pose),
        *display_coordinate));
  }
  m_teach_point_list_panel->Set_Point_Pose (
    display_pose, has_display_pose);
}

void Robot_Model_Panel_Controller::On_Teach_Point_Selection_Changed ( )
{
  Update_Trajectory_Status ( );
  Update_Teach_Point_Details ( );
  const int selection = Selected_Teach_Point_Index ( );
  if( selection != wxNOT_FOUND && selection >= 0 &&
      !Is_Trajectory_Active ( ) )
  {
    Apply_Teach_Point_Bindings_Keeping_Display (
      static_cast<std::size_t> (selection));
  }
}

void Robot_Model_Panel_Controller::On_Teach_Pose_Coordinate_Changed (
  int selection)
{
  if( selection < 0 ||
      static_cast<std::size_t> (selection) >=
        m_tool_configuration.tools.size ( ) )
  {
    return;
  }
  m_teach_pose_coordinate_id =
    m_tool_configuration.tools[static_cast<std::size_t> (selection)].id;
  Update_Teach_Point_Details ( );
}

void Robot_Model_Panel_Controller::Bind_Template_To_Teach_Point_Cloud (
  std::size_t point_index)
{
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( point_index >= points.size ( ) ||
      points[point_index].point_cloud_path.empty ( ) ||
      !m_point_cloud_overlay_toolbar )
  {
    return;
  }

  robot_model::Template_Configuration configuration;
  std::string error_message;
  if( !robot_model::Load_Template_Configuration (
        robot_model::Template_Configuration_Path ( ),
        &configuration,
        &error_message) )
  {
    wxMessageBox (
      wxString::FromUTF8 (error_message.c_str ( )),
      wxString::FromUTF8 (u8"模板配置加载失败"),
      wxOK | wxICON_ERROR,
      this);
    return;
  }
  if( configuration.templates.empty ( ) )
  {
    wxMessageBox (
      wxString::FromUTF8 (u8"还没有可用模板，请先在设置中创建模板。"),
      wxString::FromUTF8 (u8"绑定模板"),
      wxOK | wxICON_INFORMATION,
      this);
    return;
  }

  wxArrayString choices;
  int current_selection = 0;
  for( std::size_t index = 0;
       index < configuration.templates.size ( );
       ++index )
  {
    choices.Add (wxString::FromUTF8 (
      configuration.templates[index].name.c_str ( )));
    if( points[point_index].has_template &&
        points[point_index].template_id ==
          configuration.templates[index].id )
    {
      current_selection = static_cast<int> (index);
    }
  }
  wxSingleChoiceDialog dialog (
    this,
    wxString::FromUTF8 (u8"请选择当前点云要绑定的模板"),
    wxString::FromUTF8 (u8"绑定模板"),
    choices);
  dialog.SetSelection (current_selection);
  if( dialog.ShowModal ( ) != wxID_OK ||
      dialog.GetSelection ( ) == wxNOT_FOUND )
  {
    return;
  }

  const auto& selected = configuration.templates[
    static_cast<std::size_t> (dialog.GetSelection ( ))];
  const std::string cloud_path = points[point_index].point_cloud_path;
  const std::string cloud_name = points[point_index].point_cloud_name;
  if( !m_point_cloud_overlay_toolbar->Bind_Point_Cloud_Template (
        std::filesystem::u8path (cloud_path),
        cloud_name,
        selected,
        &error_message) )
  {
    m_point_cloud_overlay_toolbar->Refresh_Template_Configuration ( );
    wxMessageBox (
      wxString::FromUTF8 (error_message.c_str ( )),
      wxString::FromUTF8 (u8"模板绑定失败"),
      wxOK | wxICON_ERROR,
      this);
    return;
  }

  const std::size_t updated =
    m_teach_point_store.Apply_Template_To_Point_Cloud (
      m_current_model_id,
      cloud_path,
      selected.id,
      selected.name,
      selected.reference_pose);
  if( updated > 0 )
  {
    Set_Progress_Dirty (true);
  }
  Update_Trajectory_Point_List ( );
  if( m_status_text )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"点云已绑定模板：") +
      wxString::FromUTF8 (selected.name.c_str ( )) +
      wxString::Format (
        wxString::FromUTF8 (u8"（已更新 %zu 个示教点）"),
        updated));
  }
}

void Robot_Model_Panel_Controller::Unbind_Template_From_Teach_Point_Cloud (
  std::size_t point_index)
{
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( point_index >= points.size ( ) ||
      points[point_index].point_cloud_path.empty ( ) ||
      !m_point_cloud_overlay_toolbar )
  {
    return;
  }
  const std::string cloud_path = points[point_index].point_cloud_path;
  std::string error_message;
  if( !m_point_cloud_overlay_toolbar->Unbind_Point_Cloud_Template (
        std::filesystem::u8path (cloud_path), &error_message) )
  {
    m_point_cloud_overlay_toolbar->Refresh_Template_Configuration ( );
    wxMessageBox (
      wxString::FromUTF8 (error_message.c_str ( )),
      wxString::FromUTF8 (u8"模板解绑失败"),
      wxOK | wxICON_ERROR,
      this);
    return;
  }

  const std::size_t updated =
    m_teach_point_store.Apply_Template_To_Point_Cloud (
      m_current_model_id,
      cloud_path,
      { },
      { },
      { });
  if( updated > 0 )
  {
    Set_Progress_Dirty (true);
  }
  Update_Trajectory_Point_List ( );
  if( m_status_text )
  {
    m_status_text->SetLabel (wxString::Format (
      wxString::FromUTF8 (
        u8"点云已解绑模板，已清除 %zu 个示教点的模板关系"),
      updated));
  }
}

bool Robot_Model_Panel_Controller::Apply_Teach_Point_Bindings (
  std::size_t index,
  bool require_point_cloud)
{
  const auto& points = m_teach_point_store.Points (m_current_model_id);
  if( index >= points.size ( ) ) return false;
  const auto& point = points[index];

  if( point.has_coordinate_frame && !point.coordinate_frame_id.empty ( ) )
  {
    m_interaction_tool_id = point.coordinate_frame_id;
    if( m_interaction_coordinate_choice )
    {
      for( std::size_t choice_index = 0;
           choice_index < m_tool_configuration.tools.size ( );
           ++choice_index )
      {
        if( m_tool_configuration.tools[choice_index].id ==
            point.coordinate_frame_id )
        {
          m_interaction_coordinate_choice->SetSelection (
            static_cast<int> (choice_index));
          break;
        }
      }
    }
    if( m_view )
    {
      robot_model::Tool_Coordinate_Profile coordinate;
      coordinate.id = point.coordinate_frame_id;
      coordinate.name = point.coordinate_frame_name;
      coordinate.flange_from_tool_pose =
        point.flange_from_coordinate_pose;
      m_view->Set_Interaction_Coordinate (coordinate);
    }
  }

  if( require_point_cloud && point.point_cloud_path.empty ( ) )
  {
    if( m_status_text )
    {
      m_status_text->SetLabel (wxString::Format (
        wxString::FromUTF8 (u8"P[%zu] 没有绑定点云，播放已暂停"),
        point.id));
    }
    return false;
  }
  if( !point.point_cloud_path.empty ( ) &&
      m_point_cloud_overlay_toolbar )
  {
    std::string error_message;
    const bool loaded =
      m_point_cloud_overlay_toolbar->Load_Bound_Point_Cloud (
          std::filesystem::u8path (point.point_cloud_path),
          point.point_cloud_name,
          &error_message);
    if( !loaded )
    {
      if( m_status_text )
      {
        m_status_text->SetLabel (
          wxString::FromUTF8 (u8"加载绑定点云失败：") +
          wxString::FromUTF8 (error_message.c_str ( )));
      }
      return false;
    }
  }
  return true;
}

bool Robot_Model_Panel_Controller::Apply_Teach_Point_Bindings_Keeping_Display (
  std::size_t index,
  bool require_point_cloud)
{
  const auto display_page = m_display_page;
  const bool applied =
    Apply_Teach_Point_Bindings (index, require_point_cloud);
  if( m_display_page != display_page )
    Select_Display_Page (display_page);
  return applied;
}

void Robot_Model_Panel_Controller::Sync_Trajectory_From_Teach_Points ( )
{
  std::vector<std::array<double, 6>> joint_points;
  const auto& teach_points = m_teach_point_store.Points (m_current_model_id);
  joint_points.reserve (teach_points.size ( ));
  for( const auto& point : teach_points )
  {
    joint_points.push_back (point.joint_angles_deg);
  }
  m_trajectory_session.Set_Points (std::move (joint_points));
}

int Robot_Model_Panel_Controller::Selected_Teach_Point_Index ( ) const
{
  return m_teach_point_list_panel
    ? m_teach_point_list_panel->Selected_Point_Index ( )
    : wxNOT_FOUND;
}

std::vector<int> Robot_Model_Panel_Controller::Selected_Teach_Point_Indices ( ) const
{
  return m_teach_point_list_panel
    ? m_teach_point_list_panel->Selected_Point_Indices ( )
    : std::vector<int> { };
}

std::size_t Robot_Model_Panel_Controller::Current_Progress_Point_Index ( ) const
{
  return robot_model::Find_Matching_Joint_Point_Index (
    m_trajectory_session.Points ( ),
    Read_Joint_Input_Angles ( ),
    0.5);
}

bool Robot_Model_Panel_Controller::Read_Current_Teach_Point (
  std::array<double, 6>* joint_angles,
  robot_model::XyzabcPose* world_pose) const
{
  if( !joint_angles || !world_pose || !m_view ||
      m_current_model_id.empty ( ) )
  {
    return false;
  }
  robot_model::Matrix4 world_from_flange = { };
  if( !m_view->Get_World_From_Flange (&world_from_flange) )
  {
    return false;
  }
  *joint_angles = Read_Joint_Input_Angles ( );
  *world_pose =
    robot_model::Build_Xyzabc_From_Zyx_Matrix (world_from_flange);
  return true;
}

bool Robot_Model_Panel_Controller::Capture_Current_Teach_Bindings (
  std::string* point_cloud_path,
  std::string* point_cloud_name,
  robot_model::Tool_Coordinate_Profile* coordinate,
  robot_model::Template_Profile* template_profile)
{
  if( !point_cloud_path || !point_cloud_name || !coordinate ||
      !template_profile ||
      !m_point_cloud_overlay_toolbar ||
      !m_point_cloud_overlay_toolbar->Has_Point_Cloud ( ) )
  {
    if( m_status_text )
    {
      m_status_text->SetLabel (wxString::FromUTF8 (
        u8"请先加载或采集点云，再记录 Progress 点"));
    }
    return false;
  }

  std::filesystem::path bound_path;
  std::string error_message;
  if( !m_point_cloud_overlay_toolbar->Current_Point_Cloud_Binding (
        &bound_path, point_cloud_name, &error_message) )
  {
    if( m_status_text )
    {
      m_status_text->SetLabel (
        wxString::FromUTF8 (u8"绑定点云失败：") +
        wxString::FromUTF8 (error_message.c_str ( )));
    }
    return false;
  }

  *point_cloud_path = bound_path.u8string ( );
  *coordinate = Interaction_Tool ( );
  *template_profile = { };
  bool has_template_binding = false;
  if( !m_point_cloud_overlay_toolbar->Current_Template_Profile (
        template_profile, &has_template_binding) &&
      has_template_binding )
  {
    if( m_status_text )
    {
      m_status_text->SetLabel (wxString::FromUTF8 (
        u8"当前点云绑定的模板已被删除，请重新绑定模板"));
    }
    return false;
  }
  return true;
}

void Robot_Model_Panel_Controller::Set_Progress_Dirty (bool dirty)
{
  if( m_current_model_id.empty ( ) ) return;
  if( dirty )
  {
    Invalidate_Completed_Progress ( );
    m_dirty_progress_models.insert (m_current_model_id);
  }
  else
  {
    m_dirty_progress_models.erase (m_current_model_id);
  }
  if( m_teach_point_list_panel )
  {
    m_teach_point_list_panel->Set_Dirty (dirty);
  }
}

void Robot_Model_Panel_Controller::Invalidate_Completed_Progress ( )
{
  if( !m_progress_completed ) return;
  if( m_progress_run_controller.Is_Motion_Active() ) return;
  m_progress_completed = false;
  if( m_run_progress_panel )
    m_run_progress_panel->Set_Progress_Ready (false);
  if( m_right_tool_panel )
  {
    m_right_tool_panel->Set_Run_Tool_Enabled (false);
  }
}

double Robot_Model_Panel_Controller::Get_Trajectory_Speed_Mps ( ) const
{
  if( m_trajectory_panel == nullptr )
  {
    return 1.0;
  }
  return std::clamp (
    m_trajectory_panel->Speed_Meters_Per_Second ( ), 0.0, 2.0);
}

int Robot_Model_Panel_Controller::Get_Trajectory_Timer_Interval_Ms ( ) const
{
  const double speed_mps = Get_Trajectory_Speed_Mps ( );
  if( speed_mps <= 0.0 ) return TRAJECTORY_TIMER_MS;
  const int interval = static_cast<int> (
    std::lround (static_cast<double> (TRAJECTORY_TIMER_MS) / speed_mps));
  return std::max (interval, 1);
}

bool Robot_Model_Panel_Controller::Is_Trajectory_Active ( ) const
{
  return m_trajectory_session.Is_Active ( );
}

void Robot_Model_Panel_Controller::Stop_Trajectory_Playback ( )
{
  if( m_trajectory_timer.IsRunning ( ) )
  {
    m_trajectory_timer.Stop ( );
  }

  m_trajectory_session.Stop ( );
  m_teach_step_preview_active = false;
  m_speed_zero_paused_playback = false;
  m_playback_waypoint_frame_indices.clear ( );
  m_playback_waypoint_point_indices.clear ( );
  m_next_playback_waypoint_index = 0;
  m_playback_cloud_switches.clear ( );
  m_next_playback_cloud_switch = 0;
  m_waiting_for_playback_cloud = false;
  m_playback_cloud_switch_blocked = false;
  Update_Trajectory_Status ( );
  Set_Joint_Controls_Enabled (!m_hardware_step_preview_active);
}

void Robot_Model_Panel_Controller::Resize_Right_Tool (int requested_width)
{
  if( !m_content_splitter || !m_right_tool_panel ||
      !m_content_splitter->IsSplit ( ) ) return;

  const int total_width = m_content_splitter->GetClientSize ( ).x;
  if( total_width <= RIGHT_TOOL_COLLAPSED_WIDTH ) return;

  const int maximum_right_width = std::max (
    RIGHT_TOOL_COLLAPSED_WIDTH,
    total_width - DISPLAY_MINIMUM_WIDTH);
  const int right_width = std::clamp (
    requested_width,
    RIGHT_TOOL_COLLAPSED_WIDTH,
    maximum_right_width);
  m_content_splitter->SetSashPosition (total_width - right_width, true);
}

void Robot_Model_Panel_Controller::Resize_Teach_Point_List (bool collapsed)
{
  if( !m_workspace_splitter || !m_teach_point_list_panel ||
      !m_workspace_splitter->IsSplit ( ) ) return;

  const int total_width = m_workspace_splitter->GetClientSize ( ).x;
  if( total_width <= TEACH_POINT_COLLAPSED_WIDTH ) return;

  if( collapsed )
  {
    const int current_width = m_teach_point_list_panel->GetSize ( ).x;
    if( current_width > TEACH_POINT_COLLAPSED_WIDTH + 40 )
    {
      m_expanded_teach_point_width = current_width;
    }
  }

  const int maximum_left_width = std::max (
    TEACH_POINT_COLLAPSED_WIDTH,
    total_width - WORKSPACE_MINIMUM_WIDTH);
  const int requested_width = collapsed
    ? TEACH_POINT_COLLAPSED_WIDTH
    : std::max (m_expanded_teach_point_width,
                TEACH_POINT_DEFAULT_EXPANDED_WIDTH);
  const int left_width = std::clamp (
    requested_width,
    TEACH_POINT_COLLAPSED_WIDTH,
    maximum_left_width);
  m_workspace_splitter->SetSashPosition (left_width, true);
}

const robot_model::Tool_Coordinate_Profile&
Robot_Model_Panel_Controller::Active_Tool ( ) const
{
  const auto* tool = robot_model::Find_Tool_Coordinate (
    m_tool_configuration,
    m_tool_configuration.active_tool_id);
  if( tool )
  {
    return *tool;
  }
  static const robot_model::Tool_Coordinate_Profile fallback = {
    "flange", "Flange", {}};
  return fallback;
}

const robot_model::Tool_Coordinate_Profile&
Robot_Model_Panel_Controller::Interaction_Tool ( ) const
{
  const auto* tool = robot_model::Find_Tool_Coordinate (
    m_tool_configuration,
    m_interaction_tool_id);
  return tool ? *tool : Active_Tool ( );
}

void Robot_Model_Panel_Controller::On_Interaction_Coordinate_Changed (wxCommandEvent&)
{
  if( !m_interaction_coordinate_choice ) return;
  const int selection = m_interaction_coordinate_choice->GetSelection ( );
  if( selection == wxNOT_FOUND ||
      static_cast<std::size_t> (selection) >=
        m_tool_configuration.tools.size ( ) )
  {
    return;
  }

  m_interaction_tool_id =
    m_tool_configuration.tools[static_cast<std::size_t> (selection)].id;
  if( m_view )
  {
    m_view->Set_Interaction_Coordinate (Interaction_Tool ( ));
  }
  if( m_cartesian_pose_panel )
  {
    m_cartesian_pose_panel->Set_Control_Frame_Name (
      Interaction_Tool ( ).name);
    Update_Cartesian_Pose ( );
  }
  if( m_status_text )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"操作坐标系：") +
      wxString::FromUTF8 (Interaction_Tool ( ).name.c_str ( )));
  }
}

void Robot_Model_Panel_Controller::Refresh_Interaction_Coordinate_Choices ( )
{
  robot_model::Normalize_Tool_Coordinate_Configuration (
    &m_tool_configuration);
  if( !robot_model::Find_Tool_Coordinate (
        m_tool_configuration,
        m_interaction_tool_id) )
  {
    m_interaction_tool_id = m_tool_configuration.active_tool_id;
  }

  if( m_interaction_coordinate_choice )
  {
    m_interaction_coordinate_choice->Clear ( );
    int selection = wxNOT_FOUND;
    for( std::size_t index = 0;
         index < m_tool_configuration.tools.size ( );
         ++index )
    {
      const auto& tool = m_tool_configuration.tools[index];
      m_interaction_coordinate_choice->Append (
        wxString::FromUTF8 (tool.name.c_str ( )));
      if( tool.id == m_interaction_tool_id )
      {
        selection = static_cast<int> (index);
      }
    }
    if( selection != wxNOT_FOUND )
    {
      m_interaction_coordinate_choice->SetSelection (selection);
    }
  }
}

void Robot_Model_Panel_Controller::Refresh_Teach_Pose_Coordinate_Choices ( )
{
  if( !robot_model::Find_Tool_Coordinate (
        m_tool_configuration,
        m_teach_pose_coordinate_id) )
  {
    m_teach_pose_coordinate_id = m_tool_configuration.active_tool_id;
  }
  if( !m_teach_point_list_panel ) return;

  std::vector<wxString> names;
  names.reserve (m_tool_configuration.tools.size ( ));
  int selection = wxNOT_FOUND;
  for( std::size_t index = 0;
       index < m_tool_configuration.tools.size ( );
       ++index )
  {
    const auto& tool = m_tool_configuration.tools[index];
    names.push_back (wxString::FromUTF8 (tool.name.c_str ( )));
    if( tool.id == m_teach_pose_coordinate_id )
    {
      selection = static_cast<int> (index);
    }
  }
  m_teach_point_list_panel->Set_Pose_Coordinate_Choices (
    names, selection);
  Update_Teach_Point_Details ( );
}

void Robot_Model_Panel_Controller::Apply_Active_Tool ( )
{
  robot_model::Normalize_Tool_Coordinate_Configuration (
    &m_tool_configuration);
  Refresh_Interaction_Coordinate_Choices ( );
  Refresh_Teach_Pose_Coordinate_Choices ( );
  const auto& tool = Active_Tool ( );
  if( m_view )
  {
    m_view->Set_Tool_Coordinate (tool);
    m_view->Set_Interaction_Coordinate (Interaction_Tool ( ));
  }
  if( m_cartesian_pose_panel )
  {
    m_cartesian_pose_panel->Set_Control_Frame_Name (
      Interaction_Tool ( ).name);
  }
  if( m_tool_panel )
  {
    m_tool_panel->Set_Tool_Coordinates (m_tool_configuration);
  }
  Apply_Tool_Visualization ( );
}

void Robot_Model_Panel_Controller::Apply_Tool_Visualization ( )
{
  robot_model::Normalize_Tool_Visualization_Configuration (
    &m_tool_visualization_configuration);
  const auto* bound_tool = robot_model::Find_Tool_Coordinate (
    m_tool_configuration,
    m_tool_visualization_configuration.fov.tool_coordinate_id);
  if( m_view )
  {
    m_view->Set_Tool_Frame_Configuration (
      m_tool_visualization_configuration.tool_frame);
    m_view->Set_Fov_Configuration (
      m_tool_visualization_configuration.fov,
      bound_tool);
  }
}

void Robot_Model_Panel_Controller::Load_Model_List ( )
{
  const auto root = robot_model::Find_Robot_Root ( );
  m_models.clear ( );

  if( root.empty ( ) || !std::filesystem::exists (root) )
  {
    m_status_text->SetLabel ("Resource/Robot not found");
    return;
  }

  m_models = robot_model::Scan_Models_In_Directory (root);
  std::sort (
    m_models.begin ( ), m_models.end ( ),
    [] (const auto& lhs, const auto& rhs)
    {
      return lhs.display_name < rhs.display_name;
    });

  if( m_models.empty ( ) )
  {
    m_status_text->SetLabel ("No robot model folders found");
  }
}

void Robot_Model_Panel_Controller::Load_Default_Model ( )
{
  if( Load_Bound_Robot_Model ( ) )
    return;

  const auto default_model = std::find_if (
    m_models.begin ( ), m_models.end ( ),
    [] (const robot_model::Robot_Model_Info& model)
    {
      return model_id (model) == DEFAULT_ROBOT_MODEL_ID;
    });
  if( default_model == m_models.end ( ) )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"默认机械臂 KR10_R1100_2 未找到"));
    if( m_reset_robot_button ) m_reset_robot_button->Enable (false);
    m_right_tool_panel->Set_Robot_Tool_Enabled (false);
    return;
  }

  std::string error_message;
  const auto model_index = static_cast<size_t> (
    std::distance (m_models.begin ( ), default_model));
  if( !Load_Model (model_index, &error_message) )
  {
    m_status_text->SetLabel (
      wxString::FromUTF8 (u8"默认机械臂模型加载失败"));
    m_right_tool_panel->Set_Robot_Tool_Enabled (false);
  }
}

bool Robot_Model_Panel_Controller::Load_Bound_Robot_Model (
  std::string* error_message)
{
  const auto& connection_configuration =
    m_robot_connection->Configuration();
  const auto bound_model = std::find_if (
    m_models.begin ( ),
    m_models.end ( ),
    [this, &connection_configuration]
    (const robot_model::Robot_Model_Info& model)
    {
      return model_id(model) == connection_configuration.model_id;
    });
  if( bound_model == m_models.end ( ) )
  {
    if( error_message )
      *error_message =
        "The bound robot model was not found: " +
        connection_configuration.model_id;
    return false;
  }
  if( m_current_model_id == connection_configuration.model_id &&
      m_view && m_view->Has_Current_Model ( ) )
  {
    if( error_message ) error_message->clear ( );
    return true;
  }
  return Load_Model (
    static_cast<std::size_t> (
      std::distance (m_models.begin ( ), bound_model)),
    error_message);
}

bool Robot_Model_Panel_Controller::Load_Model (
  size_t model_index,
  std::string* error_message)
{
  if( error_message )
  {
    error_message->clear ( );
  }
  if( model_index >= m_models.size ( ) )
  {
    if( error_message )
    {
      *error_message = "Invalid robot model selection";
    }
    return false;
  }

  const auto& model = m_models[model_index];
  if( !validate_model_files (model, error_message) )
  {
    return false;
  }

  Invalidate_Completed_Progress ( );

  if( Is_Trajectory_Active ( ) )
  {
    Stop_Trajectory_Playback ( );
  }

  m_view->Load_Model (model);
  Apply_Joint_Limits (m_view->Kinematic_Params ( ));
  if( m_cartesian_pose_panel )
  {
    m_cartesian_pose_panel->Set_Position_Range (
      cartesian_position_limit_mm (m_view->Kinematic_Params ( )));
  }
  if( m_point_cloud_overlay_toolbar &&
      m_point_cloud_overlay_toolbar->Has_Point_Cloud ( ) )
  {
    m_point_cloud_overlay_toolbar->Attach_Renderer (
      m_view->Scene_Renderer ( ));
  }
  if( m_camera_pose_controller.Is_Visible ( ) )
  {
    m_camera_pose_controller.Attach_Renderer (m_view->Scene_Renderer ( ));
  }

  m_current_model_id = model_id (model);
  Sync_Trajectory_From_Teach_Points ( );
  Update_Trajectory_Point_List ( );
  Update_Trajectory_Status ( );
  m_model_name_text->SetLabel (
    wxString::FromUTF8 (u8"当前机械臂：") +
    wxString::FromUTF8 (model.display_name.c_str ( )));
  m_status_text->SetLabel (wxString::FromUTF8 (u8"模型加载成功"));
  if( m_reset_robot_button ) m_reset_robot_button->Enable (true);
  m_right_tool_panel->Set_Robot_Tool_Enabled (true);
  Refresh_Kuka_Status_Table ( );
  Select_Display_Page (Main_Display_Page::Robot);
  return true;
}

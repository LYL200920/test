#include "run_image_processing.h"
#include "camera_params.h"

#include <wx/image.h>
#include <wx/init.h>
#include <wx/log.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

struct Temporary_Directory
{
  std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("run_images_" + std::to_string(
      std::chrono::high_resolution_clock::now().time_since_epoch().count()));
  Temporary_Directory() { std::filesystem::create_directory(path); }
  ~Temporary_Directory()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
};

application::Run_Captured_Frame Capture(int id = 1)
{
  const std::vector<std::uint8_t> pixels = {11, 22, 33, 11, 22, 33,
                                          11, 22, 33, 11, 22, 33};
  application::Run_Captured_Frame capture;
  capture.frame = std::make_shared<jutze_camera::camera_frame>(
    "test", "serial", 2, 2,
    static_cast<unsigned int>(jutze_camera::pixel_format::rgb8),
    42, pixels.data(), pixels.size());
  capture.point.id = id;
  capture.point.has_world_pose = true;
  capture.point.world_pose = {};
  return capture;
}

application::Run_Captured_Frame Colored_Capture(int id, unsigned char color)
{
  auto capture = Capture(id);
  const std::vector<std::uint8_t> pixels(12, color);
  capture.frame = std::make_shared<jutze_camera::camera_frame>(
    "test", "serial", 2, 2,
    static_cast<unsigned int>(jutze_camera::pixel_format::rgb8),
    42, pixels.data(), pixels.size());
  return capture;
}

robot_model::Mosaic_Group Group(std::size_t id, std::vector<std::size_t> points)
{
  robot_model::Mosaic_Group group;
  group.id = id;
  group.name = "group " + std::to_string(id);
  group.point_ids = std::move(points);
  return group;
}

void Test_Save_Only()
{
  Temporary_Directory directory;
  application::Run_Image_Processing_Options options;
  options.image_directory = directory.path;
  options.save_original_images = true;
  auto capture = Capture(7);
  std::weak_ptr<const jutze_camera::camera_frame> released = capture.frame;
  std::vector<application::Run_Captured_Frame> captures;
  captures.push_back(std::move(capture));
  const auto result = application::Process_Run_Images(std::move(captures), options);
  Require(result.success && result.processed_count == 1, "Save-only failed");
  Require(released.expired(), "Worker retained the input frame");
  Require(result.mosaic.rgb.empty(), "Save-only retained an unused mosaic");
  Require(result.mosaic_path.empty(), "Save-only exposed a nonexistent mosaic");
  const auto path = directory.path / "P007_frame_42.png";
  wxImage image(wxString(path.wstring()), wxBITMAP_TYPE_PNG);
  Require(image.IsOk() && image.GetWidth() == 2 && image.GetHeight() == 2,
          "Original PNG dimensions or filename changed");
  Require(image.GetData()[0] == 11 && image.GetData()[1] == 22 &&
            image.GetData()[2] == 33, "Saved pixels changed");
  Require(!std::filesystem::exists(directory.path / "mosaic.png"),
          "Save-only unexpectedly generated mosaic");
}

void Test_Mosaic(bool save_originals)
{
  Temporary_Directory directory;
  application::Run_Image_Processing_Options options;
  options.image_directory = directory.path;
  options.build_mosaic = true;
  options.save_original_images = save_originals;
  options.fov_configuration.length_mm = 2;
  options.fov_configuration.width_mm = 2;
  const auto result = application::Process_Run_Images({Capture()}, options);
  Require(result.success && result.processed_count == 1, "Mosaic failed");
  Require(result.groups.size() == 1 && result.groups[0].success &&
            result.groups[0].group_id == 0 &&
            result.groups[0].mosaic_path == result.mosaic_path &&
            result.groups[0].mosaic.rgb == result.mosaic.rgb,
          "Default group did not preserve legacy result fields");
  Require(result.mosaic.width > 0 && result.mosaic.width <= 360 &&
            result.mosaic.height > 0 && result.mosaic.height <= 260,
          "Preview dimensions invalid");
  Require(result.mosaic.rgb[0] == 11 && result.mosaic.rgb[1] == 22 &&
            result.mosaic.rgb[2] == 33, "Mosaic pixels changed");
  Require(result.mosaic_path == directory.path / "mosaic.png" &&
            std::filesystem::exists(result.mosaic_path),
          "Full mosaic path was not returned");
  Require(std::filesystem::exists(directory.path / "P001_frame_42.png") ==
            save_originals, "Original image flag ignored");
}

void Test_Mosaic_Placement()
{
  Temporary_Directory directory;
  application::Run_Image_Processing_Options options;
  options.image_directory = directory.path;
  options.build_mosaic = true;
  options.fov_configuration.length_mm = 2;
  options.fov_configuration.width_mm = 2;
  auto second = Capture(2);
  second.point.world_pose[1] = -2;
  const std::vector<std::uint8_t> pixels(12, 200);
  second.frame = std::make_shared<jutze_camera::camera_frame>(
    "test", "serial", 2, 2,
    static_cast<unsigned int>(jutze_camera::pixel_format::rgb8),
    43, pixels.data(), pixels.size());
  const auto result = application::Process_Run_Images({Capture(), second}, options);
  Require(result.success && result.processed_count == 2,
          "Two-image mosaic failed");
  Require(result.mosaic.width == 4 && result.mosaic.height == 2,
          "FOV-axis placement dimensions changed");
  Require(result.mosaic.rgb[0] == 11 && result.mosaic.rgb[2 * 3] == 200,
          "FOV pixel-axis sign or image order changed");
}

void Test_Full_Resolution_Path()
{
  Temporary_Directory directory;
  application::Run_Image_Processing_Options options;
  options.image_directory = directory.path;
  options.build_mosaic = true;
  options.fov_configuration.length_mm = 400;
  options.fov_configuration.width_mm = 300;
  auto capture = Capture();
  const std::vector<std::uint8_t> pixels(400 * 300 * 3, 120);
  capture.frame = std::make_shared<jutze_camera::camera_frame>(
    "test", "serial", 400, 300,
    static_cast<unsigned int>(jutze_camera::pixel_format::rgb8),
    42, pixels.data(), pixels.size());
  const auto result = application::Process_Run_Images({capture}, options);
  Require(result.success, "Full-resolution mosaic failed");
  wxImage saved(wxString(result.mosaic_path.wstring()), wxBITMAP_TYPE_PNG);
  Require(saved.IsOk() && saved.GetWidth() == 400 && saved.GetHeight() == 300,
          "Viewer path points to a reduced preview instead of the full mosaic");
  Require(result.mosaic.width <= 360 && result.mosaic.height <= 260,
          "Panel preview should remain bounded");
}

void Test_Failures()
{
  Temporary_Directory directory;
  application::Run_Image_Processing_Options options;
  options.image_directory = directory.path;
  options.save_original_images = true;
  auto result = application::Process_Run_Images({}, options);
  Require(!result.success && !result.message.empty(), "Empty batch accepted");
  result = application::Process_Run_Images({{}}, options);
  Require(!result.success && result.processed_count == 0, "Null frame accepted");

  auto invalid = Capture();
  const std::uint8_t byte = 1;
  invalid.frame = std::make_shared<jutze_camera::camera_frame>(
    "test", "serial", 2, 2,
    static_cast<unsigned int>(jutze_camera::pixel_format::rgb8), 42, &byte, 1);
  result = application::Process_Run_Images({invalid}, options);
  Require(!result.success && result.processed_count == 0 &&
            !result.message.empty(), "Truncated frame accepted");

  options.image_directory = directory.path / "missing";
  result = application::Process_Run_Images({Capture()}, options);
  Require(!result.success && !result.message.empty(), "Save failure ignored");

  options.image_directory = directory.path;
  options.save_original_images = false;
  options.build_mosaic = true;
  invalid = Capture();
  invalid.point.has_world_pose = false;
  result = application::Process_Run_Images({invalid}, options);
  Require(!result.success && !result.message.empty(), "Missing mosaic pose accepted");

  options.build_mosaic = false;
  options.save_original_images = true;
  result = application::Process_Run_Images({Capture(), {}}, options);
  Require(!result.success && result.processed_count == 1,
          "Partial batch count or failure was lost");
}
void Test_Group_Isolation_And_Order()
{
  Temporary_Directory directory;
  application::Run_Image_Processing_Options options;
  options.image_directory = directory.path;
  options.build_mosaic = true;
  options.save_original_images = true;
  options.fov_configuration.length_mm = 2;
  options.fov_configuration.width_mm = 2;
  options.mosaic_configuration.groups = {
    Group(8, {42}), Group(3, {7}), Group(11, {7, 42})};
  auto first = Colored_Capture(42, 40);
  auto second = Colored_Capture(7, 180);
  second.point.world_pose[1] = -2;
  auto result = application::Process_Run_Images({first, second}, options);
  Require(result.success && result.groups.size() == 3 && result.processed_count == 2,
          "Overlapping group membership failed or processed inputs repeatedly");
  Require(result.groups[0].group_id == 8 && result.groups[1].group_id == 3 &&
            result.groups[2].group_id == 11 && result.groups[0].name == "group 8",
          "Configured group order or names were lost");
  Require(result.groups[0].mosaic.width == 2 && result.groups[1].mosaic.width == 2 &&
            result.groups[0].mosaic.rgb[0] == 40 && result.groups[1].mosaic.rgb[0] == 180,
          "Images leaked between independent groups");
  Require(result.groups[2].mosaic.width == 4 && result.groups[2].mosaic.rgb[0] == 40 &&
            result.groups[2].mosaic.rgb[6] == 180,
          "Point ID membership changed capture order or placement");
  Require(result.mosaic_path == directory.path / "mosaic_group_8.png" &&
            result.mosaic.rgb == result.groups[0].mosaic.rgb,
          "Legacy result does not use the first successful group");
  Require(!std::filesystem::exists(directory.path / "mosaic.png"),
          "Explicit groups wrote the implicit mosaic file");
  std::size_t files = 0;
  for( const auto& file : std::filesystem::directory_iterator(directory.path) )
    if( file.is_regular_file() ) ++files;
  Require(files == 5, "Expected two original files and three group files");

  // At identical poses overlap ties must follow captures, not point_ids order.
  second.point.world_pose = {};
  options.mosaic_configuration.groups = {Group(12, {7, 42})};
  result = application::Process_Run_Images({first, second}, options);
  Require(result.success && result.mosaic.rgb[0] == 40,
          "Overlap tie breaking reordered captures by member IDs");

  // A different orientation makes an incorrect first-camera selection visible.
  second.point.world_pose[1] = -4;
  second.point.world_pose[3] = 90;
  result = application::Process_Run_Images({first, second}, options);
  Require(result.success && result.mosaic.width == 6 && result.mosaic.height <= 3,
          "First-camera reference did not use the first capture");
}

void Test_Base_Axes_And_Reversal()
{
  Temporary_Directory directory;
  application::Run_Image_Processing_Options options;
  options.image_directory = directory.path;
  options.build_mosaic = true;
  options.fov_configuration.length_mm = 4;
  options.fov_configuration.width_mm = 4;
  auto capture = Capture(19);
  std::vector<std::uint8_t> pixels;
  for( unsigned int y = 0; y < 4; ++y )
    for( unsigned int x = 0; x < 4; ++x )
      for( int channel = 0; channel < 3; ++channel )
        pixels.push_back(static_cast<std::uint8_t>(10 + y * 40 + x * 5));
  capture.frame = std::make_shared<jutze_camera::camera_frame>(
    "test", "serial", 4, 4,
    static_cast<unsigned int>(jutze_camera::pixel_format::rgb8),
    42, pixels.data(), pixels.size());
  auto group = Group(1, {19});
  group.reference = robot_model::Mosaic_Reference::Robot_Base;
  group.horizontal_axis = robot_model::Coordinate_Axis::Y;
  group.vertical_axis = robot_model::Coordinate_Axis::X;
  options.mosaic_configuration.groups = {group};
  auto result = application::Process_Run_Images({capture}, options);
  Require(result.success && result.mosaic.width == 4 && result.mosaic.height == 4 &&
            result.mosaic.rgb[0] == 100 && result.mosaic.rgb[9] == 90,
          "Base axes did not project camera image vectors");
  options.mosaic_configuration.groups[0].horizontal_axis = robot_model::Coordinate_Axis::X;
  options.mosaic_configuration.groups[0].vertical_axis = robot_model::Coordinate_Axis::Y;
  result = application::Process_Run_Images({capture}, options);
  Require(result.success && result.mosaic.rgb[0] == 100 && result.mosaic.rgb[9] == 20,
          "Swapping base axes did not transpose image vectors");
  options.mosaic_configuration.groups[0] = group;
  options.mosaic_configuration.groups[0].reverse_horizontal = true;
  result = application::Process_Run_Images({capture}, options);
  Require(result.success && result.mosaic.rgb[0] == 90 && result.mosaic.rgb[9] == 100,
          "Horizontal reversal did not reverse source pixels");
  options.mosaic_configuration.groups[0].reverse_vertical = true;
  result = application::Process_Run_Images({capture}, options);
  Require(result.success && result.mosaic.rgb[0] == 10 && result.mosaic.rgb[9] == 20,
          "Vertical reversal did not reverse source pixels");

  options.mosaic_configuration.groups[0].reference = robot_model::Mosaic_Reference::First_Camera;
  result = application::Process_Run_Images({capture}, options);
  Require(result.success && result.mosaic.rgb[0] == 100,
          "First-camera reversal did not reverse both pixel directions");

  // Centers must use the same signed basis as both source image vectors.
  options.fov_configuration.length_mm = 2;
  options.fov_configuration.width_mm = 2;
  auto first = Colored_Capture(19, 30);
  auto second = Colored_Capture(25, 210);
  second.point.world_pose[1] = 2;
  group.point_ids = {25, 19};
  options.mosaic_configuration.groups = {group};
  result = application::Process_Run_Images({first, second}, options);
  Require(result.success && result.mosaic.width == 4 && result.mosaic.rgb[0] == 30 &&
            result.mosaic.rgb[6] == 210, "Base center projection is inconsistent");
  options.mosaic_configuration.groups[0].reverse_horizontal = true;
  result = application::Process_Run_Images({first, second}, options);
  Require(result.success && result.mosaic.rgb[0] == 210 && result.mosaic.rgb[6] == 30,
          "Horizontal reversal did not reverse center placement");

  // Exercise both other robot-base planes with FOV axes in those planes.
  for( const auto vertical : {robot_model::Coordinate_Axis::X, robot_model::Coordinate_Axis::Y} )
  {
    options.fov_configuration.length_axis = robot_model::Coordinate_Axis::Z;
    options.fov_configuration.width_axis = vertical;
    group.horizontal_axis = robot_model::Coordinate_Axis::Z;
    group.vertical_axis = vertical;
    options.mosaic_configuration.groups = {group};
    second.point.world_pose = {};
    second.point.world_pose[2] = 2;
    result = application::Process_Run_Images({first, second}, options);
    Require(result.success && result.mosaic.width == 4 && result.mosaic.rgb[6] == 210,
            "ZX/ZY plane projection failed");
  }
}

void Test_Point_Corrections()
{
  Temporary_Directory directory;
  application::Run_Image_Processing_Options options;
  options.image_directory = directory.path;
  options.build_mosaic = true;
  options.save_original_images = true;
  options.fov_configuration.length_mm = 4;
  options.fov_configuration.width_mm = 4;
  options.fov_configuration.length_axis = robot_model::Coordinate_Axis::X;
  options.fov_configuration.width_axis = robot_model::Coordinate_Axis::Y;
  auto first = Colored_Capture(19, 30);
  auto second = Capture(25);
  std::vector<std::uint8_t> pixels;
  for( unsigned int y = 0; y < 4; ++y )
    for( unsigned int x = 0; x < 4; ++x )
      for( int channel = 0; channel < 3; ++channel )
        pixels.push_back(static_cast<std::uint8_t>(80 + y * 20 + x * 5));
  second.frame = std::make_shared<jutze_camera::camera_frame>(
    "test", "serial", 4, 4,
    static_cast<unsigned int>(jutze_camera::pixel_format::rgb8),
    42, pixels.data(), pixels.size());
  // Both images use one pixel/mm, so integer millimetre shifts are exact.
  const std::vector<std::uint8_t> first_pixels(4 * 4 * 3, 30);
  first.frame = std::make_shared<jutze_camera::camera_frame>(
    "test", "serial", 4, 4,
    static_cast<unsigned int>(jutze_camera::pixel_format::rgb8),
    42, first_pixels.data(), first_pixels.size());
  first.point.joint_angles_deg = {1, 2, 3, 4, 5, 6};
  second.point.joint_angles_deg = {6, 5, 4, 3, 2, 1};
  const std::vector<application::Run_Captured_Frame> captures = {first, second};
  const auto before = captures;
  for( const auto reference : {robot_model::Mosaic_Reference::First_Camera,
       robot_model::Mosaic_Reference::Robot_Base} )
  {
    for( int flags = 0; flags < 4; ++flags )
    {
      auto group = Group(1, {25, 19});
      group.reference = reference;
      group.reverse_horizontal = (flags & 1) != 0;
      group.reverse_vertical = (flags & 2) != 0;
      auto unchanged = group;
      unchanged.id = 2;
      auto single = group;
      single.id = 3;
      single.point_ids = {25};
      options.mosaic_configuration.groups = {group, unchanged, single};
      const auto baseline = application::Process_Run_Images(captures, options);
      Require(baseline.success, "Correction baseline failed");
      for( const double sign : {-1.0, 1.0} )
      {
        options.mosaic_configuration.groups[0].corrections = {{25, sign * 8, sign * 4}};
        const auto result = application::Process_Run_Images(captures, options);
        Require(result.success, "Corrected mosaic failed");
        const auto& mosaic = result.groups[0].mosaic;
        Require(mosaic.width == 12 && mosaic.height == 8,
                "Correction was not applied in final-plane millimetres");
        const unsigned int second_x = sign > 0 ? 8 : 0;
        const unsigned int second_y = sign > 0 ? 4 : 0;
        const unsigned int first_x = sign > 0 ? 0 : 8;
        const unsigned int first_y = sign > 0 ? 0 : 4;
        for( unsigned int y = 0; y < 4; ++y )
          for( unsigned int x = 0; x < 4; ++x )
            for( unsigned int channel = 0; channel < 3; ++channel )
            {
              Require(mosaic.rgb[((y + second_y) * mosaic.width + x + second_x) * 3 + channel] ==
                        baseline.groups[2].mosaic.rgb[(y * 4 + x) * 3 + channel],
                      "Correction changed source basis, orientation or target point");
              Require(mosaic.rgb[((y + first_y) * mosaic.width + x + first_x) * 3 + channel] == 30,
                      "Correction moved or changed the uncorrected point");
            }
        Require(mosaic.rgb[(3 * mosaic.width + 5) * 3] == 0,
                "Correction gap was not preserved");
        Require(result.groups[1].mosaic.width == baseline.groups[1].mosaic.width &&
                  result.groups[1].mosaic.height == baseline.groups[1].mosaic.height &&
                  result.groups[1].mosaic.rgb == baseline.groups[1].mosaic.rgb,
                "Correction leaked into another group sharing the same point");
        wxImage original(wxString((directory.path / "P025_frame_42.png").wstring()),
          wxBITMAP_TYPE_PNG);
        Require(original.IsOk() && original.GetWidth() == 4 && original.GetHeight() == 4 &&
                  std::equal(pixels.begin(), pixels.end(), original.GetData()),
                "Mosaic correction changed the original image");
      }
    }
  }
  auto translated = second;
  translated.point.world_pose[0] = -4;
  auto additive = Group(4, {19, 25});
  additive.corrections = {{25, 8, 4}};
  options.mosaic_configuration.groups = {additive};
  const auto translated_result = application::Process_Run_Images({first, translated}, options);
  Require(translated_result.success && translated_result.mosaic.width == 16 &&
            translated_result.mosaic.height == 8 &&
            translated_result.mosaic.rgb[(4 * 16 + 12) * 3] == pixels[0],
          "Correction replaced rather than added to the projected center");

  for( std::size_t i = 0; i < captures.size(); ++i )
  {
    const auto& actual = captures[i].point;
    const auto& expected = before[i].point;
    Require(actual.id == expected.id && actual.type == expected.type &&
              actual.robot_model_id == expected.robot_model_id &&
              actual.joint_angles_deg == expected.joint_angles_deg &&
              actual.world_pose == expected.world_pose &&
              actual.has_world_pose == expected.has_world_pose &&
              actual.flange_from_coordinate_pose == expected.flange_from_coordinate_pose &&
              actual.template_reference_pose == expected.template_reference_pose &&
              captures[i].frame == before[i].frame,
            "Mosaic correction modified input point/motion data");
  }
}

void Test_Invalid_Point_Corrections()
{
  Temporary_Directory directory;
  application::Run_Image_Processing_Options options;
  options.image_directory = directory.path;
  options.build_mosaic = true;
  options.fov_configuration.length_mm = 2;
  options.fov_configuration.width_mm = 2;
  const std::vector<std::vector<robot_model::Mosaic_Point_Correction>> invalid = {
    {{0, 0, 0}}, {{2, 0, 0}}, {{1, 0, 0}, {1, 1, 2}},
    {{1, std::numeric_limits<double>::quiet_NaN(), 0}},
    {{1, 0, std::numeric_limits<double>::quiet_NaN()}},
    {{1, std::numeric_limits<double>::infinity(), 0}},
    {{1, 0, -std::numeric_limits<double>::infinity()}}
  };
  for( const auto& corrections : invalid )
  {
    auto group = Group(1, {1});
    group.corrections = corrections;
    options.mosaic_configuration.groups = {group, Group(2, {1})};
    const auto result = application::Process_Run_Images({Capture(1), Capture(2)}, options);
    Require(!result.success && !result.groups[0].success && result.groups[1].success &&
              !result.groups[0].message.empty() && result.groups[0].mosaic.rgb.empty(),
            "Invalid correction accepted or prevented another group from completing");
  }
}

void Test_Group_Failures()
{
  Temporary_Directory directory;
  application::Run_Image_Processing_Options options;
  options.image_directory = directory.path;
  options.build_mosaic = true;
  options.fov_configuration.length_mm = 2;
  options.fov_configuration.width_mm = 2;
  options.mosaic_configuration.groups = {Group(4, {77}), Group(5, {1})};
  auto result = application::Process_Run_Images({Capture()}, options);
  Require(!result.success && result.groups.size() == 2 && !result.groups[0].success &&
            result.groups[0].message.find("77") != std::string::npos &&
            result.groups[1].success && result.mosaic_path == result.groups[1].mosaic_path,
          "Missing member must fail only its group and expose the first success");
  Require(result.groups[0].mosaic_path.empty() && result.groups[0].mosaic.rgb.empty(),
          "Failed group exposed a mosaic");

  options.mosaic_configuration.groups = {Group(4, {77, 1}), Group(5, {1})};
  auto invalid = Capture(77);
  for( int failure = 0; failure < 5; ++failure )
  {
    invalid = Capture(77);
    if( failure == 0 ) invalid.frame.reset();
    if( failure == 1 ) invalid.point.has_world_pose = false;
    if( failure == 2 ) invalid.point.world_pose[0] = std::numeric_limits<double>::quiet_NaN();
    if( failure == 3 ) invalid.point.world_pose[3] = std::numeric_limits<double>::infinity();
    if( failure == 4 ) invalid.point.world_pose[0] = std::numeric_limits<double>::max();
    result = application::Process_Run_Images({Capture(), invalid}, options);
    Require(!result.success && !result.groups[0].success && result.groups[1].success &&
              result.groups[0].message.find("77") != std::string::npos,
            "Invalid member was silently skipped or poisoned another group");
  }

  auto base = Group(6, {1});
  base.reference = robot_model::Mosaic_Reference::Robot_Base;
  base.horizontal_axis = robot_model::Coordinate_Axis::X;
  base.vertical_axis = robot_model::Coordinate_Axis::Z;
  options.mosaic_configuration.groups = {base, Group(7, {1})};
  result = application::Process_Run_Images({Capture()}, options);
  Require(!result.success && !result.groups[0].success && result.groups[1].success &&
            result.groups[0].message.find("P1") != std::string::npos,
          "Perpendicular projection was silently dropped");
  base.vertical_axis = robot_model::Coordinate_Axis::Y;
  options.mosaic_configuration.groups = {base};
  invalid = Capture();
  invalid.point.world_pose[4] = 89.9999999;
  result = application::Process_Run_Images({invalid}, options);
  Require(!result.success && result.message.find("P1") != std::string::npos,
          "Nearly perpendicular projection was accepted");

  for( int failure = 0; failure < 8; ++failure )
  {
    auto bad = Group(2, {1});
    if( failure == 0 ) bad.point_ids.clear();
    if( failure == 1 ) bad.point_ids = {1, 1};
    if( failure == 2 ) bad.id = 0;
    if( failure == 3 ) bad.reference = static_cast<robot_model::Mosaic_Reference>(99);
    if( failure == 4 )
    {
      bad.reference = robot_model::Mosaic_Reference::Robot_Base;
      bad.horizontal_axis = bad.vertical_axis;
    }
    if( failure == 5 ) bad.name.clear();
    if( failure == 6 ) bad.point_ids = {0};
    if( failure == 7 )
    {
      bad.reference = robot_model::Mosaic_Reference::Robot_Base;
      bad.horizontal_axis = static_cast<robot_model::Coordinate_Axis>(99);
    }
    options.mosaic_configuration.groups = {bad};
    result = application::Process_Run_Images({Capture()}, options);
    Require(!result.success && !result.groups[0].message.empty(),
            "Invalid group configuration was accepted");
  }
  options.mosaic_configuration.groups = {Group(2, {1}), Group(2, {1})};
  result = application::Process_Run_Images({Capture()}, options);
  Require(!result.success && !result.groups[0].success && !result.groups[1].success,
          "Duplicate group IDs overwrote an output");
  options.mosaic_configuration.groups = {Group(8, {1}), Group(9, {1})};
  std::filesystem::create_directory(directory.path / "mosaic_group_8.png");
  result = application::Process_Run_Images({Capture()}, options);
  Require(!result.success && !result.groups[0].success && result.groups[1].success &&
            result.mosaic_path == result.groups[1].mosaic_path,
          "One group save failure prevented another group from completing");
  options.mosaic_configuration.groups = {Group(10, {77})};
  invalid = Capture(77);
  invalid.point.type = robot_model::Robot_Teach_Point_Type::Transition;
  result = application::Process_Run_Images({invalid}, options);
  Require(!result.success, "Explicit non-motion member was accepted");
  options.mosaic_configuration.groups.clear();
  result = application::Process_Run_Images({Capture(), invalid}, options);
  Require(result.success && result.mosaic.width == 2,
          "Default group must include motion points only");
  invalid = Capture(77);
  invalid.point.has_world_pose = false;
  result = application::Process_Run_Images({Capture(), invalid}, options);
  Require(!result.success && result.message.find("77") != std::string::npos,
          "Default group silently omitted a member without a pose");
  options.fov_configuration.length_mm = std::numeric_limits<double>::infinity();
  result = application::Process_Run_Images({Capture()}, options);
  Require(!result.success, "Infinite FOV was silently normalized");
  options.fov_configuration.length_mm = 2;
  options.flange_from_camera_pose[0] = std::numeric_limits<double>::quiet_NaN();
  result = application::Process_Run_Images({Capture()}, options);
  Require(!result.success && result.message.find("P1") != std::string::npos,
          "Invalid camera calibration pose was accepted");
}
} // namespace

int main()
{
  wxInitializer initializer;
  if (!initializer.IsOk()) return 1;
  wxInitAllImageHandlers();
  wxLogNull suppress_expected_file_errors;
  try
  {
    Test_Save_Only();
    Test_Mosaic(false);
    Test_Mosaic(true);
    Test_Mosaic_Placement();
    Test_Full_Resolution_Path();
    Test_Failures();
    Test_Group_Isolation_And_Order();
    Test_Base_Axes_And_Reversal();
    Test_Group_Failures();
    Test_Point_Corrections();
    Test_Invalid_Point_Corrections();
  }
  catch (const std::exception &error)
  {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }
  std::cout << "Run image processing tests passed.\n";
  return 0;
}

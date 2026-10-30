#include "run_image_processing.h"
#include "camera_params.h"

#include <wx/image.h>
#include <wx/init.h>
#include <wx/log.h>

#include <chrono>
#include <filesystem>
#include <iostream>
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
  Require(result.mosaic.width > 0 && result.mosaic.width <= 360 &&
            result.mosaic.height > 0 && result.mosaic.height <= 260,
          "Preview dimensions invalid");
  Require(result.mosaic.rgb[0] == 11 && result.mosaic.rgb[1] == 22 &&
            result.mosaic.rgb[2] == 33, "Mosaic pixels changed");
  Require(std::filesystem::exists(directory.path / "mosaic.png"),
          "Full mosaic was not saved");
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
    Test_Failures();
  }
  catch (const std::exception &error)
  {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }
  std::cout << "Run image processing tests passed.\n";
  return 0;
}

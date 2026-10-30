#ifndef includeguard_run_image_processing_h_includeguard
#define includeguard_run_image_processing_h_includeguard

#include "camera_2d_image_converter.h"
#include "robot_teach_point_store.h"
#include "tool_visualization.h"
#include "mosaic_configuration.h"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace application
{
struct Run_Captured_Frame
{
  std::shared_ptr<const jutze_camera::camera_frame> frame;
  robot_model::Robot_Teach_Point point;
};

struct Run_Image_Processing_Options
{
  std::filesystem::path image_directory;
  bool save_original_images = false;
  bool build_mosaic = false;
  robot_model::Fov_Visualization_Configuration fov_configuration;
  robot_model::XyzabcPose flange_from_camera_pose{};
  robot_model::Mosaic_Configuration mosaic_configuration;
};

struct Mosaic_Group_Result
{
  std::size_t group_id = 0;
  std::string name;
  bool success = false;
  std::string message;
  Camera_2D_Display_Image mosaic;
  std::filesystem::path mosaic_path;
};

struct Run_Image_Processing_Result
{
  bool success = false;
  std::string message;
  Camera_2D_Display_Image mosaic;
  std::size_t processed_count = 0;
  std::filesystem::path mosaic_path;
  std::vector<Mosaic_Group_Result> groups;
};

// Synchronous worker operation: owns the supplied frames, never accesses a window.
// PNG handlers must be registered at application startup before calling this.
// Saves mosaic.png in compatibility mode, otherwise mosaic_group_<id>.png.
// Returned images are bounded 360x260 previews; legacy fields use the first
// successful group. Group order follows the configuration (implicit group ID 0).
Run_Image_Processing_Result Process_Run_Images(
  std::vector<Run_Captured_Frame> captures,
  const Run_Image_Processing_Options &options);
} // namespace application

#endif

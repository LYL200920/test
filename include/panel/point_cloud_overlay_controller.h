#ifndef includeguard_point_cloud_overlay_controller_h_includeguard
#define includeguard_point_cloud_overlay_controller_h_includeguard

#include "point_cloud_document.h"
#include "point_cloud_selection.h"

#include <array>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

class Camera_Service;
class vtkRenderer;

struct Point_Cloud_Overlay_Result
{
  bool success = false;
  std::size_t source_point_count = 0;
  std::size_t filtered_point_count = 0;
  std::size_t point_count = 0;
  double median_depth_mm = 0.0;
  std::array<double, 6> world_bounds_mm = {};
  bool has_world_bounds = false;
  std::string error_message;
};

struct Point_Cloud_Save_Result
{
  bool success = false;
  std::size_t source_point_count = 0;
  std::size_t filtered_point_count = 0;
  std::size_t point_count = 0;
  double median_depth_mm = 0.0;
  std::array<double, 6> world_bounds_mm = {};
  std::filesystem::path file_path;
  std::string error_message;
};

class Point_Cloud_Overlay_Controller
{
public:
  explicit Point_Cloud_Overlay_Controller(Camera_Service &camera_service);
  ~Point_Cloud_Overlay_Controller();

  Point_Cloud_Overlay_Result Load_Latest(vtkRenderer *renderer);
  Point_Cloud_Save_Result Save_Current_To_File(const std::filesystem::path &path, const std::string &robot_model_id);
  Point_Cloud_Overlay_Result Load_File(const std::filesystem::path &path,
                                       const std::string &expected_robot_model,
                                       vtkRenderer *renderer);
  void Attach_Renderer(vtkRenderer *renderer);
  void Clear();
  bool Has_Point_Cloud() const;
  void Set_Interactive_LOD(bool enabled);
  std::size_t Select_In_Display_Rect(vtkRenderer *renderer,
                                     int minimum_x,
                                     int minimum_y,
                                     int maximum_x,
                                     int maximum_y,
                                     point_cloud::Point_Selection_Mode mode);
  std::size_t Select_In_Display_Polygon(
    vtkRenderer *renderer,
    const std::vector<point_cloud::Display_Point> &polygon,
    point_cloud::Point_Selection_Mode mode);
  std::size_t Selection_Count() const;
  void Clear_Selection();
  std::size_t Delete_Selected(std::string *error_message = nullptr);
  bool Undo(std::string *error_message = nullptr);
  bool Redo(std::string *error_message = nullptr);
  bool Can_Undo() const;
  bool Can_Redo() const;
  std::size_t Displayed_Point_Count() const;
  std::size_t Interaction_Point_Count() const;
  std::shared_ptr<const std::vector<float>> Collision_Obstacle_Xyz() const;

private:
  class Implementation;
  std::unique_ptr<Implementation> m_implementation;
};

#endif

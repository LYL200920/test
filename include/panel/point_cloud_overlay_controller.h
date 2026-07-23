#ifndef includeguard_point_cloud_overlay_controller_h_includeguard
#define includeguard_point_cloud_overlay_controller_h_includeguard

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
  Point_Cloud_Save_Result Save_Latest_To_File(const std::filesystem::path &path, const std::string &robot_model_id);
  Point_Cloud_Overlay_Result Load_File(const std::filesystem::path &path,
                                       const std::string &expected_robot_model,
                                       vtkRenderer *renderer);
  void Attach_Renderer(vtkRenderer *renderer);
  void Clear();
  bool Has_Point_Cloud() const;
  void Set_Interactive_LOD(bool enabled);
  std::size_t Displayed_Point_Count() const;
  std::size_t Interaction_Point_Count() const;
  std::shared_ptr<const std::vector<float>> Collision_Obstacle_Xyz() const;

private:
  class Implementation;
  std::unique_ptr<Implementation> m_implementation;
};

#endif

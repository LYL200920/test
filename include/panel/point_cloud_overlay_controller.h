#ifndef includeguard_point_cloud_overlay_controller_h_includeguard
#define includeguard_point_cloud_overlay_controller_h_includeguard

#include "point_cloud_renderer.h"

#include <cstddef>
#include <string>

class Camera_Service;
class vtkRenderer;

struct Point_Cloud_Overlay_Result
{
  bool success = false;
  std::size_t point_count = 0;
  std::string error_message;
};

// Application-level coordinator for camera frame conversion, eye-to-hand
// transformation and robot-scene point-cloud rendering.
class Point_Cloud_Overlay_Controller
{
public:
  explicit Point_Cloud_Overlay_Controller (Camera_Service& camera_service);

  Point_Cloud_Overlay_Result Load_Latest (vtkRenderer* renderer);
  void Attach_Renderer (vtkRenderer* renderer);
  void Clear ( );
  bool Has_Point_Cloud ( ) const;

private:
  Camera_Service& m_camera_service;
  point_cloud::Point_Cloud_Renderer m_renderer;
};

#endif

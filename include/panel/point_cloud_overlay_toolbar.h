#ifndef includeguard_point_cloud_overlay_toolbar_h_includeguard
#define includeguard_point_cloud_overlay_toolbar_h_includeguard

#include "point_cloud_overlay_controller.h"

#include <wx/panel.h>
#include <wx/string.h>

#include <functional>
#include <string>
#include <vector>

class Camera_Service;
class vtkRenderer;
class wxButton;
class wxCheckBox;
class wxSpinCtrlDouble;

class Point_Cloud_Overlay_Toolbar : public wxPanel
{
public:
  struct Callbacks
  {
    std::function<vtkRenderer* ( )> renderer;
    std::function<std::string ( )> robot_model_id;
    std::function<void ( )> show_robot_page;
    std::function<void ( )> render_scene;
    std::function<void (const wxString&)> set_status;
    std::function<bool (bool)> set_camera_pose_visible;
    std::function<bool (
      std::shared_ptr<const std::vector<float>>, std::string*)>
      set_collision_obstacle_points;
    std::function<void ( )> clear_collision_obstacle_points;
    std::function<void (bool)> set_collision_enabled;
    std::function<bool ( )> collision_rebuild_in_progress;
    std::function<bool (
      double, double, double, bool,
      std::size_t*, std::size_t*, std::string*)> apply_collision_settings;
  };

  Point_Cloud_Overlay_Toolbar (
    wxWindow* parent,
    Camera_Service& camera_service,
    Callbacks callbacks);

  void Attach_Renderer (vtkRenderer* renderer);
  bool Has_Point_Cloud ( ) const;
  void Set_Camera_Connected (bool connected);
  void Set_Interactive_LOD (bool enabled);

private:
  void On_Load_Latest (wxCommandEvent& event);
  void On_Save_Latest (wxCommandEvent& event);
  void On_Load_File (wxCommandEvent& event);
  void On_Clear (wxCommandEvent& event);
  void On_Toggle_Camera_Pose (wxCommandEvent& event);
  void On_Toggle_Collision (wxCommandEvent& event);
  void On_Apply_Collision_Settings (wxCommandEvent& event);
  void Report_Error (const wxString& title, const std::string& message);
  vtkRenderer* Renderer ( ) const;
  std::string Robot_Model_Id ( ) const;
  void Show_And_Render ( );
  void Set_Status (const wxString& status);
  bool Sync_Collision_Obstacle_Points (std::string* error_message);

private:
  Point_Cloud_Overlay_Controller m_controller;
  Callbacks m_callbacks;
  wxButton* m_load_latest_button = nullptr;
  wxButton* m_save_latest_button = nullptr;
  wxButton* m_camera_pose_button = nullptr;
  wxCheckBox* m_collision_enabled_checkbox = nullptr;
  wxSpinCtrlDouble* m_clearance_ctrl = nullptr;
  wxSpinCtrlDouble* m_voxel_size_ctrl = nullptr;
  wxSpinCtrlDouble* m_robot_exclusion_ctrl = nullptr;
  wxCheckBox* m_exclude_robot_points = nullptr;
  bool m_camera_pose_visible = false;
  bool m_collision_enabled = true;
};

#endif

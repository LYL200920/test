#ifndef includeguard_point_cloud_overlay_toolbar_h_includeguard
#define includeguard_point_cloud_overlay_toolbar_h_includeguard

#include "point_cloud_overlay_controller.h"

#include <wx/panel.h>
#include <wx/string.h>

#include <functional>
#include <string>

class Camera_Service;
class vtkRenderer;
class wxButton;

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
  };

  Point_Cloud_Overlay_Toolbar (
    wxWindow* parent,
    Camera_Service& camera_service,
    Callbacks callbacks);

  void Attach_Renderer (vtkRenderer* renderer);
  bool Has_Point_Cloud ( ) const;
  void Set_Camera_Connected (bool connected);

private:
  void On_Load_Latest (wxCommandEvent& event);
  void On_Save_Latest (wxCommandEvent& event);
  void On_Load_File (wxCommandEvent& event);
  void On_Clear (wxCommandEvent& event);
  void On_Toggle_Camera_Pose (wxCommandEvent& event);
  void Report_Error (const wxString& title, const std::string& message);
  vtkRenderer* Renderer ( ) const;
  std::string Robot_Model_Id ( ) const;
  void Show_And_Render ( );
  void Set_Status (const wxString& status);

private:
  Point_Cloud_Overlay_Controller m_controller;
  Callbacks m_callbacks;
  wxButton* m_load_latest_button = nullptr;
  wxButton* m_save_latest_button = nullptr;
  wxButton* m_camera_pose_button = nullptr;
  bool m_camera_pose_visible = false;
};

#endif

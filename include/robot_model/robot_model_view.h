#ifndef includeguard_robot_model_view_h_includeguard
#define includeguard_robot_model_view_h_includeguard

#include "robot_model_data.h"
#include "robot_render_controller.h"
#include "collision_index_rebuild_coordinator.h"
#include "coordinate_frame_renderer.h"
#include "flange_interaction_controller.h"
#include "flange_drag_update_coordinator.h"
#include "flange_drag_motion_executor.h"
#include "robot_drag_performance_collector.h"
#include "tool_coordinate.h"

#include <wx/glcanvas.h>
#include <wx/overlay.h>
#include <wx/timer.h>

#include <memory>
#include <array>
#include <functional>
#include <string>
#include <vector>

class vtkGenericRenderWindowInteractor;
class vtkRenderer;

namespace robot_model
{
  class Vtk_Scene;
} // namespace robot_model

class Robot_Model_View : public wxGLCanvas
{
public:
  using Flange_Interaction_Mode = robot_model::Flange_Interaction_Mode;
  enum class Point_Cloud_Selection_Shape
  {
    Rectangle,
    Polygon
  };

  explicit Robot_Model_View(wxWindow *parent, wxWindowID id = wxID_ANY);
  ~Robot_Model_View() override;

  void Load_Model(const robot_model::Robot_Model_Info &model);
  void Set_Joint_State(const robot_model::Robot_Joint_State &state);
  robot_model::Robot_Joint_State_Apply_Result Try_Set_Joint_State(const robot_model::Robot_Joint_State &state);
  const robot_model::Robot_Joint_State_Apply_Result &Last_Joint_State_Apply_Result() const;
  bool Has_Current_Model() const;
  const robot_model::Robot_Kinematic_Params &Kinematic_Params() const;
  const robot_model::Robot_Joint_State &Joint_State() const;
  vtkRenderer *Scene_Renderer();
  void Render_Scene();
  void Set_Flange_Frame_Visible(bool visible);
  bool Has_Flange_Pose() const;
  bool Get_World_From_Flange(robot_model::Matrix4 *pose) const;
  bool Get_World_From_Tool(robot_model::Matrix4 *pose) const;
  void Set_Tool_Coordinate(
    const robot_model::Tool_Coordinate_Profile &tool);
  bool Set_Collision_Obstacle_Points(const std::vector<float> &xyz, std::string *error_message = nullptr);
  bool Set_Collision_Obstacle_Points(std::shared_ptr<const std::vector<float>> xyz, std::string *error_message = nullptr);
  void Clear_Collision_Obstacle_Points();
  bool Has_Collision_Obstacle_Points() const;
  std::size_t Collision_Obstacle_Point_Count() const;
  bool Set_Collision_Settings(const robot_model::Robot_Collision_Settings &settings, std::string *error_message = nullptr);
  void Set_Collision_Enabled(bool enabled);
  bool Collision_Enabled() const;
  const robot_model::Robot_Collision_Settings &Collision_Settings() const;
  const robot_model::Robot_Collision_Point_Cloud_Stats &Collision_Point_Cloud_Stats() const;
  bool Collision_Rebuild_In_Progress() const;
  void Set_On_Collision_Rebuild_Completed(std::function<void(bool, const robot_model::Robot_Collision_Point_Cloud_Stats &, const std::string &)> callback);
  robot_model::Robot_Pose_IK_Result Move_Flange_To_Pose(const robot_model::Matrix4 &target_world_from_flange, const robot_model::Robot_Pose_IK_Options &options = {});
  void Set_World_Frame_Visible(bool visible);
  void Set_Flange_Interaction_Mode(Flange_Interaction_Mode mode);
  void Set_On_Flange_Dragged(std::function<void(const robot_model::Robot_Position_IK_Result &)> callback);
  void Set_On_Flange_Pose_Dragged(std::function<void(const robot_model::Robot_Pose_IK_Result &)> callback);
  void Set_On_Flange_Drag_State_Changed(std::function<void(bool)> callback);
  void Set_On_Flange_Drag_Performance(std::function<void(const robot_model::Robot_Drag_Performance_Stats &)> callback);
  void Set_Point_Cloud_Edit_Mode(
    bool enabled,
    Point_Cloud_Selection_Shape shape =
      Point_Cloud_Selection_Shape::Rectangle);
  bool Point_Cloud_Edit_Mode() const { return m_point_cloud_edit_mode; }
  void Set_On_Point_Cloud_Area_Selected(
    std::function<void(int, int, int, int, bool, bool)> callback);
  void Set_On_Point_Cloud_Polygon_Selected(
    std::function<void(
      const std::vector<std::array<int, 2>> &, bool, bool)> callback);
  void Set_On_Point_Cloud_Delete(std::function<void()> callback);
  void Set_On_Point_Cloud_Undo(std::function<void()> callback);
  void Set_On_Point_Cloud_Redo(std::function<void()> callback);
  void Clear_Point_Cloud_Selection_Outline();

private:
  void Ensure_VTK();
  void On_Size(wxSizeEvent &event);
  void On_Paint(wxPaintEvent &event);
  void On_Erase_Background(wxEraseEvent &event);
  void On_Mouse_Move(wxMouseEvent &event);
  void On_Mouse_Wheel(wxMouseEvent &event);
  void On_Left_Down(wxMouseEvent &event);
  void On_Left_Up(wxMouseEvent &event);
  void On_Right_Down(wxMouseEvent &event);
  void On_Right_Up(wxMouseEvent &event);
  void On_Middle_Down(wxMouseEvent &event);
  void On_Middle_Up(wxMouseEvent &event);
  void On_Mouse_Capture_Lost(wxMouseCaptureLostEvent &event);
  void On_Key_Down(wxKeyEvent &event);
  void On_Drag_Update_Timer(wxTimerEvent &event);

  void Render();
  vtkGenericRenderWindowInteractor *Interactor();
  void Set_Interactor_Event(const wxMouseEvent &event);
  void Update_Flange_Frame();
  void Update_Flange_Hover(const wxMouseEvent &event);
  void Queue_Flange_Drag_Update(int display_x, int display_y);
  void Apply_Pending_Flange_Drag_Update();
  void Finish_Flange_Drag_Performance();
  bool Queue_Collision_Rebuild(robot_model::Collision_Index_Build_Request request, bool settings_change);
  void Draw_Point_Cloud_Selection_Rectangle();
  void Clear_Point_Cloud_Selection_Rectangle();
  void Draw_Point_Cloud_Selection_Polygon();
  void Cancel_Point_Cloud_Polygon_Selection();

private:
  wxGLContext *m_gl_context = nullptr;
  std::unique_ptr<robot_model::Vtk_Scene> m_scene;
  robot_model::Robot_Render_Controller m_render_controller;
  robot_model::Coordinate_Frame_Renderer m_world_frame_renderer{500.0, 20.0, true};
  robot_model::Coordinate_Frame_Renderer m_tool_frame_renderer{140.0, 10.0, true};
  robot_model::Tool_Coordinate_Profile m_tool_coordinate{
    "flange", "Flange", {}};
  robot_model::Flange_Interaction_Controller m_flange_interaction;
  std::function<void(const robot_model::Robot_Position_IK_Result &)> m_on_flange_dragged;
  std::function<void(const robot_model::Robot_Pose_IK_Result &)> m_on_flange_pose_dragged;
  wxTimer m_drag_update_timer;
  robot_model::Flange_Drag_Update_Coordinator m_drag_update_coordinator;
  robot_model::Flange_Drag_Motion_Executor m_drag_motion_executor;
  robot_model::Collision_Index_Rebuild_Coordinator m_collision_index_rebuild_coordinator;
  std::function<void(bool, const robot_model::Robot_Collision_Point_Cloud_Stats &, const std::string &)> m_on_collision_rebuild_completed;
  std::function<void(bool)> m_on_flange_drag_state_changed;
  std::function<void(const robot_model::Robot_Drag_Performance_Stats &)> m_on_flange_drag_performance;
  robot_model::Robot_Drag_Performance_Collector m_drag_performance_collector;
  double m_last_render_time_ms = 0.0;
  bool m_point_cloud_edit_mode = false;
  Point_Cloud_Selection_Shape m_point_cloud_selection_shape =
    Point_Cloud_Selection_Shape::Rectangle;
  bool m_point_cloud_selecting = false;
  wxPoint m_point_cloud_selection_start;
  wxPoint m_point_cloud_selection_end;
  wxOverlay m_point_cloud_selection_overlay;
  std::function<void(int, int, int, int, bool, bool)>
    m_on_point_cloud_area_selected;
  std::vector<wxPoint> m_point_cloud_polygon_vertices;
  wxPoint m_point_cloud_polygon_preview;
  bool m_point_cloud_polygon_has_preview = false;
  bool m_point_cloud_polygon_closed = false;
  bool m_point_cloud_polygon_add = false;
  bool m_point_cloud_polygon_toggle = false;
  std::function<void(
    const std::vector<std::array<int, 2>> &, bool, bool)>
    m_on_point_cloud_polygon_selected;
  std::function<void()> m_on_point_cloud_delete;
  std::function<void()> m_on_point_cloud_undo;
  std::function<void()> m_on_point_cloud_redo;
};

#endif

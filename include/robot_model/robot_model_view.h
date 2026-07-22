#ifndef includeguard_robot_model_view_h_includeguard
#define includeguard_robot_model_view_h_includeguard

#include "robot_model_data.h"
#include "robot_render_controller.h"
#include "coordinate_frame_renderer.h"
#include "flange_interaction_controller.h"

#include <wx/glcanvas.h>
#include <wx/timer.h>

#include <chrono>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

class vtkGenericRenderWindowInteractor;
class vtkRenderer;

namespace robot_model
{
class Vtk_Scene;
} // namespace robot_model

struct Robot_Drag_Performance_Stats
{
  std::size_t update_count = 0;
  std::size_t blocked_update_count = 0;
  std::size_t checked_pose_count = 0;
  std::size_t obstacle_candidate_points = 0;
  std::size_t obstacle_distance_queries = 0;
  std::size_t self_exact_pair_queries = 0;
  std::size_t obstacle_blocked_update_count = 0;
  std::size_t self_blocked_update_count = 0;
  std::size_t ground_blocked_update_count = 0;
  double total_update_time_ms = 0.0;
  double maximum_update_time_ms = 0.0;
  double total_ik_time_ms = 0.0;
  double total_collision_time_ms = 0.0;
  double total_render_time_ms = 0.0;
};

class Robot_Model_View : public wxGLCanvas
{
public:
  using Flange_Interaction_Mode = robot_model::Flange_Interaction_Mode;

  explicit Robot_Model_View (wxWindow* parent, wxWindowID id = wxID_ANY);
  ~Robot_Model_View ( ) override;

  void Load_Model (const robot_model::Robot_Model_Info& model);
  void Set_Joint_State (const robot_model::Robot_Joint_State& state);
  robot_model::Robot_Joint_State_Apply_Result Try_Set_Joint_State (
    const robot_model::Robot_Joint_State& state);
  const robot_model::Robot_Joint_State_Apply_Result&
  Last_Joint_State_Apply_Result ( ) const;
  bool Has_Current_Model ( ) const;
  const robot_model::Robot_Kinematic_Params& Kinematic_Params ( ) const;
  const robot_model::Robot_Joint_State& Joint_State ( ) const;
  vtkRenderer* Scene_Renderer ( );
  void Render_Scene ( );
  void Set_Flange_Frame_Visible (bool visible);
  bool Has_Flange_Pose ( ) const;
  bool Get_World_From_Flange (robot_model::Matrix4* pose) const;
  bool Set_Collision_Obstacle_Points (
    const std::vector<float>& xyz,
    std::string* error_message = nullptr);
  bool Set_Collision_Obstacle_Points (
    std::shared_ptr<const std::vector<float>> xyz,
    std::string* error_message = nullptr);
  void Clear_Collision_Obstacle_Points ( );
  bool Has_Collision_Obstacle_Points ( ) const;
  std::size_t Collision_Obstacle_Point_Count ( ) const;
  bool Set_Collision_Settings (
    const robot_model::Robot_Collision_Settings& settings,
    std::string* error_message = nullptr);
  const robot_model::Robot_Collision_Settings& Collision_Settings ( ) const;
  const robot_model::Robot_Collision_Point_Cloud_Stats&
  Collision_Point_Cloud_Stats ( ) const;
  bool Collision_Rebuild_In_Progress ( ) const;
  void Set_On_Collision_Rebuild_Completed (
    std::function<void (
      bool,
      const robot_model::Robot_Collision_Point_Cloud_Stats&,
      const std::string&)> callback);
  robot_model::Robot_Pose_IK_Result Move_Flange_To_Pose (
    const robot_model::Matrix4& target_world_from_flange,
    const robot_model::Robot_Pose_IK_Options& options = { });
  void Set_World_Frame_Visible (bool visible);
  void Set_Flange_Interaction_Mode (Flange_Interaction_Mode mode);
  void Set_On_Flange_Dragged (
    std::function<void (const robot_model::Robot_Position_IK_Result&)> callback);
  void Set_On_Flange_Pose_Dragged (
    std::function<void (const robot_model::Robot_Pose_IK_Result&)> callback);
  void Set_On_Flange_Drag_State_Changed (
    std::function<void (bool)> callback);
  void Set_On_Flange_Drag_Performance (
    std::function<void (const Robot_Drag_Performance_Stats&)> callback);

private:
  void Ensure_VTK ( );
  void On_Size (wxSizeEvent& event);
  void On_Paint (wxPaintEvent& event);
  void On_Erase_Background (wxEraseEvent& event);
  void On_Mouse_Move (wxMouseEvent& event);
  void On_Mouse_Wheel (wxMouseEvent& event);
  void On_Left_Down (wxMouseEvent& event);
  void On_Left_Up (wxMouseEvent& event);
  void On_Right_Down (wxMouseEvent& event);
  void On_Right_Up (wxMouseEvent& event);
  void On_Middle_Down (wxMouseEvent& event);
  void On_Middle_Up (wxMouseEvent& event);
  void On_Mouse_Capture_Lost (wxMouseCaptureLostEvent& event);
  void On_Drag_Update_Timer (wxTimerEvent& event);

  void Render ( );
  vtkGenericRenderWindowInteractor* Interactor ( );
  void Set_Interactor_Event (const wxMouseEvent& event);
  void Update_Flange_Frame ( );
  void Update_Flange_Hover (const wxMouseEvent& event);
  void Queue_Flange_Drag_Update (int display_x, int display_y);
  void Apply_Pending_Flange_Drag_Update ( );
  void Finish_Flange_Drag_Performance ( );
  bool Queue_Collision_Rebuild (
    robot_model::Robot_Collision_Rebuild_Request request);
  void Collision_Rebuild_Worker_Loop ( );

private:
  wxGLContext* m_gl_context = nullptr;
  std::unique_ptr<robot_model::Vtk_Scene> m_scene;
  robot_model::Robot_Render_Controller m_render_controller;
  robot_model::Coordinate_Frame_Renderer m_world_frame_renderer {
    500.0, 20.0, true };
  robot_model::Flange_Interaction_Controller m_flange_interaction;
  std::function<void (const robot_model::Robot_Position_IK_Result&)>
    m_on_flange_dragged;
  std::function<void (const robot_model::Robot_Pose_IK_Result&)>
    m_on_flange_pose_dragged;
  wxTimer m_drag_update_timer;
  std::chrono::steady_clock::time_point m_last_drag_update_time;
  int m_pending_drag_x = 0;
  int m_pending_drag_y = 0;
  bool m_has_pending_drag_update = false;
  struct Pending_Collision_Rebuild
  {
    std::uint64_t generation = 0;
    robot_model::Robot_Collision_Rebuild_Request request;
  };
  std::thread m_collision_rebuild_worker;
  mutable std::mutex m_collision_rebuild_mutex;
  std::condition_variable m_collision_rebuild_condition;
  std::optional<Pending_Collision_Rebuild> m_pending_collision_rebuild;
  std::optional<robot_model::Robot_Collision_Rebuild_Request>
    m_latest_collision_rebuild_request;
  std::shared_ptr<std::atomic_bool> m_active_collision_rebuild_cancel;
  std::atomic<std::uint64_t> m_collision_rebuild_generation { 0 };
  std::atomic<bool> m_collision_rebuild_busy { false };
  bool m_stop_collision_rebuild_worker = false;
  std::function<void (
    bool,
    const robot_model::Robot_Collision_Point_Cloud_Stats&,
      const std::string&)> m_on_collision_rebuild_completed;
  std::function<void (bool)> m_on_flange_drag_state_changed;
  std::function<void (const Robot_Drag_Performance_Stats&)>
    m_on_flange_drag_performance;
  Robot_Drag_Performance_Stats m_drag_performance;
  double m_last_render_time_ms = 0.0;
  bool m_collect_drag_performance = false;
};

#endif

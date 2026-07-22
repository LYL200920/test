#include "robot_model_view.h"

#include "vtk_scene.h"

#include <wx/dcclient.h>
#include <wx/msw/window.h>

#include <GL/gl.h>

#include <utility>

Robot_Model_View::Robot_Model_View (wxWindow* parent, wxWindowID id)
  : wxGLCanvas (parent, id, nullptr, wxDefaultPosition, wxDefaultSize,
                wxBORDER_SIMPLE | wxFULL_REPAINT_ON_RESIZE),
    m_drag_update_timer (this)
{
  SetBackgroundStyle (wxBG_STYLE_PAINT);
  m_gl_context = new wxGLContext (this);

  Bind (wxEVT_SIZE, &Robot_Model_View::On_Size, this);
  Bind (wxEVT_PAINT, &Robot_Model_View::On_Paint, this);
  Bind (wxEVT_ERASE_BACKGROUND, &Robot_Model_View::On_Erase_Background, this);
  Bind (wxEVT_MOTION, &Robot_Model_View::On_Mouse_Move, this);
  Bind (wxEVT_MOUSEWHEEL, &Robot_Model_View::On_Mouse_Wheel, this);
  Bind (wxEVT_LEFT_DOWN, &Robot_Model_View::On_Left_Down, this);
  Bind (wxEVT_LEFT_UP, &Robot_Model_View::On_Left_Up, this);
  Bind (wxEVT_RIGHT_DOWN, &Robot_Model_View::On_Right_Down, this);
  Bind (wxEVT_RIGHT_UP, &Robot_Model_View::On_Right_Up, this);
  Bind (wxEVT_MIDDLE_DOWN, &Robot_Model_View::On_Middle_Down, this);
  Bind (wxEVT_MIDDLE_UP, &Robot_Model_View::On_Middle_Up, this);
  Bind (wxEVT_MOUSE_CAPTURE_LOST,
        &Robot_Model_View::On_Mouse_Capture_Lost, this);
  Bind (wxEVT_TIMER, &Robot_Model_View::On_Drag_Update_Timer, this,
        m_drag_update_timer.GetId ( ));
  m_collision_rebuild_worker = std::thread (
    &Robot_Model_View::Collision_Rebuild_Worker_Loop, this);
}

Robot_Model_View::~Robot_Model_View ( )
{
  m_drag_update_timer.Stop ( );
  ++m_collision_rebuild_generation;
  {
    std::lock_guard<std::mutex> lock (m_collision_rebuild_mutex);
    m_stop_collision_rebuild_worker = true;
    if( m_active_collision_rebuild_cancel )
      m_active_collision_rebuild_cancel->store (true);
    if( m_pending_collision_rebuild &&
        m_pending_collision_rebuild->request.cancel_requested )
    {
      m_pending_collision_rebuild->request.cancel_requested->store (true);
    }
    m_pending_collision_rebuild.reset ( );
  }
  m_collision_rebuild_condition.notify_one ( );
  if( m_collision_rebuild_worker.joinable ( ) )
    m_collision_rebuild_worker.join ( );
  m_flange_interaction.Detach_Renderer ( );
  m_world_frame_renderer.Detach_Renderer ( );
  m_render_controller.Detach_Scene ( );
  m_scene.reset ( );
  delete m_gl_context;
  m_gl_context = nullptr;
}

void Robot_Model_View::Load_Model (const robot_model::Robot_Model_Info& model)
{
  ++m_collision_rebuild_generation;
  m_collision_rebuild_busy.store (false);
  m_latest_collision_rebuild_request.reset ( );
  {
    std::lock_guard<std::mutex> lock (m_collision_rebuild_mutex);
    if( m_active_collision_rebuild_cancel )
      m_active_collision_rebuild_cancel->store (true);
    if( m_pending_collision_rebuild &&
        m_pending_collision_rebuild->request.cancel_requested )
    {
      m_pending_collision_rebuild->request.cancel_requested->store (true);
    }
    m_pending_collision_rebuild.reset ( );
  }
  Ensure_VTK ( );
  m_render_controller.Load_Model (model);
  if( m_scene )
  {
    m_flange_interaction.Attach_Renderer (m_scene->Renderer ( ));
    m_world_frame_renderer.Attach_Renderer (m_scene->Renderer ( ));
    m_world_frame_renderer.Set_Visible (m_world_frame_renderer.Is_Visible ( ));
  }
  Update_Flange_Frame ( );
  Render ( );
}

void Robot_Model_View::Set_Joint_State (
  const robot_model::Robot_Joint_State& state)
{
  m_render_controller.Set_Joint_State (state);
  Update_Flange_Frame ( );
  Render ( );
}

robot_model::Robot_Joint_State_Apply_Result
Robot_Model_View::Try_Set_Joint_State (
  const robot_model::Robot_Joint_State& state)
{
  const auto result = m_render_controller.Try_Set_Joint_State (state);
  if( result.state_changed || result.scene_changed )
  {
    Update_Flange_Frame ( );
    Render ( );
  }
  return result;
}

const robot_model::Robot_Joint_State_Apply_Result&
Robot_Model_View::Last_Joint_State_Apply_Result ( ) const
{
  return m_render_controller.Last_Joint_State_Apply_Result ( );
}

bool Robot_Model_View::Has_Current_Model ( ) const
{
  return m_render_controller.Has_Current_Model ( );
}

const robot_model::Robot_Kinematic_Params&
Robot_Model_View::Kinematic_Params ( ) const
{
  return m_render_controller.Kinematic_Params ( );
}

const robot_model::Robot_Joint_State& Robot_Model_View::Joint_State ( ) const
{
  return m_render_controller.Joint_State ( );
}

vtkRenderer* Robot_Model_View::Scene_Renderer ( )
{
  Ensure_VTK ( );
  return m_scene ? m_scene->Renderer ( ) : nullptr;
}

void Robot_Model_View::Render_Scene ( )
{
  Render ( );
}

void Robot_Model_View::Set_Flange_Frame_Visible (bool visible)
{
  Ensure_VTK ( );
  m_flange_interaction.Set_Frame_Visible (visible);
  Update_Flange_Frame ( );
  Render_Scene ( );
}

bool Robot_Model_View::Has_Flange_Pose ( ) const
{
  return m_render_controller.Has_Flange_Pose ( );
}

bool Robot_Model_View::Get_World_From_Flange (
  robot_model::Matrix4* pose) const
{
  if( !pose || !m_render_controller.Has_Flange_Pose ( ) ) return false;
  *pose = m_render_controller.World_From_Flange ( );
  return true;
}

bool Robot_Model_View::Set_Collision_Obstacle_Points (
  const std::vector<float>& xyz,
  std::string* error_message)
{
  return Set_Collision_Obstacle_Points (
    std::make_shared<const std::vector<float>> (xyz), error_message);
}

bool Robot_Model_View::Set_Collision_Obstacle_Points (
  std::shared_ptr<const std::vector<float>> xyz,
  std::string* error_message)
{
  robot_model::Robot_Collision_Rebuild_Request request;
  if( !m_render_controller.Create_Collision_Points_Rebuild_Request (
        std::move (xyz), &request, error_message) )
  {
    return false;
  }
  m_latest_collision_rebuild_request = request;
  return Queue_Collision_Rebuild (std::move (request));
}

void Robot_Model_View::Clear_Collision_Obstacle_Points ( )
{
  ++m_collision_rebuild_generation;
  m_collision_rebuild_busy.store (false);
  m_latest_collision_rebuild_request.reset ( );
  {
    std::lock_guard<std::mutex> lock (m_collision_rebuild_mutex);
    if( m_active_collision_rebuild_cancel )
      m_active_collision_rebuild_cancel->store (true);
    if( m_pending_collision_rebuild &&
        m_pending_collision_rebuild->request.cancel_requested )
    {
      m_pending_collision_rebuild->request.cancel_requested->store (true);
    }
    m_pending_collision_rebuild.reset ( );
  }
  m_render_controller.Clear_Collision_Obstacle_Points ( );
}

bool Robot_Model_View::Has_Collision_Obstacle_Points ( ) const
{
  return m_render_controller.Has_Collision_Obstacle_Points ( );
}

std::size_t Robot_Model_View::Collision_Obstacle_Point_Count ( ) const
{
  return m_render_controller.Collision_Obstacle_Point_Count ( );
}

bool Robot_Model_View::Set_Collision_Settings (
  const robot_model::Robot_Collision_Settings& settings,
  std::string* error_message)
{
  robot_model::Robot_Collision_Rebuild_Request request;
  if( !m_render_controller.Create_Collision_Settings_Rebuild_Request (
        settings, &request, error_message) )
  {
    return false;
  }
  if( m_latest_collision_rebuild_request )
  {
    request = *m_latest_collision_rebuild_request;
    request.settings = settings;
  }
  m_latest_collision_rebuild_request = request;
  return Queue_Collision_Rebuild (std::move (request));
}

const robot_model::Robot_Collision_Settings&
Robot_Model_View::Collision_Settings ( ) const
{
  return m_render_controller.Collision_Settings ( );
}

const robot_model::Robot_Collision_Point_Cloud_Stats&
Robot_Model_View::Collision_Point_Cloud_Stats ( ) const
{
  return m_render_controller.Collision_Point_Cloud_Stats ( );
}

bool Robot_Model_View::Collision_Rebuild_In_Progress ( ) const
{
  return m_collision_rebuild_busy.load ( );
}

void Robot_Model_View::Set_On_Collision_Rebuild_Completed (
  std::function<void (
    bool,
    const robot_model::Robot_Collision_Point_Cloud_Stats&,
    const std::string&)> callback)
{
  m_on_collision_rebuild_completed = std::move (callback);
}

robot_model::Robot_Pose_IK_Result Robot_Model_View::Move_Flange_To_Pose (
  const robot_model::Matrix4& target_world_from_flange,
  const robot_model::Robot_Pose_IK_Options& options)
{
  const auto result = m_render_controller.Move_Flange_To_Pose (
    target_world_from_flange,
    options);
  Update_Flange_Frame ( );
  Render ( );
  return result;
}

void Robot_Model_View::Set_World_Frame_Visible (bool visible)
{
  Ensure_VTK ( );
  robot_model::Matrix4 identity = { };
  for( int index = 0; index < 4; ++index ) identity[index][index] = 1.0;
  m_world_frame_renderer.Set_World_From_Frame (identity);
  m_world_frame_renderer.Set_Visible (visible);
  Render_Scene ( );
}

void Robot_Model_View::Set_Flange_Interaction_Mode (
  Flange_Interaction_Mode mode)
{
  Ensure_VTK ( );
  m_flange_interaction.Set_Mode (mode);
  Update_Flange_Frame ( );
  Render_Scene ( );
}

void Robot_Model_View::Set_On_Flange_Dragged (
  std::function<void (const robot_model::Robot_Position_IK_Result&)> callback)
{
  m_on_flange_dragged = std::move (callback);
}

void Robot_Model_View::Set_On_Flange_Pose_Dragged (
  std::function<void (const robot_model::Robot_Pose_IK_Result&)> callback)
{
  m_on_flange_pose_dragged = std::move (callback);
}

void Robot_Model_View::Set_On_Flange_Drag_State_Changed (
  std::function<void (bool)> callback)
{
  m_on_flange_drag_state_changed = std::move (callback);
}

void Robot_Model_View::Set_On_Flange_Drag_Performance (
  std::function<void (const Robot_Drag_Performance_Stats&)> callback)
{
  m_on_flange_drag_performance = std::move (callback);
}

void Robot_Model_View::Ensure_VTK ( )
{
  if( m_scene )
  {
    return;
  }

  if( !IsShownOnScreen ( ) )
  {
    return;
  }

  SetCurrent (*m_gl_context);
  m_scene = std::make_unique<robot_model::Vtk_Scene> ( );
  m_scene->Init (this, m_gl_context);
  m_render_controller.Attach_Scene (m_scene.get ( ));
  m_flange_interaction.Attach_Renderer (m_scene->Renderer ( ));
  m_world_frame_renderer.Attach_Renderer (m_scene->Renderer ( ));
  robot_model::Matrix4 identity = { };
  for( int index = 0; index < 4; ++index ) identity[index][index] = 1.0;
  m_world_frame_renderer.Set_World_From_Frame (identity);
  Update_Flange_Frame ( );
}

void Robot_Model_View::Update_Flange_Frame ( )
{
  if( m_render_controller.Has_Flange_Pose ( ) )
  {
    m_flange_interaction.Update_Flange_Pose (
      &m_render_controller.World_From_Flange ( ));
  }
  else
  {
    m_flange_interaction.Update_Flange_Pose (nullptr);
  }
}

void Robot_Model_View::On_Size (wxSizeEvent& event)
{
  Ensure_VTK ( );
  const auto size = GetClientSize ( );
  if( m_scene && size.x > 0 && size.y > 0 )
  {
    m_scene->Resize (size.x, size.y);
    Render ( );
  }
  event.Skip ( );
}

void Robot_Model_View::On_Paint (wxPaintEvent&)
{
  wxPaintDC dc (this);
  Ensure_VTK ( );
  Render ( );
}

void Robot_Model_View::On_Erase_Background (wxEraseEvent&)
{
}

void Robot_Model_View::Render ( )
{
  const auto started_at = std::chrono::steady_clock::now ( );
  m_last_render_time_ms = 0.0;
  if( m_scene && m_scene->Is_Ready ( ) )
  {
    SetCurrent (*m_gl_context);
    const auto size = GetClientSize ( );
    glViewport (0, 0, size.x, size.y);
    glClearColor (0.02f, 0.08f, 0.16f, 1.0f);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m_scene->Render ( );
    m_last_render_time_ms = std::chrono::duration<double, std::milli> (
      std::chrono::steady_clock::now ( ) - started_at).count ( );
  }
}

vtkGenericRenderWindowInteractor* Robot_Model_View::Interactor ( )
{
  Ensure_VTK ( );
  return m_scene ? m_scene->Get_Interactor ( ) : nullptr;
}

void Robot_Model_View::Set_Interactor_Event (const wxMouseEvent& event)
{
  if( !m_scene )
  {
    return;
  }

  auto* interactor = m_scene->Get_Interactor ( );
  if( !interactor )
  {
    return;
  }

  const auto pos = event.GetPosition ( );
  interactor->SetEventInformationFlipY (
    pos.x, pos.y,
    event.ControlDown ( ) ? 1 : 0,
    event.ShiftDown ( ) ? 1 : 0
  );
}

void Robot_Model_View::On_Mouse_Move (wxMouseEvent& event)
{
  if( m_flange_interaction.Is_Dragging ( ) )
  {
    const auto position = event.GetPosition ( );
    const auto size = GetClientSize ( );
    Queue_Flange_Drag_Update (
      position.x, size.y - 1 - position.y);
    return;
  }

  if( !event.LeftIsDown ( ) && !event.RightIsDown ( ) &&
      !event.MiddleIsDown ( ) )
  {
    Update_Flange_Hover (event);
  }

  auto* interactor = Interactor ( );
  if( !interactor )
  {
    return;
  }

  Set_Interactor_Event (event);
  interactor->MouseMoveEvent ( );
  Render ( );
}

void Robot_Model_View::On_Mouse_Wheel (wxMouseEvent& event)
{
  auto* interactor = Interactor ( );
  if( !interactor )
  {
    return;
  }

  Set_Interactor_Event (event);
  if( event.GetWheelRotation ( ) > 0 )
  {
    interactor->MouseWheelForwardEvent ( );
  }
  else
  {
    interactor->MouseWheelBackwardEvent ( );
  }
  Render ( );
}

void Robot_Model_View::On_Left_Down (wxMouseEvent& event)
{
  Ensure_VTK ( );
  if( m_scene )
  {
    const auto pos = event.GetPosition ( );
    if( m_scene->Handle_View_Cube_Click (pos.x, pos.y) )
    {
      Render ( );
      return;
    }
  }

  if( m_render_controller.Has_Flange_Pose ( ) )
  {
    const auto position = event.GetPosition ( );
    const auto size = GetClientSize ( );
    const auto& pose = m_render_controller.World_From_Flange ( );
    if( m_flange_interaction.Begin_Drag (
          position.x, size.y - 1 - position.y, pose) )
    {
      SetFocus ( );
      if( !HasCapture ( ) )
      {
        CaptureMouse ( );
      }
      m_drag_performance = { };
      m_collect_drag_performance = true;
      if( m_on_flange_drag_state_changed )
        m_on_flange_drag_state_changed (true);
      return;
    }
  }

  auto* interactor = m_scene ? m_scene->Get_Interactor ( ) : nullptr;
  if( !interactor )
  {
    return;
  }

  SetFocus ( );
  if( !HasCapture ( ) )
  {
    CaptureMouse ( );
  }
  Set_Interactor_Event (event);
  interactor->LeftButtonPressEvent ( );
  Render ( );
}

void Robot_Model_View::On_Left_Up (wxMouseEvent& event)
{
  if( m_flange_interaction.Is_Dragging ( ) )
  {
    const auto position = event.GetPosition ( );
    const auto size = GetClientSize ( );
    m_pending_drag_x = position.x;
    m_pending_drag_y = size.y - 1 - position.y;
    m_has_pending_drag_update = true;
    Apply_Pending_Flange_Drag_Update ( );
    m_flange_interaction.End_Drag ( );
    if( m_on_flange_drag_state_changed )
      m_on_flange_drag_state_changed (false);
    Finish_Flange_Drag_Performance ( );
    if( HasCapture ( ) ) ReleaseMouse ( );
    Render ( );
    return;
  }

  if( auto* interactor = Interactor ( ) )
  {
    Set_Interactor_Event (event);
    interactor->LeftButtonReleaseEvent ( );
    Render ( );
  }
  if( HasCapture ( ) )
  {
    ReleaseMouse ( );
  }
}

void Robot_Model_View::On_Right_Down (wxMouseEvent& event)
{
  auto* interactor = Interactor ( );
  if( !interactor )
  {
    return;
  }

  SetFocus ( );
  if( !HasCapture ( ) )
  {
    CaptureMouse ( );
  }
  Set_Interactor_Event (event);
  interactor->RightButtonPressEvent ( );
  Render ( );
}

void Robot_Model_View::On_Right_Up (wxMouseEvent& event)
{
  if( auto* interactor = Interactor ( ) )
  {
    Set_Interactor_Event (event);
    interactor->RightButtonReleaseEvent ( );
    Render ( );
  }
  if( HasCapture ( ) )
  {
    ReleaseMouse ( );
  }
}

void Robot_Model_View::On_Middle_Down (wxMouseEvent& event)
{
  auto* interactor = Interactor ( );
  if( !interactor )
  {
    return;
  }

  SetFocus ( );
  if( !HasCapture ( ) )
  {
    CaptureMouse ( );
  }
  Set_Interactor_Event (event);
  interactor->MiddleButtonPressEvent ( );
  Render ( );
}

void Robot_Model_View::On_Middle_Up (wxMouseEvent& event)
{
  if( auto* interactor = Interactor ( ) )
  {
    Set_Interactor_Event (event);
    interactor->MiddleButtonReleaseEvent ( );
    Render ( );
  }
  if( HasCapture ( ) )
  {
    ReleaseMouse ( );
  }
}

void Robot_Model_View::On_Mouse_Capture_Lost (wxMouseCaptureLostEvent&)
{
  const bool was_dragging = m_flange_interaction.Is_Dragging ( );
  m_drag_update_timer.Stop ( );
  m_has_pending_drag_update = false;
  m_flange_interaction.End_Drag ( );
  if( was_dragging && m_on_flange_drag_state_changed )
    m_on_flange_drag_state_changed (false);
  if( was_dragging ) Finish_Flange_Drag_Performance ( );
  Render ( );
}

void Robot_Model_View::On_Drag_Update_Timer (wxTimerEvent&)
{
  Apply_Pending_Flange_Drag_Update ( );
}

void Robot_Model_View::Queue_Flange_Drag_Update (
  int display_x,
  int display_y)
{
  constexpr auto minimum_interval = std::chrono::milliseconds (16);
  m_pending_drag_x = display_x;
  m_pending_drag_y = display_y;
  m_has_pending_drag_update = true;

  const auto now = std::chrono::steady_clock::now ( );
  if( m_last_drag_update_time.time_since_epoch ( ).count ( ) == 0 ||
      now - m_last_drag_update_time >= minimum_interval )
  {
    Apply_Pending_Flange_Drag_Update ( );
    return;
  }

  if( !m_drag_update_timer.IsRunning ( ) )
  {
    const auto remaining = minimum_interval -
      std::chrono::duration_cast<std::chrono::milliseconds> (
        now - m_last_drag_update_time);
    m_drag_update_timer.StartOnce (
      static_cast<int> (std::max<std::int64_t> (1, remaining.count ( ))));
  }
}

void Robot_Model_View::Apply_Pending_Flange_Drag_Update ( )
{
  m_drag_update_timer.Stop ( );
  if( !m_has_pending_drag_update ||
      !m_flange_interaction.Is_Dragging ( ) )
  {
    m_has_pending_drag_update = false;
    return;
  }

  m_has_pending_drag_update = false;
  const auto update_started_at = std::chrono::steady_clock::now ( );
  robot_model::Flange_Drag_Update update;
  if( !m_flange_interaction.Update_Drag (
        m_pending_drag_x, m_pending_drag_y, &update) )
  {
    return;
  }

  bool pose_changed = false;
  double ik_time_ms = 0.0;
  robot_model::Robot_Joint_State_Apply_Result apply_result;
  if( update.target_type ==
      robot_model::Flange_Drag_Update::Target_Type::Position )
  {
    const auto result = m_render_controller.Move_Flange_To (
      update.target_position_world);
    ik_time_ms = result.solve_time_ms;
    apply_result = m_render_controller.Last_Joint_State_Apply_Result ( );
    pose_changed = apply_result.state_changed || apply_result.scene_changed;
    Update_Flange_Frame ( );
    if( m_on_flange_dragged &&
        ( pose_changed || apply_result.accepted ||
          !apply_result.collision.collided ) )
    {
      m_on_flange_dragged (result);
    }
  }
  else if( update.target_type ==
           robot_model::Flange_Drag_Update::Target_Type::Pose )
  {
    const auto result = m_render_controller.Move_Flange_To_Pose (
      update.target_world_from_flange);
    ik_time_ms = result.solve_time_ms;
    apply_result = m_render_controller.Last_Joint_State_Apply_Result ( );
    pose_changed = apply_result.state_changed || apply_result.scene_changed;
    Update_Flange_Frame ( );
    if( m_on_flange_pose_dragged &&
        ( pose_changed || apply_result.accepted ||
          !apply_result.collision.collided ) )
    {
      m_on_flange_pose_dragged (result);
    }
  }

  m_last_drag_update_time = std::chrono::steady_clock::now ( );
  if( pose_changed ) Render ( );
  if( m_collect_drag_performance )
  {
    const double update_time_ms =
      std::chrono::duration<double, std::milli> (
        std::chrono::steady_clock::now ( ) - update_started_at).count ( );
    ++m_drag_performance.update_count;
    if( !apply_result.accepted && apply_result.collision.collided )
    {
      ++m_drag_performance.blocked_update_count;
      switch( apply_result.collision.type )
      {
        case robot_model::Robot_Collision_Type::Obstacle_Point:
          ++m_drag_performance.obstacle_blocked_update_count;
          break;
        case robot_model::Robot_Collision_Type::Self_Collision:
          ++m_drag_performance.self_blocked_update_count;
          break;
        case robot_model::Robot_Collision_Type::Ground:
          ++m_drag_performance.ground_blocked_update_count;
          break;
        default:
          break;
      }
    }
    m_drag_performance.checked_pose_count +=
      apply_result.checked_pose_count;
    m_drag_performance.obstacle_candidate_points +=
      apply_result.collision_query_stats.obstacle_candidate_points;
    m_drag_performance.obstacle_distance_queries +=
      apply_result.collision_query_stats.obstacle_distance_queries;
    m_drag_performance.self_exact_pair_queries +=
      apply_result.collision_query_stats.self_exact_pair_queries;
    m_drag_performance.total_update_time_ms += update_time_ms;
    m_drag_performance.maximum_update_time_ms = std::max (
      m_drag_performance.maximum_update_time_ms, update_time_ms);
    m_drag_performance.total_ik_time_ms += ik_time_ms;
    m_drag_performance.total_collision_time_ms +=
      apply_result.collision_query_time_ms;
    if( pose_changed )
      m_drag_performance.total_render_time_ms += m_last_render_time_ms;
  }
}

void Robot_Model_View::Finish_Flange_Drag_Performance ( )
{
  if( !m_collect_drag_performance ) return;
  m_collect_drag_performance = false;
  if( m_on_flange_drag_performance )
    m_on_flange_drag_performance (m_drag_performance);
}

bool Robot_Model_View::Queue_Collision_Rebuild (
  robot_model::Robot_Collision_Rebuild_Request request)
{
  const auto generation = ++m_collision_rebuild_generation;
  request.cancel_requested = std::make_shared<std::atomic_bool> (false);
  {
    std::lock_guard<std::mutex> lock (m_collision_rebuild_mutex);
    if( m_stop_collision_rebuild_worker ) return false;
    if( m_active_collision_rebuild_cancel )
      m_active_collision_rebuild_cancel->store (true);
    if( m_pending_collision_rebuild &&
        m_pending_collision_rebuild->request.cancel_requested )
    {
      m_pending_collision_rebuild->request.cancel_requested->store (true);
    }
    m_pending_collision_rebuild = Pending_Collision_Rebuild {
      generation, std::move (request)
    };
    m_collision_rebuild_busy.store (true);
  }
  m_collision_rebuild_condition.notify_one ( );
  return true;
}

void Robot_Model_View::Collision_Rebuild_Worker_Loop ( )
{
  for( ;; )
  {
    Pending_Collision_Rebuild pending;
    {
      std::unique_lock<std::mutex> lock (m_collision_rebuild_mutex);
      m_collision_rebuild_condition.wait (lock, [this]
      {
        return m_stop_collision_rebuild_worker ||
          m_pending_collision_rebuild.has_value ( );
      });
      if( m_stop_collision_rebuild_worker ) return;
      pending = std::move ( *m_pending_collision_rebuild );
      m_pending_collision_rebuild.reset ( );
      m_active_collision_rebuild_cancel =
        pending.request.cancel_requested;
    }

    const auto cancel_token = pending.request.cancel_requested;
    auto result = std::make_shared<
      robot_model::Robot_Collision_Rebuild_Result> (
        robot_model::Robot_Render_Controller::Build_Collision_Obstacle (
          std::move (pending.request)));
    {
      std::lock_guard<std::mutex> lock (m_collision_rebuild_mutex);
      if( m_active_collision_rebuild_cancel ==
          cancel_token )
      {
        m_active_collision_rebuild_cancel.reset ( );
      }
    }
    if( result->cancelled ||
        pending.generation != m_collision_rebuild_generation.load ( ) )
    {
      continue;
    }
    CallAfter ([this, generation = pending.generation,
                result = std::move (result)]
    {
      if( generation != m_collision_rebuild_generation.load ( ) ) return;
      m_collision_rebuild_busy.store (false);
      const bool success = result->success;
      const auto error_message = result->error_message;
      const auto failed_stats = result->stats;
      if( success )
      {
        m_render_controller.Apply_Collision_Rebuild_Result (
          std::move ( *result ));
        Update_Flange_Frame ( );
        Render ( );
      }
      if( m_on_collision_rebuild_completed )
      {
        m_on_collision_rebuild_completed (
          success,
          success
            ? m_render_controller.Collision_Point_Cloud_Stats ( )
            : failed_stats,
          error_message);
      }
    });
  }
}

void Robot_Model_View::Update_Flange_Hover (const wxMouseEvent& event)
{
  const auto position = event.GetPosition ( );
  const auto size = GetClientSize ( );
  const robot_model::Matrix4* pose = m_render_controller.Has_Flange_Pose ( )
    ? &m_render_controller.World_From_Flange ( )
    : nullptr;
  bool hover_changed = false;
  const bool over_handle = m_flange_interaction.Update_Hover (
    position.x, size.y - 1 - position.y, pose, &hover_changed);
  if( !hover_changed ) return;
  SetCursor (over_handle ? wxCursor (wxCURSOR_HAND) : wxNullCursor);
  Render ( );
}

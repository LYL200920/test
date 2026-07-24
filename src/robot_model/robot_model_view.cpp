#include "robot_model_view.h"

#include "vtk_scene.h"

#include <wx/dcclient.h>
#include <wx/msw/window.h>

#include <GL/gl.h>

#include <algorithm>
#include <cstdlib>
#include <utility>

Robot_Model_View::Robot_Model_View(wxWindow *parent, wxWindowID id)
    : wxGLCanvas(parent, id, nullptr, wxDefaultPosition, wxDefaultSize, wxBORDER_SIMPLE | wxFULL_REPAINT_ON_RESIZE),
      m_drag_update_timer(this)
{
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  m_gl_context = new wxGLContext(this);

  Bind(wxEVT_SIZE, &Robot_Model_View::On_Size, this);
  Bind(wxEVT_PAINT, &Robot_Model_View::On_Paint, this);
  Bind(wxEVT_ERASE_BACKGROUND, &Robot_Model_View::On_Erase_Background, this);
  Bind(wxEVT_MOTION, &Robot_Model_View::On_Mouse_Move, this);
  Bind(wxEVT_MOUSEWHEEL, &Robot_Model_View::On_Mouse_Wheel, this);
  Bind(wxEVT_LEFT_DOWN, &Robot_Model_View::On_Left_Down, this);
  Bind(wxEVT_LEFT_UP, &Robot_Model_View::On_Left_Up, this);
  Bind(wxEVT_RIGHT_DOWN, &Robot_Model_View::On_Right_Down, this);
  Bind(wxEVT_RIGHT_UP, &Robot_Model_View::On_Right_Up, this);
  Bind(wxEVT_MIDDLE_DOWN, &Robot_Model_View::On_Middle_Down, this);
  Bind(wxEVT_MIDDLE_UP, &Robot_Model_View::On_Middle_Up, this);
  Bind(wxEVT_MOUSE_CAPTURE_LOST, &Robot_Model_View::On_Mouse_Capture_Lost, this);
  Bind(wxEVT_KEY_DOWN, &Robot_Model_View::On_Key_Down, this);
  Bind(wxEVT_TIMER, &Robot_Model_View::On_Drag_Update_Timer, this, m_drag_update_timer.GetId());
}

Robot_Model_View::~Robot_Model_View()
{
  m_drag_update_timer.Stop();
  m_collision_index_rebuild_coordinator.Shutdown();
  m_flange_interaction.Detach_Renderer();
  m_tool_frame_renderer.Detach_Renderer();
  m_world_frame_renderer.Detach_Renderer();
  m_render_controller.Detach_Scene();
  m_scene.reset();
  delete m_gl_context;
  m_gl_context = nullptr;
}

void Robot_Model_View::Load_Model(const robot_model::Robot_Model_Info &model)
{
  m_collision_index_rebuild_coordinator.Reset();
  Ensure_VTK();
  m_render_controller.Load_Model(model);
  if (m_scene)
  {
    m_flange_interaction.Attach_Renderer(m_scene->Renderer());
    m_tool_frame_renderer.Attach_Renderer(m_scene->Renderer());
    m_world_frame_renderer.Attach_Renderer(m_scene->Renderer());
    m_world_frame_renderer.Set_Visible(m_world_frame_renderer.Is_Visible());
  }
  Update_Flange_Frame();
  Render();
}

void Robot_Model_View::Set_Joint_State(const robot_model::Robot_Joint_State &state)
{
  m_render_controller.Set_Joint_State(state);
  Update_Flange_Frame();
  Render();
}

robot_model::Robot_Joint_State_Apply_Result
Robot_Model_View::Try_Set_Joint_State(const robot_model::Robot_Joint_State &state)
{
  const auto result = m_render_controller.Try_Set_Joint_State(state);
  if (result.state_changed || result.scene_changed)
  {
    Update_Flange_Frame();
    Render();
  }
  return result;
}

const robot_model::Robot_Joint_State_Apply_Result &Robot_Model_View::Last_Joint_State_Apply_Result() const
{
  return m_render_controller.Last_Joint_State_Apply_Result();
}

bool Robot_Model_View::Has_Current_Model() const
{
  return m_render_controller.Has_Current_Model();
}

const robot_model::Robot_Kinematic_Params &Robot_Model_View::Kinematic_Params() const
{
  return m_render_controller.Kinematic_Params();
}

const robot_model::Robot_Joint_State &Robot_Model_View::Joint_State() const
{
  return m_render_controller.Joint_State();
}

vtkRenderer *Robot_Model_View::Scene_Renderer()
{
  Ensure_VTK();
  return m_scene ? m_scene->Renderer() : nullptr;
}

void Robot_Model_View::Render_Scene()
{
  Render();
}

void Robot_Model_View::Set_Point_Cloud_Edit_Mode(
  bool enabled,
  Point_Cloud_Selection_Shape shape)
{
  if (m_point_cloud_edit_mode == enabled &&
      m_point_cloud_selection_shape == shape)
  {
    return;
  }
  Cancel_Point_Cloud_Polygon_Selection();
  m_point_cloud_edit_mode = enabled;
  m_point_cloud_selection_shape = shape;
  if (!enabled)
  {
    m_point_cloud_selecting = false;
    Clear_Point_Cloud_Selection_Rectangle();
    if (HasCapture())
    {
      ReleaseMouse();
    }
  }
}

void Robot_Model_View::Set_On_Point_Cloud_Area_Selected(
  std::function<void(int, int, int, int, bool, bool)> callback)
{
  m_on_point_cloud_area_selected = std::move(callback);
}

void Robot_Model_View::Set_On_Point_Cloud_Polygon_Selected(
  std::function<void(
    const std::vector<std::array<int, 2>> &, bool, bool)> callback)
{
  m_on_point_cloud_polygon_selected = std::move(callback);
}

void Robot_Model_View::Set_On_Point_Cloud_Delete(
  std::function<void()> callback)
{
  m_on_point_cloud_delete = std::move(callback);
}

void Robot_Model_View::Set_On_Point_Cloud_Undo(
  std::function<void()> callback)
{
  m_on_point_cloud_undo = std::move(callback);
}

void Robot_Model_View::Set_On_Point_Cloud_Redo(
  std::function<void()> callback)
{
  m_on_point_cloud_redo = std::move(callback);
}

void Robot_Model_View::Clear_Point_Cloud_Selection_Outline()
{
  Cancel_Point_Cloud_Polygon_Selection();
}

void Robot_Model_View::Set_Flange_Frame_Visible(bool visible)
{
  Ensure_VTK();
  m_flange_interaction.Set_Frame_Visible(visible);
  Update_Flange_Frame();
  Render_Scene();
}

bool Robot_Model_View::Has_Flange_Pose() const
{
  return m_render_controller.Has_Flange_Pose();
}

bool Robot_Model_View::Get_World_From_Flange(robot_model::Matrix4 *pose) const
{
  if (!pose || !m_render_controller.Has_Flange_Pose())
    return false;
  *pose = m_render_controller.World_From_Flange();
  return true;
}

bool Robot_Model_View::Get_World_From_Tool(robot_model::Matrix4 *pose) const
{
  if (!pose || !m_render_controller.Has_Flange_Pose())
  {
    return false;
  }
  *pose = robot_model::Build_World_From_Tool(
    m_render_controller.World_From_Flange(),
    m_tool_coordinate);
  return true;
}

void Robot_Model_View::Set_Tool_Coordinate(
  const robot_model::Tool_Coordinate_Profile &tool)
{
  m_tool_coordinate = tool;
  Update_Flange_Frame();
  Render();
}

bool Robot_Model_View::Set_Collision_Obstacle_Points(const std::vector<float> &xyz,
                                                     std::string *error_message)
{
  return Set_Collision_Obstacle_Points(
      std::make_shared<const std::vector<float>>(xyz), error_message);
}

bool Robot_Model_View::Set_Collision_Obstacle_Points(std::shared_ptr<const std::vector<float>> xyz,
                                                     std::string *error_message)
{
  if (!m_render_controller.Collision_Enabled())
  {
    return m_render_controller.Set_Collision_Obstacle_Points(std::move(xyz), error_message);
  }
  robot_model::Collision_Index_Build_Request request;
  if (!m_render_controller.Create_Collision_Points_Rebuild_Request(std::move(xyz), &request, error_message))
  {
    return false;
  }
  return Queue_Collision_Rebuild(std::move(request), false);
}

void Robot_Model_View::Clear_Collision_Obstacle_Points()
{
  m_collision_index_rebuild_coordinator.Reset();
  m_render_controller.Clear_Collision_Obstacle_Points();
}

bool Robot_Model_View::Has_Collision_Obstacle_Points() const
{
  return m_render_controller.Has_Collision_Obstacle_Points();
}

std::size_t Robot_Model_View::Collision_Obstacle_Point_Count() const
{
  return m_render_controller.Collision_Obstacle_Point_Count();
}

bool Robot_Model_View::Set_Collision_Settings(const robot_model::Robot_Collision_Settings &settings,
                                              std::string *error_message)
{
  if (!m_render_controller.Collision_Enabled())
  {
    return m_render_controller.Set_Collision_Settings(settings, error_message);
  }
  robot_model::Collision_Index_Build_Request request;
  if (!m_render_controller.Create_Collision_Settings_Rebuild_Request(settings, &request, error_message))
  {
    return false;
  }
  return Queue_Collision_Rebuild(std::move(request), true);
}

void Robot_Model_View::Set_Collision_Enabled(bool enabled)
{
  if (m_render_controller.Collision_Enabled() == enabled)
    return;
  if (!enabled)
    m_collision_index_rebuild_coordinator.Reset();
  m_render_controller.Set_Collision_Enabled(enabled);
  if (enabled && m_render_controller.Has_Collision_Obstacle_Source())
  {
    robot_model::Collision_Index_Build_Request request;
    if (m_render_controller.Create_Collision_Settings_Rebuild_Request(m_render_controller.Collision_Settings(), &request, nullptr))
    {
      Queue_Collision_Rebuild(std::move(request), true);
    }
  }
  Render();
}

bool Robot_Model_View::Collision_Enabled() const
{
  return m_render_controller.Collision_Enabled();
}

const robot_model::Robot_Collision_Settings &
Robot_Model_View::Collision_Settings() const
{
  return m_render_controller.Collision_Settings();
}

const robot_model::Robot_Collision_Point_Cloud_Stats &
Robot_Model_View::Collision_Point_Cloud_Stats() const
{
  return m_render_controller.Collision_Point_Cloud_Stats();
}

bool Robot_Model_View::Collision_Rebuild_In_Progress() const
{
  return m_collision_index_rebuild_coordinator.Busy();
}

void Robot_Model_View::Set_On_Collision_Rebuild_Completed(
    std::function<void(
        bool,
        const robot_model::Robot_Collision_Point_Cloud_Stats &,
        const std::string &)>
        callback)
{
  m_on_collision_rebuild_completed = std::move(callback);
}

robot_model::Robot_Pose_IK_Result Robot_Model_View::Move_Flange_To_Pose(const robot_model::Matrix4 &target_world_from_flange,
                                                                        const robot_model::Robot_Pose_IK_Options &options)
{
  const auto result = m_render_controller.Move_Flange_To_Pose(target_world_from_flange, options);
  Update_Flange_Frame();
  Render();
  return result;
}

void Robot_Model_View::Set_World_Frame_Visible(bool visible)
{
  Ensure_VTK();
  robot_model::Matrix4 identity = {};
  for (int index = 0; index < 4; ++index)
    identity[index][index] = 1.0;
  m_world_frame_renderer.Set_World_From_Frame(identity);
  m_world_frame_renderer.Set_Visible(visible);
  Render_Scene();
}

void Robot_Model_View::Set_Flange_Interaction_Mode(Flange_Interaction_Mode mode)
{
  Ensure_VTK();
  m_flange_interaction.Set_Mode(mode);
  Update_Flange_Frame();
  Render_Scene();
}

void Robot_Model_View::Set_On_Flange_Dragged(std::function<void(const robot_model::Robot_Position_IK_Result &)> callback)
{
  m_on_flange_dragged = std::move(callback);
}

void Robot_Model_View::Set_On_Flange_Pose_Dragged(std::function<void(const robot_model::Robot_Pose_IK_Result &)> callback)
{
  m_on_flange_pose_dragged = std::move(callback);
}

void Robot_Model_View::Set_On_Flange_Drag_State_Changed(std::function<void(bool)> callback)
{
  m_on_flange_drag_state_changed = std::move(callback);
}

void Robot_Model_View::Set_On_Flange_Drag_Performance(std::function<void(
                                                          const robot_model::Robot_Drag_Performance_Stats &)>
                                                          callback)
{
  m_on_flange_drag_performance = std::move(callback);
}

void Robot_Model_View::Ensure_VTK()
{
  if (m_scene)
  {
    return;
  }

  if (!IsShownOnScreen())
  {
    return;
  }

  SetCurrent(*m_gl_context);
  m_scene = std::make_unique<robot_model::Vtk_Scene>();
  m_scene->Init(this, m_gl_context);
  m_render_controller.Attach_Scene(m_scene.get());
  m_flange_interaction.Attach_Renderer(m_scene->Renderer());
  m_tool_frame_renderer.Attach_Renderer(m_scene->Renderer());
  m_world_frame_renderer.Attach_Renderer(m_scene->Renderer());
  robot_model::Matrix4 identity = {};
  for (int index = 0; index < 4; ++index)
    identity[index][index] = 1.0;
  m_world_frame_renderer.Set_World_From_Frame(identity);
  Update_Flange_Frame();
}

void Robot_Model_View::Update_Flange_Frame()
{
  if (m_render_controller.Has_Flange_Pose())
  {
    m_flange_interaction.Update_Flange_Pose(&m_render_controller.World_From_Flange());
    m_tool_frame_renderer.Set_World_From_Frame(
      robot_model::Build_World_From_Tool(
        m_render_controller.World_From_Flange(),
        m_tool_coordinate));
    m_tool_frame_renderer.Set_Visible(m_tool_coordinate.id != "flange");
  }
  else
  {
    m_flange_interaction.Update_Flange_Pose(nullptr);
    m_tool_frame_renderer.Set_Visible(false);
  }
}

void Robot_Model_View::On_Size(wxSizeEvent &event)
{
  Ensure_VTK();
  const auto size = GetClientSize();
  if (m_scene && size.x > 0 && size.y > 0)
  {
    m_scene->Resize(size.x, size.y);
    Render();
  }
  event.Skip();
}

void Robot_Model_View::On_Paint(wxPaintEvent &)
{
  wxPaintDC dc(this);
  Ensure_VTK();
  Render();
}

void Robot_Model_View::On_Erase_Background(wxEraseEvent &)
{
}

void Robot_Model_View::Render()
{
  const auto started_at = std::chrono::steady_clock::now();
  m_last_render_time_ms = 0.0;
  if (m_scene && m_scene->Is_Ready())
  {
    SetCurrent(*m_gl_context);
    const auto size = GetClientSize();
    glViewport(0, 0, size.x, size.y);
    glClearColor(0.02f, 0.08f, 0.16f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m_scene->Render();
    m_last_render_time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started_at).count();
  }
}

vtkGenericRenderWindowInteractor *Robot_Model_View::Interactor()
{
  Ensure_VTK();
  return m_scene ? m_scene->Get_Interactor() : nullptr;
}

void Robot_Model_View::Set_Interactor_Event(const wxMouseEvent &event)
{
  if (!m_scene)
  {
    return;
  }

  auto *interactor = m_scene->Get_Interactor();
  if (!interactor)
  {
    return;
  }

  const auto pos = event.GetPosition();
  interactor->SetEventInformationFlipY(pos.x, pos.y,
                                       event.ControlDown() ? 1 : 0,
                                       event.ShiftDown() ? 1 : 0);
}

void Robot_Model_View::On_Mouse_Move(wxMouseEvent &event)
{
  if (m_point_cloud_selecting)
  {
    m_point_cloud_selection_end = event.GetPosition();
    Draw_Point_Cloud_Selection_Rectangle();
    return;
  }
  if (m_point_cloud_edit_mode &&
      m_point_cloud_selection_shape ==
        Point_Cloud_Selection_Shape::Polygon &&
      !m_point_cloud_polygon_vertices.empty())
  {
    if (!m_point_cloud_polygon_closed)
    {
      m_point_cloud_polygon_preview = event.GetPosition();
      m_point_cloud_polygon_has_preview = true;
      Draw_Point_Cloud_Selection_Polygon();
    }
    return;
  }

  if (m_flange_interaction.Is_Dragging())
  {
    const auto position = event.GetPosition();
    const auto size = GetClientSize();
    Queue_Flange_Drag_Update(position.x, size.y - 1 - position.y);
    return;
  }

  if (!event.LeftIsDown() &&
      !event.RightIsDown() &&
      !event.MiddleIsDown())
  {
    Update_Flange_Hover(event);
  }

  auto *interactor = Interactor();
  if (!interactor)
  {
    return;
  }

  Set_Interactor_Event(event);
  interactor->MouseMoveEvent();
  Render();
}

void Robot_Model_View::On_Mouse_Wheel(wxMouseEvent &event)
{
  if (m_point_cloud_edit_mode &&
      m_point_cloud_selection_shape ==
        Point_Cloud_Selection_Shape::Polygon &&
      !m_point_cloud_polygon_vertices.empty())
  {
    return;
  }

  auto *interactor = Interactor();
  if (!interactor)
  {
    return;
  }

  Set_Interactor_Event(event);
  if (event.GetWheelRotation() > 0)
  {
    interactor->MouseWheelForwardEvent();
  }
  else
  {
    interactor->MouseWheelBackwardEvent();
  }
  Render();
}

void Robot_Model_View::On_Left_Down(wxMouseEvent &event)
{
  if (m_point_cloud_edit_mode)
  {
    SetFocus();
    if (m_point_cloud_selection_shape ==
        Point_Cloud_Selection_Shape::Polygon)
    {
      if (m_point_cloud_polygon_closed)
      {
        Cancel_Point_Cloud_Polygon_Selection();
      }
      if (m_point_cloud_polygon_vertices.empty())
      {
        m_point_cloud_polygon_add = event.ShiftDown();
        m_point_cloud_polygon_toggle = event.ControlDown();
      }
      m_point_cloud_polygon_vertices.push_back(event.GetPosition());
      m_point_cloud_polygon_preview = event.GetPosition();
      m_point_cloud_polygon_has_preview = true;
      m_point_cloud_polygon_closed = false;
      Draw_Point_Cloud_Selection_Polygon();
      return;
    }

    m_point_cloud_selecting = true;
    m_point_cloud_selection_start = event.GetPosition();
    m_point_cloud_selection_end = m_point_cloud_selection_start;
    if (!HasCapture())
    {
      CaptureMouse();
    }
    Draw_Point_Cloud_Selection_Rectangle();
    return;
  }

  Ensure_VTK();
  if (m_scene)
  {
    const auto pos = event.GetPosition();
    if (m_scene->Handle_View_Cube_Click(pos.x, pos.y))
    {
      Render();
      return;
    }
  }

  if (m_render_controller.Has_Flange_Pose())
  {
    const auto position = event.GetPosition();
    const auto size = GetClientSize();
    const auto &pose = m_render_controller.World_From_Flange();
    if (m_flange_interaction.Begin_Drag(position.x, size.y - 1 - position.y, pose))
    {
      SetFocus();
      if (!HasCapture())
      {
        CaptureMouse();
      }
      m_drag_update_coordinator.Begin();
      m_drag_performance_collector.Begin();
      if (m_on_flange_drag_state_changed)
        m_on_flange_drag_state_changed(true);
      return;
    }
  }

  auto *interactor = m_scene ? m_scene->Get_Interactor() : nullptr;
  if (!interactor)
  {
    return;
  }

  SetFocus();
  if (!HasCapture())
  {
    CaptureMouse();
  }
  Set_Interactor_Event(event);
  interactor->LeftButtonPressEvent();
  Render();
}

void Robot_Model_View::On_Left_Up(wxMouseEvent &event)
{
  if (m_point_cloud_edit_mode &&
      m_point_cloud_selection_shape ==
        Point_Cloud_Selection_Shape::Polygon)
  {
    return;
  }

  if (m_point_cloud_selecting)
  {
    m_point_cloud_selection_end = event.GetPosition();
    const wxPoint start = m_point_cloud_selection_start;
    const wxPoint end = m_point_cloud_selection_end;
    m_point_cloud_selecting = false;
    Clear_Point_Cloud_Selection_Rectangle();
    if (HasCapture())
    {
      ReleaseMouse();
    }
    if (m_on_point_cloud_area_selected)
    {
      const auto size = GetClientSize();
      m_on_point_cloud_area_selected(
        start.x,
        size.y - 1 - start.y,
        end.x,
        size.y - 1 - end.y,
        event.ShiftDown(),
        event.ControlDown());
    }
    return;
  }

  if (m_flange_interaction.Is_Dragging())
  {
    const auto position = event.GetPosition();
    const auto size = GetClientSize();
    m_drag_update_coordinator.Set_Final(position.x, size.y - 1 - position.y);
    Apply_Pending_Flange_Drag_Update();
    m_flange_interaction.End_Drag();
    if (m_on_flange_drag_state_changed)
      m_on_flange_drag_state_changed(false);
    Finish_Flange_Drag_Performance();
    if (HasCapture())
      ReleaseMouse();
    Render();
    return;
  }

  if (auto *interactor = Interactor())
  {
    Set_Interactor_Event(event);
    interactor->LeftButtonReleaseEvent();
    Render();
  }
  if (HasCapture())
  {
    ReleaseMouse();
  }
}

void Robot_Model_View::On_Right_Down(wxMouseEvent &event)
{
  if (m_point_cloud_edit_mode &&
      m_point_cloud_selection_shape ==
        Point_Cloud_Selection_Shape::Polygon)
  {
    if (m_point_cloud_polygon_closed)
    {
      return;
    }
    if (m_point_cloud_polygon_vertices.size() < 2)
    {
      Cancel_Point_Cloud_Polygon_Selection();
      return;
    }

    m_point_cloud_polygon_vertices.push_back(event.GetPosition());
    const std::vector<wxPoint> completed_vertices =
      m_point_cloud_polygon_vertices;

    std::vector<std::array<int, 2>> polygon;
    polygon.reserve(m_point_cloud_polygon_vertices.size());
    const auto size = GetClientSize();
    for (const wxPoint &vertex : m_point_cloud_polygon_vertices)
    {
      polygon.push_back({vertex.x, size.y - 1 - vertex.y});
    }
    const bool add = m_point_cloud_polygon_add;
    const bool toggle = m_point_cloud_polygon_toggle;
    Cancel_Point_Cloud_Polygon_Selection();
    if (m_on_point_cloud_polygon_selected)
    {
      m_on_point_cloud_polygon_selected(
        polygon,
        add,
        toggle);
    }
    if (m_point_cloud_edit_mode &&
        m_point_cloud_selection_shape ==
          Point_Cloud_Selection_Shape::Polygon)
    {
      m_point_cloud_polygon_vertices = completed_vertices;
      m_point_cloud_polygon_has_preview = false;
      m_point_cloud_polygon_closed = true;
      Draw_Point_Cloud_Selection_Polygon();
    }
    return;
  }

  auto *interactor = Interactor();
  if (!interactor)
  {
    return;
  }

  SetFocus();
  if (!HasCapture())
  {
    CaptureMouse();
  }
  Set_Interactor_Event(event);
  interactor->RightButtonPressEvent();
  Render();
}

void Robot_Model_View::On_Right_Up(wxMouseEvent &event)
{
  if (m_point_cloud_edit_mode &&
      m_point_cloud_selection_shape ==
        Point_Cloud_Selection_Shape::Polygon)
  {
    return;
  }

  if (auto *interactor = Interactor())
  {
    Set_Interactor_Event(event);
    interactor->RightButtonReleaseEvent();
    Render();
  }
  if (HasCapture())
  {
    ReleaseMouse();
  }
}

void Robot_Model_View::On_Middle_Down(wxMouseEvent &event)
{
  auto *interactor = Interactor();
  if (!interactor)
  {
    return;
  }

  SetFocus();
  if (!HasCapture())
  {
    CaptureMouse();
  }
  Set_Interactor_Event(event);
  interactor->MiddleButtonPressEvent();
  Render();
}

void Robot_Model_View::On_Middle_Up(wxMouseEvent &event)
{
  if (auto *interactor = Interactor())
  {
    Set_Interactor_Event(event);
    interactor->MiddleButtonReleaseEvent();
    Render();
  }
  if (HasCapture())
  {
    ReleaseMouse();
  }
}

void Robot_Model_View::On_Mouse_Capture_Lost(wxMouseCaptureLostEvent &)
{
  if (m_point_cloud_selecting)
  {
    m_point_cloud_selecting = false;
    Clear_Point_Cloud_Selection_Rectangle();
  }
  const bool was_dragging = m_flange_interaction.Is_Dragging();
  m_drag_update_timer.Stop();
  m_drag_update_coordinator.Cancel();
  m_flange_interaction.End_Drag();
  if (was_dragging && m_on_flange_drag_state_changed)
    m_on_flange_drag_state_changed(false);
  if (was_dragging)
    Finish_Flange_Drag_Performance();
  Render();
}

void Robot_Model_View::On_Key_Down(wxKeyEvent &event)
{
  if (!m_point_cloud_edit_mode)
  {
    event.Skip();
    return;
  }

  if (event.GetKeyCode() == WXK_ESCAPE)
  {
    Cancel_Point_Cloud_Polygon_Selection();
    m_point_cloud_selecting = false;
    Clear_Point_Cloud_Selection_Rectangle();
    if (HasCapture())
    {
      ReleaseMouse();
    }
    return;
  }
  if (event.GetKeyCode() == WXK_DELETE && m_on_point_cloud_delete)
  {
    m_on_point_cloud_delete();
    return;
  }
  if (event.ControlDown() &&
      (event.GetKeyCode() == 'Z' || event.GetKeyCode() == 'z') &&
      m_on_point_cloud_undo)
  {
    m_on_point_cloud_undo();
    return;
  }
  if (event.ControlDown() &&
      (event.GetKeyCode() == 'Y' || event.GetKeyCode() == 'y') &&
      m_on_point_cloud_redo)
  {
    m_on_point_cloud_redo();
    return;
  }
  event.Skip();
}

void Robot_Model_View::Draw_Point_Cloud_Selection_Rectangle()
{
  wxClientDC dc(this);
  wxDCOverlay overlay(m_point_cloud_selection_overlay, &dc);
  overlay.Clear();
  if (!m_point_cloud_selecting)
  {
    return;
  }

  dc.SetPen(wxPen(wxColour(255, 196, 32), 1, wxPENSTYLE_SHORT_DASH));
  dc.SetBrush(*wxTRANSPARENT_BRUSH);
  const int minimum_x = std::min(
    m_point_cloud_selection_start.x, m_point_cloud_selection_end.x);
  const int minimum_y = std::min(
    m_point_cloud_selection_start.y, m_point_cloud_selection_end.y);
  const int width = std::abs(
    m_point_cloud_selection_end.x - m_point_cloud_selection_start.x);
  const int height = std::abs(
    m_point_cloud_selection_end.y - m_point_cloud_selection_start.y);
  dc.DrawRectangle(minimum_x, minimum_y, width, height);
}

void Robot_Model_View::Clear_Point_Cloud_Selection_Rectangle()
{
  wxClientDC dc(this);
  wxDCOverlay overlay(m_point_cloud_selection_overlay, &dc);
  overlay.Clear();
  m_point_cloud_selection_overlay.Reset();
}

void Robot_Model_View::Draw_Point_Cloud_Selection_Polygon()
{
  wxClientDC dc(this);
  wxDCOverlay overlay(m_point_cloud_selection_overlay, &dc);
  overlay.Clear();
  if (m_point_cloud_polygon_vertices.empty())
  {
    return;
  }

  dc.SetPen(wxPen(wxColour(255, 196, 32), 2));
  for (std::size_t index = 0;
       index < m_point_cloud_polygon_vertices.size();
       ++index)
  {
    if (index > 0)
    {
      dc.DrawLine(
        m_point_cloud_polygon_vertices[index - 1],
        m_point_cloud_polygon_vertices[index]);
    }
  }
  if (m_point_cloud_polygon_closed &&
      m_point_cloud_polygon_vertices.size() >= 3)
  {
    dc.DrawLine(
      m_point_cloud_polygon_vertices.back(),
      m_point_cloud_polygon_vertices.front());
  }
  else if (m_point_cloud_polygon_has_preview)
  {
    dc.SetPen(wxPen(
      wxColour(255, 196, 32), 1, wxPENSTYLE_SHORT_DASH));
    dc.DrawLine(
      m_point_cloud_polygon_vertices.back(),
      m_point_cloud_polygon_preview);
  }
}

void Robot_Model_View::Cancel_Point_Cloud_Polygon_Selection()
{
  if (m_point_cloud_polygon_vertices.empty() &&
      !m_point_cloud_polygon_closed)
  {
    return;
  }
  m_point_cloud_polygon_vertices.clear();
  m_point_cloud_polygon_has_preview = false;
  m_point_cloud_polygon_closed = false;
  m_point_cloud_polygon_add = false;
  m_point_cloud_polygon_toggle = false;
  Clear_Point_Cloud_Selection_Rectangle();
}

void Robot_Model_View::On_Drag_Update_Timer(wxTimerEvent &)
{
  Apply_Pending_Flange_Drag_Update();
}

void Robot_Model_View::Queue_Flange_Drag_Update(int display_x, int display_y)
{
  const auto schedule = m_drag_update_coordinator.Queue(display_x, display_y);
  if (schedule.process_now)
  {
    Apply_Pending_Flange_Drag_Update();
    return;
  }

  if (schedule.timer_delay_ms > 0 && !m_drag_update_timer.IsRunning())
  {
    m_drag_update_timer.StartOnce(schedule.timer_delay_ms);
  }
}

void Robot_Model_View::Apply_Pending_Flange_Drag_Update()
{
  m_drag_update_timer.Stop();
  robot_model::Flange_Drag_Pointer pointer;
  if (!m_flange_interaction.Is_Dragging() || !m_drag_update_coordinator.Take_Pending(&pointer))
  {
    return;
  }

  const auto update_started_at = std::chrono::steady_clock::now();
  robot_model::Flange_Drag_Update update;
  if (!m_flange_interaction.Update_Drag(pointer.display_x, pointer.display_y, &update))
  {
    return;
  }

  const auto execution = m_drag_motion_executor.Execute(update, [this](const robot_model::Point3 &target)
                                                        {
                                                          robot_model::Flange_Position_Motion_Outcome outcome;
                                                          outcome.ik_result = m_render_controller.Move_Flange_To(target);
                                                          outcome.apply_result =m_render_controller.Last_Joint_State_Apply_Result();
                                                          return outcome; }, [this](const robot_model::Matrix4 &target)
                                                        {
                                                          robot_model::Flange_Pose_Motion_Outcome outcome;
                                                          outcome.ik_result = m_render_controller.Move_Flange_To_Pose(target);
                                                          outcome.apply_result = m_render_controller.Last_Joint_State_Apply_Result();
                                                          return outcome; });
  if (!execution.handled)
    return;

  const auto &apply_result = execution.apply_result;
  const bool pose_changed = execution.pose_changed;
  Update_Flange_Frame();
  if (execution.notify_result &&
      execution.target_type == robot_model::Flange_Drag_Update::Target_Type::Position &&
      m_on_flange_dragged)
  {
    m_on_flange_dragged(execution.position_result);
  }
  else if (execution.notify_result &&
           execution.target_type == robot_model::Flange_Drag_Update::Target_Type::Pose &&
           m_on_flange_pose_dragged)
  {
    m_on_flange_pose_dragged(execution.pose_result);
  }

  m_drag_update_coordinator.Mark_Processed();
  if (pose_changed)
    Render();
  if (m_drag_performance_collector.Active())
  {
    const double update_time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - update_started_at).count();
    robot_model::Robot_Drag_Performance_Sample sample;
    sample.accepted = apply_result.accepted;
    sample.collision = apply_result.collision;
    sample.checked_pose_count = apply_result.checked_pose_count;
    sample.collision_query_stats = apply_result.collision_query_stats;
    sample.update_time_ms = update_time_ms;
    sample.ik_time_ms = execution.ik_time_ms;
    sample.collision_time_ms = apply_result.collision_query_time_ms;
    sample.render_time_ms = m_last_render_time_ms;
    sample.rendered = pose_changed;
    m_drag_performance_collector.Record(sample);
  }
}

void Robot_Model_View::Finish_Flange_Drag_Performance()
{
  robot_model::Robot_Drag_Performance_Stats stats;
  if (!m_drag_performance_collector.Finish(&stats))
    return;
  m_drag_update_coordinator.Cancel();
  if (m_on_flange_drag_performance)
    m_on_flange_drag_performance(stats);
}

bool Robot_Model_View::Queue_Collision_Rebuild(robot_model::Collision_Index_Build_Request request, bool settings_change)
{
  auto dispatch = [this](std::function<void()> task)
  {
    CallAfter(std::move(task));
  };
  auto completion = [this](robot_model::Collision_Index_Build_Result completed_result)
  {
    const bool success = completed_result.success;
    const auto error_message = completed_result.error_message;
    const auto failed_stats = completed_result.stats;
    if (success)
    {
      m_render_controller.Apply_Collision_Rebuild_Result(std::move(completed_result));
      Update_Flange_Frame();
      Render();
    }
    if (m_on_collision_rebuild_completed)
    {
      m_on_collision_rebuild_completed(success,
                                       success ? m_render_controller.Collision_Point_Cloud_Stats()
                                               : failed_stats,
                                       error_message);
    }
  };
  if (settings_change)
  {
    return m_collision_index_rebuild_coordinator.Submit_Settings(std::move(request),
                                                                 std::move(dispatch),
                                                                 std::move(completion));
  }
  return m_collision_index_rebuild_coordinator.Submit_Source(std::move(request),
                                                             std::move(dispatch),
                                                             std::move(completion));
}

void Robot_Model_View::Update_Flange_Hover(const wxMouseEvent &event)
{
  const auto position = event.GetPosition();
  const auto size = GetClientSize();
  const robot_model::Matrix4 *pose = m_render_controller.Has_Flange_Pose() ? &m_render_controller.World_From_Flange()
                                                                           : nullptr;
  bool hover_changed = false;
  const bool over_handle = m_flange_interaction.Update_Hover(position.x, size.y - 1 - position.y, pose, &hover_changed);
  if (!hover_changed)
    return;
  SetCursor(over_handle ? wxCursor(wxCURSOR_HAND) : wxNullCursor);
  Render();
}

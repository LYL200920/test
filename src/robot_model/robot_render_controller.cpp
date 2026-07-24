#include "robot_render_controller.h"

#include "collision_index_build_service.h"
#include "robot_calibration_builder.h"
#include "robot_kinematic_params.h"
#include "robot_model_config_loader.h"
#include "vtk_scene.h"

#include <algorithm>
#include <cmath>

namespace robot_model
{
  namespace
  {
    constexpr double kMaximumCollisionJointStepDeg = 2.0;
    constexpr double kMaximumCollisionSpatialStepMm = 5.0;
    constexpr std::size_t kCollisionBoundaryRefinementIterations = 4;
    constexpr std::size_t kInteractiveBoundaryRefinementIterations = 2;
    constexpr std::size_t kMaximumInteractiveSweepPoseCount = 8;

    double position_distance(const Point3 &lhs, const Point3 &rhs)
    {
      const double dx = lhs[0] - rhs[0];
      const double dy = lhs[1] - rhs[1];
      const double dz = lhs[2] - rhs[2];
      return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    double orientation_distance_degrees(const Matrix4 &actual,
                                        const Matrix4 &target)
    {
      double trace = 0.0;
      for (int row = 0; row < 3; ++row)
      {
        for (int column = 0; column < 3; ++column)
        {
          trace += target[column][row] * actual[column][row];
        }
      }
      const double cosine = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
      return std::acos(cosine) * 180.0 / 3.14159265358979323846;
    }

  } // namespace

  void Robot_Render_Controller::Attach_Scene(Vtk_Scene *scene)
  {
    m_scene = scene;
    if (m_state.Robot_Model().Has_Current_Model())
    {
      Rebuild_Current_Model();
    }
  }

  void Robot_Render_Controller::Detach_Scene()
  {
    m_assembly.Clear();
    m_collision_service.Clear_Detector();
    m_forward_model = {};
    m_has_forward_model = false;
    m_scene = nullptr;
  }

  void Robot_Render_Controller::Load_Model(const Robot_Model_Info &model)
  {
    const auto params = Load_Robot_Kinematic_Params(model.xml_path, model.display_name);
    m_state.Robot_Model().Set_Current_Model(model, params);
    Rebuild_Current_Model();
  }

  void Robot_Render_Controller::Set_Joint_State(const Robot_Joint_State &joint_state)
  {
    m_state.Robot_Model().Set_Joint_State(joint_state);
    Apply_Joint_Pose();
    Update_Current_Pose_Collision();
    const auto &current_collision = m_collision_service.Current_Collision();
    const bool scene_changed = current_collision.collided ? m_assembly.Show_Collision(current_collision)
                                                          : m_assembly.Clear_Collision();
    m_last_joint_apply_result = {};
    m_last_joint_apply_result.accepted = true;
    m_last_joint_apply_result.state_changed = true;
    m_last_joint_apply_result.scene_changed = scene_changed;
  }

  Robot_Joint_State_Apply_Result
  Robot_Render_Controller::Try_Set_Joint_State(const Robot_Joint_State &joint_state)
  {
    return Try_Set_Joint_State_With_Refinement(joint_state, kCollisionBoundaryRefinementIterations);
  }

  Robot_Joint_State_Apply_Result
  Robot_Render_Controller::Try_Set_Joint_State_With_Refinement(const Robot_Joint_State &joint_state,
                                                               std::size_t boundary_refinement_iterations,
                                                               std::size_t maximum_sweep_pose_count)
  {
    m_last_joint_apply_result = {};
    auto &result = m_last_joint_apply_result;
    auto &model_state = m_state.Robot_Model();
    if (!model_state.Has_Current_Model())
      return result;

    if (!m_collision_service.Enabled() ||
        !m_has_forward_model ||
        !m_collision_service.Has_Robot_Geometry())
    {
      Set_Joint_State(joint_state);
      result.accepted = true;
      return result;
    }

    Robot_Motion_Collision_Request request;
    request.current_state = model_state.Joint_State();
    request.requested_state = joint_state;
    request.current_collision = m_collision_service.Current_Collision();
    request.options.clearance_mm = m_collision_service.Settings().clearance_mm;
    request.options.maximum_joint_step_deg = kMaximumCollisionJointStepDeg;
    request.options.maximum_spatial_step_mm = kMaximumCollisionSpatialStepMm;
    request.options.boundary_refinement_iterations = boundary_refinement_iterations;
    request.options.maximum_sweep_pose_count = maximum_sweep_pose_count;
    const auto guarded = m_collision_service.Evaluate_Motion(m_forward_model, request);

    result.accepted = guarded.accepted;
    result.state_changed = guarded.state_changed;
    result.collision_checked = true;
    result.recovery_motion = guarded.recovery_motion;
    result.clearance_mm = request.options.clearance_mm;
    result.checked_pose_count = guarded.checked_pose_count;
    result.collision_query_time_ms = guarded.collision_query_time_ms;
    result.collision_query_stats = guarded.collision_query_stats;
    result.collision = guarded.blocking_collision;
    if (guarded.state_changed)
    {
      model_state.Set_Joint_State(guarded.applied_state);
      m_assembly.Apply_Forward_Kinematics(guarded.applied_transforms);
    }
    m_collision_service.Set_Current_Collision(guarded.resulting_collision);
    const auto &current_collision = m_collision_service.Current_Collision();
    result.scene_changed = !guarded.accepted && guarded.blocking_collision.collided ? m_assembly.Show_Collision(guarded.blocking_collision)
                                                                                    : (current_collision.collided ? m_assembly.Show_Collision(current_collision)
                                                                                                                  : m_assembly.Clear_Collision());
    return result;
  }

  bool Robot_Render_Controller::Has_Current_Model() const
  {
    return m_state.Robot_Model().Has_Current_Model();
  }

  const Robot_Kinematic_Params &Robot_Render_Controller::Kinematic_Params() const
  {
    return m_state.Robot_Model().Params();
  }

  const Robot_Joint_State &Robot_Render_Controller::Joint_State() const
  {
    return m_state.Robot_Model().Joint_State();
  }

  bool Robot_Render_Controller::Has_Flange_Pose() const
  {
    return m_assembly.Has_Flange_Pose();
  }

  const Matrix4 &Robot_Render_Controller::World_From_Flange() const
  {
    return m_assembly.World_From_Flange();
  }

  bool Robot_Render_Controller::Set_Collision_Obstacle_Points(const std::vector<float> &xyz, std::string *error_message)
  {
    return Set_Collision_Obstacle_Points(std::make_shared<const std::vector<float>>(xyz), error_message);
  }

  bool Robot_Render_Controller::Set_Collision_Obstacle_Points(std::shared_ptr<const std::vector<float>> xyz, std::string *error_message)
  {
    const auto &reference_state = m_state.Robot_Model().Joint_State();
    std::vector<Matrix4> reference_world_from_parts;
    if (m_has_forward_model)
    {
      reference_world_from_parts = Compute_Forward_Kinematics(m_forward_model, reference_state).world_from_parts;
    }
    const bool rebuilt = m_collision_service.Set_Obstacle_Source(std::move(xyz),
                                                                 reference_state,
                                                                 reference_world_from_parts,
                                                                 error_message);
    if (rebuilt)
    {
      Update_Current_Pose_Collision();
      const auto &collision = m_collision_service.Current_Collision();
      if (collision.collided)
        m_assembly.Show_Collision(collision);
      else
        m_assembly.Clear_Collision();
    }
    return rebuilt;
  }

  void Robot_Render_Controller::Clear_Collision_Obstacle_Points()
  {
    m_collision_service.Clear_Obstacle_Source();
    Update_Current_Pose_Collision();
    const auto &collision = m_collision_service.Current_Collision();
    if (collision.collided)
      m_assembly.Show_Collision(collision);
    else
      m_assembly.Clear_Collision();
  }

  bool Robot_Render_Controller::Has_Collision_Obstacle_Points() const
  {
    return m_collision_service.Has_Obstacle_Points();
  }

  bool Robot_Render_Controller::Has_Collision_Obstacle_Source() const
  {
    return m_collision_service.Has_Obstacle_Source();
  }

  std::size_t Robot_Render_Controller::Collision_Obstacle_Point_Count() const
  {
    return m_collision_service.Obstacle_Point_Count();
  }

  bool Robot_Render_Controller::Set_Collision_Settings(const Robot_Collision_Settings &settings, std::string *error_message)
  {
    std::vector<Matrix4> reference_world_from_parts;
    if (m_has_forward_model)
    {
      reference_world_from_parts = Compute_Forward_Kinematics(m_forward_model, m_collision_service.Reference_Joint_State()).world_from_parts;
    }
    if (!m_collision_service.Set_Settings(settings, reference_world_from_parts, error_message))
      return false;
    Update_Current_Pose_Collision();
    const auto &collision = m_collision_service.Current_Collision();
    if (collision.collided)
      m_assembly.Show_Collision(collision);
    else
      m_assembly.Clear_Collision();
    return true;
  }

  void Robot_Render_Controller::Set_Collision_Enabled(bool enabled)
  {
    if (m_collision_service.Enabled() == enabled)
      return;
    m_collision_service.Set_Enabled(enabled);
    if (enabled)
      Update_Current_Pose_Collision();

    const auto &collision = m_collision_service.Current_Collision();
    if (collision.collided)
      m_assembly.Show_Collision(collision);
    else
      m_assembly.Clear_Collision();
  }

  bool Robot_Render_Controller::Create_Collision_Points_Rebuild_Request(std::shared_ptr<const std::vector<float>> xyz,
                                                                        Collision_Index_Build_Request *request,
                                                                        std::string *error_message) const
  {
    const auto &reference_state = m_state.Robot_Model().Joint_State();
    std::vector<Matrix4> reference_world_from_parts;
    if (m_has_forward_model)
    {
      reference_world_from_parts = Compute_Forward_Kinematics(m_forward_model, reference_state).world_from_parts;
    }
    return m_collision_service.Create_Points_Rebuild_Request(std::move(xyz),
                                                             reference_state,
                                                             reference_world_from_parts,
                                                             request, error_message);
  }

  bool Robot_Render_Controller::Create_Collision_Settings_Rebuild_Request(const Robot_Collision_Settings &settings,
                                                                          Collision_Index_Build_Request *request,
                                                                          std::string *error_message) const
  {
    std::vector<Matrix4> reference_world_from_parts;
    if (m_has_forward_model)
    {
      reference_world_from_parts = Compute_Forward_Kinematics(m_forward_model, m_collision_service.Reference_Joint_State()).world_from_parts;
    }
    return m_collision_service.Create_Settings_Rebuild_Request(settings,
                                                               reference_world_from_parts,
                                                               request, error_message);
  }

  void Robot_Render_Controller::Apply_Collision_Rebuild_Result(Collision_Index_Build_Result result)
  {
    if (!result.success)
      return;
    m_collision_service.Apply_Rebuild_Result(std::move(result));
    Update_Current_Pose_Collision();
    const auto &collision = m_collision_service.Current_Collision();
    if (collision.collided)
      m_assembly.Show_Collision(collision);
    else
      m_assembly.Clear_Collision();
  }

  bool Robot_Render_Controller::Rebuild_Collision_Obstacle_Points(std::string *error_message)
  {
    std::vector<Matrix4> reference_world_from_parts;
    if (m_has_forward_model)
    {
      reference_world_from_parts = Compute_Forward_Kinematics(m_forward_model, m_state.Robot_Model().Joint_State()).world_from_parts;
    }
    return m_collision_service.Rebuild_Obstacle_Source(m_state.Robot_Model().Joint_State(),
                                                       reference_world_from_parts,
                                                       error_message);
  }

  Robot_Position_IK_Result Robot_Render_Controller::Move_Flange_To(const Point3 &target_world)
  {
    m_last_joint_apply_result = {};
    Robot_Position_IK_Result result;
    auto &model_state = m_state.Robot_Model();
    if (!m_has_forward_model || !model_state.Has_Current_Model())
    {
      return result;
    }

    result = Solve_Flange_Position_IK(m_forward_model,
                                      model_state.Params(),
                                      model_state.Joint_State(),
                                      target_world);
    if (result.status != Robot_IK_Status::Invalid_Model &&
        std::isfinite(result.position_error_mm))
    {
      Try_Set_Joint_State_With_Refinement(result.joint_state, kInteractiveBoundaryRefinementIterations,
                                          kMaximumInteractiveSweepPoseCount);
      const auto actual = Compute_Forward_Kinematics(m_forward_model, model_state.Joint_State());
      if (actual.has_flange)
      {
        result.joint_state = model_state.Joint_State();
        result.achieved_position_world = {actual.world_from_flange[0][3],
                                          actual.world_from_flange[1][3],
                                          actual.world_from_flange[2][3]};
        result.position_error_mm = position_distance(result.achieved_position_world, target_world);
      }
    }
    return result;
  }

  Robot_Pose_IK_Result Robot_Render_Controller::Move_Flange_To_Pose(const Matrix4 &target_world_from_flange,
                                                                    const Robot_Pose_IK_Options &options)
  {
    m_last_joint_apply_result = {};
    Robot_Pose_IK_Result result;
    auto &model_state = m_state.Robot_Model();
    if (!m_has_forward_model || !model_state.Has_Current_Model())
    {
      return result;
    }
    result = Solve_Flange_Pose_IK(m_forward_model,
                                  model_state.Params(),
                                  model_state.Joint_State(),
                                  target_world_from_flange,
                                  options);
    if (result.status != Robot_IK_Status::Invalid_Model &&
        std::isfinite(result.position_error_mm) &&
        std::isfinite(result.orientation_error_deg))
    {
      Try_Set_Joint_State_With_Refinement(result.joint_state, kInteractiveBoundaryRefinementIterations,
                                          kMaximumInteractiveSweepPoseCount);
      const auto actual = Compute_Forward_Kinematics(m_forward_model, model_state.Joint_State());
      if (actual.has_flange)
      {
        result.joint_state = model_state.Joint_State();
        result.achieved_world_from_flange = actual.world_from_flange;
        const Point3 achieved_position = {actual.world_from_flange[0][3],
                                          actual.world_from_flange[1][3],
                                          actual.world_from_flange[2][3]};
        const Point3 target_position = {target_world_from_flange[0][3],
                                        target_world_from_flange[1][3],
                                        target_world_from_flange[2][3]};
        result.position_error_mm = position_distance(achieved_position, target_position);
        result.orientation_error_deg = orientation_distance_degrees(actual.world_from_flange, target_world_from_flange);
      }
    }
    return result;
  }

  void Robot_Render_Controller::Rebuild_Current_Model()
  {
    auto &robot_model_state = m_state.Robot_Model();
    if (m_scene == nullptr ||
        !m_scene->Is_Ready() ||
        !robot_model_state.Has_Current_Model())
    {
      return;
    }

    const auto &model = robot_model_state.Current_Model();
    m_scene->Remove_All_ViewProps();
    m_scene->Remove_All_Lights();
    m_assembly.Clear();
    m_collision_service.Set_Robot_Parts({});
    m_forward_model = {};
    m_has_forward_model = false;
    robot_model_state.Clear_Assembly_Calibration();

    if (model.Has_Mesh())
    {
      m_assembly.Load_Mesh(model, m_scene->Renderer());
      auto scene_collision_options = m_collision_service.Scene_Options();
      scene_collision_options.self_collision_clearance_mm = robot_model_state.Params().self_collision_clearance_mm;
      scene_collision_options.self_collision_pairs = robot_model_state.Params().self_collision_pairs;
      scene_collision_options.ground_checked_parts = robot_model_state.Params().ground_collision_parts;
      m_collision_service.Set_Scene_Options(scene_collision_options);
      m_collision_service.Set_Robot_Parts(m_assembly.Parts());
      const auto assembly_calibration = Build_Assembly_Calibration(robot_model_state.Params(),
                                                                   robot_model_state.Model_Name(),
                                                                   m_assembly.Parts());
      robot_model_state.Set_Assembly_Calibration(assembly_calibration);
      m_forward_model = Build_Forward_Kinematics_Model(m_assembly.Parts().size(),
                                                       robot_model_state.Assembly_Calibration(),
                                                       robot_model_state.Params());
      m_has_forward_model = m_forward_model.has_flange;
      Rebuild_Collision_Obstacle_Points(nullptr);
      Apply_Joint_Pose();
      Update_Current_Pose_Collision();
    }
    else
    {
      m_assembly.Build_Fallback_Demo(robot_model_state.Params(),
                                     m_scene->Renderer());
    }

    const double link_extent = robot_model_state.Params().Has_Link_Lengths() ? link_length_at(robot_model_state.Params(), 1, 1000.0) + link_length_at(robot_model_state.Params(), 3, 1000.0)
                                                                             : 2000.0;
    const double extent = std::max(5000.0, link_extent * 2.2);
    m_scene->Add_Ground_Grid(extent);
    m_scene->Reset_Camera();
  }

  void Robot_Render_Controller::Apply_Joint_Pose()
  {
    const auto &robot_model_state = m_state.Robot_Model();
    if (!robot_model_state.Has_Assembly_Calibration())
    {
      return;
    }

    if (m_has_forward_model)
    {
      m_assembly.Apply_Forward_Kinematics(Compute_Forward_Kinematics(m_forward_model, robot_model_state.Joint_State()));
    }
  }

  void Robot_Render_Controller::Update_Current_Pose_Collision()
  {
    m_collision_service.Clear_Current_Collision();
    const auto &robot_model_state = m_state.Robot_Model();
    if (!m_collision_service.Enabled() || !m_has_forward_model ||
        !robot_model_state.Has_Current_Model() ||
        !m_collision_service.Has_Robot_Geometry())
    {
      return;
    }
    const auto transforms = Compute_Forward_Kinematics(m_forward_model, robot_model_state.Joint_State());
    m_collision_service.Check_Pose(transforms.world_from_parts);
  }

} // namespace robot_model

#ifndef includeguard_robot_render_controller_h_includeguard
#define includeguard_robot_render_controller_h_includeguard

#include "robot_model_data.h"
#include "robot_collision_detector.h"
#include "robot_forward_kinematics.h"
#include "robot_inverse_kinematics.h"
#include "robot_scene_assembly.h"
#include "robot_simulation_state.h"
#include "pose_transform.h"

#include <string>
#include <memory>
#include <vector>

namespace robot_model
{

class Vtk_Scene;

struct Robot_Joint_State_Apply_Result
{
  bool accepted = false;
  bool state_changed = false;
  bool scene_changed = false;
  bool collision_checked = false;
  bool recovery_motion = false;
  double clearance_mm = 0.0;
  std::size_t checked_pose_count = 0;
  double collision_query_time_ms = 0.0;
  Robot_Collision_Query_Stats collision_query_stats;
  Robot_Collision_Result collision;
};

struct Robot_Collision_Settings
{
  double clearance_mm = 10.0;
  Robot_Collision_Point_Cloud_Options point_cloud;
};

struct Robot_Collision_Rebuild_Request
{
  std::shared_ptr<const std::vector<float>> source_xyz;
  Robot_Collision_Settings settings;
  Robot_Joint_State reference_joint_state;
  std::vector<Robot_Visual_Part> robot_parts;
  std::vector<Matrix4> reference_world_from_parts;
  Robot_Scene_Collision_Options scene_options;
  std::shared_ptr<std::atomic_bool> cancel_requested;
};

struct Robot_Collision_Rebuild_Result
{
  std::unique_ptr<Robot_Collision_Detector> detector;
  std::shared_ptr<const std::vector<float>> source_xyz;
  Robot_Collision_Settings settings;
  Robot_Joint_State reference_joint_state;
  Robot_Collision_Point_Cloud_Stats stats;
  std::string error_message;
  bool success = false;
  bool cancelled = false;
};

class Robot_Render_Controller
{
public:
  void Attach_Scene (Vtk_Scene* scene);
  void Detach_Scene ( );

  void Load_Model (const Robot_Model_Info& model);
  void Set_Joint_State (const Robot_Joint_State& joint_state);
  Robot_Joint_State_Apply_Result Try_Set_Joint_State (
    const Robot_Joint_State& joint_state);
  const Robot_Joint_State_Apply_Result& Last_Joint_State_Apply_Result ( ) const
  {
    return m_last_joint_apply_result;
  }
  bool Has_Current_Model ( ) const;
  const Robot_Kinematic_Params& Kinematic_Params ( ) const;
  const Robot_Joint_State& Joint_State ( ) const;
  bool Has_Flange_Pose ( ) const;
  const Matrix4& World_From_Flange ( ) const;
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
    const Robot_Collision_Settings& settings,
    std::string* error_message = nullptr);
  const Robot_Collision_Settings& Collision_Settings ( ) const
  {
    return m_collision_settings;
  }
  const Robot_Collision_Point_Cloud_Stats& Collision_Point_Cloud_Stats ( ) const
  {
    return m_collision_point_cloud_stats;
  }
  bool Create_Collision_Points_Rebuild_Request (
    std::shared_ptr<const std::vector<float>> xyz,
    Robot_Collision_Rebuild_Request* request,
    std::string* error_message = nullptr) const;
  bool Create_Collision_Settings_Rebuild_Request (
    const Robot_Collision_Settings& settings,
    Robot_Collision_Rebuild_Request* request,
    std::string* error_message = nullptr) const;
  static Robot_Collision_Rebuild_Result Build_Collision_Obstacle (
    Robot_Collision_Rebuild_Request request);
  void Apply_Collision_Rebuild_Result (
    Robot_Collision_Rebuild_Result result);
  Robot_Position_IK_Result Move_Flange_To (const Point3& target_world);
  Robot_Pose_IK_Result Move_Flange_To_Pose (
    const Matrix4& target_world_from_flange,
    const Robot_Pose_IK_Options& options = { });

private:
  Robot_Joint_State_Apply_Result Try_Set_Joint_State_With_Refinement (
    const Robot_Joint_State& joint_state,
    std::size_t boundary_refinement_iterations,
    std::size_t maximum_sweep_pose_count = 0);
  void Rebuild_Current_Model ( );
  void Apply_Joint_Pose ( );
  bool Rebuild_Collision_Obstacle_Points (
    std::string* error_message = nullptr);
  void Update_Current_Pose_Collision ( );

private:
  Vtk_Scene* m_scene = nullptr;
  Robot_Simulation_State m_state;
  Robot_Scene_Assembly m_assembly;
  Robot_Collision_Detector m_collision_detector;
  Robot_Collision_Settings m_collision_settings;
  Robot_Collision_Point_Cloud_Stats m_collision_point_cloud_stats;
  std::shared_ptr<const std::vector<float>> m_collision_source_xyz;
  Robot_Joint_State m_collision_reference_joint_state;
  Robot_Forward_Kinematics_Model m_forward_model;
  Robot_Joint_State_Apply_Result m_last_joint_apply_result;
  Robot_Collision_Result m_current_pose_collision;
  bool m_has_forward_model = false;
};

} // namespace robot_model

#endif

#ifndef includeguard_robot_collision_service_h_includeguard
#define includeguard_robot_collision_service_h_includeguard

#include "collision_index_build_service.h"
#include "robot_motion_collision_guard.h"

#include <memory>
#include <string>
#include <vector>

namespace robot_model
{

  // Owns collision policy and all mutable collision state. Rendering code may
  // consume Current_Collision(), but does not own or duplicate collision state.
  class Robot_Collision_Service
  {
  public:
    void Clear_Detector();
    void Set_Robot_Parts(const std::vector<Robot_Visual_Part> &parts);
    void Set_Scene_Options(const Robot_Scene_Collision_Options &options);
    const Robot_Scene_Collision_Options &Scene_Options() const;

    void Set_Enabled(bool enabled);
    bool Enabled() const { return m_enabled; }

    bool Set_Obstacle_Source(std::shared_ptr<const std::vector<float>> xyz,
                             const Robot_Joint_State &reference_joint_state,
                             const std::vector<Matrix4> &reference_world_from_parts,
                             std::string *error_message = nullptr);
    bool Rebuild_Obstacle_Source(const Robot_Joint_State &reference_joint_state,
                                 const std::vector<Matrix4> &reference_world_from_parts,
                                 std::string *error_message = nullptr);
    void Clear_Obstacle_Source();
    bool Has_Obstacle_Points() const;
    bool Has_Obstacle_Source() const;
    std::size_t Obstacle_Point_Count() const;

    bool Set_Settings(const Robot_Collision_Settings &settings,
                      const std::vector<Matrix4> &reference_world_from_parts,
                      std::string *error_message = nullptr);
    const Robot_Collision_Settings &Settings() const { return m_settings; }
    const Robot_Collision_Point_Cloud_Stats &Point_Cloud_Stats() const
    {
      return m_point_cloud_stats;
    }
    const Robot_Joint_State &Reference_Joint_State() const
    {
      return m_reference_joint_state;
    }

    bool Create_Points_Rebuild_Request(std::shared_ptr<const std::vector<float>> xyz,
                                       const Robot_Joint_State &reference_joint_state,
                                       const std::vector<Matrix4> &reference_world_from_parts,
                                       Collision_Index_Build_Request *request,
                                       std::string *error_message = nullptr) const;
    bool Create_Settings_Rebuild_Request(const Robot_Collision_Settings &settings,
                                         const std::vector<Matrix4> &reference_world_from_parts,
                                         Collision_Index_Build_Request *request,
                                         std::string *error_message = nullptr) const;
    void Apply_Rebuild_Result(Collision_Index_Build_Result result);

    bool Has_Robot_Geometry() const;
    Robot_Motion_Collision_Result Evaluate_Motion(const Robot_Forward_Kinematics_Model &model,
                                                  const Robot_Motion_Collision_Request &request);
    const Robot_Collision_Result &Check_Pose(const std::vector<Matrix4> &world_from_parts);
    void Set_Current_Collision(const Robot_Collision_Result &collision)
    {
      m_current_collision = collision;
    }
    void Clear_Current_Collision() { m_current_collision = {}; }
    const Robot_Collision_Result &Current_Collision() const
    {
      return m_current_collision;
    }

  private:
    static bool Validate_Settings(const Robot_Collision_Settings &settings, std::string *error_message);
    bool Rebuild_Obstacle_Points(const std::vector<Matrix4> &reference_world_from_parts, std::string *error_message);

  private:
    Robot_Collision_Detector m_detector;
    Robot_Motion_Collision_Guard m_motion_guard;
    Robot_Collision_Settings m_settings;
    Robot_Collision_Point_Cloud_Stats m_point_cloud_stats;
    std::shared_ptr<const std::vector<float>> m_source_xyz;
    Robot_Joint_State m_reference_joint_state;
    std::vector<Robot_Visual_Part> m_robot_parts;
    Robot_Collision_Result m_current_collision;
    bool m_enabled = true;
  };

} // namespace robot_model

#endif

#include "robot_collision_service.h"

#include <cmath>
#include <utility>

namespace robot_model
{

  void Robot_Collision_Service::Clear_Detector()
  {
    m_detector.Clear();
    m_robot_parts.clear();
    m_current_collision = {};
  }

  void Robot_Collision_Service::Set_Robot_Parts(const std::vector<Robot_Visual_Part> &parts)
  {
    m_robot_parts = parts;
    m_detector.Set_Robot_Parts(parts);
    m_current_collision = {};
  }

  void Robot_Collision_Service::Set_Scene_Options(const Robot_Scene_Collision_Options &options)
  {
    m_detector.Set_Scene_Collision_Options(options);
  }

  const Robot_Scene_Collision_Options &
  Robot_Collision_Service::Scene_Options() const
  {
    return m_detector.Scene_Collision_Options();
  }

  void Robot_Collision_Service::Set_Enabled(bool enabled)
  {
    if (m_enabled == enabled)
      return;
    m_enabled = enabled;
    m_current_collision = {};
    if (!enabled)
    {
      // Preserve the raw source for a later rebuild, but release the query
      // index so disabled means no retained active obstacle collision data.
      m_detector.Clear_Obstacle_Points();
      m_point_cloud_stats = {};
    }
  }

  bool Robot_Collision_Service::Set_Obstacle_Source(std::shared_ptr<const std::vector<float>> xyz,
                                                    const Robot_Joint_State &reference_joint_state,
                                                    const std::vector<Matrix4> &reference_world_from_parts,
                                                    std::string *error_message)
  {
    m_source_xyz = xyz ? std::move(xyz)
                       : std::make_shared<const std::vector<float>>();
    m_reference_joint_state = reference_joint_state;
    if (!m_enabled)
    {
      m_detector.Clear_Obstacle_Points();
      m_point_cloud_stats = {};
      return true;
    }
    return Rebuild_Obstacle_Points(reference_world_from_parts, error_message);
  }

  bool Robot_Collision_Service::Rebuild_Obstacle_Source(const Robot_Joint_State &reference_joint_state,
                                                        const std::vector<Matrix4> &reference_world_from_parts,
                                                        std::string *error_message)
  {
    m_reference_joint_state = reference_joint_state;
    if (!m_enabled)
      return true;
    return Rebuild_Obstacle_Points(reference_world_from_parts, error_message);
  }

  void Robot_Collision_Service::Clear_Obstacle_Source()
  {
    m_detector.Clear_Obstacle_Points();
    m_source_xyz.reset();
    m_point_cloud_stats = {};
    m_current_collision = {};
  }

  bool Robot_Collision_Service::Has_Obstacle_Points() const
  {
    return m_detector.Has_Obstacle_Points();
  }

  bool Robot_Collision_Service::Has_Obstacle_Source() const
  {
    return m_source_xyz && !m_source_xyz->empty();
  }

  std::size_t Robot_Collision_Service::Obstacle_Point_Count() const
  {
    return m_detector.Obstacle_Point_Count();
  }

  bool Robot_Collision_Service::Set_Settings(const Robot_Collision_Settings &settings,
                                             const std::vector<Matrix4> &reference_world_from_parts,
                                             std::string *error_message)
  {
    if (!Validate_Settings(settings, error_message))
      return false;
    const auto previous = m_settings;
    m_settings = settings;
    if (!m_enabled)
      return true;
    if (!Rebuild_Obstacle_Points(reference_world_from_parts, error_message))
    {
      m_settings = previous;
      Rebuild_Obstacle_Points(reference_world_from_parts, nullptr);
      return false;
    }
    return true;
  }

  bool Robot_Collision_Service::Create_Points_Rebuild_Request(std::shared_ptr<const std::vector<float>> xyz,
                                                              const Robot_Joint_State &reference_joint_state,
                                                              const std::vector<Matrix4> &reference_world_from_parts,
                                                              Collision_Index_Build_Request *request,
                                                              std::string *error_message) const
  {
    if (error_message)
      error_message->clear();
    if (!request || !xyz || xyz->empty() || xyz->size() % 3 != 0)
    {
      if (error_message)
        *error_message = "Obstacle point array is empty or malformed";
      return false;
    }
    if (!Validate_Settings(m_settings, error_message))
      return false;

    request->source_xyz = std::move(xyz);
    request->settings = m_settings;
    request->reference_joint_state = reference_joint_state;
    request->robot_parts = m_robot_parts;
    request->reference_world_from_parts = reference_world_from_parts;
    return true;
  }

  bool Robot_Collision_Service::Create_Settings_Rebuild_Request(const Robot_Collision_Settings &settings,
                                                                const std::vector<Matrix4> &reference_world_from_parts,
                                                                Collision_Index_Build_Request *request,
                                                                std::string *error_message) const
  {
    if (error_message)
      error_message->clear();
    if (!request || !Validate_Settings(settings, error_message))
      return false;

    request->source_xyz = m_source_xyz;
    request->settings = settings;
    request->reference_joint_state = m_reference_joint_state;
    request->robot_parts = m_robot_parts;
    request->reference_world_from_parts = reference_world_from_parts;
    return true;
  }

  void Robot_Collision_Service::Apply_Rebuild_Result(Collision_Index_Build_Result result)
  {
    if (!m_enabled || !result.success)
      return;
    m_detector.Set_Obstacle_Snapshot(std::move(result.obstacle_snapshot));
    m_source_xyz = std::move(result.source_xyz);
    m_settings = result.settings;
    m_reference_joint_state = result.reference_joint_state;
    m_point_cloud_stats = result.stats;
  }

  bool Robot_Collision_Service::Has_Robot_Geometry() const
  {
    return m_detector.Has_Robot_Geometry();
  }

  Robot_Motion_Collision_Result Robot_Collision_Service::Evaluate_Motion(const Robot_Forward_Kinematics_Model &model,
                                                                         const Robot_Motion_Collision_Request &request)
  {
    return m_motion_guard.Evaluate(model, m_robot_parts, m_detector, request);
  }

  const Robot_Collision_Result &Robot_Collision_Service::Check_Pose(const std::vector<Matrix4> &world_from_parts)
  {
    m_current_collision = {};
    if (m_enabled && m_detector.Has_Robot_Geometry())
    {
      m_current_collision = m_detector.Check_Pose(world_from_parts, m_settings.clearance_mm);
    }
    return m_current_collision;
  }

  bool Robot_Collision_Service::Validate_Settings(const Robot_Collision_Settings &settings, std::string *error_message)
  {
    if (!std::isfinite(settings.clearance_mm) || settings.clearance_mm < 0.0)
    {
      if (error_message)
        *error_message = "Collision clearance is invalid";
      return false;
    }
    if (!std::isfinite(settings.point_cloud.voxel_size_mm) ||
        settings.point_cloud.voxel_size_mm <= 0.0 ||
        !std::isfinite(settings.point_cloud.robot_exclusion_distance_mm) ||
        settings.point_cloud.robot_exclusion_distance_mm < 0.0)
    {
      if (error_message)
        *error_message = "Collision point-cloud options are invalid";
      return false;
    }
    return true;
  }

  bool Robot_Collision_Service::Rebuild_Obstacle_Points(const std::vector<Matrix4> &reference_world_from_parts,
                                                        std::string *error_message)
  {
    if (error_message)
      error_message->clear();
    if (!m_source_xyz || m_source_xyz->empty())
    {
      m_detector.Clear_Obstacle_Points();
      m_point_cloud_stats = {};
      return true;
    }
    return m_detector.Set_Obstacle_Points(*m_source_xyz,
                                          m_settings.point_cloud,
                                          reference_world_from_parts,
                                          &m_point_cloud_stats,
                                          error_message);
  }

} // namespace robot_model

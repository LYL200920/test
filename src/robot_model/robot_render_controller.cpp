#include "robot_render_controller.h"

#include "robot_calibration_builder.h"
#include "robot_kinematic_params.h"
#include "robot_model_config_loader.h"
#include "vtk_scene.h"

#include <algorithm>
#include <cmath>

namespace robot_model
{

void Robot_Render_Controller::Attach_Scene (Vtk_Scene* scene)
{
  m_scene = scene;
  if( m_state.Robot_Model ( ).Has_Current_Model ( ) )
  {
    Rebuild_Current_Model ( );
  }
}

void Robot_Render_Controller::Detach_Scene ( )
{
  m_assembly.Clear ( );
  m_forward_model = { };
  m_has_forward_model = false;
  m_scene = nullptr;
}

void Robot_Render_Controller::Load_Model (const Robot_Model_Info& model)
{
  const auto params = Load_Robot_Kinematic_Params (
    model.xml_path, model.display_name);
  m_state.Robot_Model ( ).Set_Current_Model (model, params);
  Rebuild_Current_Model ( );
}

void Robot_Render_Controller::Set_Joint_State (
  const Robot_Joint_State& joint_state)
{
  m_state.Robot_Model ( ).Set_Joint_State (joint_state);
  Apply_Joint_Pose ( );
}

bool Robot_Render_Controller::Has_Current_Model ( ) const
{
  return m_state.Robot_Model ( ).Has_Current_Model ( );
}

const Robot_Kinematic_Params& Robot_Render_Controller::Kinematic_Params ( ) const
{
  return m_state.Robot_Model ( ).Params ( );
}

const Robot_Joint_State& Robot_Render_Controller::Joint_State ( ) const
{
  return m_state.Robot_Model ( ).Joint_State ( );
}

bool Robot_Render_Controller::Has_Flange_Pose ( ) const
{
  return m_assembly.Has_Flange_Pose ( );
}

const Matrix4& Robot_Render_Controller::World_From_Flange ( ) const
{
  return m_assembly.World_From_Flange ( );
}

Robot_Position_IK_Result Robot_Render_Controller::Move_Flange_To (
  const Point3& target_world)
{
  Robot_Position_IK_Result result;
  auto& model_state = m_state.Robot_Model ( );
  if( !m_has_forward_model || !model_state.Has_Current_Model ( ) )
  {
    return result;
  }

  result = Solve_Flange_Position_IK (
    m_forward_model,
    model_state.Params ( ),
    model_state.Joint_State ( ),
    target_world);
  if( result.status != Robot_IK_Status::Invalid_Model &&
      std::isfinite (result.position_error_mm) )
  {
    model_state.Set_Joint_State (result.joint_state);
    Apply_Joint_Pose ( );
  }
  return result;
}

Robot_Pose_IK_Result Robot_Render_Controller::Move_Flange_To_Pose (
  const Matrix4& target_world_from_flange)
{
  Robot_Pose_IK_Result result;
  auto& model_state = m_state.Robot_Model ( );
  if( !m_has_forward_model || !model_state.Has_Current_Model ( ) )
  {
    return result;
  }
  result = Solve_Flange_Pose_IK (
    m_forward_model,
    model_state.Params ( ),
    model_state.Joint_State ( ),
    target_world_from_flange);
  if( result.status != Robot_IK_Status::Invalid_Model &&
      std::isfinite (result.position_error_mm) &&
      std::isfinite (result.orientation_error_deg) )
  {
    model_state.Set_Joint_State (result.joint_state);
    Apply_Joint_Pose ( );
  }
  return result;
}

void Robot_Render_Controller::Rebuild_Current_Model ( )
{
  auto& robot_model_state = m_state.Robot_Model ( );
  if( m_scene == nullptr || !m_scene->Is_Ready ( ) ||
      !robot_model_state.Has_Current_Model ( ) )
  {
    return;
  }

  const auto& model = robot_model_state.Current_Model ( );
  m_scene->Remove_All_ViewProps ( );
  m_scene->Remove_All_Lights ( );
  m_assembly.Clear ( );
  m_forward_model = { };
  m_has_forward_model = false;
  robot_model_state.Clear_Assembly_Calibration ( );

  if( model.Has_Mesh ( ) )
  {
    m_assembly.Load_Mesh (model, m_scene->Renderer ( ));
    const auto assembly_calibration = Build_Assembly_Calibration (
      robot_model_state.Params ( ),
      robot_model_state.Model_Name ( ),
      m_assembly.Parts ( ));
    robot_model_state.Set_Assembly_Calibration (assembly_calibration);
    m_forward_model = Build_Forward_Kinematics_Model (
      m_assembly.Parts ( ).size ( ),
      robot_model_state.Assembly_Calibration ( ),
      robot_model_state.Params ( ));
    m_has_forward_model = m_forward_model.has_flange;
    Apply_Joint_Pose ( );
  }
  else
  {
    m_assembly.Build_Fallback_Demo (robot_model_state.Params ( ),
                                    m_scene->Renderer ( ));
  }

  const double link_extent = robot_model_state.Params ( ).Has_Link_Lengths ( )
    ? link_length_at (robot_model_state.Params ( ), 1, 1000.0) +
      link_length_at (robot_model_state.Params ( ), 3, 1000.0)
    : 2000.0;
  const double extent = std::max (5000.0, link_extent * 2.2);
  m_scene->Add_Ground_Grid (extent);
  m_scene->Reset_Camera ( );
}

void Robot_Render_Controller::Apply_Joint_Pose ( )
{
  const auto& robot_model_state = m_state.Robot_Model ( );
  if( !robot_model_state.Has_Assembly_Calibration ( ) )
  {
    return;
  }

  if( m_has_forward_model )
  {
    m_assembly.Apply_Forward_Kinematics (Compute_Forward_Kinematics (
      m_forward_model, robot_model_state.Joint_State ( )));
  }
}

} // namespace robot_model

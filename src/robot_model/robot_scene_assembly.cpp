#include "robot_scene_assembly.h"

#include "robot_mesh_loader.h"
#include "robot_kinematic_params.h"
#include "robot_collision_detector.h"

#include <vtkCubeSource.h>
#include <vtkLineSource.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>

#include <algorithm>

namespace robot_model
{

  void Robot_Scene_Assembly::Load_Mesh(const Robot_Model_Info &model,
                                       vtkRenderer *renderer)
  {
    Clear();
    m_renderer = renderer;

    for (size_t i = 0; i < model.stl_files.size(); ++i)
    {
      auto part = Create_STL_Part(model.stl_files[i], static_cast<int>(i));
      if (renderer && part.actor)
      {
        renderer->AddActor(part.actor);
      }
      m_parts.push_back(std::move(part));
    }
  }

  void Robot_Scene_Assembly::Build_Fallback_Demo(const Robot_Kinematic_Params &params,
                                                 vtkRenderer *renderer)
  {
    Clear();
    m_renderer = renderer;
    auto effective_params = params;
    if (effective_params.link_lengths.empty())
    {
      effective_params.link_lengths = {400.0, 560.0, 25.0, 515.0, 90.0, 25.0};
    }

    const double orange[3] = {0.94, 0.48, 0.10};
    const double dark[3] = {0.17, 0.18, 0.20};
    const double metal[3] = {0.72, 0.74, 0.76};

    Add_Joint(0.0, 0.0, 0.0, 115.0, renderer);
    Add_Link(220.0, 220.0, 140.0, 0.0, 0.0, 70.0, dark, renderer);

    double x = 0.0;
    double z = 140.0;

    const auto l1 = link_length_at(effective_params, 0, 400.0);
    Add_Link(130.0, 130.0, l1, x, 0.0, z + l1 * 0.5, orange, renderer);
    z += l1;
    Add_Joint(x, 0.0, z, 80.0, renderer);

    const auto l2 = link_length_at(effective_params, 1, 560.0);
    Add_Link(l2, 120.0, 120.0, x + l2 * 0.5, 0.0, z, orange, renderer);
    x += l2;
    Add_Joint(x, 0.0, z, 72.0, renderer);

    const auto l3 = link_length_at(effective_params, 3, 515.0);
    Add_Link(120.0, 120.0, l3, x, 0.0, z + l3 * 0.5, orange, renderer);
    z += l3;
    Add_Joint(x, 0.0, z, 64.0, renderer);

    const auto wrist = std::max(
        link_length_at(effective_params, 4, 90.0), 120.0);
    Add_Link(wrist + 160.0, 95.0, 95.0,
             x + (wrist + 160.0) * 0.5, 0.0, z, metal, renderer);
    x += wrist + 160.0;
    Add_Joint(x, 0.0, z, 54.0, renderer);
  }

  void Robot_Scene_Assembly::Apply_Calibration(const Robot_Assembly_Calibration &calibration)
  {
    Robot_Joint_State zero_state;
    Robot_Kinematic_Params params;
    Apply_Joint_State(calibration, params, zero_state);
  }

  void Robot_Scene_Assembly::Apply_Joint_State(const Robot_Assembly_Calibration &calibration,
                                               const Robot_Kinematic_Params &params,
                                               const Robot_Joint_State &joint_state)
  {
    const auto model = Build_Forward_Kinematics_Model(m_parts.size(), calibration, params);
    Apply_Forward_Kinematics(Compute_Forward_Kinematics(model, joint_state));
  }

  void Robot_Scene_Assembly::Apply_Forward_Kinematics(const Robot_Forward_Kinematics_Result &transforms)
  {
    m_world_from_flange = transforms.world_from_flange;
    m_has_flange_pose = transforms.has_flange;
    const auto count = std::min(m_parts.size(), transforms.world_from_parts.size());
    for (size_t i = 0; i < count; ++i)
    {
      auto &part = m_parts[i];
      if (!part.local_transform)
      {
        continue;
      }

      auto matrix = vtkSmartPointer<vtkMatrix4x4>::New();
      for (int row = 0; row < 4; ++row)
      {
        for (int column = 0; column < 4; ++column)
        {
          matrix->SetElement(row, column, transforms.world_from_parts[i][row][column]);
        }
      }
      part.local_transform->SetMatrix(matrix);

      if (part.actor)
      {
        part.actor->Modified();
      }
    }
  }

  bool Robot_Scene_Assembly::Show_Collision(const Robot_Collision_Result &collision)
  {
    if (!collision.collided || collision.robot_part_index >= m_parts.size() ||
        !m_renderer)
    {
      return false;
    }

    const bool has_other_part = collision.type == Robot_Collision_Type::Self_Collision &&
                                collision.other_robot_part_index < m_parts.size();
    const int collision_type = static_cast<int>(collision.type);
    if (m_collision_visible &&
        m_collision_part_index == collision.robot_part_index &&
        m_collision_type == collision_type &&
        m_has_other_collision_part == has_other_part &&
        (!has_other_part || m_other_collision_part_index == collision.other_robot_part_index))
    {
      return false;
    }

    Clear_Collision();
    m_collision_part_index = collision.robot_part_index;
    m_collision_type = collision_type;
    m_has_other_collision_part = has_other_part;
    m_other_collision_part_index = collision.other_robot_part_index;
    auto &collision_part = m_parts[m_collision_part_index];
    if (collision_part.actor)
    {
      collision_part.actor->GetProperty()->SetColor(1.0, 0.08, 0.04);
    }
    if (m_has_other_collision_part)
    {
      auto &other_part = m_parts[m_other_collision_part_index];
      if (other_part.actor)
      {
        other_part.actor->GetProperty()->SetColor(1.0, 0.08, 0.04);
      }
    }

    if (!m_robot_contact_source)
    {
      m_robot_contact_source = vtkSmartPointer<vtkSphereSource>::New();
      m_robot_contact_source->SetRadius(10.0);
      m_robot_contact_source->SetThetaResolution(20);
      m_robot_contact_source->SetPhiResolution(12);
      auto robot_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      robot_mapper->SetInputConnection(m_robot_contact_source->GetOutputPort());
      m_robot_contact_actor = vtkSmartPointer<vtkActor>::New();
      m_robot_contact_actor->SetMapper(robot_mapper);
      m_robot_contact_actor->GetProperty()->SetColor(1.0, 0.1, 0.05);

      m_obstacle_contact_source = vtkSmartPointer<vtkSphereSource>::New();
      m_obstacle_contact_source->SetRadius(10.0);
      m_obstacle_contact_source->SetThetaResolution(20);
      m_obstacle_contact_source->SetPhiResolution(12);
      auto obstacle_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      obstacle_mapper->SetInputConnection(
          m_obstacle_contact_source->GetOutputPort());
      m_obstacle_contact_actor = vtkSmartPointer<vtkActor>::New();
      m_obstacle_contact_actor->SetMapper(obstacle_mapper);
      m_obstacle_contact_actor->GetProperty()->SetColor(1.0, 0.9, 0.05);

      m_contact_line_source = vtkSmartPointer<vtkLineSource>::New();
      auto line_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      line_mapper->SetInputConnection(m_contact_line_source->GetOutputPort());
      m_contact_line_actor = vtkSmartPointer<vtkActor>::New();
      m_contact_line_actor->SetMapper(line_mapper);
      m_contact_line_actor->GetProperty()->SetColor(1.0, 0.9, 0.05);
      m_contact_line_actor->GetProperty()->SetLineWidth(3.0);

      m_renderer->AddActor(m_robot_contact_actor);
      m_renderer->AddActor(m_obstacle_contact_actor);
      m_renderer->AddActor(m_contact_line_actor);
    }

    m_robot_contact_source->SetCenter(collision.robot_closest_point_world.data());
    m_obstacle_contact_source->SetCenter(collision.obstacle_point_world.data());
    m_contact_line_source->SetPoint1(collision.robot_closest_point_world.data());
    m_contact_line_source->SetPoint2(collision.obstacle_point_world.data());
    m_robot_contact_actor->SetVisibility(true);
    m_obstacle_contact_actor->SetVisibility(true);
    m_contact_line_actor->SetVisibility(true);
    m_collision_visible = true;
    return true;
  }

  bool Robot_Scene_Assembly::Clear_Collision()
  {
    if (!m_collision_visible)
      return false;

    if (m_collision_part_index < m_parts.size())
    {
      auto &part = m_parts[m_collision_part_index];
      if (part.actor)
      {
        part.actor->GetProperty()->SetColor(part.base_color.data());
      }
    }
    if (m_has_other_collision_part && m_other_collision_part_index < m_parts.size())
    {
      auto &part = m_parts[m_other_collision_part_index];
      if (part.actor)
      {
        part.actor->GetProperty()->SetColor(part.base_color.data());
      }
    }
    if (m_robot_contact_actor)
      m_robot_contact_actor->SetVisibility(false);
    if (m_obstacle_contact_actor)
    {
      m_obstacle_contact_actor->SetVisibility(false);
    }
    if (m_contact_line_actor)
      m_contact_line_actor->SetVisibility(false);
    m_collision_visible = false;
    m_has_other_collision_part = false;
    m_collision_type = 0;
    return true;
  }

  void Robot_Scene_Assembly::Clear()
  {
    Clear_Collision();
    if (m_renderer)
    {
      if (m_robot_contact_actor)
        m_renderer->RemoveActor(m_robot_contact_actor);
      if (m_obstacle_contact_actor)
      {
        m_renderer->RemoveActor(m_obstacle_contact_actor);
      }
      if (m_contact_line_actor)
        m_renderer->RemoveActor(m_contact_line_actor);
    }
    m_parts.clear();
    m_world_from_flange = {};
    m_has_flange_pose = false;
    m_renderer = nullptr;
    m_robot_contact_source = nullptr;
    m_obstacle_contact_source = nullptr;
    m_contact_line_source = nullptr;
    m_robot_contact_actor = nullptr;
    m_obstacle_contact_actor = nullptr;
    m_contact_line_actor = nullptr;
  }

  void Robot_Scene_Assembly::Add_Link(double x_length, double y_length,
                                      double z_length,
                                      double x, double y, double z,
                                      const double color[3],
                                      vtkRenderer *renderer)
  {
    auto source = vtkSmartPointer<vtkCubeSource>::New();
    source->SetXLength(x_length);
    source->SetYLength(y_length);
    source->SetZLength(z_length);
    source->SetCenter(x, y, z);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(source->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetAmbient(0.28);
    actor->GetProperty()->SetDiffuse(0.78);
    actor->GetProperty()->SetSpecular(0.25);
    actor->GetProperty()->SetSpecularPower(24.0);
    renderer->AddActor(actor);
  }

  void Robot_Scene_Assembly::Add_Joint(double x, double y, double z,
                                       double radius,
                                       vtkRenderer *renderer)
  {
    auto source = vtkSmartPointer<vtkSphereSource>::New();
    source->SetCenter(x, y, z);
    source->SetRadius(radius);
    source->SetThetaResolution(32);
    source->SetPhiResolution(16);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(source->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.12, 0.12, 0.13);
    actor->GetProperty()->SetAmbient(0.35);
    actor->GetProperty()->SetDiffuse(0.72);
    actor->GetProperty()->SetSpecular(0.35);
    actor->GetProperty()->SetSpecularPower(32.0);
    renderer->AddActor(actor);
  }

} // namespace robot_model

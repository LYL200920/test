#include "point_cloud_renderer.h"

#include <vtkPointData.h>
#include <vtkMatrix4x4.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>

#include <cmath>

namespace point_cloud
{
  namespace
  {

    constexpr double kDefaultPointSize = 2.0;

    bool point_is_valid(const double point[3])
    {
      return std::isfinite(point[0]) &&
             std::isfinite(point[1]) &&
             std::isfinite(point[2]) &&
             !(point[0] == 0.0 && point[1] == 0.0 && point[2] == 0.0);
    }

  } // namespace

  void Point_Cloud_Renderer::Attach_Renderer(vtkRenderer *renderer)
  {
    if (m_renderer && m_actor)
    {
      m_renderer->RemoveActor(m_actor);
    }

    m_renderer = renderer;
    Add_Actor_To_Renderer();
  }

  void Point_Cloud_Renderer::Detach_Renderer()
  {
    if (m_renderer && m_actor)
    {
      m_renderer->RemoveActor(m_actor);
    }
    m_renderer = nullptr;
  }

  bool Point_Cloud_Renderer::Load_Ply(
      const std::filesystem::path &path,
      std::string *error_message)
  {
    if (error_message)
    {
      error_message->clear();
    }
    Clear();

    if (path.empty() || !std::filesystem::exists(path))
    {
      if (error_message)
      {
        *error_message = "PLY file does not exist";
      }
      return false;
    }

    m_reader = vtkSmartPointer<vtkPLYReader>::New();
    m_reader->SetFileName(path.string().c_str());
    m_reader->Update();

    auto *output = m_reader->GetOutput();
    if (output == nullptr || output->GetNumberOfPoints() <= 0)
    {
      Clear();
      if (error_message)
      {
        *error_message = "PLY file has no points";
      }
      return false;
    }

    m_display_data = Build_Display_Data(output);
    if (m_display_data == nullptr || m_display_data->GetNumberOfPoints() <= 0)
    {
      Clear();
      if (error_message)
      {
        *error_message = "PLY file has no valid points";
      }
      return false;
    }

    m_point_count = static_cast<std::size_t>(
        m_display_data->GetNumberOfPoints());
    Build_Actor();
    return true;
  }

  bool Point_Cloud_Renderer::Set_Point_Data(
      const std::vector<float> &xyz,
      const std::vector<std::uint8_t> &rgb,
      std::string *error_message)
  {
    if (error_message)
    {
      error_message->clear();
    }
    if (xyz.empty() || (xyz.size() % 3) != 0)
    {
      if (error_message)
      {
        *error_message = "Point coordinate array is empty or malformed";
      }
      return false;
    }

    const std::size_t input_count = xyz.size() / 3;
    if (!rgb.empty() && rgb.size() != input_count * 3)
    {
      if (error_message)
      {
        *error_message = "Point color array size does not match coordinates";
      }
      return false;
    }

    auto points = vtkSmartPointer<vtkPoints>::New();
    points->SetDataTypeToFloat();
    points->Allocate(static_cast<vtkIdType>(input_count));
    vtkSmartPointer<vtkUnsignedCharArray> colors;
    if (!rgb.empty())
    {
      colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
      colors->SetName("RGB");
      colors->SetNumberOfComponents(3);
      colors->Allocate(static_cast<vtkIdType>(input_count * 3));
    }

    for (std::size_t i = 0; i < input_count; ++i)
    {
      const double point[3] = {
        static_cast<double>(xyz[i * 3]),
        static_cast<double>(xyz[i * 3 + 1]),
        static_cast<double>(xyz[i * 3 + 2])
      };
      if (!point_is_valid(point))
      {
        continue;
      }
      points->InsertNextPoint(point);
      if (colors)
      {
        colors->InsertNextTuple3(
          rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]);
      }
    }
    if (points->GetNumberOfPoints() <= 0)
    {
      if (error_message)
      {
        *error_message = "Point coordinate array has no valid points";
      }
      return false;
    }

    Clear();
    m_display_data = vtkSmartPointer<vtkPolyData>::New();
    m_display_data->SetPoints(points);
    if (colors)
    {
      m_display_data->GetPointData()->SetScalars(colors);
    }
    m_point_count = static_cast<std::size_t>(points->GetNumberOfPoints());
    Build_Actor();
    return true;
  }

  void Point_Cloud_Renderer::Set_World_From_Point_Cloud(
      const robot_model::Matrix4 &world_from_point_cloud)
  {
    if (!m_world_from_point_cloud)
    {
      m_world_from_point_cloud = vtkSmartPointer<vtkMatrix4x4>::New();
    }
    for (int row = 0; row < 4; ++row)
    {
      for (int column = 0; column < 4; ++column)
      {
        m_world_from_point_cloud->SetElement(
          row, column, world_from_point_cloud[row][column]);
      }
    }
    m_world_from_point_cloud->Modified();
    if (m_actor)
    {
      m_actor->SetUserMatrix(m_world_from_point_cloud);
      m_actor->Modified();
    }
  }

  void Point_Cloud_Renderer::Clear()
  {
    if (m_renderer && m_actor)
    {
      m_renderer->RemoveActor(m_actor);
    }

    m_actor = nullptr;
    m_mapper = nullptr;
    m_glyph_filter = nullptr;
    m_display_data = nullptr;
    m_reader = nullptr;
    m_point_count = 0;
  }

  bool Point_Cloud_Renderer::Get_World_Bounds(double bounds[6]) const
  {
    if (!m_actor || bounds == nullptr)
    {
      return false;
    }
    m_actor->GetBounds(bounds);
    return true;
  }

  vtkSmartPointer<vtkPolyData> Point_Cloud_Renderer::Build_Display_Data(
      vtkPolyData *input) const
  {
    if (input == nullptr)
    {
      return nullptr;
    }

    auto *input_scalars = input->GetPointData()
                              ? input->GetPointData()->GetScalars()
                              : nullptr;
    vtkSmartPointer<vtkDataArray> display_scalars;
    if (input_scalars)
    {
      display_scalars.TakeReference(input_scalars->NewInstance());
      display_scalars->SetName(input_scalars->GetName());
      display_scalars->SetNumberOfComponents(
          input_scalars->GetNumberOfComponents());
    }

    auto points = vtkSmartPointer<vtkPoints>::New();
    const vtkIdType point_count = input->GetNumberOfPoints();
    points->Allocate(point_count);
    if (display_scalars)
    {
      display_scalars->Allocate(point_count);
    }

    double point[3] = {};
    for (vtkIdType index = 0; index < point_count; ++index)
    {
      input->GetPoint(index, point);
      if (!point_is_valid(point))
      {
        continue;
      }

      points->InsertNextPoint(point);
      if (display_scalars)
      {
        display_scalars->InsertNextTuple(index, input_scalars);
      }
    }

    auto output = vtkSmartPointer<vtkPolyData>::New();
    output->SetPoints(points);
    if (display_scalars && display_scalars->GetNumberOfTuples() > 0)
    {
      output->GetPointData()->SetScalars(display_scalars);
    }
    return output;
  }

  void Point_Cloud_Renderer::Add_Actor_To_Renderer()
  {
    if (m_renderer && m_actor)
    {
      m_renderer->AddActor(m_actor);
    }
  }

  void Point_Cloud_Renderer::Build_Actor()
  {
    if (!m_display_data || m_display_data->GetNumberOfPoints() <= 0)
    {
      return;
    }

    m_glyph_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    m_glyph_filter->SetInputData(m_display_data);
    m_glyph_filter->Update();

    m_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_mapper->SetInputConnection(m_glyph_filter->GetOutputPort());
    m_mapper->SetColorModeToDirectScalars();
    m_mapper->SetScalarModeToUsePointData();
    m_mapper->SetScalarVisibility(
      m_display_data->GetPointData()->GetScalars() != nullptr);

    m_actor = vtkSmartPointer<vtkActor>::New();
    m_actor->SetMapper(m_mapper);
    if (m_world_from_point_cloud)
    {
      m_actor->SetUserMatrix(m_world_from_point_cloud);
    }
    m_actor->GetProperty()->SetPointSize(kDefaultPointSize);
    m_actor->GetProperty()->SetColor(0.10, 0.70, 0.95);
    m_actor->GetProperty()->SetAmbient(0.65);
    m_actor->GetProperty()->SetDiffuse(0.35);
    Add_Actor_To_Renderer();
  }

} // namespace point_cloud

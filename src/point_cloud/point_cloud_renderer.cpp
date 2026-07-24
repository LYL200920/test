#include "point_cloud_renderer.h"

#include <vtkPointData.h>
#include <vtkIdTypeArray.h>
#include <vtkMatrix4x4.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>

#include <algorithm>
#include <cmath>

namespace point_cloud
{
  namespace
  {

    constexpr double kDefaultPointSize = 1.0;
    constexpr double kDefaultOpacity = 0.35;
    constexpr double kSelectionPointSize = 5.0;
    constexpr vtkIdType kMaximumInteractivePointCount = 100000;
    constexpr const char *kOriginalPointIdsName = "OriginalPointIds";

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
    if (m_renderer && m_selection_actor)
    {
      m_renderer->RemoveActor(m_selection_actor);
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

  bool Point_Cloud_Renderer::Load_Ply(const std::filesystem::path &path, std::string *error_message)
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

    m_point_count = static_cast<std::size_t>(m_display_data->GetNumberOfPoints());
    Build_Actor();
    return true;
  }

  bool Point_Cloud_Renderer::Set_Point_Data(const std::vector<float> &xyz,
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
    auto original_point_ids = vtkSmartPointer<vtkIdTypeArray>::New();
    original_point_ids->SetName(kOriginalPointIdsName);
    original_point_ids->SetNumberOfComponents(1);
    original_point_ids->Allocate(static_cast<vtkIdType>(input_count));
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
          static_cast<double>(xyz[i * 3 + 2])};
      if (!point_is_valid(point))
      {
        continue;
      }
      points->InsertNextPoint(point);
      original_point_ids->InsertNextValue(static_cast<vtkIdType>(i));
      if (colors)
      {
        colors->InsertNextTuple3(rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]);
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
    m_display_data->GetPointData()->AddArray(original_point_ids);
    if (colors)
    {
      m_display_data->GetPointData()->SetScalars(colors);
    }
    m_point_count = static_cast<std::size_t>(points->GetNumberOfPoints());
    Build_Actor();
    return true;
  }

  std::vector<std::size_t>
  Point_Cloud_Renderer::Select_Point_Ids_In_Display_Rect(
    vtkRenderer *renderer,
    int minimum_x,
    int minimum_y,
    int maximum_x,
    int maximum_y) const
  {
    std::vector<std::size_t> selected_ids;
    if (!renderer || !m_display_data)
    {
      return selected_ids;
    }

    auto *original_point_ids = vtkIdTypeArray::SafeDownCast(
      m_display_data->GetPointData()->GetArray(kOriginalPointIdsName));
    if (!original_point_ids)
    {
      return selected_ids;
    }

    if (minimum_x > maximum_x)
    {
      std::swap(minimum_x, maximum_x);
    }
    if (minimum_y > maximum_y)
    {
      std::swap(minimum_y, maximum_y);
    }

    double local_point[3] = {};
    double local_homogeneous[4] = {};
    double world_point[4] = {};
    const vtkIdType point_count = m_display_data->GetNumberOfPoints();
    selected_ids.reserve(static_cast<std::size_t>(point_count / 8));
    for (vtkIdType point_index = 0; point_index < point_count; ++point_index)
    {
      m_display_data->GetPoint(point_index, local_point);
      local_homogeneous[0] = local_point[0];
      local_homogeneous[1] = local_point[1];
      local_homogeneous[2] = local_point[2];
      local_homogeneous[3] = 1.0;
      if (m_world_from_point_cloud)
      {
        m_world_from_point_cloud->MultiplyPoint(local_homogeneous, world_point);
      }
      else
      {
        std::copy(local_homogeneous, local_homogeneous + 4, world_point);
      }

      renderer->SetWorldPoint(world_point);
      renderer->WorldToDisplay();
      double *display_point = renderer->GetDisplayPoint();
      if (!display_point || display_point[2] < 0.0 || display_point[2] > 1.0)
      {
        continue;
      }
      if (display_point[0] >= minimum_x &&
          display_point[0] <= maximum_x &&
          display_point[1] >= minimum_y &&
          display_point[1] <= maximum_y)
      {
        selected_ids.push_back(static_cast<std::size_t>(
          original_point_ids->GetValue(point_index)));
      }
    }
    return selected_ids;
  }

  std::vector<std::size_t>
  Point_Cloud_Renderer::Select_Point_Ids_In_Display_Polygon(
    vtkRenderer *renderer,
    const std::vector<Display_Point> &polygon) const
  {
    std::vector<std::size_t> selected_ids;
    if (!renderer || !m_display_data || polygon.size() < 3)
    {
      return selected_ids;
    }

    auto *original_point_ids = vtkIdTypeArray::SafeDownCast(
      m_display_data->GetPointData()->GetArray(kOriginalPointIdsName));
    if (!original_point_ids)
    {
      return selected_ids;
    }

    double minimum_x = polygon.front().x;
    double maximum_x = polygon.front().x;
    double minimum_y = polygon.front().y;
    double maximum_y = polygon.front().y;
    for (const Display_Point &vertex : polygon)
    {
      minimum_x = std::min(minimum_x, vertex.x);
      maximum_x = std::max(maximum_x, vertex.x);
      minimum_y = std::min(minimum_y, vertex.y);
      maximum_y = std::max(maximum_y, vertex.y);
    }

    double local_point[3] = {};
    double local_homogeneous[4] = {};
    double world_point[4] = {};
    const vtkIdType point_count = m_display_data->GetNumberOfPoints();
    selected_ids.reserve(static_cast<std::size_t>(point_count / 8));
    for (vtkIdType point_index = 0; point_index < point_count; ++point_index)
    {
      m_display_data->GetPoint(point_index, local_point);
      local_homogeneous[0] = local_point[0];
      local_homogeneous[1] = local_point[1];
      local_homogeneous[2] = local_point[2];
      local_homogeneous[3] = 1.0;
      if (m_world_from_point_cloud)
      {
        m_world_from_point_cloud->MultiplyPoint(local_homogeneous, world_point);
      }
      else
      {
        std::copy(local_homogeneous, local_homogeneous + 4, world_point);
      }

      renderer->SetWorldPoint(world_point);
      renderer->WorldToDisplay();
      double *display_point = renderer->GetDisplayPoint();
      if (!display_point || display_point[2] < 0.0 || display_point[2] > 1.0)
      {
        continue;
      }
      if (display_point[0] < minimum_x ||
          display_point[0] > maximum_x ||
          display_point[1] < minimum_y ||
          display_point[1] > maximum_y)
      {
        continue;
      }
      if (Is_Point_In_Display_Polygon(
            {display_point[0], display_point[1]}, polygon))
      {
        selected_ids.push_back(static_cast<std::size_t>(
          original_point_ids->GetValue(point_index)));
      }
    }
    return selected_ids;
  }

  void Point_Cloud_Renderer::Set_Selected_Point_Ids(
    const std::vector<std::size_t> &point_ids)
  {
    if (m_renderer && m_selection_actor)
    {
      m_renderer->RemoveActor(m_selection_actor);
    }
    m_selection_actor = nullptr;
    m_selection_mapper = nullptr;
    m_selection_glyph_filter = nullptr;
    m_selection_data = nullptr;

    if (!m_display_data || point_ids.empty())
    {
      return;
    }

    auto *original_point_ids = vtkIdTypeArray::SafeDownCast(
      m_display_data->GetPointData()->GetArray(kOriginalPointIdsName));
    if (!original_point_ids)
    {
      return;
    }

    auto selected_points = vtkSmartPointer<vtkPoints>::New();
    selected_points->SetDataType(m_display_data->GetPoints()->GetDataType());
    selected_points->Allocate(static_cast<vtkIdType>(point_ids.size()));
    double point[3] = {};
    for (vtkIdType point_index = 0;
         point_index < m_display_data->GetNumberOfPoints();
         ++point_index)
    {
      const std::size_t original_id = static_cast<std::size_t>(
        original_point_ids->GetValue(point_index));
      if (!std::binary_search(point_ids.begin(), point_ids.end(), original_id))
      {
        continue;
      }
      m_display_data->GetPoint(point_index, point);
      selected_points->InsertNextPoint(point);
    }
    if (selected_points->GetNumberOfPoints() <= 0)
    {
      return;
    }

    m_selection_data = vtkSmartPointer<vtkPolyData>::New();
    m_selection_data->SetPoints(selected_points);
    m_selection_glyph_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    m_selection_glyph_filter->SetInputData(m_selection_data);
    m_selection_glyph_filter->Update();
    m_selection_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_selection_mapper->SetInputConnection(m_selection_glyph_filter->GetOutputPort());
    m_selection_mapper->ScalarVisibilityOff();
    m_selection_actor = vtkSmartPointer<vtkActor>::New();
    m_selection_actor->SetMapper(m_selection_mapper);
    if (m_world_from_point_cloud)
    {
      m_selection_actor->SetUserMatrix(m_world_from_point_cloud);
    }
    m_selection_actor->GetProperty()->SetPointSize(kSelectionPointSize);
    m_selection_actor->GetProperty()->SetOpacity(1.0);
    m_selection_actor->GetProperty()->SetColor(1.0, 0.75, 0.05);
    m_selection_actor->GetProperty()->SetLighting(false);
    if (m_renderer)
    {
      m_renderer->AddActor(m_selection_actor);
    }
  }

  void Point_Cloud_Renderer::Set_World_From_Point_Cloud(const robot_model::Matrix4 &world_from_point_cloud)
  {
    if (!m_world_from_point_cloud)
    {
      m_world_from_point_cloud = vtkSmartPointer<vtkMatrix4x4>::New();
    }
    for (int row = 0; row < 4; ++row)
    {
      for (int column = 0; column < 4; ++column)
      {
        m_world_from_point_cloud->SetElement(row, column, world_from_point_cloud[row][column]);
      }
    }
    m_world_from_point_cloud->Modified();
    if (m_actor)
    {
      m_actor->SetUserMatrix(m_world_from_point_cloud);
      m_actor->Modified();
    }
    if (m_selection_actor)
    {
      m_selection_actor->SetUserMatrix(m_world_from_point_cloud);
      m_selection_actor->Modified();
    }
  }

  void Point_Cloud_Renderer::Set_Interactive_LOD(bool enabled)
  {
    const bool use_interaction_data = enabled &&
                                      m_interaction_glyph_filter &&
                                      m_interaction_point_count < m_point_count;
    if (m_interactive_lod == use_interaction_data)
    {
      return;
    }
    m_interactive_lod = use_interaction_data;
    if (!m_mapper)
    {
      return;
    }
    m_mapper->SetInputConnection(m_interactive_lod ? m_interaction_glyph_filter->GetOutputPort()
                                                   : m_glyph_filter->GetOutputPort());
    m_mapper->Modified();
  }

  void Point_Cloud_Renderer::Clear()
  {
    if (m_renderer && m_actor)
    {
      m_renderer->RemoveActor(m_actor);
    }
    if (m_renderer && m_selection_actor)
    {
      m_renderer->RemoveActor(m_selection_actor);
    }
    if (m_renderer && m_selection_actor)
    {
      m_renderer->RemoveActor(m_selection_actor);
    }

    m_actor = nullptr;
    m_selection_actor = nullptr;
    m_selection_mapper = nullptr;
    m_selection_glyph_filter = nullptr;
    m_selection_data = nullptr;
    m_mapper = nullptr;
    m_glyph_filter = nullptr;
    m_interaction_glyph_filter = nullptr;
    m_interaction_display_data = nullptr;
    m_display_data = nullptr;
    m_reader = nullptr;
    m_point_count = 0;
    m_interaction_point_count = 0;
    m_interactive_lod = false;
  }

  std::size_t Point_Cloud_Renderer::Displayed_Point_Count() const
  {
    return m_interactive_lod ? m_interaction_point_count : m_point_count;
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

  vtkSmartPointer<vtkPolyData> Point_Cloud_Renderer::Build_Display_Data(vtkPolyData *input) const
  {
    if (input == nullptr)
    {
      return nullptr;
    }

    auto *input_scalars = input->GetPointData() ? input->GetPointData()->GetScalars()
                                                : nullptr;
    vtkSmartPointer<vtkDataArray> display_scalars;
    auto original_point_ids = vtkSmartPointer<vtkIdTypeArray>::New();
    original_point_ids->SetName(kOriginalPointIdsName);
    original_point_ids->SetNumberOfComponents(1);
    original_point_ids->Allocate(input->GetNumberOfPoints());
    if (input_scalars)
    {
      display_scalars.TakeReference(input_scalars->NewInstance());
      display_scalars->SetName(input_scalars->GetName());
      display_scalars->SetNumberOfComponents(input_scalars->GetNumberOfComponents());
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
      original_point_ids->InsertNextValue(index);
      if (display_scalars)
      {
        display_scalars->InsertNextTuple(index, input_scalars);
      }
    }

    auto output = vtkSmartPointer<vtkPolyData>::New();
    output->SetPoints(points);
    output->GetPointData()->AddArray(original_point_ids);
    if (display_scalars && display_scalars->GetNumberOfTuples() > 0)
    {
      output->GetPointData()->SetScalars(display_scalars);
    }
    return output;
  }

  vtkSmartPointer<vtkPolyData> Point_Cloud_Renderer::Build_Interaction_Data(vtkPolyData *input) const
  {
    if (!input || input->GetNumberOfPoints() <= 0)
    {
      return nullptr;
    }
    const vtkIdType input_count = input->GetNumberOfPoints();
    if (input_count <= kMaximumInteractivePointCount)
    {
      return input;
    }

    const vtkIdType stride = std::max<vtkIdType>(1, (input_count + kMaximumInteractivePointCount - 1) / kMaximumInteractivePointCount);
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->SetDataType(input->GetPoints()->GetDataType());
    points->Allocate((input_count + stride - 1) / stride);

    auto *input_scalars = input->GetPointData() ? input->GetPointData()->GetScalars()
                                                 : nullptr;
    auto *input_original_point_ids = vtkIdTypeArray::SafeDownCast(
      input->GetPointData()->GetArray(kOriginalPointIdsName));
    auto original_point_ids = vtkSmartPointer<vtkIdTypeArray>::New();
    original_point_ids->SetName(kOriginalPointIdsName);
    original_point_ids->SetNumberOfComponents(1);
    original_point_ids->Allocate((input_count + stride - 1) / stride);
    vtkSmartPointer<vtkDataArray> colors;
    if (input_scalars)
    {
      colors.TakeReference(input_scalars->NewInstance());
      colors->SetName(input_scalars->GetName());
      colors->SetNumberOfComponents(input_scalars->GetNumberOfComponents());
      colors->Allocate((input_count + stride - 1) / stride);
    }

    double point[3] = {};
    for (vtkIdType index = 0; index < input_count; index += stride)
    {
      input->GetPoint(index, point);
      points->InsertNextPoint(point);
      original_point_ids->InsertNextValue(
        input_original_point_ids
          ? input_original_point_ids->GetValue(index)
          : index);
      if (colors)
        colors->InsertNextTuple(index, input_scalars);
    }
    auto output = vtkSmartPointer<vtkPolyData>::New();
    output->SetPoints(points);
    output->GetPointData()->AddArray(original_point_ids);
    if (colors)
      output->GetPointData()->SetScalars(colors);
    return output;
  }

  void Point_Cloud_Renderer::Add_Actor_To_Renderer()
  {
    if (m_renderer && m_actor)
    {
      m_renderer->AddActor(m_actor);
    }
    if (m_renderer && m_selection_actor)
    {
      m_renderer->AddActor(m_selection_actor);
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

    m_interaction_display_data = Build_Interaction_Data(m_display_data);
    m_interaction_point_count = m_interaction_display_data ? static_cast<std::size_t>(m_interaction_display_data->GetNumberOfPoints())
                                                           : m_point_count;
    if (m_interaction_point_count < m_point_count)
    {
      m_interaction_glyph_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
      m_interaction_glyph_filter->SetInputData(m_interaction_display_data);
      m_interaction_glyph_filter->Update();
    }

    m_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_mapper->SetInputConnection(m_glyph_filter->GetOutputPort());
    m_mapper->SetColorModeToDirectScalars();
    m_mapper->SetScalarModeToUsePointData();
    m_mapper->SetScalarVisibility(m_display_data->GetPointData()->GetScalars() != nullptr);

    m_actor = vtkSmartPointer<vtkActor>::New();
    m_actor->SetMapper(m_mapper);
    if (m_world_from_point_cloud)
    {
      m_actor->SetUserMatrix(m_world_from_point_cloud);
    }
    m_actor->GetProperty()->SetPointSize(kDefaultPointSize);
    m_actor->GetProperty()->SetOpacity(kDefaultOpacity);
    m_actor->GetProperty()->SetColor(0.10, 0.70, 0.95);
    m_actor->GetProperty()->SetAmbient(0.65);
    m_actor->GetProperty()->SetDiffuse(0.35);
    Add_Actor_To_Renderer();
  }

} // namespace point_cloud

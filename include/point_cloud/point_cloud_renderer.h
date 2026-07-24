#ifndef includeguard_point_cloud_renderer_h_includeguard
#define includeguard_point_cloud_renderer_h_includeguard

#include "pose_transform.h"
#include "point_cloud_selection.h"

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkDataArray.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPLYReader.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkVertexGlyphFilter.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <cstdint>
#include <vector>

namespace point_cloud
{

  class Point_Cloud_Renderer
  {
  public:
    void Attach_Renderer(vtkRenderer *renderer);
    void Detach_Renderer();

    bool Load_Ply(const std::filesystem::path &path, std::string *error_message = nullptr);
    bool Set_Point_Data(const std::vector<float> &xyz,
                        const std::vector<std::uint8_t> &rgb = {},
                        std::string *error_message = nullptr);
    std::vector<std::size_t> Select_Point_Ids_In_Display_Rect(
      vtkRenderer *renderer,
      int minimum_x,
      int minimum_y,
      int maximum_x,
      int maximum_y) const;
    std::vector<std::size_t> Select_Point_Ids_In_Display_Polygon(
      vtkRenderer *renderer,
      const std::vector<Display_Point> &polygon) const;
    void Set_Selected_Point_Ids(const std::vector<std::size_t> &point_ids);
    void Set_World_From_Point_Cloud(const robot_model::Matrix4 &world_from_point_cloud);
    void Set_Interactive_LOD(bool enabled);
    void Clear();

    bool Has_Point_Cloud() const { return m_actor != nullptr; }
    std::size_t Point_Count() const { return m_point_count; }
    std::size_t Displayed_Point_Count() const;
    std::size_t Interaction_Point_Count() const
    {
      return m_interaction_point_count;
    }
    bool Interactive_LOD_Enabled() const { return m_interactive_lod; }
    bool Get_World_Bounds(double bounds[6]) const;

  private:
    vtkSmartPointer<vtkPolyData> Build_Display_Data(vtkPolyData *input) const;
    vtkSmartPointer<vtkPolyData> Build_Interaction_Data(vtkPolyData *input) const;
    void Build_Actor();
    void Add_Actor_To_Renderer();

  private:
    vtkRenderer *m_renderer = nullptr;
    vtkSmartPointer<vtkPLYReader> m_reader;
    vtkSmartPointer<vtkPolyData> m_display_data;
    vtkSmartPointer<vtkVertexGlyphFilter> m_glyph_filter;
    vtkSmartPointer<vtkPolyData> m_interaction_display_data;
    vtkSmartPointer<vtkVertexGlyphFilter> m_interaction_glyph_filter;
    vtkSmartPointer<vtkPolyDataMapper> m_mapper;
    vtkSmartPointer<vtkActor> m_actor;
    vtkSmartPointer<vtkPolyData> m_selection_data;
    vtkSmartPointer<vtkVertexGlyphFilter> m_selection_glyph_filter;
    vtkSmartPointer<vtkPolyDataMapper> m_selection_mapper;
    vtkSmartPointer<vtkActor> m_selection_actor;
    vtkSmartPointer<vtkMatrix4x4> m_world_from_point_cloud;
    std::size_t m_point_count = 0;
    std::size_t m_interaction_point_count = 0;
    bool m_interactive_lod = false;
  };

} // namespace point_cloud

#endif
